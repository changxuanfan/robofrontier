"""Read-only scheduling/command audit of complete Transport witnesses.

Report original timings separately from engineering handback waits. Conditional
bounds preserve the observed command set, durations and per-goal order; they do
not certify a globally optimal motion/task plan.
"""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import re


def analyze(path):
    report=json.loads(path.read_text());evaluation=report['evaluation'];trial=path.parent
    if not (report.get('passed') and report.get('full_task_attempt')):raise ValueError('Complete original-order witnesses only')
    task=json.loads((trial/'task.json').read_text());actions=json.loads((trial/'actions.json').read_text())
    events=[json.loads(line) for line in (trial/'trace.jsonl').read_text().splitlines()]
    key=lambda stamp,label,commands:(round(stamp,9),label,tuple(sorted(commands)))
    owners={key(e['sim_time_s'],e['label'],e['commands']):e.get('oracle_job') for e in events if e['event']=='action_start'}
    arm_work=defaultdict(float);goal_work=defaultdict(float);goal_commands=Counter();kinds=Counter();parks=0;park_work=0.;refinements=0;engineering_commands=0
    for action in actions:
        duration=action['end_sim_s']-action['start_sim_s'];label=action['label']
        owner=owners.get(key(action['start_sim_s'],label,action['commands']));match=re.match(r'^([A-Z]_slot[123])(?:-|$)',owner or '')
        goal=match.group(1) if match else None
        if goal:goal_work[goal]+=duration;goal_commands[goal]+=len(action['commands'])
        for robot,(operation,_) in action['commands'].items():
            kinds[operation]+=1;arm_work[robot]+=duration
            if label.startswith('park'):parks+=1;park_work+=duration
            if '-refine' in label:refinements+=1
            if label=='engineering-repair':engineering_commands+=1
    assert sum(kinds.values())==report['robot_action_count']
    start=None;handback=[];failures=Counter()
    for e in events:
        if e['event']=='transport_engineering_handback' and start is None:start=e['sim_time_s']
        elif e['event']=='transport_job_admitted' and start is not None:
            handback.append([start,e['sim_time_s']]);start=None
        if e['event']=='transport_job_failed':
            error=e.get('error','')
            category=('scope_or_route' if error.startswith(('Retained carrier has no finite','No current-state pickup','list index')) else
                      'candidate_search' if error.startswith('No bounded physical air-regrasp') else
                      'motion_screen' if error.startswith(('No collision-free','No clear free-arm','No pickup','No radial')) else
                      'recovery_boundary' if error.startswith('Dual grasp frontier') else 'other')
            failures[category]+=1
    sim=float(evaluation['sim_time_s']);wait=sum(max(0,b-a) for a,b in handback);motion=float(evaluation.get('robot_motion_s',0));moving=float(evaluation.get('any_motion_sim_s',0));overlap=float(evaluation.get('motion_overlap_sim_s',0));arms=len(task['arms'])
    fixed_bound=max(max(arm_work.values(),default=0),max(goal_work.values(),default=0),sum(arm_work.values())/arms)
    return dict(case=report['case_id'],report=str(path.resolve()),source=report['git_commit'],engineering_assisted=report.get('engineering_assisted',False),goals=evaluation['matched_goals'],arms=arms,
        commands=report['robot_action_count'],commands_by_operation=dict(kinds),commands_per_goal=report['robot_action_count']/evaluation['matched_goals'],known_failed_commands=report['failed_robot_actions'],
        wall_s=report['wall_time_s'],original_sim_s=sim,recorded_engineering_handback_sim_s=wait,sim_excluding_recorded_handback_s=sim-wait,
        peak_moving_arms=evaluation.get('peak_moving_robots'),mean_moving_arms_while_any_moves=motion/moving if moving else 0,
        fraction_motion_time_with_multiple_arms=overlap/moving if moving else 0,robot_motion_work_s=motion,any_motion_s=moving,
        explicit_park_commands=parks,explicit_park_robot_command_s=park_work,precision_refine_commands=refinements,owner_prefix_commands=engineering_commands,
        initial_private_plan_hops=sum(max(0,len(j['path'])-1) for e in events if e['event']=='transport_private_plan' for j in e['jobs']),
        explicit_air_carrier_retained_events=sum(e['event']=='reference_air_regrasp_carrier_retained' for e in events),planning_and_recovery_job_failures=dict(failures),
        fixed_command_schedule_relaxed_bound_s=fixed_bound,per_arm_command_work_s=dict(arm_work),largest_goal_command_chains=sorted([dict(goal=g,command_s=duration,commands=goal_commands[g]) for g,duration in goal_work.items()],key=lambda x:-x['command_s'])[:3],
        definitions='Motion: native20ms/max-joint-rate>0.02rad/s. Park/refine counts use actual command labels. Command-work sums batch durations; initial setup batch is a shared barrier. Handback exclusion is an analysis view, never an official timing replacement. Conditional bound ignores collision/settling/holding/resource-chain constraints and assumes unchanged observed commands and per-goal serial order; not global optimality. Explicit air events may omit repaired handovers.')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('batch',type=Path);args=parser.parse_args();batch=args.batch.resolve()
    index=json.loads((batch/'current.json').read_text());rows=[analyze(Path(r['report'])) for r in index['qualified']]
    result=dict(scope='Completed Transport original-order witnesses only; one separate physical run per row; no score stitching or near-optimality claim',cases=rows)
    (batch/'efficiency.json').write_text(json.dumps(result,indent=2)+'\n')
    lines=['# Transport efficiency audit','', 'All rows are complete original-order witnesses. These measurements do not certify near-optimal scheduling.','', '| Case | Commands | Commands/goal | Mean moving arms¹ | Peak | Multi-arm fraction¹ | Explicit park commands | Original sim s | Owner handback s |','|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in rows:lines.append(f"| {r['case']} | {r['commands']} | {r['commands_per_goal']:.1f} | {r['mean_moving_arms_while_any_moves']:.2f} | {r['peak_moving_arms']} | {r['fraction_motion_time_with_multiple_arms']:.1%} | {r['explicit_park_commands']} | {r['original_sim_s']:.1f} | {r['recorded_engineering_handback_sim_s']:.1f} |")
    lines+=['','¹Conditioned on at least one arm actually moving; native20ms joint-motion definition.','', 'Priorities supported by the code and trace:', '', '- Separate useful parallel motion from peak concurrency and passive force holding.', '- Evaluate parking after each handover: the current program waits for sender parking before receiver carrying. Parallel cleanup or validated local retreat could shorten the critical path.', '- Whole future routes are leased before pickup. Compare critical-path priority and shorter reservations against the current cheap-job-first policy; retain collision and deadlock protection.', '- Avoid repeated failed routes/grips using actual source reach, current grasp-to-socket feasibility and measured successful transfer costs.', '- Treat direct placement and air handover as preferred candidates. Count supported regrasp costs explicitly and judge buffers by measured makespan/throughput, not only action count.', '', 'Engineering handback time is preserved in original reports and excluded only in the separate analysis field. Conditional command-schedule bounds in efficiency.json ignore collision, holding and settling constraints, and are not a global optimization gap.']
    (batch/'EFFICIENCY.md').write_text('\n'.join(lines)+'\n');print(json.dumps([dict(case=r['case'],commands=r['commands'],mean_moving=round(r['mean_moving_arms_while_any_moves'],2),park_commands=r['explicit_park_commands'],owner_wait_s=round(r['recorded_engineering_handback_sim_s'],1)) for r in rows]))


if __name__=='__main__':main()
