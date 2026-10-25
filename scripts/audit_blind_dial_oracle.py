"""Audit completed blind runs without reading the answer book."""
import hashlib
import json
from pathlib import Path
import tempfile
import numpy as np
import mujoco
from robofrontier.motion_recording import MotionReader
from robofrontier.suite.dial_hold_oracle import BlindDecoder

ROOT=Path(__file__).resolve().parents[1];B=ROOT/'runs/oracle/dial-hold-blind-20261025'
rows=[]
for report in sorted(B.glob('dial-*/attempt-*/report.json')):
    directory=report.parent;r=json.loads(report.read_text());n=len(json.loads((directory/'task.json').read_text())['dials'])
    transcript=[json.loads(s) for s in (directory/'solver-io.jsonl').read_text().splitlines()]
    with tempfile.TemporaryDirectory() as tmp:
        decoder=BlindDecoder(n,tmp)
        try:
            for call in transcript:
                q=call['input'];output=call['output']
                if q['op']=='choose':
                    assert set(q)=={'op','current'}
                    actual=decoder.choose(q['current']);assert actual==output['guess']
                else:
                    assert q['op']=='feedback' and set(q)=={'op','guess','count'}
                    decoder.update(q['guess'],q['count'])
                assert len(decoder.candidates)==output['candidates']
        finally:decoder.close()
    events=[json.loads(s) for s in (directory/'trace.jsonl').read_text().splitlines()]
    releases=[e for e in events if e['event']=='action_start' and e.get('label') in ('red-release-open','crank-open-maximum')]
    reasons=[e for e in events if e['event'] in ('necessary_regrasp_admission','reference_turn_regrasp','reference_red_regrasp','reference_replan_rejected_arc','reference_replan_stopped_arc')]
    reader=MotionReader(directory/'motion');count=0;last=None
    model=mujoco.MjModel.from_xml_path(str(reader.path/'source/scene.xml'));data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id)
    for stamp,q in reader.frames():
        if count==0:assert stamp==0 and np.array_equal(q,np.asarray(data.qpos,dtype='<f4'))
        count+=1;last=stamp
    assert count==reader.meta['frames'];assert abs(reader.meta['physics_timestep_s']-.002)<1e-12
    win_seen=False
    for line in (directory/'evaluator-io.jsonl').open():
        e=json.loads(line);request=e['input'];result=e['output']
        assert set(result)=={'display','won','queries','since'}
        if result['display'] is not None and not win_seen:
            assert request['eligible'] and all(round(float(x)*180/np.pi/90)%4!=0 for x in request['angles'])
        win_seen=win_seen or result['won']
    if r['passed']:
        assert r['evaluation']['task_success'] and not r['evaluation']['violations'] and not r['unresolved_robot_actions']
        assert all(r['evaluation']['private_red_hold_ready']) and win_seen and reader.meta['status']=='complete'
    row=dict(case_id=r['case_id'],passed=r['passed'],error=r.get('error'),valid_trials=r['evaluation']['valid_trials'],
        solver_guesses=sum(c['input']['op']=='choose' for c in transcript),solver_replay_equal=True,answer_book_read_for_audit=False,
        actions=r['robot_action_count'],failed_actions=r['failed_robot_actions'],release_batches=len(releases),regrasp_reason_events=len(reasons),
        unexplained_release=bool(releases and not reasons),frames_verified=count,last_sim_s=last,wall_s=r['wall_time_s'],
        physical_control_sha256=r['physical_control_sha256'],new_evaluator_sha256=r['new_evaluator_sha256'],report=str(report))
    rows.append(row)
cases=[]
for cid in sorted({r['case_id'] for r in rows}):
    attempts=[r for r in rows if r['case_id']==cid]
    winners=[r for r in attempts if r['passed']]
    cases.append(dict(case_id=cid,passed=bool(winners),selected_report=(winners[-1] if winners else attempts[-1])['report'],attempts=len(attempts)))
payload=dict(protocol='dial-continuous-grasp-v1',completed=len(cases),total=9,passed=sum(r['passed'] for r in cases),attempt_count=len(rows),cases=cases,attempts=rows)
(B/'audit.json').write_text(json.dumps(payload,indent=2)+'\n')
print(json.dumps(payload,indent=2))
