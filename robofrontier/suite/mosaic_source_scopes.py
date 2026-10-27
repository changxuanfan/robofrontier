"""Interpret complete serial witnesses, including recorded same-world repairs.

Repair intervals are explicit serial barriers. Their physical commands remain
in the recipe; an omitted goal_attempt never means the repaired goal is copied.
"""


def source_scopes(actions, events, final_assignments, object_names):
    initial=[a for a in actions if a['label']=='clear-idle-arms']
    if len(initial)!=1 or not actions or actions[0] is not initial[0]:
        raise ValueError('Require exactly one initial parking before manipulation')
    if any(any(r['status']!='completed' for r in a['results'].values()) for a in actions):
        raise ValueError('Source needs explicit treatment of failed actions, never blind replay')
    previous=actions[0]['end_sim_s']
    for a in actions[1:]:
        if a['start_sim_s']<previous-1e-8:raise ValueError('Require a serial physical source witness')
        previous=a['end_sim_s']
    seen={};scopes=[];cursor=1
    for event in events:
        kind=event['event']
        if kind not in ('goal_attempt','mosaic_resume_from_actual_progress'):continue
        barrier=kind!='goal_attempt'
        pairs=(dict(event['protected_assignments']) if barrier else {event['goal']:event['object']})
        if any(final_assignments.get(g)!=name for g,name in pairs.items()):
            raise ValueError('Source scope and final physical object pairing disagree')
        if barrier and any(pairs.get(g)!=name for g,name in seen.items()):
            raise ValueError('Repair marker lost an earlier completed pairing')
        added={g:name for g,name in pairs.items() if g not in seen}
        commands=[]
        while cursor<len(actions) and actions[cursor]['end_sim_s']<=event['sim_time_s']+1e-8:
            commands.append(actions[cursor]);cursor+=1
        if not commands:
            if added:raise ValueError('New physical goal has no recorded command prefix')
            continue
        robots=sorted({r for a in commands for r in a['commands']})
        scopes.append(dict(goal_id=f'@review-{len(scopes)}' if barrier else event['goal'],
            object=None if barrier else event['object'],objects=list(object_names) if barrier else [event['object']],
            path=robots if barrier else event['path'],robots=robots,barrier=barrier,
            expected_assignments=added,actions=commands,end_sim_s=event['sim_time_s']))
        seen.update(added)
    if cursor!=len(actions):raise ValueError('Unassigned trailing physical actions')
    if seen!=final_assignments:raise ValueError('Recorded scope markers do not cover every final goal')
    return initial[0],scopes
