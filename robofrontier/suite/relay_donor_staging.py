"""Defer optional donor homing after a fully clear, real opening.

The hand stays at its measured joint pose. Subsequent transport still screens
that stationary hand and all other geometry. No simulated state is changed.
"""
from copy import copy
import numpy as np
import mujoco


def parked_donor_routes(t, ident, selected, donors, opening_width=.075):
    if len(donors)!=1 or len(selected)!=2:return None
    probe=copy(t);probe.data=mujoco.MjData(t.model)
    probe.data.qpos[:]=t.data.qpos
    probe.screen_data=mujoco.MjData(t.screen_model)
    probe.held={a:p for a,p in t.held.items() if a not in donors}
    fingers={t.model.body(a+'_'+s).id for a in donors for s in ('left_finger','right_finger')}
    timber=set(t.monitor.relay_v2.states[ident]['rod_geoms']);baseline={}
    for contact in t.data.contact:
        a,b=map(int,contact.geom)
        if ((a in timber and int(t.model.geom_bodyid[b]) in fingers) or
                (b in timber and int(t.model.geom_bodyid[a]) in fingers)):
            key=tuple(sorted((a,b)))
            baseline[key]=min(baseline.get(key,0.),float(contact.dist))
    for item in selected:
        robot=item['robot'];probe.data.qpos[t.arms[robot].qids]=item['contact_q']
        for suffix in ('finger_joint1','finger_joint2'):
            probe.data.joint(robot+'_'+suffix).qpos[0]=(t.task['mechanism']['cross_section_m'][0]-t.arms[robot].grip_min)/2
    initial={a:[float(probe.data.joint(a+'_'+s).qpos[0]) for s in
        ('finger_joint1','finger_joint2')] for a in donors}
    for fraction in np.linspace(0,1,21):
        for robot in donors:
            for i,suffix in enumerate(('finger_joint1','finger_joint2')):
                probe.data.joint(robot+'_'+suffix).qpos[0]=(1-fraction)*initial[robot][i]+fraction*(opening_width-t.arms[robot].grip_min)/2
        mujoco.mj_forward(t.model,probe.data)
        for contact in probe.data.contact:
            limit=min(-.0005,baseline.get(tuple(sorted(map(int,contact.geom))),0.)-.0001)
            if contact.dist<limit and any(int(t.model.geom_bodyid[g]) in fingers for g in contact.geom):return None
    routes={}
    for robot in donors:
        q=probe.data.qpos[t.arms[robot].qids].copy()
        # No target-contact allowance: the open donor must already be clear.
        if not probe.pose_clear(robot,q,(),.001):return None
        routes[robot]=[q]
    return routes
