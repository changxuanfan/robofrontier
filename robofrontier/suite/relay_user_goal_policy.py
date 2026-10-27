"""Goal contact and release planning from the current Relay contract.

These functions inspect geometry and actual support. They do not change physics,
monitor state or historical scores; completion remains the evaluator's job.
"""
import numpy as np
from robofrontier.suite.scenes import rz

from .relay_contract import BOUNDARY_TOLERANCE_M,LOW_RELEASE_GAP_M


def eligible_goal_cargo(t,data=None):
    data=t.data if data is None else data
    goal=t.task['route'][-1];center=np.asarray(goal['center_m'])
    rotation=rz(goal['nominal_yaw_rad']);half=np.asarray(goal['half_size_m'])
    names=[]
    for name,state in t.monitor.relay_v2.states.items():
        lo=np.full(2,np.inf);hi=np.full(2,-np.inf)
        for gid in state['all_geoms']:
            frame=data.geom_xmat[gid].reshape(3,3);aabb=t.model.geom_aabb[gid]
            point=rotation.T@(data.geom_xpos[gid]+frame@aabb[:3]-center)
            extent=(abs(rotation.T@frame)@aabb[3:])[:2]
            lo=np.minimum(lo,point[:2]-extent);hi=np.maximum(hi,point[:2]+extent)
        if np.all(lo>=-half-BOUNDARY_TOLERANCE_M) and np.all(hi<=half+BOUNDARY_TOLERANCE_M):names.append(name)
    return names


def terminal_contact_names(t,ident,data=None):
    if not t.task['completion'].get('allow_goal_payload_contact'):return []
    names=eligible_goal_cargo(t,data)
    return names if ident in names else []


def opening_supported(t,ident):
    relay=t.monitor.relay_v2;names=eligible_goal_cargo(t)
    if ident not in names:return False
    state=relay.states[ident]
    supported=ident in relay.supported_payloads(goal_only=True,require_all=False,inside_ids=set(names))
    gap=t.monitor.bottom(state['all_geoms'])-t.task['table_height_m']
    # A low final release need not wait for exact tabletop contact. Its actual
    # final landing/release is still checked independently after opening.
    near_surface=-.002<=gap<=t.task.get('completion',{}).get('max_release_gap_m',LOW_RELEASE_GAP_M)
    return bool(supported or near_surface)


def final_support(t,ident,team):
    state=t.monitor.relay_v2.states[ident]
    return bool(set(team)<=t.monitor.bilateral(state['rod_geoms'],positive_force=True)
        and opening_supported(t,ident))
