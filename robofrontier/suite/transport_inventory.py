"""Read-only remaining-stock reach check for engineering episodes."""
import numpy as np


class UnreachableTransportInventory(RuntimeError):
    pass


def unreachable_required_inventory(trial,padding_m=.12):
    """Conservatively detect stock far outside the admitted TCP workspace.

    The padding exceeds the object/finger grasp offsets in this suite. This
    does not change the workspace, physics, inventory or final acceptance.
    It stops a doomed attempt instead of repeatedly planning for lost stock.
    """
    workspace=trial.task['workspace_world_m'];lower=np.asarray(workspace['lower']);upper=np.asarray(workspace['upper'])
    accepted=set(trial.monitor.result().get('assignments',{}).values());rows=[]
    for obj in trial.env['objects']:
        name=obj['id']
        if name in accepted:continue
        point=np.asarray(trial.data.body(name).xpos)
        if np.any(point<lower-padding_m) or np.any(point>upper+padding_m):
            rows.append(dict(object=name,position_m=point.tolist()))
    return rows
