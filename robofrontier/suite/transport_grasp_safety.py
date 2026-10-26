"""Measured Transport pickup support and known-fault handback classification."""
import numpy as np


class PickupSupportLost(RuntimeError):
    def __init__(self,robot,name,stage,before,after,supported):
        self.robot,self.object,self.stage=robot,name,stage
        self.before_z,self.after_z,self.positive_support=float(before),float(after),bool(supported)
        super().__init__(f'Transport pickup support lost {robot}/{name} at {stage}: '
            f'z {before:.6f}->{after:.6f}, positive bilateral support={supported}')


def requires_contact_recovery(error):
    if isinstance(error,PickupSupportLost):return True
    return any(text in str(error) for text in (
        'Grasp stopped outside expected object-width tolerance',
        'Requested opening not reached','Requested aperture exceeded',
        'Bilateral cube contact missing','Bilateral cylinder contact missing',
        'Air-regrasp requires actual bilateral support'))


def guarded_pickup_lift(trial,robot,name,before_z,rotation,speed_m_s=.025):
    """Finish a short known action and verify real support before the main lift.

    No controller, world state, force bound or acceptance rule is changed.
    Failure is raised only after the submitted action has returned a result.
    """
    arm=trial.arms[robot];start=trial.data.site_xpos[arm.site].copy()
    for height,label,required in ((.02,'pick-lift-probe',.01),(.14,'pick-lift',.08)):
        trial.move(robot,start+np.array([0.,0.,height]),rotation,label,
            speed_m_s=min(speed_m_s,.015) if height==.02 else speed_m_s)
        trial.wait(.1)
        supported=robot in trial.monitor.bilateral(trial.monitor.geoms[name],positive_force=True)
        after=float(trial.data.body(name).xpos[2])
        if not supported or after<float(before_z)+required:
            raise PickupSupportLost(robot,name,label,before_z,after,supported)
