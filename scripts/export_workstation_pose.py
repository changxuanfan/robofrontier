"""Export an initial or saved-qpos pose frame without running a simulation."""
import argparse,json
from pathlib import Path
import numpy as np
from robofrontier.workstation import Workstation

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('workstation');p.add_argument('--states');p.add_argument('--frame',type=int,default=0);p.add_argument('--translation',type=float,nargs=3,default=[0,0,0]);p.add_argument('--yaw',type=float,default=0);p.add_argument('--output',required=True);a=p.parse_args()
    source=Workstation(a.workstation);q=None;t=0.
    if a.states:
        states=np.load(a.states);q=states['qpos'][a.frame];t=float(states['time_s'][a.frame])
    result=source.frame(q,t,a.translation,a.yaw);Path(a.output).write_text(json.dumps(result,indent=2)+'\n')
