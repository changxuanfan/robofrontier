"""Oracle reference preflight only; not part of a model episode."""
from pathlib import Path
from tempfile import TemporaryDirectory
import json
import numpy as np
from robofrontier.pipeline import World, ARMS, OBJECTS


def settle(w, steps):
    for _ in range(steps):w.step()


def move(w, goals, max_steps=1500):
    for a,g in goals.items():w.arms[a].goal=np.array(g);w.arms[a].speed=.06
    for step in range(max_steps):
        w.step()
        errors={a:float(np.linalg.norm(w.data.site_xpos[w.arms[a].site]-g)) for a,g in goals.items()}
        if max(errors.values())<.0025:
            for a in goals:
                r=w.arms[a];r.goal=None;r.velocity[:]=0
            settle(w,20)
            return step,errors
    raise AssertionError(('tracking',errors))


def main():
    report={'scope':'Oracle engineering preflight, not model score','checks':{}}
    with TemporaryDirectory() as d:
        p=Path(d);(p/'private').mkdir();w=World(p)
        initial=np.array([w.data.body(o[0]).xpos.copy() for o in OBJECTS]);settle(w,500)
        drift=max(np.linalg.norm(w.data.body(o[0]).xpos-initial[i]) for i,o in enumerate(OBJECTS))
        assert drift<.001;report['checks']['idle_10_sim_seconds_max_drift_m']=float(drift)
        before=w.data.time
        move(w,{a:[.42,y-.1,.56] for a,y in ARMS.items()})
        move(w,{a:[.42,y-.1,.423] for a,y in ARMS.items()})
        for r in w.arms.values():r.grip_goal=0
        settle(w,80)
        move(w,{a:[.42,y-.1,.58] for a,y in ARMS.items()})
        settle(w,100)
        lifts={a:float(w.data.body(a+'_gray').xpos[2]-.42) for a in ARMS}
        assert min(lifts.values())>.10,lifts
        report['checks']['three_simultaneous_real_contact_grasps_lift_m']=lifts
        report['checks']['concurrent_sequence_sim_seconds']=float(w.data.time-before)
        move(w,{'B':[.57,-.1,.58]})
        move(w,{'B':[.57,-.1,.423]})
        w.arms['B'].grip_goal=255;settle(w,80)
        move(w,{'B':[.4,0,.68]})
        # Shared-region reach and physical placement with A, then pickup by B.
        move(w,{'A':[.5,-.325,.58]})
        move(w,{'A':[.5,-.325,.423]})
        print('relay_A_before_open',w.data.body('A_gray').xpos.tolist(),w.arms['A'].observe(),flush=True)
        w.arms['A'].grip_goal=255;settle(w,80)
        print('relay_A_after_open',w.data.body('A_gray').xpos.tolist(),flush=True)
        move(w,{'A':[.4,-.65,.68]})
        move(w,{'B':[.5,-.325,.58]})
        move(w,{'B':[.5,-.325,.423]})
        print('relay_B_before_close',w.data.body('A_gray').xpos.tolist(),w.arms['B'].observe(),flush=True)
        w.arms['B'].grip_goal=0;settle(w,80)
        print('relay_B_after_close',w.data.body('A_gray').xpos.tolist(),w.arms['B'].observe(),flush=True)
        move(w,{'B':[.5,-.325,.58]});settle(w,100)
        lift=float(w.data.body('A_gray').xpos[2]-.42)
        assert lift>.10,lift
        report['checks']['adjacent_relay_A_to_B_lift_m']=lift
        report['checks']['finite_state']=bool(np.isfinite(w.data.qpos).all());assert report['checks']['finite_state']
    path=Path('runs/pipeline-preflight.json');path.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
