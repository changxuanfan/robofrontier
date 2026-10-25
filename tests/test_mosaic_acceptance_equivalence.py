"""Differential geometry checks against the preserved scalar evaluator."""
import json,math,unittest
from types import SimpleNamespace
import numpy as np
import mujoco
from robofrontier.suite.v2 import cases,directory
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite.evaluate_v2 import placement_candidates
from robofrontier.robot_control import matrix_quat
from robofrontier.workstation import Workstation
from tests.fixtures.mosaic_scalar_reference import placement_candidates as scalar


def world(case_id):
    source=Workstation(directory(next(c for c in cases() if c['id']==case_id)));source.frame()
    task=json.loads((source.path/'task.json').read_text())
    return Monitor(SimpleNamespace(model=source.model,data=source.data,task=task,arms={}))


def seat_all(mon):
    remaining=list(range(len(mon.objects)))
    for goal in mon.task['goals']:
        j=next(j for j in remaining if all(mon.objects[j].get(k)==v for k,v in goal['requires'].items()))
        remaining.remove(j);q=mon.d.joint(mon.objects[j]['id']+'_free').qpos
        q[:3]=goal['position_m'];q[3:]=[1,0,0,0]
    mujoco.mj_forward(mon.m,mon.d)


class MosaicAcceptanceEquivalenceTests(unittest.TestCase):
    def compare(self,mon,stable):
        positions=mon.d.xpos[mon.bodies].copy()
        expected=scalar(mon,positions,stable);actual=placement_candidates(mon,positions,stable)
        np.testing.assert_array_equal(actual,expected)
        return actual

    def test_all_nine_scenes_and_disturbed_placements(self):
        rng=np.random.default_rng(10052)
        for case in [c for c in cases() if c['family']=='mosaic']:
            with self.subTest(case=case['id']):
                mon=world(case['id']);n=len(mon.names)
                self.compare(mon,[True]*n);seat_all(mon)
                self.assertEqual(int(self.compare(mon,[True]*n).sum()),n)
                for j in range(min(n,24)):
                    q=mon.d.joint(mon.names[j]+'_free').qpos
                    q[:2]+=rng.uniform(-.013,.013,2);q[2]+=rng.uniform(-.001,.001)
                    angle=rng.choice([0.,math.radians(5),math.radians(8),math.radians(9)])
                    c,s=math.cos(angle),math.sin(angle)
                    q[3:]=matrix_quat(np.array([[c,-s,0],[s,c,0],[0,0,1]]))
                mujoco.mj_forward(mon.m,mon.d)
                self.compare(mon,rng.random(n)>.15)
                # No stale requirement/colour cache after metadata changes.
                mon.objects[0]['color']='unmatched-test-colour'
                mon.task['goals'][0]['requires']['extra']='required-value'
                self.compare(mon,[True]*n)
                self.assertFalse(self.compare(mon,[False]*n).any())

    def test_float_and_contact_boundaries(self):
        for level in (1,2,3):
            mon=world(f'mosaic-a04-6x12-d{level}');g=mon.task['goals'][0]
            j=next(i for i,o in enumerate(mon.objects) if all(o.get(k)==v for k,v in g['requires'].items()))
            stable=[False]*len(mon.names);stable[j]=True;q=mon.d.joint(mon.names[j]+'_free').qpos
            offsets=[0.,.005,np.nextafter(.005,0),np.nextafter(.005,np.inf),.0103,.0103001]
            for x in offsets:
                for dz in (0.,.00015,.00025,.0008,.003,np.nextafter(.003,np.inf)):
                    q[:]=np.r_[np.asarray(g['position_m'])+[x,0,dz],[1,0,0,0]]
                    mujoco.mj_forward(mon.m,mon.d);self.compare(mon,stable)


if __name__=='__main__':unittest.main()
