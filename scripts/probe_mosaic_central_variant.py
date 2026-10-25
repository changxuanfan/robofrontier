"""Unconstrained central placement with measured-attitude/retreat lookahead."""
import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np

from scripts.probe_mosaic_cartesian_approach import base,Trial,FROZEN,ROOT


class CentralVariant(Trial):
    def place(self,robot,position,object_rotation=None,precise=False,pose_provider=None):
        name=self.held[robot];body=self.data.body(name);rotation=body.xmat.reshape(3,3).copy()
        arm=self.arms[robot];tool=self.data.site_xmat[arm.site].reshape(3,3).copy()
        offset=rotation.T@(self.data.site_xpos[arm.site]-body.xpos)
        height=self.release_clearance_height(robot,tool,float((rotation@offset)[2]))
        original=np.asarray(position).copy();grid=self.task['grid'];half=np.array([grid['columns'],grid['rows']])*grid['pitch_m']/2
        axis=int(np.argmax(abs(original[:2])/half));selected=None
        for steps in (0,1,2,3):
            point=original.copy();point[axis]-=np.sign(point[axis])*grid['pitch_m']*steps
            if np.any(abs(point[:2])>.5*half):
                # Still allow the original point; candidates move inward only.
                pass
            release=point+rotation@offset+[0,0,height]
            for seed in (arm.qref,self.home_q[robot]):
                q=self.screen_ik(robot,release,tool,seed=seed)
                back=None if q is None else self.screen_ik(robot,release+[0,0,.10],tool,seed=q)
                if back is None:continue
                margin=lambda joints:float(np.min(np.minimum(joints-arm.ik.limits[:,0],arm.ik.limits[:,1]-joints)))
                if min(margin(q),margin(back))<.01:continue
                selected=point;break
            if selected is not None:break
        if selected is None:raise RuntimeError('No finite central release and withdrawal IK pair')
        self.central_variant=dict(original_requested_m=original.tolist(),selected_m=selected.tolist(),
            preserve_measured_cube_orientation=True,withdrawal_ik_reserved=True,minimum_joint_margin_rad=.01)
        self.event('unit_central_variant',robot=robot,object=name,**self.central_variant)
        return super().place(robot,selected,rotation,False,None)

    def close(self,report):
        report['central_variant']=getattr(self,'central_variant',None)
        if report['central_variant'] is not None:
            report['original_requested_position_m']=report.get('requested_position_m')
            report['requested_position_m']=report['central_variant']['selected_m']
            closest=min(self.task['goals'],key=lambda g:np.linalg.norm(np.asarray(g['position_m'])-report['requested_position_m']))
            report['chosen_central_cell']=closest['goal_id']
        report['central_variant_source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        return super().close(report)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--items',nargs='+',required=True);p.add_argument('--output',type=Path,default=base.DEFAULT_OUT/'recovery-central-v1');a=p.parse_args()
    sources={str(p.relative_to(FROZEN)):p.read_bytes() for p in FROZEN.rglob('*') if p.is_file() and p.suffix in ('.py','.json')}
    for name in ['scripts/probe_mosaic_cartesian_approach.py','scripts/probe_mosaic_central_variant.py']:sources[name]=(ROOT/name).read_bytes()
    for name,payload in sources.items():
        dest=a.output/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        if dest.exists():
            if dest.read_bytes()!=payload:raise ValueError('Changed frozen source '+name)
        else:
            temp=dest.with_name(dest.name+f'.{os.getpid()}.tmp');temp.write_bytes(payload);os.replace(temp,dest)
    case=next(c for c in json.loads((ROOT/'configs/suites/frontier-mosaic-r13.json').read_text())['cases'] if c['id']==a.case)
    coverage=json.loads((ROOT/'runs/engineering/mosaic-r13-coverage-20261025'/case['id']/'report.json').read_text());base.Trial=CentralVariant
    for name in a.items:
        r=base.unit(case,name,a.output.resolve(),sources,coverage)
        print('CENTRAL RETEST',a.case,name,r['unit_passed'],r.get('error'),flush=True)
