"""Independent historical-unit continuation: choose a measured tool-axis exit.

Uses the original frozen physical controller/evaluator. The released cube stays
a fixed world obstacle during screening, rather than a stale carried prediction.
This does not change the environment or constitute an artwork Oracle pass.
"""
import argparse
from copy import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from scripts.probe_mosaic_cartesian_approach import base,Trial,FROZEN,ROOT
from robofrontier.suite.oracle import joint_edge_clear
from robofrontier.run_setup import write_json

SOURCE_SHA256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


class WithdrawVariant(Trial):
    def move(self,robot,target,rotation=None,label='move',precise=False,speed_m_s=None):
        if label!='place-withdraw':
            return super().move(robot,target,rotation,label,precise,speed_m_s)
        arm=self.arms[robot];name=self.held[robot]
        start=self.data.site_xpos[arm.site].copy()
        tool=self.data.site_xmat[arm.site].reshape(3,3).copy()
        proxy=copy(self);proxy.held=dict(self.held);proxy.held.pop(robot)
        delta=np.asarray(self.task['arms'][robot]['base_position_m'])-start
        diagonal=np.r_[delta[:2],.10]
        diagonal*=.10/np.linalg.norm(diagonal)
        for kind,offset in [('negative_actual_tool_z',-tool[:,2]*.10),('toward_base_up',diagonal)]:
            q=self.data.qpos[arm.qids].copy();valid=True;minimum=float('inf')
            for fraction in np.linspace(0,1,21)[1:]:
                point=start+offset*fraction
                nxt=self.screen_ik(robot,point,tool,seed=q)
                if (nxt is None or np.any(nxt<arm.ik.limits[:,0]) or np.any(nxt>arm.ik.limits[:,1]) or
                        not joint_edge_clear(q,nxt,lambda value:Trial.pose_clear(proxy,robot,value,allowed_objects=(name,)),step_rad=.005)):
                    valid=False;break
                q=nxt
                minimum=min(minimum,float(np.min(np.minimum(q-arm.ik.limits[:,0],arm.ik.limits[:,1]-q))))
            if not valid:continue
            self.withdraw_variant=dict(kind=kind,actual_start_m=start.tolist(),target_m=(start+offset).tolist(),
                released_object_screened_as_fixed_world_obstacle=True,samples=20,joint_sample_step_rad=.005,
                minimum_original_ik_margin_rad=minimum,controller_joint_limits_unchanged=True)
            self.event('unit_measured_withdrawal',robot=robot,object=name,**self.withdraw_variant)
            return super().move(robot,start+offset,tool,label,False,.035)
        raise RuntimeError('No screened measured-attitude withdrawal candidate')

    def close(self,report):
        report['withdraw_variant']=getattr(self,'withdraw_variant',None)
        report['withdraw_variant_source_sha256']=SOURCE_SHA256
        return super().close(report)


class ScreenedApproach(WithdrawVariant):
    def execute(self,label,commands,allow_failure=False):
        if label!='unit-approach':return super().execute(label,commands,allow_failure)
        robot=next(iter(commands));joints=np.asarray(commands[robot][1]['joint_rad'])
        target,rotation=self.arms[robot].ik.pose(joints)
        self.event('unit_slow_screened_approach',robot=robot,joint_speed_rad_s=.2,
            scope='Fresh canonical unit; full-arm route screening, no replay of old uncertain execution')
        self.move_free(robot,target,rotation,'unit-screened-approach',target_joints=joints,
            joint_speed_rad_s=.2,route_joint_step_rad=.005)
        return {}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',required=True)
    p.add_argument('--items',nargs='+',required=True);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--screened-approach',action='store_true');a=p.parse_args()
    sources={str(p.relative_to(FROZEN)):p.read_bytes() for p in FROZEN.rglob('*') if p.is_file() and p.suffix in ('.py','.json')}
    for name in ('scripts/probe_mosaic_cartesian_approach.py','scripts/probe_mosaic_withdraw_variant.py'):
        sources[name]=(ROOT/name).read_bytes()
    for name,payload in sources.items():
        dest=a.output/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        if dest.exists() and dest.read_bytes()!=payload:raise ValueError('Changed frozen source '+name)
        if not dest.exists():dest.write_bytes(payload)
    case=next(c for c in json.loads((ROOT/'configs/suites/frontier-mosaic-r13.json').read_text())['cases'] if c['id']==a.case)
    coverage=json.loads((ROOT/'runs/engineering/mosaic-r13-coverage-20261025'/case['id']/'report.json').read_text())
    base.Trial=ScreenedApproach if a.screened_approach else WithdrawVariant
    rows=[]
    for name in a.items:
        result=base.unit(case,name,a.output.resolve(),sources,coverage)
        rows.append(dict(case_id=a.case,object=name,passed=result['unit_passed'],error=result.get('error')))
        write_json(a.output/f'{a.case}-summary.json',dict(results=rows,scope='Historical frozen executor independent units; not HEAD artwork qualification'))
        print(json.dumps(rows[-1]),flush=True)
