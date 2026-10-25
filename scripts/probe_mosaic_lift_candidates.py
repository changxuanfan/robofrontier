"""Fresh units with bounded legal lift-goal selection after IK rejection."""
import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np

# Reuse the audited frozen-module overlay, never the evolving shared code.
from scripts.probe_mosaic_cartesian_approach import base,Trial,FROZEN,ROOT


class LiftCandidates(Trial):
    def move(self,robot,target,rotation=None,label='move',precise=False,speed_m_s=None):
        if label!='unit-lift':return super().move(robot,target,rotation,label,precise,speed_m_s)
        origin=self.data.site_xpos[self.arms[robot].site].copy()
        measured=self.data.site_xmat[self.arms[robot].site].reshape(3,3).copy()
        for height in (.14,.12,.16,.18,.10):
            before_time=self.time;before_actions=len(self.actions)
            self.event('unit_lift_candidate',robot=robot,height_m=height,source='Measured current tool pose; ordinary Cartesian command')
            try:
                return super().move(robot,origin+[0,0,height],measured,'unit-lift-selected',precise,speed_m_s)
            except ValueError as error:
                # Continue only when validation rejected BEFORE any execution.
                if self.time!=before_time or len(self.actions)!=before_actions or self.arms[robot].job is not None:raise
                self.event('unit_lift_validation_rejected',robot=robot,height_m=height,reason=str(error))
        raise RuntimeError('No verified IK for the finite lift candidates')

    def close(self,report):
        previous=ROOT/'runs/engineering/mosaic-r13-physical-pick-place-20261025/batch-v4'/self.task['id']/self.directory.parent.name/'attempt-001/report.json'
        report['lift_variant']=dict(name='measured-pose-finite-lift-v1',source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),previous_report=str(previous),
            no_reset_or_unknown_replay=True,ordinary_robot_parameters_only=True)
        return super().close(report)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--items',nargs='+',required=True);p.add_argument('--output',type=Path,default=base.DEFAULT_OUT/'recovery-lift-v1');a=p.parse_args()
    sources={str(p.relative_to(FROZEN)):p.read_bytes() for p in FROZEN.rglob('*') if p.is_file()}
    for name in ['scripts/probe_mosaic_cartesian_approach.py','scripts/probe_mosaic_lift_candidates.py']:sources[name]=(ROOT/name).read_bytes()
    for name,payload in sources.items():
        dest=a.output/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        if dest.exists():
            if dest.read_bytes()!=payload:raise ValueError('Changed frozen source '+name)
        else:
            temp=dest.with_name(dest.name+f'.{os.getpid()}.tmp');temp.write_bytes(payload);os.replace(temp,dest)
    cases=json.loads((ROOT/'configs/suites/frontier-mosaic-r13.json').read_text())['cases'];case=next(c for c in cases if c['id']==a.case)
    coverage=json.loads((ROOT/'runs/engineering/mosaic-r13-coverage-20261025'/case['id']/'report.json').read_text());base.Trial=LiftCandidates
    for name in a.items:
        r=base.unit(case,name,a.output.resolve(),sources,coverage)
        print('LIFT RETEST',a.case,name,r['unit_passed'],r.get('error'),flush=True)
