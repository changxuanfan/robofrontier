"""Fresh independent retest with a screened Cartesian empty-hand approach.

Do not alter/replay an uncertain old call. Keep its sealed world report and
all physical parameters. Only the ordinary motion interface/route changes.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

import numpy as np

# Other task families can evolve in this shared checkout. Load the exact
# evaluator used by the still-running baseline rather than reverting it.
FROZEN=(Path(__file__).resolve().parents[1]/'runs/engineering/mosaic-r13-physical-pick-place-20261025/batch-v4/source').resolve()
import robofrontier as root_package
root_package.__path__=[str(FROZEN/'robofrontier'),*list(root_package.__path__)]
import scripts as scripts_package
scripts_package.__path__=[str(FROZEN/'scripts'),*list(scripts_package.__path__)]
import robofrontier.suite as suite_package
suite_package.__path__=[str(FROZEN/'robofrontier/suite'),*list(suite_package.__path__)]
spec=importlib.util.spec_from_file_location('robofrontier.suite.evaluate',FROZEN/'robofrontier/suite/evaluate.py')
frozen_evaluator=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=frozen_evaluator;spec.loader.exec_module(frozen_evaluator);suite_package.evaluate=frozen_evaluator

import scripts.probe_mosaic_pick_place as base
from robofrontier.suite.oracle import Trial,joint_edge_clear
from robofrontier.robot_control import matrix_quat,quat_matrix
from robofrontier.suite.v2 import ROOT
from robofrontier.run_setup import write_json


class CartesianApproach(Trial):
    def execute(self,label,commands,allow_failure=False):
        if label!='unit-approach':return super().execute(label,commands,allow_failure)
        robot=next(iter(commands));name=self.directory.parent.name
        cover=json.loads((ROOT/'runs/engineering/mosaic-r13-coverage-20261025'/self.task['id']/'report.json').read_text())
        witness=next(x['witness'] for x in cover['sources'] if x['id']==name)
        target_r=np.asarray(witness['tool_rotation']);target=self.arms[robot].ik.pose(np.asarray(witness['hover_joint_rad']))[0]
        initial=self.data.qpos[self.arms[robot].qids].copy();start,r0=self.arms[robot].ik.pose(initial)
        qa,qb=matrix_quat(r0),matrix_quat(target_r)
        if qa@qb<0:qb=-qb
        variants=[qb,-qb] if abs(qa@qb)<1e-6 else [qb]
        approved=False
        for last in variants:
            q=initial.copy();ok=True;count=0
            for fraction in np.linspace(0,1,33)[1:]:
                quat=(1-fraction)*qa+fraction*last;quat/=np.linalg.norm(quat)
                nxt=self.screen_ik(robot,start,quat_matrix(quat),seed=q)
                if nxt is None or not joint_edge_clear(q,nxt,lambda x:self.pose_clear(robot,x)):
                    ok=False;break
                q=nxt;count+=1
            if not ok:continue
            for fraction in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(target-start)/.01))+1))[1:]:
                nxt=self.screen_ik(robot,start+(target-start)*fraction,target_r,seed=q)
                if nxt is None or not joint_edge_clear(q,nxt,lambda x:self.pose_clear(robot,x)):
                    ok=False;break
                q=nxt;count+=1
            if ok:approved=True;break
        position_first=False
        if not approved:
            # An orientation can be awkward at the old home point. First
            # reach the screened hover position while retaining the old wrist
            # angle, then rotate there using the real Cartesian interface.
            near=np.asarray(witness['hover_joint_rad']).copy();near[-1]=initial[-1]
            reached,near_r=self.arms[robot].ik.pose(near)
            if np.linalg.norm(reached-target)<.0001 and joint_edge_clear(initial,near,lambda x:self.pose_clear(robot,x)):
                first,last=matrix_quat(near_r),matrix_quat(target_r)
                if first@last<0:last=-last
                q=near.copy();ok=True
                for fraction in np.linspace(0,1,49)[1:]:
                    quat=(1-fraction)*first+fraction*last;quat/=np.linalg.norm(quat)
                    nxt=self.screen_ik(robot,target,quat_matrix(quat),seed=q)
                    if nxt is None or not joint_edge_clear(q,nxt,lambda x:self.pose_clear(robot,x)):
                        ok=False;break
                    q=nxt
                if ok:approved=True;position_first=True
        if not approved:raise RuntimeError('Cartesian approach failed full-arm geometric admission')
        self.event('unit_cartesian_approach',robot=robot,object=name,samples=count,
            scope='Existing Cartesian pose commands; original controller completion criteria and physics unchanged')
        if position_first:
            super().execute('unit-position-first-hover',{robot:('move',dict(joint_rad=near.tolist(),joint_speed_rad_s=.65))})
            self.move(robot,target,target_r,'unit-contact-orient-at-hover',speed_m_s=.10)
        else:
            self.move(robot,start,target_r,'unit-contact-orient-hover',speed_m_s=.10)
            self.move(robot,target,target_r,'unit-contact-cartesian-hover',speed_m_s=.10)
        return {}

    def close(self,report):
        previous=ROOT/'runs/engineering/mosaic-r13-physical-pick-place-20261025/batch-v4'/self.task['id']/self.directory.parent.name/'attempt-001/report.json'
        report['approach_variant']=dict(name='screened-empty-hand-cartesian-v3-frozen-evaluator',source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            previous_independent_episode_report=str(previous),previous_report_sha256=hashlib.sha256(previous.read_bytes()).hexdigest() if previous.exists() else None,
            old_uncertain_execution_not_replayed_or_cleared=True,explicit_fresh_canonical_episode=True)
        return super().close(report)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--items',nargs='+',required=True);p.add_argument('--output',type=Path,default=base.DEFAULT_OUT/'recovery-cartesian-v3');args=p.parse_args()
    sources={str(p.relative_to(FROZEN)):p.read_bytes() for p in FROZEN.rglob('*') if p.is_file()}
    for module_name in ['scripts.probe_mosaic_pick_place','robofrontier.suite.oracle','robofrontier.suite.evaluate','robofrontier.suite.provenance','robofrontier.robot_control','robofrontier.sim','robofrontier.motion_check','robofrontier.task_config']:
        module=sys.modules[module_name]
        if not Path(module.__file__).resolve().is_relative_to(FROZEN):raise ValueError('Baseline module not frozen '+module_name)
    sources['scripts/probe_mosaic_cartesian_approach.py']=Path(__file__).read_bytes()
    for name,payload in sources.items():
        archive=args.output/'source'/name;archive.parent.mkdir(parents=True,exist_ok=True)
        if archive.exists():
            if archive.read_bytes()!=payload:raise ValueError('Changed frozen variant source '+name)
        else:
            temp=archive.with_name(archive.name+f'.{os.getpid()}.tmp');temp.write_bytes(payload);os.replace(temp,archive)
    cases=json.loads((ROOT/'configs/suites/frontier-mosaic-r13.json').read_text())['cases'];case=next(c for c in cases if c['id']==args.case)
    coverage=json.loads((ROOT/'runs/engineering/mosaic-r13-coverage-20261025'/case['id']/'report.json').read_text());base.Trial=CartesianApproach
    if hashlib.sha256((ROOT/case['directory']/'scene.xml').read_bytes()).hexdigest()!=coverage['source_scene_sha256']:raise ValueError('Scene changed since coverage')
    for name in args.items:
        r=base.unit(case,name,args.output.resolve(),sources,coverage)
        print('RETEST',case['id'],name,r['unit_passed'],r.get('error'),flush=True)
