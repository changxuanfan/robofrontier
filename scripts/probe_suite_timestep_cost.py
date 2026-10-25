"""Equal-duration passive suite timing; no Oracle and no source-file edits.

Only in-memory timestep changes for Cable cases. No rendering, inference,
recording, or compile-time cost is included in timed stepping.
"""
import json,time
from pathlib import Path
import mujoco,numpy as np
from robofrontier.suite.v2 import ROOT,CATALOG
from robofrontier.workstation import Workstation
from robofrontier.robot_control import PandaArm
from robofrontier.physics_health import require_valid_step
from robofrontier.run_setup import write_json

def time_case(source,task,dt,seconds=.1,repeats=3):
 model=source.model;model.opt.timestep=dt;measurements=[]
 for _ in range(repeats):
  data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
  arms=[PandaArm(model,data,a) for a in task['arms']];stride=round(.02/dt);started=time.perf_counter()
  for step in range(round(seconds/dt)):
   if step%stride==0:
    for arm in arms:arm.control(.02)
   before=float(data.time);mujoco.mj_step(model,data);require_valid_step(data,before)
  measurements.append(time.perf_counter()-started)
 return dict(timestep_ms=dt*1000,simulation_s=seconds,repeats=repeats,wall_samples_s=measurements,median_wall_s=float(np.median(measurements)))

if __name__=='__main__':
 rows=[];cases=json.loads(CATALOG.read_text())['cases']
 for case in cases:
  source=Workstation(ROOT/case['directory'],verify_assets=False);task=json.loads((source.path/'task.json').read_text());original=float(source.model.opt.timestep);baseline=time_case(source,task,original)
  cable=case['family']=='relay' and case['level']>1
  comparison=time_case(source,task,.0002) if cable else baseline
  row=dict(case_id=case['id'],family=case['family'],cable=cable,current=baseline,cable_0p2ms=comparison);rows.append(row)
  print(json.dumps(dict(case_id=case['id'],current_wall_s=baseline['median_wall_s'],comparison_wall_s=comparison['median_wall_s'])),flush=True)
 current=sum(r['current']['median_wall_s'] for r in rows);comparison=sum(r['cable_0p2ms']['median_wall_s'] for r in rows);cable=sum(r['current']['median_wall_s'] for r in rows if r['cable'])
 families={f:dict(current=sum(r['current']['median_wall_s'] for r in rows if r['family']==f),comparison=sum(r['cable_0p2ms']['median_wall_s'] for r in rows if r['family']==f)) for f in ('transport','mosaic','relay','dial')}
 report=dict(scope='36 isolated scenes, each0.1 simulated seconds, median of3 repeats. Passive pose holding, no task commands, no Oracle. Timed stepping includes standard controller updates; excludes loading, recording, rendering and model inference. Initial-contact short workload only, not full-run cost prediction.',current_total_wall_s=current,comparison_total_wall_s=comparison,total_ratio=comparison/current,current_cable_cost_fraction=cable/current,per_family= families,cases=rows,formal_parameters_changed=False,oracle_started=False)
 write_json(ROOT/'docs/generated/frontier-v2/mechanism-checks/whole-suite-timestep-cost.json',report);print(json.dumps({k:v for k,v in report.items() if k!='cases'}),flush=True)
