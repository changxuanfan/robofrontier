"""Four independent passive physics/recording processes; no Oracle or agents."""
from concurrent.futures import ProcessPoolExecutor
import json,time,resource,sys
from pathlib import Path
import mujoco,numpy as np
from robofrontier.suite.v2 import cases,directory,ROOT
from robofrontier.workstation import Workstation
from robofrontier.robot_control import PandaArm
from robofrontier.motion_recording import MotionWriter
from robofrontier.run_setup import write_json

def probe(job):
 ident,output=job;source=Workstation(directory(next(c for c in cases() if c['id']==ident)));source.frame();task=json.loads((source.path/'task.json').read_text());arms=[PandaArm(source.model,source.data,a) for a in task['arms']];writer=MotionWriter(Path(output)/ident,source.path,source.model,source.data,cache=ROOT/'runs/state-assets')
 dt=source.model.opt.timestep;started=time.monotonic()
 for step in range(round(.20/dt)):
  if step%round(.02/dt)==0:
   for arm in arms:arm.control(.02)
  before=float(source.data.time);mujoco.mj_step(source.model,source.data)
  if source.data.time<=before or not np.isfinite(source.data.qpos).all():raise RuntimeError('Passive capacity probe unstable '+ident)
  writer.capture(source.data)
 state=writer.close();wall=time.monotonic()-started
 return dict(case_id=ident,simulation_s=float(source.data.time),physics_and_recording_wall_s=wall,realtime_factor=float(source.data.time)/wall,peak_process_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/(2**20 if sys.platform=='darwin' else 1024),recorded_frames=state['frames'],write_wall_s=state['write_wall_s'],passed=True)
if __name__=='__main__':
 import multiprocessing
 parent=ROOT/'runs/engineering'/('parallel-capacity-'+time.strftime('%Y%m%d-%H%M%S'));parent.mkdir(parents=True)
 ids=['transport-a12-d3','mosaic-a08-12x24-d3','relay-a16-d3','dial-n08-d3'];started=time.monotonic()
 with ProcessPoolExecutor(4,mp_context=multiprocessing.get_context('spawn')) as pool:rows=list(pool.map(probe,[(i,str(parent)) for i in ids]))
 report=dict(scope='Four processes, 0.2 simulated seconds each, passive pose holding + recording; no task actions, agents or full Oracle. Short-sample/load-specific performance, not a long-run guarantee.',passed=all(r['passed'] for r in rows),wall_s=time.monotonic()-started,processes=rows)
 write_json(ROOT/'docs/generated/frontier-v2/parallel-capacity.json',report);print(json.dumps(report),flush=True)
