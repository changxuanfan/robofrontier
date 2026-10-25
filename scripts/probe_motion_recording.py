"""Two-second passive record; zero robot task commands, no full-task Oracle."""
from pathlib import Path
import json,time
import mujoco,numpy as np
from robofrontier.suite.v2 import cases,directory,ROOT
from robofrontier.workstation import Workstation
from robofrontier.robot_control import PandaArm
from robofrontier.motion_recording import MotionWriter,MotionReader
from robofrontier.run_setup import write_json
if __name__=='__main__':
 source=directory(next(c for c in cases() if c['id']=='relay-a08-d3'));w=Workstation(source);w.frame();arms=[PandaArm(w.model,w.data,r) for r in json.loads((source/'task.json').read_text())['arms']]
 output=ROOT/'runs/engineering'/('motion-probe-'+time.strftime('%Y%m%d-%H%M%S'));output.mkdir(parents=True)
 writer=MotionWriter(output/'motion',source,w.model,w.data,cache=ROOT/'runs/state-assets');states=[w.data.qpos.astype(np.float32).copy()];started=time.monotonic();dt=w.model.opt.timestep
 for step in range(round(2/dt)):
  if step%round(.02/dt)==0:
   for arm in arms:arm.control(.02)
  before=float(w.data.time);mujoco.mj_step(w.model,w.data)
  if w.data.time<=before or not np.isfinite(w.data.qpos).all():raise RuntimeError('Passive numerical failure')
  if writer.capture(w.data):states.append(w.data.qpos.astype(np.float32))
 writer.close();decoded=[q for _,q in MotionReader(output/'motion').frames()];np.testing.assert_array_equal(states,decoded)
 compressed=sum(c['bytes'] for c in writer.meta['chunks']);raw=len(states)*w.model.nq*4
 write_json(output/'probe.json',dict(scope='Passive fixture recording only; no manipulation commands, no full-task Oracle',simulation_s=w.data.time,wall_s=time.monotonic()-started,states=len(states),raw_float32_bytes=raw,compressed_chunk_bytes=compressed,ratio=compressed/raw,roundtrip_exact_after_float32=True))
 write_json(ROOT/'docs/generated/frontier-v2/motion-probe-location.json',dict(recording=str(output/'motion'),report=str(output/'probe.json')));print(output,flush=True)
