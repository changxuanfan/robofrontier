"""Verify authentic recorded task completion without stopping the physical owner."""
import argparse,json,math,time
from pathlib import Path
import numpy as np,mujoco
from robofrontier.motion_recording import MotionReader,digest
from scripts.diagnose_relay_descent import diagnostic_world
from robofrontier.suite.relay_user_goal_policy import eligible_goal_cargo

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('final_request');a=p.parse_args();run=a.run.resolve();feedback_path=run/'recovery/results'/(a.final_request+'.json');feedback=json.loads(feedback_path.read_text());assert feedback['status']=='completed' and not feedback.get('unresolved');cutoff=feedback['end_sim_s']+1.;current=json.loads((run/'user-goal-report.json').read_text());assert current['passed'];reader=MotionReader(run/'motion');assert reader.meta['chunks'][-1]['end_sim_s']>=cutoff
 interval=math.ceil(1/reader.meta['sample_hz']/reader.meta['physics_timestep_s'])*reader.meta['physics_timestep_s'];previous=None;first=None;last=None;frames=0;chunks=0;gap=0.;began=time.monotonic()
 for chunk in reader.meta['chunks']:
  if chunk['start_sim_s']>cutoff:break
  file=reader.path/chunk['file'];assert digest(file)==chunk['sha256']
  with np.load(file,allow_pickle=False) as z:
   q=np.bitwise_xor(np.bitwise_xor.accumulate(z['xor'],axis=0),z['base']).view('<f4');times=z['time_s'];assert q.shape==(chunk['frames'],reader.meta['nq']) and len(times)==len(q) and np.isfinite(q).all() and np.isfinite(times).all() and np.all(np.diff(times)>0)
   if previous is not None:assert times[0]>previous;gap=max(gap,float(times[0]-previous))
   if len(times)>1:gap=max(gap,float(np.max(np.diff(times))))
   previous=float(times[-1]);first=float(times[0]) if first is None else first;count=int(np.searchsorted(times,cutoff,side='right'));frames+=count
   if count:last=float(times[count-1]),q[count-1].copy()
  chunks+=1
  if chunk['end_sim_s']>=cutoff:break
 assert first==0. and last is not None and 0<=cutoff-last[0]<=interval+1e-7 and gap<=interval+1e-7
 # Reconstruct the authentic final movie sample in a private FK model only.
 t,o=diagnostic_world(run);assert t.data is not getattr(t,'_live_data',None);assert o['world_id']==feedback['world_id'] and not o['evaluation']['violations'];assert all(not b['actual_support'] for b in o['payloads'].values());t.data.qpos[:]=last[1];mujoco.mj_forward(t.model,t.data);accepted=set(eligible_goal_cargo(t));grounded=t.monitor.relay_v2.supported_payloads(goal_only=True,require_all=False,inside_ids=accepted);rows={}
 for ident,state in t.monitor.relay_v2.states.items():
  transfers=o['payloads'][ident]['state']['handovers'];handover=any(v['sim_time_s']<=last[0]+interval and set(v['receivers'])-set(v['senders']) for v in transfers);rows[ident]=dict(inside_goal=ident in accepted,released=t.monitor.released(state['all_geoms']),grounded=ident in grounded,qualified_handover=bool(handover));assert all(rows[ident].values()),rows
 later=[]
 for path in (run/'recovery/accepted').glob('*.json'):
  result_path=run/'recovery/results'/path.name
  if not result_path.exists():continue
  req=json.loads(path.read_text());res=json.loads(result_path.read_text())
  if res['start_sim_s']>cutoff and req['op'] not in ['wait']:later.append(req['id'])
 assert not later,'Post-completion physical commands require a new capture: '+str(later)
 result=dict(passed=True,case_id=reader.meta['case_id'],world_id=o['world_id'],rule=current['rule_revision'],boundary_tolerance_m=.05,completion_cutoff_s=cutoff,last_recorded_sample_s=last[0],sample_quantization_s=cutoff-last[0],first_recorded_sample_s=first,verified_prefix_frames=frames,verified_chunks=chunks,max_sample_gap_s=gap,permitted_gap_s=interval+1e-7,frame_goal_geometry=rows,final_original_request=str(feedback_path),final_request_sha256=digest(feedback_path),source_scene_sha256=reader.meta['source_scene_sha256'],original_stream_status=reader.meta['status'],original_process_kept_running=True,post_cutoff_physical_requests=later,scope='Every original chunk through task-completion cutoff checksum/shape/time/finite-state verified. Authentic final sample independently inside user50 Goal, released, grounded and previously handed over. Private FK only: zero integration, no new physical episode, no live state/history writes. Original process continues and strict native history is preserved.',wall_s=time.monotonic()-began)
 (run/'RECORDING-VERIFIED-THROUGH-USER-GOAL.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:result[k] for k in ['case_id','passed','completion_cutoff_s','verified_prefix_frames','verified_chunks','wall_s']}),flush=True)
if __name__=='__main__':main()
