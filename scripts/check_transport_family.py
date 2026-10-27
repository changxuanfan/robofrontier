"""Read-only Transport family, source edit and merge-boundary audit."""
import argparse,json
from pathlib import Path
import mujoco,numpy as np
from robofrontier.motion_recording import MotionReader,digest
from robofrontier.motion_timeline import ranges
from robofrontier.run_setup import write_json
from robofrontier.suite.transport_policy_package import load_json,resolve,verify_oracle,assert_variant_parity,table_geometry,verify_geometric_evidence
from robofrontier.suite.transport_motion_edit import TransportEdit
ROOT=Path(__file__).resolve().parents[1]


def check(config,package,edits):
    cfg=load_json(config);manifest=load_json(package/'engineering/manifest.json')
    if manifest['family_config_sha256']!=digest(config):raise ValueError('Package config changed')
    if manifest['formal_gpt_ready']or manifest['gpt_run_prepared']:raise ValueError('Family package must not assert shared startup readiness')
    for name,checksum in manifest['public_files'].items():
        if digest(package/'public'/name)!=checksum:raise ValueError('Public package checksum mismatch')
    evidence=verify_geometric_evidence(cfg,ROOT);rows=[]
    for row in cfg['cases']:
        trial,report,source=verify_oracle(row,ROOT);original=mujoco.MjModel.from_xml_path(str(source/'scene.xml'));folder=resolve(ROOT,row['physical_scene']);variant=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
        compatibility=load_json(folder/'transport-compatibility.json');table_names=['table_'+str(i)for i in range(4)]if len(load_json(folder/'task.json')['arms'])>4 else[]
        assert_variant_parity(original,variant,table_names)
        geometry=table_geometry(variant);before=table_geometry(original)
        if before['central_opening_size_m']is not None:np.testing.assert_allclose(geometry['central_opening_size_m'],np.array(before['central_opening_size_m'])*.5,rtol=0,atol=1e-12)
        edit=TransportEdit(edits/row['case_id']/'edit.json');reader=MotionReader(trial/'motion')
        if edit.meta['source_scene_sha256']!=row['scene_sha256']:raise ValueError('Edit attached to wrong original scene')
        for action in load_json(trial/'actions.json'):
            for key in ('start_sim_s','end_sim_s'):
                i=min(len(edit.times)-1,int(np.searchsorted(edit.times,action[key])))
                if not edit.keep[i]:raise ValueError('Action boundary removed')
        # Independent spot checks of long removed intervals: real joint/hand
        # transport must not be hidden under a passive-object jitter label.
        registry=load_json(source/'workstation.json');arm_cols=[];finger_cols=[]
        for j in registry['joints']:
            name=j['id'];prefix=name.split('_')[0]
            if len(prefix)==1 and prefix.isalpha():
                if '_finger_joint'in name:finger_cols.append(j['qpos_address'])
                elif '_joint'in name:arm_cols.append(j['qpos_address'])
        intervals=[(a,b)for a,b in ranges(~edit.keep)if (b-a)/edit.meta['source_hz']>=4]
        samples=sorted({int(i)for a,b in intervals for i in np.linspace(a,b-1,17).astype(int)})
        snapshots={i:q for i,_,q in reader.selected_frames(samples)}if samples else{}
        largest_arm=largest_finger=0.
        for a,b in intervals:
            q=np.array([snapshots[int(i)]for i in np.linspace(a,b-1,17).astype(int)])
            largest_arm=max(largest_arm,float(np.ptp(q[:,arm_cols],axis=0).max(initial=0)))
            largest_finger=max(largest_finger,float(np.ptp(q[:,finger_cols],axis=0).max(initial=0)))
        if largest_arm>.02 or largest_finger>.002:raise ValueError(f'Long removed interval has meaningful robot excursion: {row["case_id"]}, {largest_arm}, {largest_finger}')
        # Source-native selection preserves all process change timestamps.
        previous=None;event_count=0
        for line in (trial/'process.jsonl').open():
            v=json.loads(line)
            if 'result'not in v:continue
            state=json.dumps({k:v['result'].get(k)for k in ('all_goals_met','task_success','matched_goals','assignments','violations')},sort_keys=True)
            if state!=previous:
                i=min(len(edit.times)-1,int(np.searchsorted(edit.times,v['sim_time_s'])))
                if not edit.keep[i]:raise ValueError('Task progress/failure event removed')
                event_count+=1;previous=state
        rows.append(dict(case_id=row['case_id'],r14_opening_m=geometry['central_opening_size_m'],compiled_parity=True,
            original_full_pass=True,engineering_assisted=row['engineering_assisted'],r14_full_execution_verified=False,
            raw_source_frames=edit.meta['source_frames'],retained_source_frames=edit.meta['retained_frames'],removed_source_frames=edit.meta['removed_frames'],
            long_removed_intervals_checked=len(intervals),source_states_spot_checked=len(samples),max_removed_arm_excursion_rad=largest_arm,
            max_removed_finger_excursion_m=largest_finger,critical_process_events_preserved=event_count))
        print('CHECKED',row['case_id'],len(intervals),'long cuts',flush=True)
    return dict(schema='robofrontier.transport-family-audit.v1',passed=True,case_count=len(rows),cases=rows,
        source_geometric_evidence=evidence,shared_harness_ready=False,gpt_run_started=False,physics_steps=0,
        scope='Static R14 parity, original witness binding, source cut/event validation and17-pose spot checks per long removed interval; not a continuous proof or new physical task pass')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'configs/task_families/transport/half-opening-r14.json');p.add_argument('--package',type=Path,required=True);p.add_argument('--edits',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=check(a.config,a.package,a.edits);write_json(a.output,result);print('FAMILY_AUDIT_PASSED',result['case_count'])
