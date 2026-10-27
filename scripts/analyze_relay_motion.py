"""Analyze recorded scene motion once; all backgrounds reuse the same cut mask."""
import argparse,json,time
from collections import OrderedDict
from pathlib import Path
import mujoco,numpy as np
from PIL import Image,ImageDraw,ImageFont
from robofrontier.motion_recording import MotionReader,digest
from robofrontier.relay_video_motion import MotionThresholds,classify_motion,ranges,image_motion_evidence


def load_samples(run,out,every=3):
    cache=out/'samples.npz';identity=out/'source.json';reader=MotionReader(run/'motion');task=json.loads((run/'task.json').read_text())
    prefix=run/'RECORDING-VERIFIED-THROUGH-USER-GOAL.json'
    if prefix.exists():end=json.loads(prefix.read_text())['completion_cutoff_s'];terminal=None
    else:
        native_report=json.loads((run/'report.json').read_text());user_report=run/'user-goal-report.json'
        assert native_report.get('passed') or (user_report.exists() and json.loads(user_report.read_text())['passed'])
        end=native_report['evaluation']['sim_time_s']
        with np.load(run/'event-states.npz',allow_pickle=False) as events:terminal=events['qpos'][-1].copy()
    if cache.exists():
        info=json.loads(identity.read_text());assert info['scene_sha256']==reader.meta['source_scene_sha256'] and abs(info['end_sim_s']-end)<1e-7
        with np.load(cache,allow_pickle=False) as z:return reader,task,z['times'].copy(),z['qpos'].copy(),info
    qs=[];stamps=[];offset=0;last=None;raw_count=0
    for chunk in reader.meta['chunks']:
        if chunk['start_sim_s']>end:break
        file=reader.path/chunk['file'];assert digest(file)==chunk['sha256']
        with np.load(file,allow_pickle=False) as z:
            q=np.bitwise_xor(np.bitwise_xor.accumulate(z['xor'],axis=0),z['base']).view('<f4');times=z['time_s'];valid=times<=end+1e-8;ids=np.flatnonzero(valid);raw_count+=len(ids)
            if len(ids):last=float(times[ids[-1]]),q[ids[-1]].copy()
            mask=valid&((np.arange(len(q))+offset)%every==0);qs.append(q[mask].copy());stamps.append(times[mask].copy());offset+=len(q)
        if chunk['end_sim_s']>=end:break
    times=np.concatenate(stamps);q=np.concatenate(qs);del qs,stamps
    if last[0]>times[-1]+1e-8:times=np.r_[times,last[0]];q=np.vstack([q,last[1]])
    if terminal is not None and end>times[-1]+1e-8:times=np.r_[times,end];q=np.vstack([q,terminal.astype(np.float32)])
    info=dict(case_id=task['id'],source=str(run),scene_sha256=reader.meta['source_scene_sha256'],end_sim_s=end,analysis_hz=reader.meta['sample_hz']/every,source_sample_every=every,raw_prefix_frames=raw_count,source_stream_status=reader.meta['status'],scope='Original committed recording chunks; closed cases also retain exact final native event. No new physical episode or integration.')
    np.savez_compressed(cache,times=times,qpos=q);identity.write_text(json.dumps(info,indent=2)+'\n');return reader,task,times,q,info


def layout_for(model,task):
    layout=dict(robot_hinges=[],robot_slides=[],primary_free=[],primary_hinges=[],pendant_free=[])
    for robot in task['arms']:
        hinge=[];slide=[]
        for j in range(model.njnt):
            if not (model.joint(j).name or '').startswith(robot+'_'):continue
            address=int(model.jnt_qposadr[j]);kind=model.jnt_type[j]
            (slide if kind==mujoco.mjtJoint.mjJNT_SLIDE else hinge).append(address)
        if hinge:layout['robot_hinges'].append(hinge)
        if slide:layout['robot_slides'].append(slide)
    primary_bodies=set();pendant_bodies=set()
    for item in task['mechanism']['payload_assemblies']:
        layout['primary_free'].append(int(model.jnt_qposadr[model.joint(item['id']+'_free').id]));primary_bodies.update(model.body(n).id for n in item.get('beam_bodies',[item.get('beam_body',item['id'])]))
        if item.get('elastic_hinge'):layout['primary_hinges'].append(int(model.jnt_qposadr[model.joint(item['elastic_hinge']['joint']).id]))
        if item.get('weight_body'):
            bid=model.body(item['weight_body']).id;pendant_bodies.add(bid);layout['pendant_free'].append(int(model.jnt_qposadr[int(model.body_jntadr[bid])]))
    groups=dict(robot=set(),primary=set(),pendant=set())
    for g in range(model.ngeom):
        bid=int(model.geom_bodyid[g]);name=model.body(bid).name or ''
        if name.split('_',1)[0] in task['arms']:groups['robot'].add(g);continue
        ancestor=bid
        while ancestor:
            if ancestor in pendant_bodies:groups['pendant'].add(g);break
            if ancestor in primary_bodies:groups['primary'].add(g);break
            ancestor=int(model.body_parentid[ancestor])
    return layout,groups


def configure_scene(renderer):
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False;renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False


def visual_audit(model,task,times,q,masks,groups,out):
    audit_path=out/'visual-audit.json';mask_path=out/'visual-keep.npy'
    if audit_path.exists() and mask_path.exists():return np.load(mask_path),json.loads(audit_path.read_text())
    keep=masks['keep'].copy();data=mujoco.MjData(model);cache=OrderedDict();records=[];examples=[];font=ImageFont.truetype('/System/Library/Fonts/STHeiti Medium.ttc',16)
    candidates=[(lo,hi) for lo,hi in ranges(~keep) if np.any(masks['visual_candidates'][lo:hi+1])]
    substantive=[(lo,hi) for lo,hi in candidates if times[hi]-times[lo]>=.4]
    print(task['id'],'visual waiting regions',len(substantive),'short numeric gaps',len(candidates)-len(substantive),flush=True)
    with mujoco.Renderer(model,width=384,height=240,max_geom=max(5000,model.ngeom+1000)) as renderer:
        def frame(i):
            if i in cache:value=cache.pop(i);cache[i]=value;return value
            data.qpos[:]=q[i];mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data);mujoco.mj_camlight(model,data);renderer.update_scene(data,camera='global');configure_scene(renderer);rgb=renderer.render().copy();renderer.enable_segmentation_rendering();seg=renderer.render().copy();renderer.disable_segmentation_rendering();ids=np.where(seg[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM),seg[:,:,0],-1);cache[i]=(rgb,ids)
            while len(cache)>48:cache.popitem(last=False)
            return rgb,ids
        for number,(lo,hi) in enumerate(substantive):
            centers=sorted({lo+(hi-lo)//2,lo+int(np.argmax(masks['robot_score'][lo:hi+1]))});checks=[];rescue=False
            for center in centers:
                for window in [.25,2.]:
                    left=max(lo,int(np.searchsorted(times,times[center]-window/2)));right=min(hi,int(np.searchsorted(times,times[center]+window/2)))
                    if left>=right:continue
                    a,aid=frame(left);b,bid=frame(right);evidence=image_motion_evidence(a,b,aid,bid,groups['robot'],groups['primary'],groups['pendant']);effective=evidence['robot_effective'] or evidence['primary_effective']
                    if effective:keep[left:right+1]=True;rescue=True
                    check=dict(first_s=float(times[left]),last_s=float(times[right]),window_s=window,keep=bool(effective),**evidence);checks.append(check)
                    if len(examples)<16 and (evidence['pendant_changed_pixels']>3 or effective):
                        panel=Image.new('RGB',(768,280),'#f5f5f2');panel.paste(Image.fromarray(a),(0,40));panel.paste(Image.fromarray(b),(384,40));draw=ImageDraw.Draw(panel);draw.text((8,3),f'{times[left]:.2f}s → {times[right]:.2f}s | '+('保留：发现臂/木杆实际动作' if effective else '删除：仅砝码摆动/无有效臂动作'),font=font,fill='#263848');draw.text((8,22),f'robot shift {evidence["robot_max_geom_shift_px"]:.2f}px | weight changed {evidence["pendant_changed_pixels"]} pixels',font=font,fill='#263848');name=f'audit-{len(examples):02d}.jpg';panel.save(out/name,quality=90);examples.append(name)
            records.append(dict(first_s=float(times[lo]),last_s=float(times[hi]),rescued=bool(rescue),checks=checks))
            if (number+1)%25==0:print(task['id'],'visual audit',number+1,'/',len(substantive),flush=True)
    result=dict(candidate_regions=len(candidates),image_checked_regions=len(records),short_numeric_gaps=len(candidates)-len(records),rescued_regions=sum(v['rescued'] for v in records),examples=examples,records=records,method='Actual RGB and geom-ID masks at0.25s/2s windows; compare robot/primary visible geometry centroids/silhouettes, exclude both-frame pendant occlusion; no shadows/reflections, so pendant shadows cannot fake arm motion.')
    np.save(mask_path,keep);audit_path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');return keep,result


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--analyse-only',action='store_true');a=p.parse_args();run=a.run.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);started=time.monotonic();reader,task,times,q,info=load_samples(run,out);model=mujoco.MjModel.from_xml_path(str(run/'scene.xml'));model.geom_rgba[:]=reader.meta['initial_rgba'];model.vis.quality.offsamples=0;layout,groups=layout_for(model,task);cfg=MotionThresholds();mask_file=out/'state-motion.npz'
    if mask_file.exists():
        with np.load(mask_file,allow_pickle=False) as z:masks={k:z[k].copy() for k in z.files}
    else:masks=classify_motion(q,info['analysis_hz'],layout,cfg);np.savez_compressed(mask_file,**masks)
    state_summary=dict(case_id=task['id'],samples=len(times),source_duration_s=float(times[-1]-times[0]),state_motion_s=float(np.count_nonzero(masks['keep'])/info['analysis_hz']),weight_only_wait_s=float(np.count_nonzero(masks['weight_only'])/info['analysis_hz']),robot_motion_s=float(np.count_nonzero(masks['robot'])/info['analysis_hz']),primary_motion_s=float(np.count_nonzero(masks['primary'])/info['analysis_hz']),candidate_visual_regions=len(ranges(masks['visual_candidates'])),thresholds=cfg.manifest(),layout=layout);(out/'state-summary.json').write_text(json.dumps(state_summary,indent=2)+'\n');print(json.dumps({k:v for k,v in state_summary.items() if k not in ['thresholds','layout']}),flush=True)
    if a.analyse_only:return
    keep,audit=visual_audit(model,task,times,q,masks,groups,out)
    assert np.all(keep[masks['robot']]) and np.all(keep[masks['primary']])
    manifest=dict(**{k:v for k,v in info.items() if k!='scope'},input_recording_scope=info['scope'],rule='scene-motion-two-windows-visual-weight-exception-v1',
        thresholds=cfg.manifest(),motion_analysis=state_summary,
        visual_audit={k:v for k,v in audit.items() if k!='records'},
        scope='Background-independent motion analysis only. Export edited tracks and 20x/30fps videos with scripts.export_relay_edited.',
        physics_stepped=False,raw_trajectories_unchanged=True,wall_s=time.monotonic()-started)
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(task['id'],'motion analysis ready; no video encoded',flush=True)
if __name__=='__main__':main()
