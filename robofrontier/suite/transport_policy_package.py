"""Transport-only scene revision and policy input preflight.

Creates no pipeline run, roles, credentials, simulator stepping or inference.
Original qualification and the new geometry variant remain separate evidence.
"""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from ..motion_recording import digest
from ..run_setup import write_json
from ..workstation import Workstation
from . import v2
from .table_presentation import scale_rectangular_opening_for_render

SCHEMA = 'robofrontier.transport-family.v1'
PUBLIC_SCHEMA = 'robofrontier.transport-policy-input.v1'
EXPECTED_CASES = {f'transport-a{arms:02d}-d{level}' for arms in (4,8,12) for level in (1,2,3)}
MECHANICAL_KEYS = ('kind','cube_side_m','cylinder_diameter_m','cylinder_height_m',
    'cube_body_mass_kg','cube_material','cylinder_body_mass_kg','blade_size_m',
    'blade_spacing_m','slot_opening_m','slot_depth_m','board_thickness_m','spring_board',
    'supplies','board_size_m','board_material_density_kg_m3','board_outward_shift_from_r10_m',
    'target_cue_span_m','target_cue_scale','target_center_pitch_m','target_cue_gap_m','transfer')


def load_json(path):
    return json.loads(Path(path).read_text())


def resolve(root,path):
    return Path(path) if Path(path).is_absolute() else Path(root)/path


def table_geometry(model):
    ids=[i for i in range(model.ngeom) if (model.geom(i).name or '').startswith('table_')
         and (model.geom(i).name or '')[6:].isdigit()]
    if not ids:raise ValueError('Transport tabletop geometry is missing')
    low=np.min(model.geom_pos[ids,:2]-model.geom_size[ids,:2],axis=0)
    high=np.max(model.geom_pos[ids,:2]+model.geom_size[ids,:2],axis=0)
    opening=None
    if len(ids)==4:
        vertical=[i for i in ids if abs(model.geom_pos[i,0])<1e-9 and abs(model.geom_pos[i,1])>1e-9]
        horizontal=[i for i in ids if abs(model.geom_pos[i,1])<1e-9 and abs(model.geom_pos[i,0])>1e-9]
        if len(vertical)!=2 or len(horizontal)!=2:raise ValueError('Unknown rectangular tabletop layout')
        opening=[2*min(abs(model.geom_pos[i,0])-model.geom_size[i,0] for i in horizontal),
                 2*min(abs(model.geom_pos[i,1])-model.geom_size[i,1] for i in vertical)]
    return dict(tabletop_z_m=float(max(model.geom_pos[i,2]+model.geom_size[i,2] for i in ids)),
        outer_bounds_xy_m=[low.tolist(),high.tolist()],outer_size_xy_m=(high-low).tolist(),
        central_opening_size_m=opening,central_opening_is_insertion_socket=False,
        boxes_world=[dict(id=model.geom(i).name,center_m=model.geom_pos[i].tolist(),
                         half_size_m=model.geom_size[i].tolist(),quaternion_wxyz=model.geom_quat[i].tolist()) for i in ids])


def verify_oracle(row,root):
    trial=resolve(root,row['oracle_trial']);report_path=trial/'report.json';report=load_json(report_path)
    if digest(report_path)!=row['report_sha256']:raise ValueError('Oracle report checksum mismatch')
    if report['case_id']!=row['case_id'] or not report['passed'] or not report.get('full_task_attempt'):
        raise ValueError('Expected the selected complete Transport witness')
    evaluation=report['evaluation']
    if not evaluation['all_goals_met'] or not evaluation['process_compliant'] or evaluation.get('violations') or report['unresolved_robot_actions']:
        raise ValueError('Oracle witness has unresolved goals/actions or violations')
    if report.get('live_object_state_writes_after_initialization'):raise ValueError('Invalid live state writes in evidence')
    for field in ('scene_sha256','physical_control_sha256','environment_logic_sha256'):
        if report[field]!=row[field]:raise ValueError('Oracle provenance mismatch: '+field)
    if bool(report.get('engineering_assisted'))!=row['engineering_assisted']:raise ValueError('Assistance provenance mismatch')
    source=trial/'motion/source'
    if digest(source/'scene.xml')!=row['scene_sha256']:raise ValueError('Oracle frozen scene changed')
    return trial,report,source


def assert_variant_parity(original,variant,table_names):
    """Only tabletop positions/sizes may differ in the compiled model."""
    if (original.nq,original.nv,original.nu,original.nbody,original.ngeom)!=(variant.nq,variant.nv,variant.nu,variant.nbody,variant.ngeom):
        raise ValueError('Variant changed model registries')
    for kind,count in [('body',original.nbody),('joint',original.njnt),('geom',original.ngeom)]:
        if [getattr(original,kind)(i).name for i in range(count)]!=[getattr(variant,kind)(i).name for i in range(count)]:raise ValueError('Variant changed '+kind+' identities')
    for field in ('body_parentid','body_pos','body_quat','body_mass','body_inertia','body_ipos','body_iquat','jnt_type','jnt_axis','jnt_range','jnt_pos','jnt_stiffness',
                  'dof_damping','dof_frictionloss','dof_armature','qpos_spring','key_qpos','key_ctrl','actuator_gainprm','actuator_biasprm',
                  'actuator_ctrlrange','actuator_forcerange','eq_data','geom_type','geom_bodyid','geom_friction','geom_solref','geom_solimp','geom_quat','geom_contype','geom_conaffinity'):
        if not np.allclose(getattr(original,field),getattr(variant,field),rtol=0,atol=1e-12):raise ValueError('Variant changed '+field)
    untouched=[i for i in range(original.ngeom)if original.geom(i).name not in table_names]
    for field in ('geom_pos','geom_size'):
        if not np.allclose(getattr(original,field)[untouched],getattr(variant,field)[untouched],rtol=0,atol=1e-12):raise ValueError('Variant changed non-table '+field)
    if original.opt.timestep!=variant.opt.timestep:raise ValueError('Variant changed physics timestep')
    for field in ('gravity','integrator','solver','iterations','noslip_iterations','tolerance','impratio','enableflags','disableflags'):
        if not np.array_equal(np.asarray(getattr(original.opt,field)),np.asarray(getattr(variant.opt,field))):raise ValueError('Variant changed solver '+field)


def verify_geometric_evidence(cfg,root):
    evidence=cfg['source_half_opening_audit'];path=resolve(root,evidence['path'])
    if digest(path)!=evidence['sha256']:raise ValueError('Half-opening audit checksum mismatch')
    index=load_json(path)
    if not index['passed']or index['new_contact_frames']!=0 or index['qualified_hole_cases']!=6:raise ValueError('Invalid half-opening source audit')
    rows={str(resolve(root,row['oracle_trial']).resolve()):row for row in cfg['cases']}
    count=0
    for item in index['cases']:
        row=rows.get(str(Path(item['source_trial']).resolve()))
        if row is None or 'half_opening_audit'not in row or digest(resolve(root,row['half_opening_audit']))!=row['half_opening_audit_sha256']:raise ValueError('Geometric audit does not match pinned source')
        audit=load_json(resolve(root,row['half_opening_audit']));meta=load_json(resolve(root,row['oracle_trial'])/'motion/stream.json')
        if Path(audit['source_trial']).resolve()!=resolve(root,row['oracle_trial']).resolve():raise ValueError('Geometric audit source mismatch')
        if not audit['passed']or audit['added_material_contact_frames']!=0 or audit['frames']!=meta['frames']:raise ValueError('Incomplete source geometry audit')
        count+=audit['frames']
    if count!=index['frames']:raise ValueError('Source geometry audit count mismatch')
    return dict(source_frames=count,source_cases=6,new_contact_frames=0,new_physical_execution=False)


def materialize_scene_assets(source,destination):
    """Restore ignored R14 assets from the bound witness without shared inodes.

    Existing scene metadata and assets are verified and never overwritten. This
    lets a merge checkout materialize its inputs without copying another worktree.
    """
    source=Path(source).resolve();destination=Path(destination).resolve()
    original=load_json(source/'workstation.json');variant=load_json(destination/'workstation.json')
    if digest(destination/'scene.xml')!=variant['scene_sha256']:raise ValueError('R14 scene checksum mismatch')
    if variant['assets']!=original['assets']:raise ValueError('R14 assets differ from the bound source')
    pending=[]
    for asset in variant['assets']:
        relative=Path(asset['path']);src=(source/relative).resolve();target=destination/relative
        if relative.is_absolute()or '..'in relative.parts or relative.parts[0]!='assets':raise ValueError('Invalid Transport asset path')
        if not src.is_relative_to(source)or not target.resolve().is_relative_to(destination):raise ValueError('Asset outside workstation')
        if digest(src)!=asset['sha256']:raise ValueError('Source asset checksum mismatch: '+str(relative))
        if target.exists():
            if digest(target)!=asset['sha256']:raise ValueError('Existing R14 asset checksum mismatch: '+str(relative))
            if target.samefile(src):raise ValueError('Editable R14 asset aliases frozen evidence: '+str(relative))
        else:pending.append((src,target))
    for src,target in pending:
        target.parent.mkdir(parents=True,exist_ok=True)
        with src.open('rb')as incoming,target.open('xb')as outgoing:shutil.copyfileobj(incoming,outgoing)
    return len(pending)


def build_half_scene(source,destination,root,render_previews=True):
    """Clone a frozen R13 source into an explicit R14 physical half-opening scene."""
    source=Workstation(source);destination=Path(destination);root=Path(root)
    if destination.exists():raise FileExistsError(destination)
    destination.mkdir(parents=True)
    for asset in source.manifest['assets']:
        target=destination/asset['path'];target.parent.mkdir(parents=True,exist_ok=True)
        # Editable worktree assets must not alias immutable Oracle evidence.
        shutil.copyfile(source.path/asset['path'],target)
    model=source.model;old_geometry=table_geometry(model)
    variant=scale_rectangular_opening_for_render(model,.5)
    tree=ET.parse(source.path/'scene.xml')
    for name in (variant or {}).get('table_geoms',[]):
        geom=model.geom(name);element=tree.getroot().find(f"worldbody/geom[@name='{name}']")
        if element is None:raise ValueError('Expected a world-root tabletop box')
        element.set('pos',v2.vector(geom.pos));element.set('size',v2.vector(geom.size))
    ET.indent(tree);tree.write(destination/'scene.xml',encoding='unicode')
    env=load_json(source.path/'environment.json');case=deepcopy(env['suite_case'])
    case.update(scene_revision=14,source_scene_revision=13,directory=str(destination.relative_to(root)))
    env['suite_case']=case;new_geometry=table_geometry(model)
    env['tables']=[dict(center_m=b['center_m'],half_size_m=b['half_size_m'])for b in new_geometry['boxes_world']]
    env['transport_variant']=dict(id='r14-half-opening',source_scene_sha256=source.manifest['scene_sha256'],
        linear_opening_scale=.5,physical_geometry_changed=variant is not None,source_task_pass_inherited=False)
    write_json(destination/'environment.json',env)
    task=load_json(source.path/'task.json');task.update(scale=case,scene_asset=str(destination.relative_to(root)),scene_sha256=digest(destination/'scene.xml'))
    # These still enter the existing shared preview gate. Formal startup is merge work.
    task.update(execution_readiness='environment_preview_only',oracle_verified=False)
    write_json(destination/'task.json',task)
    compiled=mujoco.MjModel.from_xml_path(str(destination/'scene.xml'));data=mujoco.MjData(compiled)
    mujoco.mj_resetDataKeyframe(compiled,data,compiled.key('initial').id);mujoco.mj_forward(compiled,data)
    original=mujoco.MjModel.from_xml_path(str(source.path/'scene.xml'))
    assert_variant_parity(original,compiled,(variant or {}).get('table_geoms',[]))
    # Serialization must exactly reproduce the previously audited presentation model.
    for field in ('geom_pos','geom_size'):
        if not np.allclose(getattr(compiled,field),getattr(model,field),rtol=0,atol=1e-12):raise ValueError('R14 differs from audited half-opening geometry')
    np.testing.assert_array_equal(compiled.key_qpos,original.key_qpos)
    state=v2.manifest(compiled,data,case,destination,env)
    compatibility=dict(schema='robofrontier.transport-half-opening-check.v1',case_id=task['id'],
        source_scene_sha256=source.manifest['scene_sha256'],variant_scene_sha256=state['scene_sha256'],
        original_geometry=old_geometry,physical_geometry=new_geometry,compiled_parity_passed=True,
        identical_to_half_opening_fk_model=True,simulated_seconds_advanced=0.,
        original_oracle_pass_inherited=False,physical_execution_verified=False,
        scope='Compiled tabletop-only revision and exact parity with the source half-opening FK model; no new physics execution')
    write_json(destination/'transport-compatibility.json',compatibility)
    write_json(destination/'validation.json',dict(status='compiled_tabletop_variant',oracle_verified=False,policy_verified=False,
        frozen_input=True,nq=compiled.nq,nv=compiled.nv,ngeom=compiled.ngeom,physics_sha256=state['physics_sha256'],
        scope=compatibility['scope']))
    if render_previews:
        from PIL import Image
        with mujoco.Renderer(compiled,width=1280,height=800,max_geom=max(5000,compiled.ngeom+1000))as renderer:
            for view in ('global','global_rear','top'):
                renderer.update_scene(data,camera=view);Image.fromarray(renderer.render()).save(destination/f'preview-{view}.png')
    return compatibility


def public_input(task,model,acceptance):
    """Allowlist public static facts; never include Oracle provenance or qpos."""
    supplies={}
    for obj in task['objects']:
        key=(obj['source'],obj['color'],obj['shape']);supplies[key]=supplies.get(key,0)+1
    return dict(schema=PUBLIC_SCHEMA,id=task['id'],family='transport',level=task['level'],scene_revision=task['scale']['scene_revision'],
        kind=task['kind'],suite_version=task['suite_version'],coordinate_frame=task['coordinate_frame'],
        arms=deepcopy(task['arms']),workspace_world_m=deepcopy(task['workspace_world_m']),
        table_geometry=table_geometry(model),table_height_m=task['table_height_m'],
        mechanism={k:deepcopy(task['mechanism'][k])for k in MECHANICAL_KEYS if k in task['mechanism']},
        initial_supplies=[dict(source=k[0],color=k[1],shape=k[2],count=n)for k,n in sorted(supplies.items())],
        orders=[{k:deepcopy(g[k])for k in ('goal_id','robot','requires','target_cue')if k in g}for g in task['goals']],
        rules=deepcopy(task['rules']),objective=task['objective'],observation_contract=deepcopy(task['observation_contract']),
        gripper_profile=task['gripper_profile'],cameras=deepcopy(task['cameras']),
        static_camera_mounts=[dict(id=model.camera(i).name,
            frame='world' if int(model.cam_bodyid[i])==0 else 'body:'+model.body(int(model.cam_bodyid[i])).name,
            position_m=model.cam_pos[i].tolist(),quaternion_wxyz=model.cam_quat[i].tolist(),
            vertical_fov_deg=float(model.cam_fovy[i]),fixed_world=int(model.cam_bodyid[i])==0)
            for i in range(model.ncam)],
        completion=dict(**deepcopy(task['completion']),acceptance=deepcopy(acceptance)),
        family_reference='transport.md',execution_readiness=task['execution_readiness'])


def build_package(config_path,output,root):
    """Write reviewable family inputs, not a prepared/launchable harness run."""
    root=Path(root);cfg=load_json(config_path);output=Path(output)
    if cfg.get('schema')!=SCHEMA or cfg.get('family')!='transport':raise ValueError('Wrong Transport family config')
    if {c['case_id']for c in cfg['cases']}!=EXPECTED_CASES or len(cfg['cases'])!=9:raise ValueError('Expected exactly the nine Transport cases')
    witness_path=resolve(root,cfg['witness_registry']['path'])
    if digest(witness_path)!=cfg['witness_registry']['sha256']:raise ValueError('Witness registry checksum mismatch')
    witnesses={row['case_id']:row for row in load_json(witness_path)['cases']}
    if set(witnesses)!=EXPECTED_CASES:raise ValueError('Incomplete witness registry')
    for row in cfg['cases']:
        if any(row.get(key)!=value for key,value in witnesses[row['case_id']].items()):raise ValueError('Variant witness reference differs from registry')
    if output.exists():raise FileExistsError(output)
    # Complete all validation before writing a partially trusted public package.
    prepared=[];source_cfg=load_json(resolve(root,cfg['controller_config']))
    geometry_evidence=verify_geometric_evidence(cfg,root)
    for row in cfg['cases']:
        trial,report,source=verify_oracle(row,root)
        if load_json(trial/'executor-source/effective-harness.json')!=source_cfg:raise ValueError('Control config differs from Oracle witness')
        for name,checksum in report['environment_logic_provenance']['sources'].items():
            if digest(root/name)!=checksum:raise ValueError('Current evaluator differs from selected Oracle: '+name)
        scene=resolve(root,row['physical_scene']);w=Workstation(scene);task=load_json(scene/'task.json')
        if digest(scene/'scene.xml')!=row['physical_scene_sha256']or digest(scene/'workstation.json')!=row['workstation_manifest_sha256']:raise ValueError('Pinned R14 scene/registry changed')
        compatibility=load_json(scene/'transport-compatibility.json')
        if compatibility['source_scene_sha256']!=row['scene_sha256']or compatibility['variant_scene_sha256']!=digest(scene/'scene.xml'):raise ValueError('R14 variant source mismatch')
        if task['id']!=row['case_id']or task['scale']['scene_revision']!=14:raise ValueError('Wrong physical task revision')
        if task['scene_sha256']!=digest(scene/'scene.xml'):raise ValueError('Physical task scene checksum mismatch')
        original_task=load_json(source/'task.json')
        for field in ('kind','id','family','level','arms','objects','goals','physics','mechanism','rules','completion','workspace_world_m','gripper_profile','observation_contract','cameras'):
            if task[field]!=original_task[field]:raise ValueError('Variant changed task contract: '+field)
        prepared.append((row,task,w,report,compatibility))
    output.mkdir(parents=True);public=output/'public';private=output/'engineering';public.mkdir();private.mkdir()
    reference=resolve(root,cfg['policy_reference']);shutil.copyfile(reference,public/'transport.md')
    rows=[]
    for row,task,w,report,compatibility in prepared:
        projected=public_input(task,w.model,cfg['acceptance']);write_json(public/(row['case_id']+'.json'),projected)
        write_json(w.path/'public-task.json',projected)
        rows.append(dict(case_id=row['case_id'],physical_scene_sha256=task['scene_sha256'],
            original_scene_sha256=row['scene_sha256'],engineering_assisted=row['engineering_assisted'],
            original_report_sha256=row['report_sha256'],original_complete_goals=report['evaluation']['matched_goals'],
            physical_variant_full_execution_verified=False,compiled_parity_verified=compatibility['compiled_parity_passed']))
    manifest=dict(schema='robofrontier.transport-package.v1',status='family_preflight_complete_shared_integration_pending',
        case_count=9,physical_revision=14,physical_opening_scale=.5,display_opening_scale_relative_to_r13=.5,
        display_opening_scale_relative_to_physical_r14=1.,family_config_sha256=digest(config_path),
        policy_reference_sha256=digest(reference),formal_gpt_ready=False,gpt_run_prepared=False,
        shared_integration_pending=cfg['shared_integration_pending'],source_geometric_compatibility=geometry_evidence,cases=rows,
        public_files={str(p.relative_to(public)):digest(p)for p in sorted(public.iterdir())if p.is_file()})
    write_json(private/'manifest.json',manifest)
    return manifest
