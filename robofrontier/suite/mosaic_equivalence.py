"""Exact L2/L3 physical-input gate for fresh engineering command candidates.

This only establishes input equivalence. It never copies a success flag or
restores a trajectory into a live world; target execution/evaluation is separate.
"""
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np


FIELDS='qpos0 key_qpos key_qvel key_ctrl body_parentid body_weldid body_mass body_inertia body_pos body_quat body_ipos body_iquat body_gravcomp jnt_type jnt_bodyid jnt_pos jnt_axis jnt_range jnt_stiffness jnt_limited jnt_solref jnt_solimp dof_bodyid dof_damping dof_armature dof_frictionloss dof_solref dof_solimp actuator_trnid actuator_trntype actuator_gaintype actuator_biastype actuator_dyntype actuator_gainprm actuator_biasprm actuator_dynprm actuator_ctrlrange actuator_forcerange actuator_gear actuator_ctrllimited actuator_forcelimited exclude_signature pair_geom1 pair_geom2 pair_solref pair_solimp pair_friction eq_type eq_data mesh_vert mesh_face'.split()
FIELDS+='eq_obj1id eq_obj2id eq_active0 eq_solref eq_solimp jnt_qposadr jnt_dofadr dof_jntid actuator_actrange actuator_actlimited actuator_lengthrange'.split()
GEOMS='geom_type geom_bodyid geom_dataid geom_pos geom_quat geom_size geom_friction geom_solref geom_solimp geom_margin geom_gap geom_contype geom_conaffinity geom_condim geom_priority geom_solmix geom_rbound'.split()
OPTIONS='timestep gravity integrator solver iterations tolerance noslip_iterations noslip_tolerance cone jacobian impratio density viscosity wind disableflags enableflags'.split()
TASK='arms objects goals cube_size_m table_height_m completion workspace_world_m'.split()


def compare_models(a,b):
    checks={name:bool(np.array_equal(getattr(a,name),getattr(b,name))) for name in FIELDS}
    masks=[np.flatnonzero((m.geom_contype!=0)|(m.geom_conaffinity!=0)) for m in (a,b)]
    for name in GEOMS:checks['physical.'+name]=bool(np.array_equal(getattr(a,name)[masks[0]],getattr(b,name)[masks[1]]))
    for name in OPTIONS:checks['option.'+name]=bool(np.array_equal(getattr(a.opt,name),getattr(b.opt,name)))
    checks['body_names']=[a.body(i).name for i in range(a.nbody)]==[b.body(i).name for i in range(b.nbody)]
    return checks


def check(source,target):
    source,target=Path(source).resolve(),Path(target).resolve()
    tasks=[json.loads((p/'task.json').read_text()) for p in (source,target)]
    if ([t.get('family') for t in tasks]!=['mosaic','mosaic'] or [t['level'] for t in tasks]!=[2,3]):
        raise ValueError('This equivalence gate only covers Mosaic L2 -> L3')
    models=[mujoco.MjModel.from_xml_path(str(p/'scene.xml')) for p in (source,target)]
    checks=compare_models(*models)
    checks.update({'task.'+name:tasks[0][name]==tasks[1][name] for name in TASK})
    return dict(equivalent_for_command_candidate=all(checks.values()),checks=checks,
        differences=[name for name,value in checks.items() if not value],
        source_scene_sha256=[hashlib.sha256((p/'scene.xml').read_bytes()).hexdigest() for p in (source,target)],
        case_ids=[t['id'] for t in tasks],mujoco_version=mujoco.__version__,
        scope='Exact compiled physical arrays and acceptance-relevant task data; visual grid may differ. No physics step, checkpoint restoration, task pass or robustness claim.')


def check_prefix_candidate(source,target):
    """Also permit fresh same-case execution of a retained partial witness."""
    source,target=Path(source).resolve(),Path(target).resolve()
    tasks=[json.loads((p/'task.json').read_text()) for p in (source,target)]
    if tasks[0]['id']!=tasks[1]['id']:return check(source,target)
    if any(t.get('family')!='mosaic' for t in tasks):raise ValueError('Mosaic prefix candidates only')
    models=[mujoco.MjModel.from_xml_path(str(p/'scene.xml')) for p in (source,target)]
    checks=compare_models(*models);checks['complete_same_case_task']=tasks[0]==tasks[1]
    return dict(equivalent_for_command_candidate=all(checks.values()),checks=checks,
        differences=[name for name,value in checks.items() if not value],
        source_scene_sha256=[hashlib.sha256((p/'scene.xml').read_bytes()).hexdigest() for p in (source,target)],
        case_ids=[t['id'] for t in tasks],mujoco_version=mujoco.__version__,
        scope='Exact same-case compiled physical inputs and complete task equality; new native execution only, never a restored checkpoint or copied result')
