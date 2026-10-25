"""Offline, read-only reconstruction of a frozen workstation's visible poses.

This is an export boundary, never a policy tool. It advances no dynamics and
neither imports a factory nor claims a USD/Isaac conversion has been tested.
"""
import hashlib
import json
import math
from pathlib import Path
import mujoco
import numpy as np
from .robot_control import matrix_quat


class Workstation:
    def __init__(self,path,verify_assets=True):
        self.path=Path(path).resolve();self.manifest=json.loads((self.path/'workstation.json').read_text())
        xml=self.path/self.manifest['scene']
        if hashlib.sha256(xml.read_bytes()).hexdigest()!=self.manifest['scene_sha256']:raise ValueError('Frozen scene checksum mismatch')
        for asset in self.manifest['assets']:
            p=(self.path/asset['path']).resolve()
            if not p.is_relative_to(self.path) or not p.is_file():raise ValueError('Asset missing or outside workstation: '+asset['path'])
            if verify_assets and hashlib.sha256(p.read_bytes()).hexdigest()!=asset['sha256']:raise ValueError('Frozen asset checksum mismatch: '+asset['path'])
        self.model=mujoco.MjModel.from_xml_path(str(xml));self.data=mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model,self.data,self.model.key('initial').id)
        self.initial_qpos=self.data.qpos.copy()
        if [self.model.body(i).name for i in range(1,self.model.nbody)]!=[b['id'] for b in self.manifest['bodies']]:raise ValueError('Frozen body identity mismatch')

    def frame(self,qpos=None,sim_time_s=0.,translation_m=(0.,0.,0.),yaw_rad=0.):
        q=self.initial_qpos if qpos is None else np.asarray(qpos,dtype=float)
        shift=np.asarray(translation_m,dtype=float)
        if q.shape!=(self.model.nq,) or not np.isfinite(q).all():raise ValueError('State does not match the frozen model')
        if shift.shape!=(3,) or not np.isfinite(shift).all() or not math.isfinite(yaw_rad) or not math.isfinite(sim_time_s):raise ValueError('Invalid rigid placement/time')
        c,s=math.cos(yaw_rad),math.sin(yaw_rad);rotation=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        self.data.qpos[:]=q;self.data.time=sim_time_s
        mujoco.mj_kinematics(self.model,self.data);mujoco.mj_comPos(self.model,self.data);mujoco.mj_camlight(self.model,self.data)
        def pose(position,matrix):return dict(position_m=(rotation@position+shift).tolist(),quaternion_wxyz=matrix_quat(rotation@matrix.reshape(3,3)).tolist())
        bodies={self.model.body(i).name:pose(self.data.xpos[i],self.data.xmat[i]) for i in range(1,self.model.nbody)}
        geometries={entry['id']:pose(self.data.geom_xpos[entry['source_index']],self.data.geom_xmat[entry['source_index']]) for entry in self.manifest['geometries']}
        return dict(schema='robofrontier.pose-frame.v1',case_id=self.manifest['case']['id'],source_scene_sha256=self.manifest['scene_sha256'],simulation_time_s=sim_time_s,
                    units='metres',quaternion_order='wxyz',placement=dict(translation_m=shift.tolist(),yaw_rad=yaw_rad,scale=1),bodies=bodies,geometries=geometries,
                    scope='Recorded-state FK/rigid placement only; no physics steps, IK, contacts, factory composition or resimulation')
