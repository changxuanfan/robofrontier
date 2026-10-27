"""Offline placement/edit tests use tiny scenes, without generated factory assets."""
import hashlib,json,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET
import mujoco,numpy as np
from robofrontier.factory_workstations import FactoryPlayback,vec
from robofrontier.robot_control import matrix_quat
from robofrontier.workstation import Workstation

class FactoryMotionEditTests(unittest.TestCase):
    def test_half_table_and_all_static_dynamic_parts_share_one_rigid_frame(self):
        xml='''<mujoco><worldbody>
        <geom name="table_0" type="box" pos="0 -1.5 .775" size="2 .5 .025"/>
        <geom name="table_1" type="box" pos="0 1.5 .775" size="2 .5 .025"/>
        <geom name="table_2" type="box" pos="-1.5 0 .775" size=".5 1 .025"/>
        <geom name="table_3" type="box" pos="1.5 0 .775" size=".5 1 .025"/>
        <body name="base" pos="2 0 .8"><geom name="base_geom" type="box" size=".1 .1 .1"/>
        <body name="arm" pos="0 0 .2"><joint name="hinge"/><geom name="arm_geom" type="box" pos=".2 0 .2" size=".03 .03 .2"/></body></body>
        <body name="item" pos="1 1 1"><freejoint/><geom name="item_geom" type="box" size=".05 .04 .03"/></body>
        </worldbody></mujoco>'''
        m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_kinematics(m,d)
        entries=[dict(id=m.geom(i).name,source_index=i)for i in range(m.ngeom)]
        source=types.SimpleNamespace(model=m,data=d,initial_qpos=d.qpos.copy(),manifest=dict(case=dict(id='transport-test'),geometries=entries,scene_sha256='source'))
        source.frame=types.MethodType(Workstation.frame,source)
        root=ET.Element('mujoco');world=ET.SubElement(root,'worldbody');bodies={};geoms={};oldshift=np.array([3.,4.,0.])
        for bid in range(m.nbody):
            name=m.body(bid).name or 'world';target='ws_'+name;bodies[name]=target
            body=ET.SubElement(world,'body',name=target,pos=vec(d.xpos[bid]+oldshift),quat=vec(matrix_quat(d.xmat[bid].reshape(3,3))))
            if name in ['arm','item']:body.set('mocap','true')
            for gid in range(m.ngeom):
                if m.geom_bodyid[gid]!=bid:continue
                original=m.geom(gid).name;geoms[original]='ws_'+original
                ET.SubElement(body,'geom',name=geoms[original],type='box',pos=vec(m.geom_pos[gid]),size=vec(m.geom_size[gid]),contype='0',conaffinity='0')
        with tempfile.TemporaryDirectory()as td,patch('mujoco.mj_step',side_effect=AssertionError('No physics')):
            path=Path(td)/'scene.xml';path.write_text(ET.tostring(root,encoding='unicode'))
            (path.parent/'manifest.json').write_text(json.dumps(dict(scene_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),placements=[dict(case_id='transport-test',translation_m=oldshift.tolist(),yaw_rad=0.)],workstation_names={'transport-test':dict(source_scene_sha256='source',body_names=bodies,geom_names=geoms)})))
            play=FactoryPlayback(path);before=source.data.qpos.copy()
            config=play.configure_workstation('transport-test',source,translation_m=[-5.,2.,.6],yaw_rad=1.1)
            self.assertEqual(config['table']['new_opening_m'],[1.,1.])
            q=before.copy();q[0]=.4;q[-7:-4]+=[.1,.2,.3]
            frame=play.apply('transport-test',source,q,2.)
            for name,target in geoms.items():
                gid=play.model.geom(target).id
                np.testing.assert_allclose(play.data.geom_xpos[gid],frame['geometries'][name]['position_m'],atol=1e-12)
                np.testing.assert_allclose(play.model.geom_size[gid],source.model.geom(name).size,atol=1e-12)
            with self.assertRaisesRegex(ValueError,'already configured'):play.configure_workstation('transport-test',source)

if __name__=='__main__':unittest.main()
