"""Visual-only workstation composition and recorded-pose factory playback.

The factory has no physical task joints or contacts. Source physics runs remain
independent; only frozen geometry and recorded FK poses enter this renderer.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from .robot_control import matrix_quat,quat_matrix
from .workstation import Workstation
from .run_setup import write_json


def vec(values):return ' '.join(repr(float(x)) for x in np.asarray(values).ravel())


def placements(catalog,regions,root,aisle=1.2,source_overrides=None):
    """Size-aware 3x3 shelves; physical hardware is never scaled."""
    result=[]
    for region in regions:
        family=region['id'];entries=[];gap=float(region.get('workstation_gap_m',aisle))
        for case in catalog['cases']:
            if case['family']!=family:continue
            source=Workstation((source_overrides or {}).get(case['id'],root/case['directory']));m,d=source.model,source.data;source.frame()
            corners=[]
            for g in range(m.ngeom):
                name=m.geom(g).name
                if int(m.geom_type[g])==mujoco.mjtGeom.mjGEOM_PLANE or name.startswith(('sky','floor')):continue
                if not(m.geom_contype[g] or m.geom_conaffinity[g]):continue
                box=m.geom_aabb[g];r=d.geom_xmat[g].reshape(3,3);center=d.geom_xpos[g]+r@box[:3];half=abs(r)@box[3:]
                corners.extend((center[:2]-half[:2],center[:2]+half[:2]))
            bases=np.asarray([b['position_parent_m'][:2] for b in source.manifest['bodies'] if b['id'].endswith('_link0')])
            low=np.minimum(np.min(corners,axis=0),bases.min(axis=0)-1.)
            high=np.maximum(np.max(corners,axis=0),bases.max(axis=0)+1.)
            entries.append(dict(case=case,low=low,high=high,size=high-low))
        if not entries:continue
        scales=sorted({e['case'].get('dials',e['case']['arms']) for e in entries})
        def cell(row,col):
            level,scale=(col+1,scales[row]) if family=='transport' else (row+1,scales[col])
            return next(e for e in entries if e['case']['level']==level and e['case'].get('dials',e['case']['arms'])==scale)
        grid=[[cell(row,col) for col in range(3)] for row in range(3)]
        widths=[max(grid[r][c]['size'][0] for r in range(3)) for c in range(3)]
        heights=[max(grid[r][c]['size'][1] for c in range(3)) for r in range(3)]
        x0,y0,x1,y1=region['bounds_xy_m'];total=np.array([sum(widths)+2*gap,sum(heights)+2*gap])
        origin=(np.array([x0+x1,y0+y1])-total)/2
        if np.any(origin<np.array([x0,y0])+.6):raise ValueError('Region too small: '+family)
        for row in range(3):
            for col in range(3):
                e=grid[row][col];center=origin+[sum(widths[:col])+col*gap+widths[col]/2,sum(heights[:row])+row*gap+heights[row]/2]
                shift=center-(e['low']+e['high'])/2;case=e['case']
                result.append(dict(case_id=case['id'],family=family,source_directory=case['directory'],translation_m=[*shift,0.],yaw_rad=0.,scale=1.,
                    planning_bounds_xy_m=[*(e['low']+shift),*(e['high']+shift)],inter_cell_gap_m=gap,planning_scope='Initial physical bounds plus indicative 1m base radius; actual carried-motion swept bounds remain to be checked'))
    return result


class Assets:
    def __init__(self,asset,folder):self.asset=asset;self.folder=folder;self.signatures={};self.counter=0
    def import_source(self,source):
        xml=ET.parse(source.path/'scene.xml').getroot();compiler=xml.find('compiler');mapping={}
        for original in sorted(xml.findall('asset/*'),key=lambda e:e.tag=='material'):
            if original.tag=='texture' and original.get('type')=='skybox':continue
            e=deepcopy(original);old=e.attrib.pop('name',None);e.attrib.pop('class',None)
            if e.tag=='material':
                mid=source.model.mat(old).id
                for key,field in [('rgba','mat_rgba'),('specular','mat_specular'),('shininess','mat_shininess'),('reflectance','mat_reflectance'),('emission','mat_emission')]:
                    e.set(key,vec(np.atleast_1d(getattr(source.model,field)[mid])))
            for key in ('texture','material','mesh'):
                if e.get(key) in mapping:e.set(key,mapping[e.get(key)])
            if e.get('file'):
                prefix=compiler.get('meshdir','') if e.tag=='mesh' else compiler.get('texturedir','') if e.tag=='texture' else ''
                path=(source.path/prefix/e.get('file')).resolve();checksum=hashlib.sha256(path.read_bytes()).hexdigest()
                target=self.folder/'workstation-assets'/(checksum+path.suffix);target.parent.mkdir(exist_ok=True)
                if not target.exists():shutil.copyfile(path,target)
                e.set('file',str(target.relative_to(self.folder)))
            signature=(e.tag,tuple(sorted(e.attrib.items())),ET.tostring(e,encoding='unicode') if len(e) else '')
            if signature not in self.signatures:
                name=f'ws_asset_{self.counter}';self.counter+=1;e.set('name',name);self.asset.append(e);self.signatures[signature]=name
            if old:mapping[old]=self.signatures[signature]
        return mapping


def compose(shell,catalog_path,output,table_opening_scale=None,source_overrides=None,source_root=None,placement_overrides=None):
    shell=Path(shell).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    root_dir=Path(source_root).resolve() if source_root else Path(__file__).resolve().parents[1];catalog=json.loads(Path(catalog_path).read_text())
    shell_manifest=json.loads((shell.parent/'manifest.json').read_text());plan=placements(catalog,shell_manifest['regions'],root_dir,source_overrides=source_overrides)
    overrides=placement_overrides or {}
    if set(overrides)-{p['case_id'] for p in plan}:raise ValueError('Unknown workstation placement override')
    for place in plan:
        if place['case_id'] not in overrides:continue
        values=overrides[place['case_id']]
        if set(values)-{'translation_m','yaw_rad'}:raise ValueError('Only rigid translation/yaw may change')
        shift=np.asarray(values.get('translation_m',place['translation_m']),float)
        yaw=float(values.get('yaw_rad',place['yaw_rad']))
        if shift.shape!=(3,) or not np.isfinite(shift).all() or not np.isfinite(yaw):raise ValueError('Invalid rigid placement')
        # Placement bounds describe source XY footprints, never a scale change.
        a,b,c,d=place['planning_bounds_xy_m'];old=np.asarray(place['translation_m'])
        corners=np.array([[x,y,0.] for x in (a,c) for y in (b,d)])-old
        angle=yaw-place['yaw_rad'];co,si=math.cos(angle),math.sin(angle)
        moved=corners@np.array([[co,-si,0],[si,co,0],[0,0,1.]]).T+shift
        place.update(translation_m=shift.tolist(),yaw_rad=yaw,
            planning_bounds_xy_m=[*moved[:,:2].min(axis=0),*moved[:,:2].max(axis=0)])
    root=ET.parse(shell).getroot();shutil.copytree(shell.parent/'assets',output/'assets')
    if (shell.parent/'build-config.json').exists():shutil.copyfile(shell.parent/'build-config.json',output/'build-config.json')
    option=root.find('option')
    if option is None:option=ET.SubElement(root,'option')
    flag=option.find('flag')
    if flag is None:flag=ET.SubElement(option,'flag')
    flag.set('contact','disable')
    assets=Assets(root.find('asset'),output);world=root.find('worldbody');mapping={}
    def floor_label(name,text,position,half):
        image=Image.new('RGB',(1600,256),(216,220,216));draw=ImageDraw.Draw(image)
        font=ImageFont.truetype(str(root_dir/'assets/robo_factory/fonts/BarlowCondensed-SemiBold.ttf'),150)
        draw.text((800,128),text,font=font,anchor='mm',fill=(35,57,67))
        file=output/'workstation-assets'/(name+'.png');file.parent.mkdir(exist_ok=True);image.save(file)
        ET.SubElement(root.find('asset'),'texture',name=name,type='2d',file=str(file.relative_to(output)))
        ET.SubElement(root.find('asset'),'material',name=name,texture=name,texuniform='false',specular='0')
        ET.SubElement(world,'geom',name=name,type='box',pos=vec(position),size=vec([*half,.0001]),material=name,contype='0',conaffinity='0',mass='0')
    for region in shell_manifest['regions']:
        x0,y0,x1,y1=region['bounds_xy_m'];floor_label('ws_region_'+region['id'],region['label'],[(x0+x1)/2,y1-.6,.0007],[3.6,.5])
    types={int(v):str(k)[7:].lower() for k,v in mujoco.mjtGeom.__members__.items()}
    # enum names are mjGEOM_BOX etc; conversion strips mjGEOM_.
    for place in plan:
        source=Workstation((source_overrides or {}).get(place['case_id'],root_dir/place['source_directory']));source.frame();m,d=source.model,source.data;amap=assets.import_source(source)
        from .suite.table_presentation import scale_rectangular_opening_for_render
        if place['family']=='transport':
            item=next(c for c in catalog['cases'] if c['id']==place['case_id'])
            scale_rectangular_opening_for_render(m,item.get('table_opening_scale',1.) if table_opening_scale is None else table_opening_scale)
        t=np.asarray(place['translation_m']);yaw=place['yaw_rad'];c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        prefix=place['case_id']+'__';bodymap={};geommap={};animated=set(source.manifest['motion_recording_contract']['dynamic_bodies'])
        for bid in range(m.nbody):
            # Source cameras show groups0/1/2. Collision-only robot meshes in
            # group3 would overlap their visual skins and cause z-fighting.
            ids=[g for g in range(m.ngeom) if int(m.geom_bodyid[g])==bid and int(m.geom_group[g])<=2 and int(m.geom_type[g])!=mujoco.mjtGeom.mjGEOM_PLANE and not m.geom(g).name.startswith(('sky','floor'))]
            if not ids:continue
            name=m.body(bid).name or 'world';r=np.eye(3) if not bid else d.xmat[bid].reshape(3,3);p=np.zeros(3) if not bid else d.xpos[bid]
            body=ET.SubElement(world,'body',name=prefix+name,pos=vec(rotation@p+t),quat=vec(matrix_quat(rotation@r)))
            if name in animated:body.set('mocap','true')
            bodymap[name]=body.get('name')
            for g in ids:
                entry=source.manifest['geometries'][g];local_r=quat_matrix(m.geom_quat[g]);local_p=m.geom_pos[g].copy();typ=int(m.geom_type[g])
                kwargs=dict(name=prefix+entry['id'],type=types[typ],size=vec(m.geom_size[g]),pos=vec(local_p),quat=vec(matrix_quat(local_r)),
                    rgba=vec(m.geom_rgba[g]),contype='0',conaffinity='0',mass='0',group='0')
                if typ==mujoco.mjtGeom.mjGEOM_MESH:
                    mid=int(m.geom_dataid[g]);raw_r=local_r@quat_matrix(m.mesh_quat[mid]).T;raw_p=local_p-raw_r@m.mesh_pos[mid]
                    kwargs.update(mesh=amap[m.mesh(mid).name],pos=vec(raw_p),quat=vec(matrix_quat(raw_r)))
                if m.geom_matid[g]>=0:kwargs['material']=amap[m.mat(int(m.geom_matid[g])).name]
                if entry['id'].startswith('table_') and 'leg' not in entry['id']:
                    if root.find("asset/material[@name='factory_bench_wood']") is not None:kwargs['material']='factory_bench_wood'
                ET.SubElement(body,'geom',**kwargs);geommap[entry['id']]=kwargs['name']
        mapping[place['case_id']]=dict(body_names=bodymap,geom_names=geommap,source_scene_sha256=source.manifest['scene_sha256'])
        lowx,lowy,highx,highy=place['planning_bounds_xy_m']
        floor_label('ws_caption_'+place['case_id'],place['case_id'].upper(),[(lowx+highx)/2,lowy+.18,.0007],[min(1.4,(highx-lowx)/2-.10),.13])
    ET.indent(root);scene=output/'scene.xml';ET.ElementTree(root).write(scene,encoding='unicode')
    manifest=deepcopy(shell_manifest);manifest.update(id=shell_manifest['id']+'-workstations',scope=f'{len(plan)} visual workstations; recorded-state playback only; zero task dynamics',
        scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),placements=plan,workstation_names=mapping,task_workstations=len(plan),source_catalog=str(Path(catalog_path).resolve()),table_opening_linear_scale=table_opening_scale,source_overrides={k:str(Path(v).resolve())for k,v in (source_overrides or {}).items()})
    manifest['workstation_assets']=[dict(path=str(p.relative_to(output)),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size) for p in sorted((output/'workstation-assets').iterdir())]
    write_json(output/'manifest.json',manifest)
    return scene


class FactoryPlayback:
    def __init__(self,scene,model=None,data=None):
        self.path=Path(scene);self.manifest=json.loads((self.path.parent/'manifest.json').read_text())
        if hashlib.sha256(self.path.read_bytes()).hexdigest()!=self.manifest['scene_sha256']:raise ValueError('Factory scene checksum mismatch')
        self.model=model if model is not None else mujoco.MjModel.from_xml_path(str(self.path));self.data=data if data is not None else mujoco.MjData(self.model)
        if self.model.nv or self.model.nu:raise ValueError('Presentation must contain no task dynamics')
        self.sources={};self.presentations={};self._recorded_maps={}
    def configure_workstation(self,case_id,source,table_opening_scale=.5,translation_m=None,yaw_rad=None):
        """Configure a visual-only instance; source recordings stay immutable.

        One rigid transform moves the whole workstation, including static table
        and robot bases. Moving bodies subsequently use the same transform in FK.
        Relative layout, geometry scale and qpos must remain source-compatible.
        """
        if case_id in self.presentations:raise ValueError('Workstation presentation is already configured')
        if not 0<float(table_opening_scale)<=1:raise ValueError('Opening scale must be in (0,1]')
        entry=self.manifest['workstation_names'][case_id]
        if entry['source_scene_sha256']!=source.manifest['scene_sha256']:raise ValueError('Presentation source revision differs from factory')
        place=next(p for p in self.manifest['placements'] if p['case_id']==case_id)
        shift=np.asarray(place['translation_m'] if translation_m is None else translation_m,dtype=float)
        yaw=float(place['yaw_rad'] if yaw_rad is None else yaw_rad)
        if shift.shape!=(3,)or not np.isfinite(shift).all()or not np.isfinite(yaw):raise ValueError('Invalid rigid workstation placement')
        old_yaw=place['yaw_rad'];old_shift=np.asarray(place['translation_m'])
        angle=yaw-old_yaw;c,s=np.cos(angle),np.sin(angle);delta=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        for target in entry['body_names'].values():
            bid=self.model.body(target).id;mid=int(self.model.body_mocapid[bid])
            self.model.body_pos[bid]=delta@(self.model.body_pos[bid]-old_shift)+shift
            self.model.body_quat[bid]=matrix_quat(delta@quat_matrix(self.model.body_quat[bid]))
            if mid>=0:
                self.data.mocap_pos[mid]=delta@(self.data.mocap_pos[mid]-old_shift)+shift
                self.data.mocap_quat[mid]=matrix_quat(delta@quat_matrix(self.data.mocap_quat[mid]))
        place['translation_m']=shift.tolist();place['yaw_rad']=yaw
        from .suite.table_presentation import scale_rectangular_opening_for_render
        table=scale_rectangular_opening_for_render(source.model,table_opening_scale)
        if table:
            for name in table['table_geoms']:
                src=source.model.geom(name).id;dst=self.model.geom(entry['geom_names'][name]).id
                for field in ['geom_pos','geom_size','geom_rbound','geom_aabb']:
                    getattr(self.model,field)[dst]=getattr(source.model,field)[src]
        self.presentations[case_id]=dict(table=table,translation_m=shift.tolist(),yaw_rad=yaw)
        return self.presentations[case_id]
    def apply_arrays(self,case_id,source,qpos,time_s,verify=False,commit=True):
        """Vectorized offline FK/placement, without building per-geom JSON.

        Compiled maps are private to this frozen source/target pair. Joint angles
        stay local; one common rigid transform moves bodies and the workstation.
        """
        entry=self.manifest['workstation_names'][case_id]
        if source.manifest['scene_sha256']!=entry['source_scene_sha256']:
            raise ValueError('Replay source revision differs from factory workstation')
        key=(case_id,id(source.model))
        if key not in self._recorded_maps:
            place=next(p for p in self.manifest['placements'] if p['case_id']==case_id)
            if place.get('scale',1)!=1:raise ValueError('Recorded hardware cannot be scaled')
            yaw=place['yaw_rad'];c,s=np.cos(yaw),np.sin(yaw)
            rotation=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
            pairs=[(source.model.body(name).id,int(self.model.body_mocapid[self.model.body(target).id])) for name,target in entry['body_names'].items()]
            pairs=np.asarray([(a,b) for a,b in pairs if b>=0],dtype=int)
            # Workstation IDs also cover unnamed imported robot geoms. They
            # are registry identities (e.g. A_link0:geom0), not MuJoCo names.
            registry={g['id']:g['source_index'] for g in source.manifest['geometries']}
            geoms=np.asarray([(registry[name],self.model.geom(target).id) for name,target in entry['geom_names'].items()],dtype=int)
            self._recorded_maps[key]=(rotation,np.asarray(place['translation_m']),pairs,geoms)
        rotation,shift,pairs,geoms=self._recorded_maps[key]
        qpos=np.asarray(qpos)
        if qpos.shape!=(source.model.nq,) or not np.isfinite(qpos).all() or not np.isfinite(time_s):
            raise ValueError('Invalid recorded state')
        source.data.qpos[:]=qpos;source.data.time=time_s
        mujoco.mj_kinematics(source.model,source.data);mujoco.mj_comPos(source.model,source.data);mujoco.mj_camlight(source.model,source.data)
        self.data.mocap_pos[pairs[:,1]]=source.data.xpos[pairs[:,0]]@rotation.T+shift
        # A shared world yaw is left-multiplied into each body's wxyz rotation.
        yaw=np.arctan2(rotation[1,0],rotation[0,0]);c,s=np.cos(yaw/2),np.sin(yaw/2)
        q=source.data.xquat[pairs[:,0]];w,x,y,z=q.T
        self.data.mocap_quat[pairs[:,1]]=np.column_stack([c*w-s*z,c*x-s*y,c*y+s*x,c*z+s*w])
        if verify and not commit:raise ValueError('Deferred verification requires a committed factory frame')
        if commit:self.commit_frame(time_s)
        if verify:
            return self.verify_station(case_id,source)
    def verify_station(self,case_id,source):
        """Verify committed geometry without changing any station or clock."""
        rotation,shift,pairs,geoms=self._recorded_maps[(case_id,id(source.model))]
        expected=source.data.geom_xpos[geoms[:,0]]@rotation.T+shift
        pe=float(np.max(np.linalg.norm(self.data.geom_xpos[geoms[:,1]]-expected,axis=1)))
        expected_r=np.einsum('ij,njk->nik',rotation,source.data.geom_xmat[geoms[:,0]].reshape(-1,3,3))
        re=float(np.max(abs(self.data.geom_xmat[geoms[:,1]].reshape(-1,3,3)-expected_r)))
        if pe>1e-6 or re>1e-6:raise ValueError(('Factory rigid pose mismatch',pe,re))
        return dict(position_error_m=pe,rotation_matrix_error=re)
    def commit_frame(self,presentation_time_s):
        if not np.isfinite(presentation_time_s) or presentation_time_s<0:raise ValueError('Invalid presentation time')
        self.data.time=presentation_time_s
        mujoco.mj_kinematics(self.model,self.data);mujoco.mj_camlight(self.model,self.data)
    def apply(self,case_id,source,qpos,time_s):
        entry=self.manifest['workstation_names'][case_id]
        if source.manifest['scene_sha256']!=entry['source_scene_sha256']:raise ValueError('Replay source revision differs from factory workstation')
        place=next(p for p in self.manifest['placements'] if p['case_id']==case_id)
        frame=source.frame(qpos,time_s,place['translation_m'],place['yaw_rad'])
        for name,target in entry['body_names'].items():
            bid=self.model.body(target).id;mid=int(self.model.body_mocapid[bid])
            if mid>=0:self.data.mocap_pos[mid]=frame['bodies'][name]['position_m'];self.data.mocap_quat[mid]=frame['bodies'][name]['quaternion_wxyz']
        self.data.time=time_s;mujoco.mj_kinematics(self.model,self.data);mujoco.mj_camlight(self.model,self.data)
        return frame
    def appearance(self,case_id,rgba):
        names=self.manifest['workstation_names'][case_id]['geom_names']
        for name,value in rgba.items():
            if name in names:self.model.geom_rgba[self.model.geom(names[name]).id]=value
