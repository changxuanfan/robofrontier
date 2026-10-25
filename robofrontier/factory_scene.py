"""Build a portable static factory background, independent of task workstations."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'configs/backgrounds/robo_factory.json'
LAYERS = {'structure': 0, 'roof': 1, 'north': 2, 'south': 3, 'west': 4, 'east': 5}


def _vec(x):
    return ' '.join(f'{float(v):.8g}' for v in x)


def _hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _font(size):
    path = ROOT / 'assets/robo_factory/fonts/BarlowCondensed-SemiBold.ttf'
    return ImageFont.truetype(str(path), size) if path.exists() else ImageFont.load_default(size=size)


def _texture_assets(folder, seed):
    """Deterministic, tileable authored finishes; no image API or external service."""
    folder.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    size = 1024
    noise = np.zeros((size, size), dtype=np.float32)
    for n, weight in [(8, 2.6), (32, 1.5), (128, .8), (1024, .55)]:
        a = rng.random((n, n), dtype=np.float32)
        a = np.asarray(Image.fromarray(a).resize((size, size), Image.Resampling.BICUBIC))
        noise += (a - .5) * weight
    # Periodic high-frequency texture, with restrained coarse mottling.
    y, x = np.mgrid[:size, :size]
    field = noise + .7 * np.sin(2*np.pi*x/size) * np.sin(4*np.pi*y/size)
    gray = np.clip(150 + 4.5*field, 132, 167).astype(np.uint8)
    color = np.stack([gray, np.clip(gray.astype(int)+1, 0, 255), gray], axis=-1).astype(np.uint8)
    Image.fromarray(color).save(folder/'concrete-color.png')
    rough = np.clip(158 + field*8, 132, 188).astype(np.uint8)
    Image.fromarray(np.repeat(rough[..., None], 3, axis=2)).save(folder/'concrete-roughness.png')
    grain=np.sin(y*.16+np.sin(x*.008)*2.5+noise*1.2)*.025+noise*.018
    wood=np.clip(.90+grain,.76,.98)
    Image.fromarray(np.uint8(np.repeat(wood[...,None],3,axis=2)*255)).save(folder/'bench-grain.png')
    # A tiny normal variation; the 4m tile represents fine grain, not deep bumps.
    smooth = np.asarray(Image.fromarray(gray).filter(ImageFilter.GaussianBlur(1.1))).astype(float)
    dy, dx = np.gradient(smooth)
    normal = np.stack([-dx*.7, -dy*.7, np.full_like(dx, 127)], axis=-1)
    normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
    Image.fromarray(np.uint8(np.clip((normal*.5+.5)*255, 0, 255))).save(folder/'concrete-normal.png')
    # Wall paint with quiet roller variation, suitable for reuse across many bays.
    paint = np.clip(184 + field*1.7, 176, 194).astype(np.uint8)
    Image.fromarray(np.stack([paint, paint, np.clip(paint.astype(int)-3, 0, 255)], -1).astype(np.uint8)).save(folder/'wall-paint.png')
    # Ribs are texture detail; roof thickness and framing remain real geometry.
    rib = np.clip(96 + 9*np.cos(x/size*2*np.pi*24) + noise*.5, 75, 110).astype(np.uint8)
    Image.fromarray(np.stack([rib, rib, np.clip(rib.astype(int)+2, 0, 255)], -1).astype(np.uint8)).save(folder/'roof-ribs.png')
    stripe = Image.new('RGB', (256, 512), (236, 169, 37))
    draw = ImageDraw.Draw(stripe)
    for offset in range(-512, 768, 120):
        draw.polygon([(0, offset), (256, offset+160), (256, offset+215), (0, offset+55)], fill=(30, 34, 35))
    stripe.save(folder/'hazard-stripe.png')
    # Filterable shutter grooves: sub-pixel black 3D slats broke into dots in
    # distant views. A mipmapped finish preserves the broad panel silhouette.
    yy=np.arange(2048)[:,None]
    phase=(yy/2048*4.76/.13)%1
    crease=np.exp(-((phase-.5)/.045)**2)
    highlight=np.exp(-((phase-.42)/.08)**2)
    shutter=np.repeat(np.clip(174-29*crease+6*highlight,0,255),1024,axis=1)
    Image.fromarray(np.uint8(np.repeat(shutter[...,None],3,axis=2))).save(folder/'shutter-ribs.png')
    source=folder.parent/'source/polyhaven'
    for stem in ['concrete-color','concrete-roughness','concrete-normal']:
        cached=source/(stem+'.jpg')
        if cached.exists():
            pixels=np.asarray(Image.open(cached).convert('RGB')).astype(float)
            if stem=='concrete-color':pixels=pixels*.50+pixels.mean(axis=(0,1),keepdims=True)*.50
            elif stem=='concrete-normal':pixels=pixels*.16+np.array([128,128,255])*.84
            Image.fromarray(np.uint8(np.clip(pixels,0,255))).save(folder/(stem+'.png'))
    # Repeated maintenance graphics are shared flat textures, not hundreds of tiny meshes.
    for name in ['process','safety','tools']:
        image=Image.new('RGB',(1024,512),(221,224,218) if name!='tools' else (79,91,96))
        draw=ImageDraw.Draw(image)
        if name=='tools':
            for xx in range(16,1024,22):
                for yy in range(16,512,22):draw.ellipse((xx-2,yy-2,xx+2,yy+2),fill=(43,53,58))
            draw.rectangle((0,0,1024,62),fill=(33,54,69))
            draw.text((26,10),'MAINTENANCE  /  TOOL CONTROL',font=_font(38),fill=(233,236,230))
            for i,xx in enumerate(range(75,980,110)):
                top=105+(i%3)*20;bottom=300+(i%2)*70
                draw.line((xx,top+25,xx,bottom),fill=(173,181,180),width=16)
                draw.arc((xx-23,top,xx+23,top+46),0,295,fill=(204,210,207),width=13)
                draw.rounded_rectangle((xx-13,bottom-70,xx+13,bottom+10),7,fill=(22,42,56))
                draw.text((xx-27,438),f'{i+1:02}',font=_font(24),fill=(200,208,204))
        else:
            draw.rectangle((0,0,1024,76),fill=(24,63,86) if name=='process' else (206,149,42))
            draw.text((28,13),'OPERATIONS / DAILY STATUS' if name=='process' else 'SAFETY / AUTHORIZED PERSONNEL',font=_font(42),fill=(239,241,234) if name=='process' else (31,35,36))
            if name=='process':
                for xx in [30,345,660,995]:draw.line((xx,115,xx,470),fill=(156,169,172),width=2)
                for yy in [155,210,265,320,375,430,470]:draw.line((30,yy,995,yy),fill=(156,169,172),width=2)
                for i,text in enumerate(['PLAN','IN PROGRESS','VERIFIED']):draw.text((50+i*315,106),text,font=_font(31),fill=(43,62,69))
                for col,rows in [(0,[0,2,3]),(1,[1,2]),(2,[0,1,3,4])]:
                    for row in rows:
                        x=52+col*315;y=170+row*55
                        draw.rectangle((x,y,x+220,y+27),fill=[(111,143,160),(201,171,93),(131,158,133)][col])
                        draw.line((x+12,y+9,x+160,y+9),fill=(67,84,87),width=2)
            else:
                for xx,symbol,title in [(174,'!','KEEP AISLES CLEAR'),(512,'I','PPE REQUIRED'),(850,'+','FIRST AID')]:
                    draw.ellipse((xx-66,130,xx+66,262),fill=(29,79,108) if symbol!='!' else (216,164,55))
                    draw.text((xx,194),symbol,font=_font(96),anchor='mm',fill=(239,240,232) if symbol!='!' else (40,43,40))
                    draw.text((xx,310),title,font=_font(27),anchor='mm',fill=(36,48,52))
                    for yy in [356,378,400]:draw.line((xx-98,yy,xx+98,yy),fill=(117,132,135),width=3)
        image.save(folder/('panel_'+name+'.png'))
    return sorted(folder.glob('*.png'))


def _rx(angle):
    return np.array([[1, 0, 0], [0, math.cos(angle), -math.sin(angle)], [0, math.sin(angle), math.cos(angle)]])


def _rz(angle):
    return np.array([[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])


def _floor_finish(folder, config):
    """A restrained whole-floor finish at the existing 2K texture budget.

    Keep fine sourced grain, but put maintenance patches and broad roughness
    variation in building coordinates so they do not repeat every few metres.
    This is appearance only; the floor geometry and collision policy are unchanged.
    """
    size=2048
    rng=np.random.default_rng(config['seed']+303)
    def repeat_source(name):
        tile=Image.open(folder/(name+'.png')).convert('RGB').resize((256,256),Image.Resampling.LANCZOS)
        return np.tile(np.asarray(tile,dtype=float),(8,8,1))
    color=repeat_source('concrete-color');gray=color.mean(2)
    small=Image.fromarray(rng.random((24,24),dtype=np.float32))
    macro=np.asarray(small.resize((size,size),Image.Resampling.BICUBIC))-.5
    finish=128+(gray-gray.mean())*.45+macro*8
    rough=150+(repeat_source('concrete-roughness').mean(2)-132)*.35+macro*12
    # Soft-edged, slightly different maintenance patches; no repeating dirt tiles.
    W,D=config['building']['width_m'],config['building']['depth_m']
    for x,y,w,h,tone in [(-.23*W,-.24*D,2.0,1.5,3),(.31*W,.20*D,2.4,1.8,-3),(-.37*W,.32*D,1.8,1.2,-2)]:
        mask=Image.new('L',(size,size));draw=ImageDraw.Draw(mask)
        draw.rectangle(((x-w/2+W/2)/W*size,(y-h/2+D/2)/D*size,(x+w/2+W/2)/W*size,(y+h/2+D/2)/D*size),fill=255)
        alpha=np.asarray(mask.filter(ImageFilter.GaussianBlur(2)),dtype=float)/255
        finish+=alpha*tone;rough+=alpha*7
    # A few quiet scuffs in the perimeter service strips, not in task slots.
    scuffs=Image.new('L',(size,size));draw=ImageDraw.Draw(scuffs)
    for side in [-1,1]:
        for y in [-.34*D,-.08*D,.21*D,.36*D]:
            x=side*(W/2-1.8)
            px=(x+W/2)/W*size;py=(y+D/2)/D*size
            draw.arc((px-12,py-7,px+12,py+7),20,140,fill=100,width=1)
    wear=np.asarray(scuffs.filter(ImageFilter.GaussianBlur(.65)),dtype=float)/255
    finish-=wear*8;rough+=wear*9
    Image.fromarray(np.uint8(np.clip(np.repeat(finish[...,None],3,axis=2),0,255))).save(folder/'concrete-color.png')
    Image.fromarray(np.uint8(np.clip(np.repeat(rough[...,None],3,axis=2),0,255))).save(folder/'concrete-roughness.png')
    normal=repeat_source('concrete-normal')*.35+np.array([128,128,255])*.65
    Image.fromarray(np.uint8(np.clip(normal,0,255))).save(folder/'concrete-normal.png')


class _Builder:
    def __init__(self, config, assets, output):
        self.cfg, self.assets_path, self.output = config, assets, output
        b = config['building']
        self.width, self.depth, self.height = b['width_m'], b['depth_m'], b['eave_height_m']
        self.root = ET.Element('mujoco', model=config['id'])
        ET.SubElement(self.root, 'compiler', angle='radian', autolimits='true', inertiafromgeom='false')
        ET.SubElement(self.root, 'option', timestep='.002', gravity='0 0 -9.81')
        self.asset = ET.SubElement(self.root, 'asset')
        self.world = ET.SubElement(self.root, 'worldbody')
        self.defaults = ET.SubElement(self.root, 'default')
        background_class=ET.SubElement(self.defaults,'default',{'class':'factory_visual'})
        ET.SubElement(background_class, 'geom', contype='0', conaffinity='0', density='0')
        v = ET.SubElement(self.root, 'visual')
        ET.SubElement(v, 'global', offwidth='1920', offheight='1920')
        ET.SubElement(v, 'quality', shadowsize=str(config['lighting']['shadow_map_size']), offsamples='4', numslices='16', numstacks='8')
        ET.SubElement(v, 'headlight', ambient='.23 .23 .23', diffuse='.42 .42 .42', specular='.12 .12 .12')
        ET.SubElement(v, 'map', znear='.005', zfar='5', shadowclip='2', shadowscale='1')
        ET.SubElement(self.root, 'statistic', center='0 0 2.5', extent=str(max(self.width, self.depth)*.55))
        custom = ET.SubElement(self.root, 'custom')
        for name, data in [('cg.exposure', config['lighting']['exposure']), ('ao.enabled', 1), ('shadows.map_size', config['lighting']['shadow_map_size'])]:
            ET.SubElement(custom, 'numeric', name='filament.'+name, data=str(data))
        for name, value in [('ao.quality', config['lighting']['ambient_occlusion_quality']), ('cg.tone_mapping', 'aces')]:
            ET.SubElement(custom, 'text', name='filament.'+name, data=value)
        self.meshes, self.mesh_triangles, self.serial = {}, 0, 0
        self.objects, self.cameras = [], []
        self.props = []
        self.batching = {}
        self._materials()

    def _materials(self):
        colors = {
            'floor': ((.72, .73, .72, 1), .0, .50),
            'wall': ((.78, .78, .75, 1), .0, .87),
            'band': ((.28, .32, .34, 1), .12, .62),
            'concrete': ((.55, .56, .54, 1), .0, .84),
            'steel': ((.09, .105, .12, 1), .18, .58),
            'steel_light': ((.55, .58, .59, 1), .85, .36),
            'roof': ((.61, .63, .64, 1), .65, .62),
            'yellow': ((.95, .58, .065, 1), .0, .68),
            'black': ((.045, .052, .058, 1), .1, .6),
            'cabinet': ((.67, .69, .68, 1), .03, .52),
            'cabinet_dark': ((.29, .33, .34, 1), .03, .55),
            'blue': ((.035, .19, .39, 1), .0, .60),
            'bin_inside': ((.025, .07, .135, 1), .0, .78),
            'label': ((.88, .87, .8, 1), .0, .8),
            'cardboard': ((.47, .33, .19, 1), .0, .92),
            'tape': ((.57, .44, .27, 1), .0, .5),
            'red': ((.65, .035, .018, 1), .12, .34),
            'green': ((.04, .20, .075, 1), .0, .87),
            'leaf_light': ((.1, .29, .12, 1), .0, .78),
            'pot': ((.21, .23, .22, 1), .1, .8),
            'glass': ((.56, .66, .71, .23), .05, .45),
            'lamp': ((1, .98, .91, 1), .0, .2),
            'bench_wood': ((.55,.37,.22,1), .0, .48),
            'exit': ((.02, .42, .16, 1), .0, .65)
        }
        for name, (rgba, metal, rough) in colors.items():
            spec = {'name': 'factory_'+name, 'rgba': _vec(rgba), 'metallic': str(metal), 'roughness': str(rough), 'specular': '.25', 'shininess': '.25'}
            if name == 'lamp': spec['emission'] = '1'
            if name == 'exit': spec['emission'] = '.2'
            material = ET.SubElement(self.asset, 'material', **spec)
            if name=='bench_wood':
                ET.SubElement(self.asset,'texture',name='factory_bench_grain',type='2d',file='assets/textures/bench-grain.png')
                material.set('texture','factory_bench_grain');material.set('texuniform','true');material.set('texrepeat','2 2')
            if name == 'floor':
                if self.cfg['appearance'].get('whole_floor_finish',False):
                    material.set('texuniform', 'false');material.set('texrepeat', '1 1')
                else:
                    material.set('texuniform', 'true');material.set('texrepeat', _vec([1/self.cfg['appearance']['floor_tile_m']]*2))
                for filename, role in [('concrete-color.png', 'rgb'), ('concrete-roughness.png', 'roughness'), ('concrete-normal.png', 'normal')]:
                    texture = 'factory_'+Path(filename).stem.replace('-', '_')
                    ET.SubElement(self.asset, 'texture', name=texture, type='2d', file='assets/textures/'+filename)
                    ET.SubElement(material, 'layer', texture=texture, role=role)
            elif name in ('wall', 'roof'):
                filename = 'wall-paint.png' if name == 'wall' else 'roof-ribs.png'
                texture = 'factory_'+name+'_surface'
                ET.SubElement(self.asset, 'texture', name=texture, type='2d', file='assets/textures/'+filename)
                material.set('texture', texture);material.set('texuniform', 'true');material.set('texrepeat', '.3 .3' if name == 'wall' else '1 1')
        ET.SubElement(self.asset, 'texture', name='factory_sky', type='skybox', builtin='gradient', rgb1='.65 .70 .75', rgb2='.8 .82 .83', width='512', height='3072')
        ET.SubElement(self.asset, 'texture', name='factory_hazard', type='2d', file='assets/textures/hazard-stripe.png')
        ET.SubElement(self.asset, 'material', name='factory_hazard_material', texture='factory_hazard', texuniform='false', roughness='.65')
        ET.SubElement(self.asset,'texture',name='factory_shutter_ribs',type='2d',file='assets/textures/shutter-ribs.png')
        ET.SubElement(self.asset,'material',name='factory_shutter',texture='factory_shutter_ribs',texuniform='false',rgba='.75 .79 .8 1',metallic='.62',roughness='.43')
        ET.SubElement(self.asset,'mesh',name='factory_shutter_face',inertia='shell',vertex='-2.5 -2.38 0 2.5 -2.38 0 2.5 2.38 0 -2.5 2.38 0',face='0 1 2 0 2 3',texcoord='0 1 1 1 1 0 0 0')
        self.mesh_triangles+=2

    def body(self, name, pos=(0, 0, 0), yaw=0, group='structure', category='architecture'):
        body = ET.SubElement(self.world, 'body', name='factory_'+name, childclass='factory_visual', pos=_vec(pos), quat=_vec([math.cos(yaw/2), 0, 0, math.sin(yaw/2)]))
        self.objects.append(dict(id='factory_'+name, category=category, translation_m=list(pos), yaw_rad=yaw, layer=group))
        return body

    def _name(self):
        self.serial += 1
        return 'factory_geom_'+str(self.serial)

    def _bevel_mesh(self, half, bevel):
        half = np.array(half, float);bevel = min(bevel, float(half.min())*.45)
        key = tuple(np.round([*half, bevel], 6))
        if key in self.meshes: return self.meshes[key]
        vertices = []
        for sx in [-1, 1]:
            for sy in [-1, 1]:
                for sz in [-1, 1]:
                    signs=np.array([sx, sy, sz])
                    for axis in range(3):
                        p=half-bevel;p=p.copy();p[axis]=half[axis];vertices.append(p*signs)
        vertices=np.unique(vertices,axis=0);hull=ConvexHull(vertices);faces=[]
        for tri, equation in zip(hull.simplices, hull.equations):
            tri=tri.copy();a,b,c=vertices[tri]
            if np.dot(np.cross(b-a,c-a),equation[:3])<0:tri[[1,2]]=tri[[2,1]]
            faces.append(tri)
        name='factory_bevel_'+str(len(self.meshes));file=self.assets_path/'meshes'/f'{name}.obj';file.parent.mkdir(exist_ok=True)
        file.write_text('\n'.join(['v '+_vec(v) for v in vertices]+['f '+' '.join(str(int(x)+1) for x in tri) for tri in faces])+'\n')
        ET.SubElement(self.asset,'mesh',name=name,file='assets/meshes/'+file.name)
        self.meshes[key]=name;self.mesh_triangles+=len(faces)
        return name

    def box(self, parent, pos, half, material, group='structure', bevel=0, rotation=None):
        attrs=dict(name=self._name(),pos=_vec(pos),material='factory_'+material,group=str(LAYERS[group]))
        if rotation is not None:attrs['xyaxes']=_vec([*rotation[:,0],*rotation[:,1]])
        if bevel>0:attrs.update(type='mesh',mesh=self._bevel_mesh(half,bevel))
        else:attrs.update(type='box',size=_vec(half))
        return ET.SubElement(parent,'geom',**attrs)

    def cylinder(self, parent, pos, radius, halfheight, material, group='structure', rotation=None):
        attrs=dict(name=self._name(),type='cylinder',pos=_vec(pos),size=_vec([radius,halfheight]),material='factory_'+material,group=str(LAYERS[group]))
        if rotation is not None:attrs['xyaxes']=_vec([*rotation[:,0],*rotation[:,1]])
        return ET.SubElement(parent,'geom',**attrs)

    def line(self, parent, a, b, radius, material, group='structure'):
        return ET.SubElement(parent,'geom',name=self._name(),type='capsule',fromto=_vec([*a,*b]),size=str(radius),material='factory_'+material,group=str(LAYERS[group]))

    def sign(self, parent, name, text, pos, size, yaw=0, group='structure', bg=(188,188,181), fg=(28,32,33), painted=False):
        pixels_w=min(1024,max(128,int(size[0]*320)));pixels_h=min(512,max(64,int(size[1]*320)))
        image=Image.new('RGBA' if painted else 'RGB',(pixels_w,pixels_h),(0,0,0,0) if painted else bg);draw=ImageDraw.Draw(image);font=_font(int(pixels_h*.70))
        bounds=draw.textbbox((0,0),text,font=font);scale=min(1,pixels_w*.92/max(1,bounds[2]-bounds[0]));font=_font(max(12,int(pixels_h*.70*scale)))
        draw.text((pixels_w/2,pixels_h/2),text,font=font,anchor='mm',fill=(*fg,255) if painted else fg)
        file=self.assets_path/'textures'/('sign_'+name+'.png');image.save(file)
        tex='factory_sign_'+name
        texture=ET.SubElement(self.asset,'texture',name=tex,type='2d',file='assets/textures/'+file.name)
        if painted:texture.set('nchannel','4')
        ET.SubElement(self.asset,'material',name=tex,texture=tex,texuniform='false',roughness='.85',rgba='1 1 1 .999' if painted else '1 1 1 1')
        # Plane local XY, outward normal +Z, rotated into an upright sign.
        w,h=size;mesh=tex+'_quad'
        ET.SubElement(self.asset,'mesh',name=mesh,inertia='shell',vertex=_vec([-w/2,-h/2,0,w/2,-h/2,0,w/2,h/2,0,-w/2,h/2,0]),face='0 1 2 0 2 3',texcoord='0 1 1 1 1 0 0 0')
        rot=_rz(yaw)@_rx(math.pi/2)
        ET.SubElement(parent,'geom',name=self._name(),type='mesh',mesh=mesh,pos=_vec(pos),xyaxes=_vec([*rot[:,0],*rot[:,1]]),material=tex,group=str(LAYERS[group]))
        self.mesh_triangles+=2

    def panel(self, parent, name, pos, size, group='structure', yaw=0):
        """Reuse a pre-authored panel texture across many real mounted boards."""
        tex='factory_panel_'+name
        if self.asset.find(f"material[@name='{tex}']") is None:
            ET.SubElement(self.asset,'texture',name=tex,type='2d',file='assets/textures/panel_'+name+'.png')
            ET.SubElement(self.asset,'material',name=tex,texture=tex,texuniform='false',roughness='.8')
        mesh=tex+'_quad_'+hashlib.sha256(_vec(size).encode()).hexdigest()[:10]
        if self.asset.find(f"mesh[@name='{mesh}']") is None:
            w,h=size
            ET.SubElement(self.asset,'mesh',name=mesh,inertia='shell',vertex=_vec([-w/2,-h/2,0,w/2,-h/2,0,w/2,h/2,0,-w/2,h/2,0]),face='0 1 2 0 2 3',texcoord='0 1 1 1 1 0 0 0')
            self.mesh_triangles+=2
        rot=_rz(yaw)@_rx(math.pi/2)
        ET.SubElement(parent,'geom',name=self._name(),type='mesh',mesh=mesh,pos=_vec(pos),xyaxes=_vec([*rot[:,0],*rot[:,1]]),material=tex,group=str(LAYERS[group]))

    def static_batches(self):
        """Combine opaque plain-colour boxes per semantic body/material/layer.

        No cross-body transforms, textured surfaces, transparency or cylinders
        are merged. Source bounds remain independently computable for auditing.
        These visual-only meshes are never suitable as a physical collision hull.
        """
        material_ok={m.get('name') for m in self.asset.findall('material')
                     if not m.get('texture') and not m.findall('layer')
                     and float(m.get('rgba','1 1 1 1').split()[3])==1}
        sources={}
        for m in self.asset.findall('mesh'):
            if m.get('file'):
                vertices=[];faces=[]
                for row in (self.assets_path/Path(m.get('file')).relative_to('assets')).read_text().splitlines():
                    a=row.split()
                    if a and a[0]=='v':vertices.append([float(v) for v in a[1:4]])
                    elif a and a[0]=='f':faces.append([int(v.split('/')[0])-1 for v in a[1:4]])
                sources[m.get('name')]=(np.asarray(vertices),np.asarray(faces))
        signs=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]],float)
        cube_faces=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
        before=len(list(self.world.iter('geom')));batches=0;merged=0
        for body in self.world.findall('body'):
            groups={}
            for geom in body.findall('geom'):
                if geom.get('material') not in material_ok:continue
                if geom.get('type')=='box' or geom.get('mesh') in sources:
                    groups.setdefault((geom.get('material'),geom.get('group')),[]).append(geom)
            for (mat,group),geoms in groups.items():
                if len(geoms)<3:continue
                verts=[];faces=[];offset=0
                for geom in geoms:
                    if geom.get('type')=='box':v=signs*np.fromstring(geom.get('size'),sep=' ');f=cube_faces
                    else:v,f=sources[geom.get('mesh')]
                    if geom.get('xyaxes'):
                        axes=np.fromstring(geom.get('xyaxes'),sep=' ').reshape(2,3);rot=np.column_stack([*axes,np.cross(*axes)])
                        v=v@rot.T
                    v=v+np.fromstring(geom.get('pos','0 0 0'),sep=' ')
                    verts.extend(v);faces.extend(f+offset);offset+=len(v);body.remove(geom)
                name=body.get('name')+'_'+mat.removeprefix('factory_')+'_g'+group+'_batch'
                file=self.assets_path/'meshes'/(name+'.obj')
                file.write_text('\n'.join(['v '+_vec(v) for v in verts]+['f '+' '.join(str(int(i)+1) for i in tri) for tri in faces])+'\n')
                ET.SubElement(self.asset,'mesh',name=name,file='assets/meshes/'+file.name)
                ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,material=mat,group=group)
                self.mesh_triangles+=len(faces);batches+=1;merged+=len(geoms)
        used={g.get('mesh') for g in self.world.iter('geom') if g.get('mesh')}
        for mesh in list(self.asset.findall('mesh')):
            if mesh.get('name') not in used:self.asset.remove(mesh)
        self.mesh_triangles=0
        for mesh in self.asset.findall('mesh'):
            if mesh.get('name') in sources:
                self.mesh_triangles+=len(sources[mesh.get('name')][1])
            elif mesh.get('face'):
                self.mesh_triangles+=len(mesh.get('face').split())//3
            else:
                file=self.assets_path/Path(mesh.get('file')).relative_to('assets')
                self.mesh_triangles+=sum(row.startswith('f ') for row in file.read_text().splitlines())
        self.batching=dict(source_geoms=before,render_geoms=len(list(self.world.iter('geom'))),batches=batches,merged_source_geoms=merged,scope='Opaque static boxes/chamfers; within body, material and visibility layer only')

    def wall_geometry(self):
        W,D,H=self.width,self.depth,self.height
        for side,y,yaw,layer in [('north',D/2,0,'north'),('south',-D/2,math.pi,'south')]:
            body=self.body(side+'_wall',[0,y,0],yaw,layer)
            # A roller door is a physically present visual insert in a real opening.
            for x,half in [(-(W+5.2)/4,(W-5.2)/4),((W+5.2)/4,(W-5.2)/4)]:
                self.box(body,[x,0,H/2],[half,.09,H/2],'wall',layer)
                self.box(body,[x,-.105,.72],[half,.025,.72],'band',layer)
            self.box(body,[0,0,(H+4.8)/2],[2.6,.09,(H-4.8)/2],'wall',layer)
            door=self.body(side+'_door',[0,y,0],yaw,layer)
            self.box(door,[0,-.055,2.38],[2.5,.06,2.38],'steel_light',layer)
            rot=_rx(math.pi/2)
            ET.SubElement(door,'geom',name=self._name(),type='mesh',mesh='factory_shutter_face',pos='0 -.124 2.38',xyaxes=_vec([*rot[:,0],*rot[:,1]]),material='factory_shutter',group=str(LAYERS[layer]))
            for x in [-2.61,2.61]:self.box(door,[x,-.08,2.47],[.09,.12,2.47],'steel',layer)
            self.box(door,[0,-.08,4.92],[2.7,.14,.12],'steel',layer)
            for x in [-1.5,0,1.5]:
                self.box(door,[x,-.13,2.8],[.43,.018,.19],'black',layer,bevel=.025)
                self.box(door,[x,-.153,2.8],[.39,.008,.15],'glass',layer)
            self.sign(door,side+'_exit','EXIT',[3.5,-.17,2.75],[.65,.20],group=layer,bg=(14,104,50),fg=(235,240,230))
            self.box(door,[3.5,-.14,1.25],[.12,.07,.18],'cabinet_dark',layer,bevel=.012)
            self.cylinder(door,[3.5,-.23,1.35],.035,.02,'green',layer,rotation=_rx(math.pi/2))
            self.sign(body,side+'_left','TRANSPORT' if side=='north' else 'DIAL LAB',[-W*.27,-.12,4.6],[5.3,.72],group=layer,painted=True)
            self.sign(body,side+'_right','MOSAIC' if side=='north' else 'AIR RELAY',[W*.27,-.12,4.6],[5.3,.72],group=layer,painted=True)
            if side=='north':self.sign(body,'brand','ROBO FACTORY',[0,-.12,6.0],[8.0,1.0],group=layer,painted=True)
        # Side walls are assembled around glazed openings instead of textured solid boxes.
        for side,x,yaw,layer in [('east',W/2,-math.pi/2,'east'),('west',-W/2,math.pi/2,'west')]:
            body=self.body(side+'_wall',[x,0,0],yaw,layer)
            self.box(body,[0,0,.65],[D/2,.09,.65],'wall',layer)
            self.box(body,[0,-.105,.65],[D/2,.018,.65],'band',layer)
            self.box(body,[0,0,6.0],[D/2,.09,1.2],'wall',layer)
            n=round(D/6);step=D/n
            for i in range(n):
                cx=-D/2+(i+.5)*step;window_w=step-.5
                for xx in [cx-step/2,cx+step/2]:self.box(body,[xx,0,3.05],[.25,.09,1.75],'wall',layer)
                self.box(body,[cx,.005,3.05],[window_w/2,.018,1.73],'glass',layer)
                for xx in [cx-window_w/2,cx,cx+window_w/2]:self.box(body,[xx,-.055,3.05],[.035,.035,1.77],'steel_light',layer)
                for zz in [1.3,3.05,4.8]:self.box(body,[cx,-.055,zz],[window_w/2+.03,.035,.035],'steel_light',layer)
                self.box(body,[cx,-.12,1.28],[window_w/2+.05,.13,.035],'steel_light',layer)

    def shell(self):
        W,D,H=self.width,self.depth,self.height
        floor=self.body('floor')
        self.box(floor,[0,0,-.07],[W/2,D/2,.07],'floor')
        # Thin inset seams and sparse patches; no workstation-sized grid.
        spacing=self.cfg['appearance']['floor_joint_spacing_m']
        for x in np.arange(-W/2+spacing,W/2,spacing):self.box(floor,[x,0,.0008],[.004,D/2,.0008],'band')
        for y in np.arange(-D/2+spacing,D/2,spacing):self.box(floor,[0,y,.0008],[W/2,.004,.0008],'band')
        self.wall_geometry()
        # Real perimeter steel columns, flanges and protective feet.
        for side,x in [('west',-W/2+.25),('east',W/2-.25)]:
            n=round(D/6)
            for i,y in enumerate(np.linspace(-D/2+.2,D/2-.2,n+1)):
                b=self.body(f'{side}_column_{i}',[x,y,0],category='structure')
                self.box(b,[0,0,H/2],[.075,.14,H/2],'steel')
                for xx in [-.12,.12]:self.box(b,[xx,0,H/2],[.025,.18,H/2],'steel')
                self.box(b,[0,0,.035],[.25,.25,.035],'steel_light',bevel=.015)
                self.box(b,[0,0,.5],[.18,.2,.45],'yellow',bevel=.025)
                for z in [.3,.65]:self.box(b,[0,-.205,z],[.18,.012,.045],'black')
        roof=self.body('roof',group='roof');rise=self.cfg['building']['roof_rise_m'];slope=math.atan2(rise,W/2)
        for side in [-1,1]:
            r=np.array([[math.cos(slope),0,side*math.sin(slope)],[0,1,0],[-side*math.sin(slope),0,math.cos(slope)]])
            self.box(roof,[side*W/4,0,H+rise/2],[math.hypot(W/2,rise)/2,D/2+.15,.055],'roof','roof',rotation=r)
        for y in np.linspace(-D/2+.25,D/2-.25,round(D/6)+1):
            self.line(roof,[-W/2,y,H],[0,y,H+rise],.07,'steel','roof');self.line(roof,[0,y,H+rise],[W/2,y,H],.07,'steel','roof')
            for z in [H-.28,H-.48]:self.box(roof,[0,y,z],[W/2,.11,.025],'steel','roof')
            self.box(roof,[0,y,H-.38],[W/2,.025,.1],'steel','roof')
            nodes=np.linspace(-W/2,W/2,13)
            for j,(a,b) in enumerate(zip(nodes,nodes[1:])):
                topx=b if j%2==0 else a;bottomx=a if j%2==0 else b
                self.line(roof,[bottomx,y,H-.38],[topx,y,H+rise*(1-abs(topx)/(W/2))],.027,'steel','roof')
        for x in np.arange(-W/2+2,W/2,3.5):
            z=H+rise*(1-abs(x)/(W/2))-.09;self.box(roof,[x,0,z],[.045,D/2,.065],'steel_light','roof')

    def infrastructure(self):
        W,D,H=self.width,self.depth,self.height
        b=self.body('lighting_and_trays',group='roof',category='infrastructure')
        for x in np.linspace(-W/2+5,W/2-5,7):
            for y in np.linspace(-D/2+4,D/2-4,6):
                z=H-.95
                self.box(b,[x,y,z],[1.15,.13,.055],'steel_light','roof',bevel=.015)
                self.box(b,[x,y,z-.06],[1.08,.10,.008],'lamp','roof')
                for dx in [-.8,.8]:self.line(b,[x+dx,y,z+.05],[x+dx,y,H+.05],.006,'steel','roof')
        for x in [-W/2+1.2,W/2-1.2,-3.2,3.2]:
            z=H-.65
            for dx in [-.18,.18]:self.box(b,[x+dx,0,z],[.025,D/2-.4,.055],'steel_light','roof')
            for y in np.arange(-D/2+.5,D/2,.6):self.box(b,[x,y,z-.045],[.18,.025,.015],'steel_light','roof')
            for k in range(3):self.line(b,[x-.1+k*.08,-D/2+.5,z+.015],[x-.1+k*.08,D/2-.5,z+.015],.012,'black' if k!=1 else 'yellow','roof')
        # Pipe headers run along the actual perimeter, with bends and descent supports.
        for side in [-1,1]:
            x=side*(W/2-.32)
            for zz,mat in [(H-1.05,'steel_light'),(H-1.32,'black')]:
                self.line(b,[x,-D/2+.4,zz],[x,D/2-.4,zz],.035,mat,'roof')
            for y in np.arange(-D/2+2,D/2,6):self.line(b,[x,y,H-1.45],[x,y,H-.3],.009,'steel','roof')

    def paint(self):
        b=self.body('area_markings',category='markings');half=self.cfg['appearance']['region_line_width_m']/2
        for region in self.cfg['regions']:
            x0,y0,x1,y1=region['bounds_xy_m']
            for y in [y0,y1]:self.box(b,[(x0+x1)/2,y,.002],[abs(x1-x0)/2,half,.001],'yellow')
            for x in [x0,x1]:self.box(b,[x,(y0+y1)/2,.002],[half,abs(y1-y0)/2,.001],'yellow')
        # Main circulation marks only: no task-table rectangles.
        for x in [-.85,.85]:
            for y in np.arange(-self.depth/2+2,self.depth/2-2,3.5):self.box(b,[x,y,.002],[half,.55,.001],'yellow')

    def cabinet(self,name,pos,yaw=0,kind='electrical'):
        b=self.body(name,pos,yaw,category='perimeter_equipment')
        w,d,h=(1.1,.55,2.0) if kind=='electrical' else (.92,.50,.95)
        self.box(b,[0,0,h/2],[w/2,d/2,h/2-.035],'cabinet',bevel=.025)
        self.box(b,[0,0,.05],[w/2-.045,d/2-.025,.05],'black')
        if kind=='electrical':
            for xx in [-w/4,w/4]:
                self.box(b,[xx,-d/2-.008,h/2],[w/4-.012,.014,h/2-.08],'cabinet',bevel=.008)
                self.box(b,[xx+.15,-d/2-.032,1.03],[.012,.018,.09],'black',bevel=.006)
            self.box(b,[0,-d/2-.035,1.62],[.13,.017,.09],'cabinet_dark',bevel=.008)
            self.box(b,[0,-d/2-.055,1.62],[.1,.004,.06],'blue')
            for j in range(6):self.box(b,[.33,-d/2-.026,.22+j*.027],[.14,.006,.004],'black')
            self.sign(b,name+'_label','RF / POWER',[0,-d/2-.029,1.87],[.38,.065],bg=(160,168,168))
        else:
            for j in range(5):
                z=.18+j*.14;self.box(b,[0,-d/2-.015,z],[w/2-.035,.018,.06],'cabinet_dark',bevel=.007)
                self.box(b,[0,-d/2-.04,z+.025],[w*.33,.015,.009],'steel_light')
        R=_rz(yaw);half=np.abs(R[:2,:2])@np.array([w/2,d/2]);self.props.append(dict(id='factory_'+name,bounds_xy_m=[pos[0]-half[0],pos[1]-half[1],pos[0]+half[0],pos[1]+half[1]]))

    def bin(self,parent,pos,w=.32,d=.35,h=.22):
        x,y,z=pos
        self.box(parent,[x,y,z+.012],[w/2,d/2,.012],'blue',bevel=.008)
        for xx in [-w/2+.012,w/2-.012]:self.box(parent,[x+xx,y,z+h/2],[.012,d/2,h/2],'blue',bevel=.006)
        self.box(parent,[x,y+d/2-.012,z+h/2],[w/2,.012,h/2],'blue')
        self.box(parent,[x,y-d/2+.012,z+h*.28],[w/2,.012,h*.28],'blue',bevel=.006)
        self.box(parent,[x,y-d/2-.002,z+.035],[w*.2,.003,.018],'label')

    def rack(self,name,pos,yaw=0):
        b=self.body(name,pos,yaw,category='perimeter_equipment');w,d,h=2.0,.65,2.25
        for x in [-w/2,w/2]:
            for y in [-d/2,d/2]:
                self.box(b,[x,y,h/2],[.025,.025,h/2],'steel_light')
                self.box(b,[x,y,.015],[.055,.055,.015],'steel')
        for z in [.17,.7,1.23,1.76]:
            self.box(b,[0,0,z],[w/2,d/2,.025],'steel')
            for i,x in enumerate([-.72,-.25,.25,.72]):
                if i==0 and z<.3:
                    self.box(b,[x,0,z+.2],[.20,.24,.18],'cardboard',bevel=.008)
                    self.box(b,[x,0,z+.385],[.025,.24,.003],'tape')
                else:self.bin(b,[x,-.03,z+.03],w=.38,d=.47,h=.27)
        for xx in [-w/2,w/2]:self.box(b,[xx,0,2.23],[.025,d/2,.025],'steel_light')
        for x in [-.45,.40]:
            self.box(b,[x,.04,2.0],[.26,.22,.20],'cardboard',bevel=.008)
            self.box(b,[x,.04,2.204],[.025,.22,.003],'tape')
        self.line(b,[-w/2,d/2,.15],[w/2,d/2,h],.012,'steel')
        self.line(b,[w/2,d/2,.15],[-w/2,d/2,h],.012,'steel')
        R=_rz(yaw);half=np.abs(R[:2,:2])@np.array([w/2+.06,d/2+.06]);self.props.append(dict(id='factory_'+name,bounds_xy_m=[pos[0]-half[0],pos[1]-half[1],pos[0]+half[0],pos[1]+half[1]]))

    def plant(self,name,pos):
        b=self.body(name,pos,category='perimeter_equipment');self.cylinder(b,[0,0,.22],.22,.22,'pot')
        for k in range(10):
            a=k*2.399;tip=np.array([.36*math.cos(a),.36*math.sin(a),1.05+.35*math.sin(k*1.1)])
            self.line(b,[0,0,.4],tip,.008,'green')
            for q in [.5,.75,1.0]:
                p=tip*q;direction=np.array([math.cos(a),math.sin(a),.3]);end=p+direction*.20
                self.line(b,p,end,.035,'leaf_light' if k%3==0 else 'green')
        self.props.append(dict(id='factory_'+name,bounds_xy_m=[pos[0]-.6,pos[1]-.6,pos[0]+.6,pos[1]+.6]))

    def extinguisher(self,name,pos,yaw):
        b=self.body(name,pos,yaw,category='perimeter_equipment')
        self.box(b,[0,.03,.50],[.16,.035,.38],'red',bevel=.015)
        self.cylinder(b,[0,-.10,.46],.075,.23,'red')
        self.box(b,[0,-.10,.72],[.055,.04,.025],'black',bevel=.01)
        self.line(b,[.07,-.1,.72],[.12,-.1,.35],.012,'black')
        self.box(b,[0,-.178,.47],[.043,.004,.08],'label')

    def equipment(self):
        W,D=self.width,self.depth
        if not self.cfg['appearance']['include_perimeter_storage']:return
        for side in [-1,1]:
            # Front of local furniture is -Y. Rotate to face the room from each side.
            yaw=-side*math.pi/2;x=side*(W/2-.58)
            for i,y in enumerate(np.linspace(-D/2+3.5,D/2-3.5,6)):
                if i%3==0:
                    self.cabinet(f'side_{side}_electrical_{i}',[x,y,0],yaw)
                    self.extinguisher(f'side_{side}_fire_{i}',[side*(W/2-.25),y+1.1,0],yaw)
                elif i%3==1:self.rack(f'side_{side}_rack_{i}',[x,y,0],yaw)
                else:self.cabinet(f'side_{side}_drawer_{i}',[x,y,0],yaw,'drawer')
            if self.cfg['appearance']['include_plants']:
                for y in [-D/2+1.3,D/2-1.3]:self.plant(f'plant_{side}_{int(y)}',[side*(W/2-.7),y,0])
        for side in [-1,1]:
            y=side*(D/2-.6);yaw=0 if side==1 else math.pi
            for i,x in enumerate([-17,-13,-8,8,13,17]):
                if abs(x)>W/2-2:continue
                if i%2:self.cabinet(f'end_{side}_cabinet_{i}',[x,y,0],yaw)
                else:self.rack(f'end_{side}_rack_{i}',[x,y,0],yaw)
            for x in [-4.3,4.3]:self.extinguisher(f'end_{side}_fire_{x}',[x,side*(D/2-.18),0],yaw)
        # Conduit drops join the perimeter headers to selected cabinets.
        b=self.body('electrical_conduits',category='infrastructure')
        for side in [-1,1]:
            for y in [-D/2+3.5,D/2-3.5]:
                x=side*(W/2-.2);self.line(b,[x,y,2.0],[x,y,self.height-1.05],.018,'steel_light')
                for z in [2.3,3.6,4.9]:self.box(b,[x,y,z],[.04,.035,.015],'steel')

    def cart(self,name,pos,yaw=0):
        b=self.body(name,pos,yaw,category='perimeter_equipment')
        for z in [.25,.85]:
            self.box(b,[0,0,z],[.58,.33,.025],'steel_light',bevel=.015)
            for y in [-.32,.32]:self.box(b,[0,y,z+.045],[.58,.016,.045],'cabinet_dark')
        for x in [-.54,.54]:
            for y in [-.28,.28]:
                self.box(b,[x,y,.54],[.018,.018,.43],'steel_light')
                self.cylinder(b,[x,y,.10],.09,.025,'black',rotation=_rx(math.pi/2))
        for z in [.28,.88]:
            for x in [-.28,.25]:self.bin(b,[x,-.02,z],w=.42,d=.48,h=.22)
        self.line(b,[.6,-.3,.85],[.6,-.3,1.08],.018,'steel_light')
        self.line(b,[.6,.3,.85],[.6,.3,1.08],.018,'steel_light')
        self.line(b,[.6,-.3,1.08],[.6,.3,1.08],.018,'steel_light')

    def tool_station(self,name,pos,yaw=0):
        """A compact maintenance drawer unit and real framed shadow board."""
        self.cabinet(name+'_base',pos,yaw,'drawer')
        b=self.body(name+'_board',pos,yaw,category='perimeter_equipment')
        for x in [-.46,.46]:self.box(b,[x,.18,1.42],[.025,.025,.5],'steel_light')
        self.box(b,[0,.18,1.58],[.55,.035,.43],'steel_light',bevel=.015)
        self.panel(b,'tools',[0,.141,1.58],[1.05,.79])
        # A few tangible handles stand proud of the cheap printed silhouette detail.
        for x in [-.32,-.16,.02,.20]:
            self.box(b,[x,.105,1.50],[.017,.018,.10],'blue',bevel=.008)
            self.line(b,[x,.105,1.60],[x,.105,1.72],.009,'steel_light')

    def hose_reel(self,name,pos,yaw=0):
        b=self.body(name,pos,yaw,category='perimeter_equipment')
        self.box(b,[0,.08,.62],[.04,.06,.56],'steel_light')
        self.box(b,[0,.06,.035],[.22,.21,.035],'steel_light',bevel=.015)
        for y in [-.04,-.16]:self.cylinder(b,[0,y,1.08],.21,.018,'blue',rotation=_rx(math.pi/2))
        self.cylinder(b,[0,-.10,1.08],.17,.055,'black',rotation=_rx(math.pi/2))
        self.cylinder(b,[0,-.185,1.08],.045,.018,'steel_light',rotation=_rx(math.pi/2))
        for a,c in [([.16,-.1,1.02],[.24,-.1,.65]),([.24,-.1,.65],[.18,-.1,.30]),([.18,-.1,.30],[-.08,-.1,.22])]:self.line(b,a,c,.014,'black')

    def detail_equipment(self):
        W,D,H=self.width,self.depth,self.height
        # Fill perimeter gaps only: leave the future task regions and aisle untouched.
        for side in [-1,1]:
            yaw=-side*math.pi/2;x=side*(W/2-.70)
            for i,y in enumerate([-14,-7,0,7,14]):
                if i in [0,4]:self.rack(f'detail_{side}_rack_{i}',[x,y,0],yaw)
                elif i in [1,3]:self.tool_station(f'detail_{side}_tools_{i}',[x,y,0],yaw)
                else:self.cart(f'detail_{side}_cart',[x,y,0],yaw)
            for i,y in enumerate([-5.4,5.4]):self.hose_reel(f'detail_{side}_reel_{i}',[side*(W/2-.48),y,0],yaw)
            # Structural IDs and real conduit junctions, not floating billboard labels.
            for i,y in enumerate(np.linspace(-D/2+.2,D/2-.2,round(D/6)+1)):
                b=self.body(f'column_plate_{side}_{i}',[side*(W/2-.09),y,0],yaw,category='infrastructure')
                self.sign(b,f'bay_{side}_{i}',f'{"A" if side<0 else "B"}{i+1:02}',[0,-.34,3.25],[.46,.54])
                self.box(b,[0,-.23,1.9],[.14,.08,.18],'cabinet',bevel=.012)
                self.line(b,[0,-.23,2.08],[0,-.23,H-1.2],.018,'steel_light')
                self.panel(b,'safety',[0,-.318,1.9],[.23,.30])
        for side in [-1,1]:
            y=side*(D/2-.72);yaw=0 if side==1 else math.pi
            for x in [-20.5,20.5]:self.rack(f'detail_end_{side}_rack_{x}',[x,y,0],yaw)
            for x in [-10.5,10.5]:self.tool_station(f'detail_end_{side}_tools_{x}',[x,y,0],yaw)
            for x in [-5.5,5.5]:self.cart(f'detail_end_{side}_cart_{x}',[x,y,0],yaw)
            b=self.body(f'end_wall_detail_{side}',[0,side*D/2,0],yaw,'north' if side==1 else 'south',category='infrastructure')
            layer='north' if side==1 else 'south'
            for x in [-18,-12,-6,6,12,18]:
                self.box(b,[x,-.10,3.7],[.007,.005,3.45],'cabinet_dark',layer)
            for x,kind in [(-13,'process'),(13,'safety')]:
                self.box(b,[x,-.14,3.05],[1.4,.045,.66],'steel_light',layer,bevel=.02)
                self.panel(b,kind,[x,-.189,3.05],[2.72,1.24],layer)
            for x in [-7.0,7.0]:
                self.box(b,[x,-.18,2.6],[.22,.08,.3],'cabinet',layer,bevel=.015)
                for dx in [-.09,.09]:self.line(b,[x+dx,-.17,2.9],[x+dx,-.17,H-1.1],.017,'steel_light',layer)
                self.panel(b,'safety',[x,-.267,2.6],[.34,.45],layer)
        # Ventilation above the windows; joints, outlets and hangers give functional scale.
        b=self.body('ventilation',group='roof',category='infrastructure')
        for side in [-1,1]:
            x=side*(W/2-1.0);z=H-1.6
            self.box(b,[x,0,z],[.28,D/2-.6,.24],'steel_light','roof',bevel=.025)
            for y in np.arange(-D/2+1.5,D/2-1,3):
                self.box(b,[x,y,z],[.30,.025,.26],'cabinet_dark','roof')
                self.line(b,[x-.32,y,z-.27],[x-.32,y,H-.1],.009,'steel','roof')
                self.line(b,[x+.32,y,z-.27],[x+.32,y,H-.1],.009,'steel','roof')
            for y in np.arange(-D/2+4,D/2-2,6):
                self.box(b,[x-side*.29,y,z],[.012,.42,.15],'black','roof')
                for dy in np.linspace(-.38,.38,12):self.box(b,[x-side*.31,y+dy,z],[.018,.012,.135],'steel_light','roof')
        # Cross-hall utility lines with sparse clips; no additional real lights/shadow maps.
        for y in [-D/2+1.5,D/2-1.5]:
            self.line(b,[-W/2+.5,y,H-1.15],[W/2-.5,y,H-1.15],.035,'steel_light','roof')
            self.line(b,[-W/2+.5,y,H-1.38],[W/2-.5,y,H-1.38],.023,'red','roof')
            for x in np.arange(-W/2+2,W/2,4):self.line(b,[x,y,H-1.46],[x,y,H-.15],.008,'steel','roof')

    def workstation_dressing(self):
        """Factory-only furniture fitted to real solid tabletop footprints."""
        self.workstation_props=[]
        settings=self.cfg.get('workstation_dressing',{})
        if not settings.get('enabled'):return
        import mujoco
        from .factory_workstations import placements
        from .workstation import Workstation
        catalog_path=ROOT/settings['catalog'];catalog=json.loads(catalog_path.read_text())
        self.dressing_catalog_sha256=_hash(catalog_path)
        plan=placements(catalog,self.cfg['regions'],ROOT)
        for place in plan:
            source=Workstation(ROOT/place['source_directory']);source.frame();m,d=source.model,source.data
            task=json.loads((source.path/'task.json').read_text());height=task['table_height_m']
            if height<.70:continue
            table_ids=[i for i in range(m.ngeom) if m.geom(i).name.startswith('table_') and 'leg' not in m.geom(i).name]
            original=m.geom_group.copy();m.geom_group[:]=5;m.geom_group[table_ids]=0
            mask=np.array([1,0,0,0,0,0],np.uint8)
            def solid(x,y):
                hit=mujoco.mj_ray(m,d,np.array([x,y,height+.05]),np.array([0.,0.,-1.]),mask,True,-1,None)
                return 0<=hit<.09
            legs=np.array([d.geom_xpos[i,:2] for i in range(m.ngeom) if m.geom(i).name.startswith('table_leg_')])
            env=json.loads((source.path/'environment.json').read_text());chosen=[]
            for table in env['tables']:
                cx,cy,_=table['center_m'];hx,hy,_=table['half_size_m']
                for x in np.arange(cx-hx+.65,cx+hx-.60,1.35):
                    for y in np.arange(cy-hy+.50,cy+hy-.45,1.15):
                        if len(chosen)>=min(6,max(2,task['scale']['arms']//2)):break
                        if any(np.linalg.norm(np.array([x,y])-q)<1.15 for q in chosen):continue
                        if len(legs) and np.min(np.linalg.norm(legs-np.array([x,y]),axis=1))<.70:continue
                        if not all(solid(x+dx,y+dy) for dx in (-.46,.46) for dy in (-.28,.28)):continue
                        chosen.append(np.array([x,y]))
            m.geom_group[:]=original
            for index,p in enumerate(chosen):
                position=[p[0]+place['translation_m'][0],p[1]+place['translation_m'][1],0.]
                name='underbench_'+place['case_id']+'_'+str(index)
                b=self.body(name,position,category='workstation_storage')
                # Furniture has real thickness, feet, pulls, seams and ventilation.
                if index%3!=2:
                    self.box(b,[0,0,.34],[.43,.25,.29],'cabinet',bevel=.014)
                    for x in (-.37,.37):
                        for y in (-.19,.19):self.cylinder(b,[x,y,.035],.025,.035,'black')
                    for j,z in enumerate((.18,.36,.54)):
                        self.box(b,[0,-.257,z],[.41,.009,.078],'cabinet_dark' if j==0 else 'cabinet',bevel=.006)
                        self.line(b,[-.13,-.279,z+.025],[.13,-.279,z+.025],.009,'steel')
                    for z in np.arange(.16,.52,.055):self.box(b,[.433,.03,z],[.0015,.15,.005],'black')
                else:
                    for z in (.08,.37,.65):self.box(b,[0,0,z],[.45,.27,.018],'steel_light',bevel=.006)
                    for x in (-.43,.43):
                        for y in (-.25,.25):self.box(b,[x,y,.34],[.018,.018,.32],'steel')
                    for x in (-.22,.22):self.bin(b,[x,-.01,.11],w=.34,d=.43,h=.20)
                    self.box(b,[.04,.03,.51],[.31,.20,.10],'cardboard',bevel=.008)
                    self.box(b,[.04,.03,.612],[.025,.20,.001],'tape')
                self.workstation_props.append(dict(id=name,case_id=place['case_id'],position_m=position,
                    footprint_half_m=[.46,.28],top_m=.67,physical=False,placement='Below verified solid tabletop; not a swept-motion certificate'))
        # Region-corner staging islands stay outside all planned robot/load bays.
        for region in self.cfg['regions']:
            x0,y0,x1,y1=region['bounds_xy_m'];occupied=[p['planning_bounds_xy_m'] for p in plan if p['family']==region['id']]
            for index,(x,y) in enumerate(((x0+.7,y0+.65),(x1-.7,y1-.65))):
                if any(x+.45>a and x-.45<c and y+.38>b and y-.38<d for a,b,c,d in occupied):continue
                name='staging_'+region['id']+'_'+str(index);b=self.body(name,[x,y,0],category='floor_storage')
                for yy in (-.27,0,.27):self.box(b,[0,yy,.055],[.42,.045,.045],'cardboard',bevel=.008)
                for xx in (-.34,-.17,0,.17,.34):self.box(b,[xx,0,.115],[.045,.33,.016],'cardboard',bevel=.003)
                for j,(xx,yy,hh) in enumerate(((-.20,-.12,.20),(.20,.08,.26),(-.12,.10,.18))):
                    zz=.13+hh/2+(0 if j<2 else .23)
                    self.box(b,[xx,yy,zz],[.17,.14,hh/2],'cardboard',bevel=.006)
                    self.box(b,[xx,yy,zz+hh/2+.001],[.025,.14,.001],'tape')
                self.workstation_props.append(dict(id=name,position_m=[x,y,0],footprint_half_m=[.45,.38],top_m=.57,physical=False,placement='Region corner outside initial planning envelopes'))

    def lights(self):
        cfg=self.cfg['lighting']
        ET.SubElement(self.world,'light',name='factory_environment',type='image',texture='factory_sky',intensity=str(cfg['environment_intensity']))
        ET.SubElement(self.world,'light',name='factory_daylight',type='directional',pos='-18 -10 16',dir='.6 .32 -1',intensity=str(cfg['sun_intensity']),diffuse='1 .95 .86',castshadow='true')
        positions=cfg.get('fill_positions_m',[[-12,-10,self.height-.8],[12,-10,self.height-.8],[-12,10,self.height-.8],[12,10,self.height-.8]])
        for i,pos in enumerate(positions):
            ET.SubElement(self.world,'light',name=f'factory_ceiling_fill_{i}',type='point',pos=_vec(pos),intensity=str(cfg['ceiling_cluster_intensity']),diffuse='.94 .97 1',bulbradius='.8',castshadow='false')

    def camera(self,name,position,target,fovy=55,hidden=(),orthographic=False,height=None):
        z=np.asarray(position,float)-target;z/=np.linalg.norm(z);x=np.cross([0,0,1],z)
        if np.linalg.norm(x)<1e-8:x=np.array([1.,0,0])
        x/=np.linalg.norm(x);y=np.cross(z,x)
        attrs=dict(name='factory_camera_'+name,pos=_vec(position),xyaxes=_vec([*x,*y]),fovy=str(height if orthographic else fovy))
        if orthographic:attrs['projection']='orthographic'
        ET.SubElement(self.world,'camera',**attrs)
        self.cameras.append(dict(id=name,model_camera=attrs['name'],position_m=list(position),target_m=list(target),vertical_fov_deg=None if orthographic else fovy,projection='orthographic' if orthographic else 'perspective',orthographic_height_m=height if orthographic else None,hidden_layers=list(hidden)))

    def views(self):
        W,D,H=self.width,self.depth,self.height
        self.camera('interior_south',[0,-D/2+1.5,2.2],[0,D/4,2.7],68)
        self.camera('interior_north',[0,D/2-1.5,2.2],[0,-D/4,2.7],68)
        self.camera('interior_diagonal',[-W/2+2,-D/2+2,3.6],[W/5,D/5,2.4],62)
        self.camera('interior_east',[W/2-1.5,-4,2.0],[-W/3,4,2.6],65)
        self.camera('detail_storage',[-W/2+5,-D/2+7.5,1.65],[-W/2+.6,-D/2+10.5,1.2],52)
        self.camera('detail_brand',[0,D/2-7,5.4],[0,D/2,5.9],65)
        self.camera('detail_ceiling',[0,-5,2.0],[4,7,H-.8],62)
        self.camera('top',[0,0,H+25],[0,0,0],hidden=['roof','north','south','west','east'],orthographic=True,height=D+3)
        for name,p,h in [('from_south',[0,-D*1.25,D*.80],['roof','south']),('from_north',[0,D*1.25,D*.80],['roof','north']),('from_east',[W*1.25,0,W*.80],['roof','east']),('from_west',[-W*1.25,0,W*.80],['roof','west'])]:
            self.camera(name,p,[0,0,1.3],43,hidden=h)
        self.camera('interactive',[0,-D/2+1.5,2.2],[0,D/4,2.7],68)

    def finish(self):
        self.shell();self.infrastructure();self.paint();self.equipment()
        if self.cfg['appearance'].get('perimeter_detail',False):self.detail_equipment()
        self.workstation_dressing()
        self.lights();self.views()
        if self.cfg['appearance'].get('batch_static_geometry',False):self.static_batches()
        path=self.output/'scene.xml';ET.indent(self.root);ET.ElementTree(self.root).write(path,encoding='unicode')
        used_mesh_files={Path(m.get('file')).name for m in self.asset.findall('mesh') if m.get('file')}
        for file in (self.assets_path/'meshes').glob('factory_*.obj'):
            if file.name not in used_mesh_files:file.unlink()
        asset_files=[]
        for file in sorted(self.assets_path.rglob('*')):
            if file.is_file() and file.name!='asset-manifest.json':asset_files.append(dict(path=str(file.relative_to(self.assets_path)),sha256=_hash(file),bytes=file.stat().st_size))
        manifest=dict(schema_version=1,id=self.cfg['id'],units='metres',frame='+X east, +Y north, +Z up',scope='Static visual factory; no task workstations or motion playback',dimensions_m=[self.width,self.depth,self.height],layers=LAYERS,regions=self.cfg['regions'],objects=self.objects,perimeter_props=self.props,cameras=self.cameras,assets=asset_files,mesh_definition_triangles=self.mesh_triangles,source='Project-authored geometry and procedural finishes; Poly Haven CC0 concrete maps; OFL Barlow Condensed font',scene_sha256=_hash(path),physics=dict(dynamic_joints=0,actuators=0,collision_filter='All display geoms use contype=0, conaffinity=0; ground for future physics must be integrated explicitly.'))
        manifest['static_batching']=self.batching
        manifest['workstation_dressing']=getattr(self,'workstation_props',[])
        if hasattr(self,'dressing_catalog_sha256'):manifest['dressing_catalog_sha256']=self.dressing_catalog_sha256
        manifest['composition_contract']={'default_class':'factory_visual','source_sections':['asset','worldbody','named default factory_visual'],'preserve_target_sections':['compiler','option','actuator','equality','tendon','task cameras'],'frame_transform':'Rigid translation and yaw of the factory relative to the canonical task, or of a recorded workstation in a presentation; no scale change.','warning':'This standalone shell is not a task replacement. Do not copy its global compiler/options over a running task.'}
        (self.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        return path,manifest


def build_factory(config_path=DEFAULT_CONFIG, output=None):
    """Return a generated scene and manifest; no task imports or simulation steps."""
    config_path=Path(config_path).resolve();config=json.loads(config_path.read_text())
    default_folder={'robo-factory-v4':'v4-shell','robo-factory-v5':'v5-shell'}.get(config['id'],'scene')
    output=Path(output or ROOT/'outputs/robo-factory'/default_folder).resolve();output.mkdir(parents=True,exist_ok=True)
    b=config['building'];W,D=b['width_m'],b['depth_m']
    if min(W,D,b['eave_height_m'])<=0:raise ValueError('Building dimensions must be positive')
    for r in config['regions']:
        x0,y0,x1,y1=r['bounds_xy_m']
        if not (-W/2<x0<x1<W/2 and -D/2<y0<y1<D/2):raise ValueError('Region outside building: '+r['id'])
    assets=ROOT/'assets/robo_factory';(assets/'meshes').mkdir(parents=True,exist_ok=True);(assets/'textures').mkdir(exist_ok=True)
    _texture_assets(assets/'textures',config['seed'])
    if config['appearance'].get('whole_floor_finish',False):_floor_finish(assets/'textures',config)
    scene,manifest=_Builder(config,assets,output).finish()
    # Local, relative assets make the resulting scene directory portable.
    (assets/'asset-manifest.json').write_text(json.dumps({'generated_by':'robofrontier.factory_scene','seed':config['seed'],'third_party':[{'id':'Barlow Condensed','source':'https://github.com/google/fonts/tree/main/ofl/barlowcondensed','license':'SIL Open Font License 1.1','license_file':'fonts/OFL.txt'},{'id':'concrete_floor_02','source':'https://polyhaven.com/a/concrete_floor_02','license':'CC0 1.0','metadata':'source/polyhaven/source.json'}],'assets':manifest['assets']},indent=2)+'\n')
    shutil.copytree(assets,output/'assets',dirs_exist_ok=True)
    # Output folders may be reused; do not retain obsolete generated mesh copies.
    active_meshes={file.name for file in (assets/'meshes').glob('factory_*.obj')}
    for file in (output/'assets/meshes').glob('factory_*.obj'):
        if file.name not in active_meshes:file.unlink()
    shutil.copy2(config_path,output/'build-config.json')
    return scene,manifest
