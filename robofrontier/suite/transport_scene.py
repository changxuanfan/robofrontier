"""Transport revisions: unrestricted staging and visible orders, without new control.

Clone the immutable r10 workstation. Robot/object poses, socket dimensions, joints,
materials and acceptance remain unchanged. Board outlines grow, with declared
mass/inertia changes; colored order cues are non-colliding, massless paint.
"""
from copy import deepcopy
import json
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from . import v2
from .scenes import box, prism_mesh
from ..run_setup import write_json

BOARD_HALF = (.055*1.3, .16*1.3)
CUE_SPAN = .060*1.4
BOARD_OUTWARD_SHIFT = .05
PAPER = (.94,.94,.90,1)
INK = (.14,.20,.25,1)

RULES = [
    'Deliver the three requested source-identity items to each destination board. Station IDs identify inventory and destination orders, not exclusive robot ownership.',
    'Transfers may be direct airborne handovers or safe stable tabletop staging. There are no marked or mandatory intermediate regions, predefined buffer positions, prescribed routes or fixed transfer pairs. The planner chooses whether to wait, stage or hand over. Establish receiving support before releasing an airborne item.',
    'Each final position shows the required color and shape: a square for a cube and a circle for a cylinder, 1.4 times the object width or diameter. These are visual identity cues, not enlarged acceptance tolerances. Slot numbers and source identities disambiguate orders.',
    'L1: release stably at the final board targets. L2 and L3: both flat blades must enter the real blind slots and the item must be released, seated and stable. L2 has a fixed board; L3 has the same three inline sockets on a passive spring-supported z/roll/pitch board. Preserve prior placements and allow gripper opening and withdrawal clearance.',
    'Cubes have 60 mm sides; cylinders have 60 mm diameter and height. Blades initially point up or sideways. Grasp, airborne reorientation, receiving grasp and subsequent insertion must form a feasible continuous sequence. Allocation, routes and independent concurrent work are planner choices; minimize unnecessary cycling, staging and regrasping while maintaining real support and collision clearance.',
]


def _clip(poly,axis,bound,greater):
    result=[]
    for a,b in zip(poly,poly[1:]+poly[:1]):
        inside_a=(a[axis]>=bound) if greater else (a[axis]<=bound)
        inside_b=(b[axis]>=bound) if greater else (b[axis]<=bound)
        if inside_a:result.append(a)
        if inside_a!=inside_b:
            t=(bound-a[axis])/(b[axis]-a[axis])
            result.append([a[i]+t*(b[i]-a[i]) for i in range(2)])
    return result


def _paint(root,board, name, shape, color, y,holes):
    """Flat visible silhouette; its outer 12 mm remains visible after seating."""
    for suffix,half,z,rgb in [('outline',CUE_SPAN/2,.00010,INK),('color',CUE_SPAN/2-.0008,.00021,color)]:
        regions=[(-half,half,-half,half)] if not holes else [
            (-half,half,-half,-.0075),(-half,half,.0075,half),
            (-half,-.01425,-.0075,.0075),(-.00975,.00975,-.0075,.0075),(.01425,half,-.0075,.0075)]
        for i,(xlo,xhi,ylo,yhi) in enumerate(regions):
            part=name+'_'+suffix+str(i)
            if shape=='cube':
                box(board,part,[(xlo+xhi)/2,y+(ylo+yhi)/2,z],[(xhi-xlo)/2,(yhi-ylo)/2,.00004],rgb,visual=True)
            else:
                poly=[[half*math.cos(t),half*math.sin(t)] for t in np.linspace(0,2*math.pi,65)[:-1]]
                for axis,bound,greater in [(0,xlo,True),(0,xhi,False),(1,ylo,True),(1,yhi,False)]:
                    poly=_clip(poly,axis,bound,greater)
                if len(poly)<3:continue
                prism_mesh(root,part,poly,z-.00004,z+.00004)
                ET.SubElement(board,'geom',name=part,type='mesh',mesh=part,pos=v2.vector([0,y,0]),rgba=v2.vector(rgb),mass='0',contype='0',conaffinity='0')


def revise(root,env):
    world=root.find('worldbody');level=env['suite_case']['level']
    pitch=.12 if env['suite_case']['scene_revision']>=12 else .10
    board_half=(BOARD_HALF[0],BOARD_HALF[1]-.010) if env['suite_case']['scene_revision']>=13 else BOARD_HALF
    for body in list(world.findall('body')):
        if body.get('name','').startswith('transfer_'):world.remove(body)
    env['edges']=[]
    objects={o['identity']:o for o in env['objects']}
    for station in env['stations']:
        name='board_'+station['id'];board=world.find(f"body[@name='{name}']")
        # The enlarged perpendicular corner boards overlap if kept at their
        # old centers. Move the whole board and its support toward its robot;
        # Socket dimensions, supply poses and robot mounting remain intact.
        shift=v2.rz(station['yaw_rad'])@np.array([-BOARD_OUTWARD_SHIFT,0,0])
        board.set('pos',v2.vector(np.fromstring(board.get('pos'),sep=' ')+shift))
        support=world.find(f"geom[@name='{name}_spring_base']")
        if support is not None:
            support.set('pos',v2.vector(np.fromstring(support.get('pos'),sep=' ')+shift))
        for goal in env['goals']:
            if goal['board']==name:
                goal['local_position_m'][1]*=pitch/.10
                goal['position_m']=(np.fromstring(board.get('pos'),sep=' ')+v2.rz(station['yaw_rad'])@np.asarray(goal['local_position_m'])).tolist()
        for geom in list(board.findall('geom')):
            if geom.get('name','').startswith((name+'_bottom',name+'_band',name+'_plate')):
                board.remove(geom)
        hx,hy=board_half
        if level>=2:
            box(board,name+'_bottom',[0,0,-.029],[hx,hy,.003],PAPER,density='650')
            bounds=[-hy,-pitch/2,pitch/2,hy]
            for k,y in enumerate((-pitch,0,pitch)):
                lo,hi=bounds[k:k+2]
                box(board,name+f'_band{k}_below',[0,(lo+y-.0075)/2,-.013],[hx,(y-.0075-lo)/2,.013],PAPER,density='650')
                box(board,name+f'_band{k}_above',[0,(y+.0075+hi)/2,-.013],[hx,(hi-y-.0075)/2,.013],PAPER,density='650')
                for j,(a,b) in enumerate([(-hx,-.01425),(-.00975,.00975),(.01425,hx)]):
                    box(board,name+f'_band{k}_land{j}',[(a+b)/2,y,-.013],[(b-a)/2,.0075,.013],PAPER,density='650')
        else:
            box(board,name+'_plate',[0,0,-.0015],[hx,hy,.0015],PAPER,density='650')
        for goal in [g for g in env['goals'] if g['board']==name]:
            gid=goal['goal_id'];y=goal['local_position_m'][1]
            for geom in list(board.findall('geom')):
                if geom.get('name','').startswith((gid+'_markx',gid+'_marky')):
                    board.remove(geom)
            label=board.find(f"geom[@name='{gid}_label']")
            label.set('pos',v2.vector([-.060,y,.0002]))
            obj=objects[goal['requires']['identity']]
            color=(*obj['rgb'],1)
            _paint(root,board,gid+'_cue',obj['shape'],color,y,level>=2)
            goal['target_cue']=dict(color=obj['color'],shape=obj['shape'],span_m=CUE_SPAN,
                physical_role='Massless non-colliding surface paint; existing acceptance unchanged')
    env['mechanism'].update(board_size_m=[2*board_half[0],2*board_half[1]],
        board_outline_scale_from_r10=[board_half[0]/.055,board_half[1]/.16],board_material_density_kg_m3=650.,
        board_outward_shift_from_r10_m=BOARD_OUTWARD_SHIFT,
        target_cue_span_m=CUE_SPAN,target_cue_scale=1.4,
        target_center_pitch_m=pitch,target_cue_gap_m=pitch-CUE_SPAN,
        transfer=dict(marked_intermediate_regions=False,predefined_buffers=False,
            choice='Planner chooses direct airborne handover, safe tabletop staging or waiting; no fixed routes/pairs'),
        revision_scope='Board outline/order paint and 50 mm outward board/support translation. R12 increases target/socket pitch by20mm fromR11; R13 shortens the board by20mm symmetrically. Socket dimensions, springs, robots, supply poses and acceptance preserved; real board mass/inertia recomputed at unchanged density.')
    env['transport_task_rules']=deepcopy(RULES)


def build(case,replace_draft=False,render=True):
    from .reframe import build as clone
    folder=clone(case,replace_draft,render,transform=revise)
    env=json.loads((folder/'environment.json').read_text())
    model=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
    source_case=dict(case,scene_revision=10);source=v2.directory(source_case)
    old=mujoco.MjModel.from_xml_path(str(source/'scene.xml'))
    boards={}
    for station in env['stations']:
        name='board_'+station['id'];a=old.body(name).id;b=model.body(name).id
        mass=float(model.body_mass[b]);previous=float(old.body_mass[a])
        boards[name]=dict(previous_mass_kg=previous,mass_kg=mass,
            previous_principal_inertia_kg_m2=old.body_inertia[a].tolist(),
            principal_inertia_kg_m2=model.body_inertia[b].tolist(),
            com_body_m=model.body_ipos[b].tolist(),
            unloaded_linear_sag_estimate_m=mass*9.81/1400 if case['level']==3 else None,
            three_cube_sag_upper_estimate_m=(mass+3*(v2.CUBE_MASS+2*.0025*.012*.024*7800))*9.81/1400 if case['level']==3 else None)
    write_json(folder/'board-physics.json',dict(scope='Compiled mass/inertia plus static F/k estimates; no dynamics or manipulation test',
        boards=boards,spring_changed=False,contact_material_changed=False,physics_timestep_s=float(model.opt.timestep),
        estimate_limits='Sag ignores tilt, contacts and loading transients; it is not an insertion/reachability guarantee.'))
    write_json(folder/'validation.json',dict(json.loads((folder/'validation.json').read_text()),
        scope='Cloned immutable r10 plus larger boards/order paint; static compilation only. No Oracle, pickup, insertion or reachability qualification.',
        oracle_adapter_status='Existing staged reference not qualified for the revised unrestricted transport; deferred until unit checks and replanning'))
    return folder
