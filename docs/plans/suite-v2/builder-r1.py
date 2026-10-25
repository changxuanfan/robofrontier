"""Independent v2 workstation assets; no policy, Oracle or factory composition.

All sizes are SI. Generated cases are self-contained, immutable inputs to later
experiments. Static checks are explicitly separate from contact task success.
"""
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.optimize import linear_sum_assignment

from .scenes import (ROOT, FOLDERS, environment, station, rectangle, box, cylinder,
                     rz, xyz, vector, finish_base, texture, prism_mesh, annular_table,
                     build_dial)
from ..cube_transport import build_xml, initialize, add_initial_keyframe, balanced_destinations, SMALL_ORDERS
from ..robot_control import matrix_quat
from ..run_setup import write_json

CATALOG=ROOT/'configs/suites/frontier-v2.json'
PALETTE={'red':(.85,.12,.12),'blue':(.06,.34,.77),'yellow':(.98,.74,.10),
         'white':(.95,.95,.91),'cyan':(.08,.66,.68),'purple':(.49,.27,.69)}
COLORS=list(PALETTE)
SIDE=.100
DENSITY=.05/.04**3
CUBE_MASS=DENSITY*SIDE**3
TABLE=.4


def cases():
    result=[]
    for family,scales in [('transport',[4,8,12]),('mosaic',[4,6,8]),('relay',[8,12,18]),('dial',[4,6,8])]:
        for k,n in enumerate(scales):
            for level in (1,2,3):
                c=dict(family=family,level=level,suite_version=2,scene_revision=1,scale_index=k)
                if family=='mosaic':
                    rows,cols,color_count=[(6,12,4),(8,18,5),(6,24,6)][k]
                    c.update(arms=n,rows=rows,columns=cols,colors=color_count,id=f'mosaic-a{n:02d}-{rows}x{cols}-d{level}')
                elif family=='dial':
                    c.update(arms=n if level==1 else 2*n,dials=n,id=f'dial-n{n:02d}-d{level}',ring_radius_m=1.55)
                else:c.update(arms=n,id=f'{family}-a{n:02d}-d{level}')
                c['directory']=str(directory(c).relative_to(ROOT));result.append(c)
    return result


def directory(case):return ROOT/'configs/environments'/FOLDERS[case['family']]/('frontier-v2-'+case['id'])


def env_base(case,stations,tables):
    env=environment(case,stations,tables);env['schema']='robofrontier.workstation.v2'
    env['suite_case']=deepcopy(case);env['physics'].update(cube_size_m=SIDE,object_mass_kg=CUBE_MASS,
        solid_cube_density_kg_m3=DENSITY,material_scope='Declared engineering assumptions, not material measurements',
        contact_time_constant_s=.004,contact_damping_ratio=1.)
    env['gripper_profile']='offset-aluminium-40-120mm' if case['family'] in ('transport','mosaic') else 'standard-panda'
    return env


def font(size):
    bundled=ROOT/'assets/robo_factory/fonts/BarlowCondensed-SemiBold.ttf'
    return ImageFont.truetype(str(bundled),size) if bundled.exists() else ImageFont.load_default(size=size)


def label(root,folder,name,text,position,half=(.055,.035),yaw=0):
    image=Image.new('RGB',(512,256),(239,243,243));ImageDraw.Draw(image).text((256,128),text,anchor='mm',font=font(170),fill=(18,35,49))
    image.save(folder/'assets'/(name+'.png'));asset=root.find('asset')
    ET.SubElement(asset,'texture',name=name,type='2d',file='assets/'+name+'.png')
    ET.SubElement(asset,'material',name=name,texture=name,specular='0')
    box(root.find('worldbody'),name,position,[*half,.00015],visual=True,material=name,quat=vector(matrix_quat(rz(yaw))))


def outline(root,name,center,half,yaw=0,color=(.14,.34,.51,1)):
    body=ET.SubElement(root.find('worldbody'),'body',name=name,pos=vector(center),quat=vector(matrix_quat(rz(yaw))))
    for i,sign in enumerate((-1,1)):
        box(body,name+'_x'+str(i),[sign*half[0],0,0],[.001,half[1],.00012],color,visual=True)
        box(body,name+'_y'+str(i),[0,sign*half[1],0],[half[0],.001,.00012],color,visual=True)


def base_xml(env,folder):
    root=build_xml(env,folder);finish_base(root,env)
    # Source station labels are paint, never props. Do not let visual geoms
    # contribute inferred dynamic mass on any apparatus.
    for g in root.findall('.//geom'):
        if g.get('contype')=='0':g.set('mass','0')
        if (g.get('name') or '').endswith('_label') and g.get('name','')[:-6] in {s['id'] for s in env['stations']}:
            s=next(s for s in env['stations'] if s['id']==g.get('name')[:-6])
            g.set('pos',vector(xyz(s,-.14,0,.4006)))
    if env['gripper_profile'].startswith('offset-'):
        custom=root.find('custom')
        if custom is None:custom=ET.SubElement(root,'custom')
        for s in env['stations']:
            name=s['id'];ET.SubElement(custom,'numeric',name=name+'_gripper_gap_offset',data='.04')
            for suffix in ('left_finger','right_finger'):
                body=root.find(f".//body[@name='{name}_{suffix}']")
                for e in list(body):
                    if e.tag in ('geom','inertial'):body.remove(e)
                # Three non-overlapping manufactured solids. The original
                # slide, mounting origin and 54 mm finger reach are retained.
                box(body,name+'_'+suffix+'_mount',[0,.0175,.006],[.009,.0175,.006],(.85,.87,.87,1),density='2700')
                box(body,name+'_'+suffix+'_stem',[0,.030,.033],[.009,.005,.021],(.85,.87,.87,1),density='2700')
                box(body,name+'_'+suffix+'_pad',[0,.0225,.0445],[.009,.0025,.009],(.19,.22,.24,1),density='1100',friction='1 .005 .0001',condim='4')
    return root


def add_cube(root,obj,position,quat=None,attachments=None):
    body=ET.SubElement(root.find('worldbody'),'body',name=obj['id'],pos=vector(position),quat=vector(quat if quat is not None else [1,0,0,0]))
    ET.SubElement(body,'freejoint',name=obj['id']+'_free')
    box(body,obj['id']+'_geom',[0,0,0],[SIDE/2]*3,[*PALETTE[obj['color']],1],mass=CUBE_MASS,
        friction='1 .005 .0001',condim='4',solref='.004 1',solimp='.95 .99 .001 .5 2')
    if attachments=='blades':
        for i,x in enumerate((-.018,.018)):
            box(body,obj['id']+'_blade'+str(i),[x,0,-.060],[.0015,.006,.010],(.39,.47,.53,1),density='7800',friction='.45 .003 .0001',condim='4')
    if attachments=='bar':
        box(body,obj['id']+'_bottom_bar',[0,0,-.053],[.068,.05,.003],(.59,.37,.23,1),density=str(DENSITY),friction='.4 .003 .0001',condim='4')
    return body


def rocket(rows,cols,color_count):
    """Mirror-symmetric artwork with constrained, almost equal color quotas."""
    palette=COLORS[:color_count];half=rows//2;items=[];preferred=[]
    # One unbroken background color preserves a recognizable silhouette.
    # The outlines deliberately occupy enough pixels for balanced foreground
    # colors, without scattering isolated colored pixels into the background.
    outline_rows={(6,12):[(3,7),(0,9),(0,11)],(8,18):[(4,11),(0,14),(0,16),(0,17)],(6,24):[(5,19),(0,20),(0,23)]}[(rows,cols)]
    background={4:'blue',5:'cyan',6:'purple'}[color_count]
    for r in range(half):
        for col in range(cols):
            x=col/(cols-1);distance=(rows/2-.5-r)/(rows/2)
            lo,hi=outline_rows[r];mask=lo<=col<=hi
            want=background
            if mask:
                want='white'
                if x<.25:want='yellow'
                if x>.80 or distance>.60:want='red'
                if .51<=x<=.71 and distance<.60:want='red' if color_count==4 else 'blue'
                if color_count>=6 and .27<=x<=.49:want='cyan'
            items.append((r,col,mask,x,distance));preferred.append(want)
    total=rows*cols
    quotas=[total//(2*color_count)]*color_count
    for i in range(total//2-sum(quotas)):quotas[i]+=1
    slots=[c for c,q in zip(palette,quotas) for _ in range(q)]
    cost=np.empty((len(items),len(slots)))
    centers={'red':(.62,.4),'blue':(.62,.15),'yellow':(.15,.5),'white':(.9,.8),'cyan':(.36,.8),'purple':(.34,.3)}
    for i,(_,col,mask,x,y) in enumerate(items):
        for j,c in enumerate(slots):
            value=0 if c==preferred[i] else 4
            if (not mask and c!=background) or (mask and c==background):value+=1000
            cx,cy=centers[c];value+=.8*abs(x-cx)+.4*abs(y-cy)
            cost[i,j]=value+1e-7*(i*j%31)
    ii,jj=linear_sum_assignment(cost);upper=[['']*cols for _ in range(half)]
    for i,j in zip(ii,jj):r,c,*_=items[i];upper[r][c]=slots[j]
    return upper+upper[::-1]


def write_pattern(pattern,folder):
    cell=80;rows=len(pattern);cols=len(pattern[0]);im=Image.new('RGB',(cols*cell,rows*cell));draw=ImageDraw.Draw(im)
    for r,line in enumerate(pattern):
        for c,color in enumerate(line):
            rgb=tuple(round(x*255) for x in PALETTE[color]);draw.rectangle((c*cell,r*cell,(c+1)*cell-1,(r+1)*cell-1),fill=rgb,outline=(124,139,149),width=1)
    im.save(folder/'target-pattern.png')


def mosaic(case,folder):
    n=case['arms'];rows=case['rows'];cols=case['columns'];pitch=.126;w=cols*pitch/2;h=rows*pitch/2
    stations=[]
    for side in (-1,1):
        for j in range(n//2):
            x=(j-(n//2-1)/2)*.756;y=side*(h+.30);yaw=math.pi/2 if side==-1 else -math.pi/2
            stations.append(station(chr(65+len(stations)),[x,y],yaw,.27,.92))
    env=env_base(case,stations,[rectangle(w+.40,h+.88)])
    env['objects']=[];root=base_xml(env,folder);world=root.find('worldbody')
    # Robot mesh bounds at the stock's actual height, rather than a circular
    # base-radius guess, exclude initial cube/base overlap.
    planning=folder/'_stock-layout.xml';ET.ElementTree(root).write(planning,encoding='unicode')
    pm=mujoco.MjModel.from_xml_path(str(planning));pd=initialize(pm,env);planning.unlink();blocked=[]
    from ..sim import RobotIK
    stock_iks={s['id']:RobotIK(pm,s['id']+'_') for s in stations}
    stock_rotations=[]
    for axis in [np.array([1.,0,0]),np.array([0,1.,0])]:
        for sign in [-1,1]:
            yy=axis*sign;zz=np.array([0.,0,-1.]);stock_rotations.append(np.column_stack((np.cross(yy,zz),yy,zz)))
    def stock_reachable(p):
        point=np.r_[p,.482]
        ordered=sorted(stations,key=lambda s:np.linalg.norm(np.asarray(p)-s['base_position_m'][:2]))[:3]
        for s in ordered:
            ik=stock_iks[s['id']]
            for rotation in stock_rotations:
                q=pd.qpos[ik.qids].copy()
                for target in [point+[0,0,.12],point]:
                    q,error=ik.solve(target,rotation,q,140)
                    if error>.0025:break
                else:return s['id']
        return None
    for g in range(pm.ngeom):
        if not pm.geom_bodyid[g] or not(pm.geom_contype[g] or pm.geom_conaffinity[g]):continue
        r=pd.geom_xmat[g].reshape(3,3);center=pd.geom_xpos[g]+r@pm.geom_aabb[g,:3];extent=abs(r)@pm.geom_aabb[g,3:]
        if center[2]+extent[2]>=.4 and center[2]-extent[2]<=.5:blocked.append((center[:2]-extent[:2]-.052,center[:2]+extent[:2]+.052))
    # Pack only outside the assembly region. Candidate stock is clear of the
    # mounting footprints and lies in a declared local reach envelope.
    candidates=[]
    for y in np.arange(-h-.78,h+.79,.12):
        for x in np.arange(-w-.30,w+.31,.12):
            if abs(x)<w+.065 and abs(y)<h+.065:continue
            ds=[np.linalg.norm(np.array([x,y])-s['base_position_m'][:2]) for s in stations]
            if min(ds)<.29 or min(ds)>.68:continue
            if any(np.all(np.array([x,y])>lo) and np.all(np.array([x,y])<hi) for lo,hi in blocked):continue
            candidates.append([float(x),float(y)])
    pattern=rocket(rows,cols,case['colors']);stock=Counter(c for line in pattern for c in line)
    # Balanced station counts, varied color runs. No per-color permanent arm.
    colors=[c for c in PALETTE for _ in range(stock[c])];objects=[];remaining=candidates[:]
    per=rows*cols//n
    for index,s in enumerate(stations):
        for k in range(per):
            if not remaining:raise ValueError('No tabletop inventory capacity')
            def rank(p):
                distance=np.linalg.norm(np.array(p)-s['base_position_m'][:2]);depth=(np.array(p)-s['base_position_m'][:2])@np.array([math.cos(s['yaw_rad']),math.sin(s['yaw_rad'])])
                return distance+(.15 if depth<-.10 else 0)
            while remaining:
                p=min(remaining,key=rank);remaining.remove(p)
                reachable=stock_reachable(p)
                if reachable:break
            else:raise ValueError('Insufficient stock with a downward approach IK')
            color=colors[index*per+k];obj=dict(id=f'{s["id"]}_stock{k+1:02d}',source=s['id'],color=color,rgb=list(PALETTE[color]),shape='cube',initial_position_m=[*p,.45])
            objects.append(obj);add_cube(root,obj,obj['initial_position_m'])
    env['objects']=objects;floor=TABLE-SIDE/3 if case['level']==1 else TABLE
    if case['level']==1:
        # Leave a genuine opening in the table, filled by lower pocket floors.
        table=world.find("geom[@name='table_0']");world.remove(table)
        hx,hy=env['tables'][0]['half_size_m'][:2]
        for name,cx,cy,sx,sy in [('south',0,-(h+hy)/2,hx,(hy-h)/2),('north',0,(h+hy)/2,hx,(hy-h)/2),('west',-(w+hx)/2,0,(hx-w)/2,h),('east',(w+hx)/2,0,(hx-w)/2,h)]:
            box(world,'table_frame_'+name,[cx,cy,.375],[sx,sy,.025],env['table_rgba'],mass=0,material='table_oak')
        box(world,'pocket_bottom',[0,0,floor-.006],[w+.0005,h+.0005,.006],(.93,.93,.89,1),mass=0)
        for i in range(cols+1):box(world,'pocket_column_'+str(i),[(i-cols/2)*pitch,0,(floor+TABLE)/2],[.0005,h+.0005,(TABLE-floor)/2],(.74,.81,.84,1),mass=0)
        for i in range(rows+1):box(world,'pocket_row_'+str(i),[0,(i-rows/2)*pitch,(floor+TABLE)/2],[w+.0005,.0005,(TABLE-floor)/2],(.74,.81,.84,1),mass=0)
    elif case['level']==2:
        for i in range(cols+1):box(world,'printed_column_'+str(i),[(i-cols/2)*pitch,0,.4001],[.0005,h,.00008],(.28,.39,.43,1),visual=True)
        for i in range(rows+1):box(world,'printed_row_'+str(i),[0,(i-rows/2)*pitch,.4001],[w,.0005,.00008],(.28,.39,.43,1),visual=True)
    goals=[]
    for r,line in enumerate(pattern):
        for c,color in enumerate(line):goals.append(dict(goal_id=f'r{r+1}c{c+1}',row=r+1,column=c+1,requires={'color':color},position_m=[(c-(cols-1)/2)*pitch,((rows-1)/2-r)*pitch,floor+.05]))
    env['goals']=goals;env['grid']=dict(rows=rows,columns=cols,colors=case['colors'],pitch_m=pitch,cube_side_m=SIDE,pattern=pattern,
        pocket_opening_m=.125 if case['level']==1 else None,depth_m=SIDE/3 if case['level']==1 else 0,floor_z_m=floor,rim_z_m=TABLE,color_counts=dict(stock))
    env['mechanism']=dict(kind=['recessed_grid','printed_grid','unmarked_table'][case['level']-1],stock_packing='Single layer; non-exclusive station allocation',stock_count=rows*cols)
    write_pattern(pattern,folder)
    for i in range(n//2):
        x=(i-(n//2-1)/2)*.756
        ET.SubElement(world,'camera',name=f'assembly_{i+1:02d}',pos=vector([x,0,2.05]),xyaxes='1 0 0 0 1 0',fovy='40')
    return root,env


def object_rest(rotation,attachments):
    parts=[(np.zeros(3),np.full(3,.05))]
    if attachments=='blades':parts += [(np.array([x,0,-.06]),np.array([.0015,.006,.010])) for x in (-.018,.018)]
    if attachments=='bar':parts += [(np.array([0,0,-.053]),np.array([.068,.05,.003]))]
    return TABLE-min(float(rotation[2]@p-np.abs(rotation[2])@half) for p,half in parts)


def transport(case,folder):
    from ..cube_transport import COUNTS
    n=case['arms'];level=case['level'];counts=COUNTS[n];hx=1.02+.42*(max(counts[0],counts[2])-1);hy=1.02+.42*(max(counts[1],counts[3])-1)
    stations=[]
    for side,k in enumerate(counts):
        for j in range(k):
            t=(j-(k-1)/2)*.84
            x,y,yaw=[(t,-hy,math.pi/2),(hx,t,math.pi),(-t,hy,-math.pi/2),(-hx,-t,0)][side]
            stations.append(station(chr(65+len(stations)),[x,y],yaw,.28,.94))
    ix,iy=max(.1,hx-.78),max(.1,hy-.78);ox,oy=hx+.22,hy+.22
    tables=[rectangle(ox,(oy-iy)/2,0,-(oy+iy)/2),rectangle(ox,(oy-iy)/2,0,(oy+iy)/2),
            rectangle((ox-ix)/2,iy,-(ox+ix)/2,0),rectangle((ox-ix)/2,iy,(ox+ix)/2,0)]
    env=env_base(case,stations,tables);root=base_xml(env,folder);world=root.find('worldbody');env['objects']=[];goals=[]
    if n==4:wanted=[list(row) for row in SMALL_ORDERS[n]]
    else:
        destinations,_=balanced_destinations(n,0);wanted=[[chr(65+i//3) for i,d in enumerate(destinations) if d==j] for j in range(n)]
    for s,row in zip(stations,wanted):
        board=add_transport_board(root,folder,s,level,goals,row)
        for k,lateral in enumerate((-.25,0,.25)):
            attachment='blades' if level==2 else 'bar' if level==3 else None
            if level==2:
                a=[np.diag([1.,-1.,-1.]),np.array([[0,0,1.],[0,1.,0],[-1.,0,0]]),np.array([[0,0,-1.],[0,1.,0],[1.,0,0]])][k]
            elif level==3:a=rz(k*math.pi/2)@np.diag([1.,-1.,-1.])
            else:a=np.eye(3)
            rotation=rz(s['yaw_rad'])@a;z=object_rest(rotation,attachment);p=xyz(s,.30 if k==1 else .20,lateral,z)
            color=COLORS[(ord(s['id'])-65)%6];obj=dict(id=s['id']+f'_item{k+1}',source=s['id'],identity=s['id'],color=color,rgb=list(PALETTE[color]),shape='cube',initial_position_m=p,initial_quaternion_wxyz=matrix_quat(rotation).tolist())
            body=add_cube(root,obj,p,obj['initial_quaternion_wxyz'],attachment);env['objects'].append(obj)
            mat=texture(root,folder,obj['id']+'_identity',s['id'],font_size=160)
            for face,(position,rotation_face) in enumerate([([0,0,.0502],np.eye(3)),([0,.0502,0],np.array([[1,0,0],[0,0,1],[0,-1,0]])),([0,-.0502,0],np.array([[1,0,0],[0,0,-1],[0,1,0]]))]):
                box(body,obj['id']+'_print'+str(face),position,[.018,.018,.0001],visual=True,material=mat,quat=vector(matrix_quat(rotation_face)))
    edges=[]
    for i,a in enumerate(stations):
        b=stations[(i+1)%n];p=(np.asarray(xyz(a,.20,0,.4002))+xyz(b,.20,0,.4002))/2
        edge=dict(id=a['id']+'_'+b['id'],stations=[a['id'],b['id']],position_m=[*p[:2],.45],capacity=1)
        edges.append(edge);outline(root,'transfer_'+edge['id'],p,[.07,.07],color=(.20,.47,.64,1))
    env['edges']=edges;env['goals']=goals
    env['mechanism']=dict(kind=['flat_placement','two_flat_blades','spring_board_dual_pockets'][level-1],cube_side_m=SIDE,
        cube_body_mass_kg=CUBE_MASS,cube_material='Solid wood; inherited nominal density',
        blade_size_m=[.003,.012,.020] if level==2 else None,blade_spacing_m=.036 if level==2 else None,
        slot_opening_m=[.0045,.014] if level==2 else None,slot_depth_m=.021 if level==2 else None,
        bottom_bar_size_m=[.136,.100,.006] if level==3 else None,
        latch=dict(left='Fixed five-sided sleeve',right='Passive spring-return five-sided slider',entry_face='towards the cube',
            mouth_spacing_m=.116,engagement_each_m=.010,pocket_depth_m=.012,inside_width_m=.102,
            inside_height_left_m=.011,inside_height_right_m=.008,slider_travel_m=.013,slider_k_n_m=100.,slider_preload_n=.4,slider_damping_ns_m=.7) if level==3 else None,
        spring_board=dict(translation_k_n_m=5000.,translation_damping_ns_m=150.,roll_pitch_k_nm_rad=3.0,roll_pitch_damping_nms_rad=.30,
            per_axis_limit_rad=.35,translation_limit_m=[-.004,.020],support='Equivalent central passive z/roll/pitch mechanism; cosmetic coil is not FEA') if level==3 else None,
        supplies='Tabletop; no pedestals. Plug blades initially up/sideways; latch bars initially up with varied yaw.')
    from ..cube_transport import _camera
    for s in stations:_camera(world,'board_'+s['id']+'_close',xyz(s,.95,-.48,1.12),np.array(xyz(s,.59 if level==3 else .53,0,.58 if level==3 else .44)),42)
    return root,env


def add_transport_board(root,folder,s,level,goals,orders):
    name='board_'+s['id'];world=root.find('worldbody');center=xyz(s,.59 if level==3 else .53,0,.58 if level==3 else .427 if level==2 else .403)
    board=ET.SubElement(world,'body',name=name,pos=vector(center),quat=vector(matrix_quat(rz(s['yaw_rad']))))
    if level==3:
        ET.SubElement(board,'joint',name=name+'_z',type='slide',axis='0 0 -1',stiffness='5000',damping='150',springref='0',range='-.004 .020',limited='true')
        for axis,label_axis in [('1 0 0','roll'),('0 1 0','pitch')]:ET.SubElement(board,'joint',name=name+'_'+label_axis,type='hinge',axis=axis,stiffness='3',damping='.30',range='-.35 .35',limited='true')
        box(board,name+'_plate',[0,0,-.006],[.21,.18,.006],(.94,.94,.90,1),density='650')
        local_goals=[[-.10,-.115],[-.10,.115],[.12,0]]
        # A visible spring base is part of this mechanism, not a stock pedestal.
        cylinder(world,name+'_spring_base',[*center[:2],.408],[.05,.008],(.90,.92,.91,1),mass=0)
        points=[[.025*math.cos(t),.025*math.sin(t),-.160+.143*t/(12*math.pi)] for t in np.linspace(0,12*math.pi,73)]
        for k,(a,b) in enumerate(zip(points,points[1:])):ET.SubElement(board,'geom',name=name+'_coil'+str(k),type='capsule',fromto=vector(a+b),size='.002',rgba='.34 .45 .50 1',mass='0',contype='0',conaffinity='0')
    elif level==2:
        local_goals=[[0,y] for y in (-.15,0,.15)]
        # True blind rectangular holes; no solid collision box fills the slots.
        box(board,name+'_bottom',[0,0,-.024],[.075,.23,.003],(.94,.94,.90,1),density='650')
        bounds=[-.23,-.075,.075,.23]
        for k,y in enumerate((-.15,0,.15)):
            lo,hi=bounds[k:k+2]
            box(board,name+f'_band{k}_below',[0,(lo+y-.007)/2,-.0105],[.075,(y-.007-lo)/2,.0105],(.94,.94,.90,1),density='650')
            box(board,name+f'_band{k}_above',[0,(y+.007+hi)/2,-.0105],[.075,(hi-y-.007)/2,.0105],(.94,.94,.90,1),density='650')
            intervals=[(-.075,-.02025),(-.01575,.01575),(.02025,.075)]
            for j,(a,b) in enumerate(intervals):box(board,name+f'_band{k}_land{j}',[(a+b)/2,y,-.0105],[(b-a)/2,.007,.0105],(.94,.94,.90,1),density='650')
    else:
        local_goals=[[0,y] for y in (-.15,0,.15)]
        box(board,name+'_plate',[0,0,-.0015],[.075,.23,.0015],(.94,.94,.90,1),density='650')
    for k,((x,y),identity) in enumerate(zip(local_goals,orders)):
        identity=identity.upper();gid=s['id']+f'_slot{k+1}'
        if level==3:add_latch(root,board,gid,x,y)
        else:
            for j,sign in enumerate((-1,1)):
                box(board,gid+'_markx'+str(j),[x+sign*.055,y,.00015],[.0005,.055,.0001],(.23,.40,.51,1),visual=True)
                box(board,gid+'_marky'+str(j),[x,y+sign*.055,.00015],[.055,.0005,.0001],(.23,.40,.51,1),visual=True)
        mat=texture(root,folder,gid+'_number',str(k+1),font_size=150)
        box(board,gid+'_label',[x-.063,y,.0002],[.008,.012,.0001],visual=True,material=mat)
        local=[x,y,.058 if level==3 else .05]
        pos=(np.asarray(center)+rz(s['yaw_rad'])@local).tolist()
        goals.append(dict(goal_id=gid,robot=s['id'],board=name,local_position_m=local,position_m=pos,requires={'identity':identity},latch_slider=gid+'_slider_joint' if level==3 else None))
    return board


def add_latch(root,board,name,x,y):
    material=(.91,.94,.93,1);left=ET.SubElement(board,'body',name=name+'_left',pos=vector([x,y,0]))
    def sleeve(body,tag,direction,height):
        # Opening at +/-58 mm, back at +/-70 mm, 10 mm nominal engagement.
        cx=direction*.065
        box(body,tag+'_floor',[cx,0,.001],[.007,.053,.001],material,density='1200',friction='.35 .003 .0001',condim='4')
        box(body,tag+'_roof',[cx,0,.003+height],[.007,.053,.001],material,density='1200',friction='.35 .003 .0001',condim='4')
        box(body,tag+'_back',[direction*.071,0,.002+height/2],[.001,.053,height/2],material,density='1200',friction='.35 .003 .0001')
        for j,side in enumerate((-1,1)):box(body,tag+'_side'+str(j),[cx,side*.052,.002+height/2],[.007,.001,height/2],material,density='1200',friction='.35 .003 .0001')
    sleeve(left,name+'_left',-1,.011)
    right=ET.SubElement(board,'body',name=name+'_right',pos=vector([x,y,0]))
    ET.SubElement(right,'joint',name=name+'_slider_joint',type='slide',axis='1 0 0',range='0 .013',limited='true',stiffness='100',springref='-.004',damping='.7',frictionloss='.05')
    sleeve(right,name+'_right',1,.008)
    # Outside loading ramp; the interior retaining underside remains flat.
    mesh=name+'_entry_ramp';prism_mesh(root,mesh,[[.058,.012],[.072,.012],[.072,.026]],-.053,.053)
    # prism_mesh extrudes its polygon along Z. Rotate its X/Y wedge into X/Z.
    ET.SubElement(right,'geom',name=mesh,type='mesh',mesh=mesh,quat=vector(matrix_quat(np.array([[1,0,0],[0,0,-1],[0,1,0]]))),rgba=vector(material),density='1200',friction='.35 .003 .0001',condim='4')
    box(right,name+'_release_tab',[.070,.069,.012],[.007,.010,.006],(.08,.54,.59,1),density='1200',friction='.6 .003 .0001')


def compact_dial_layout(count,level):
    if level==1:
        if count==4:positions=[(-.35,.29),(.35,.29),(-.35,-.29),(.35,-.29)]
        elif count==6:positions=[(x,y) for y in (.52,0,-.52) for x in (-.40,.40)]
        else:positions=[(-.65,.55),(0,.55),(.65,.55),(-.65,0),(.65,0),(-.65,-.55),(0,-.55),(.65,-.55)]
        stations=[]
        for i,p in enumerate(positions):
            outward=np.array([0,1 if p[1]>0 else -1]) if abs(p[1])>.01 else np.array([1 if p[0]>0 else -1,0])
            base=np.asarray(p)+outward*.50;stations.append(station(chr(65+i),base,math.atan2(-outward[1],-outward[0]),.24,.88))
        return stations,[(np.array(p),0.) for p in positions]
    if count==4:modules=[(np.array([(i-1.5)*.7,0]),0.) for i in range(4)]
    elif count==6:modules=[(np.array([x,y]),0.) for y in (-.92,.92) for x in (-.7,0,.7)]
    else:modules=[(1.55*np.array([math.cos(t),math.sin(t)]),t-math.pi/2) for t in np.arange(8)*math.pi/4]
    stations=[]
    for p,angle in modules:
        for xy,yaw in [([-.25,-.65],math.pi/2),([0,.65],-math.pi/2)]:
            base=p+(rz(angle)@np.r_[xy,0])[:2];stations.append(station(chr(65+len(stations)),base,angle+yaw,.23,.86))
    return stations,modules


def dial(case,folder):
    legacy_case=deepcopy(case);legacy_case['scene_revision']=4
    root,env=build_dial(legacy_case,folder,layout_fn=compact_dial_layout)
    env['suite_case']=deepcopy(case);env['schema']='robofrontier.workstation.v2';env['gripper_profile']='standard-panda'
    world=root.find('worldbody')
    for body in list(world.findall('body')):
        if (body.get('name') or '').endswith('_parking'):world.remove(body)
    if case['level']>1:
        # White rim and blue flat handle tip form the visible tabletop rest.
        theta=math.radians(66.5);rx=np.array([[1,0,0],[0,math.cos(theta),-math.sin(theta)],[0,math.sin(theta),math.cos(theta)]])
        for entry in env['dials']:
            body=world.find(f"body[@name='{entry['id']}_module']");rotation=rz(entry['layout_yaw_rad'])@rx
            body.set('quat',vector(matrix_quat(rotation)));p=np.asarray(entry['initial_center_m']);p[2]=.513
            body.set('pos',vector(p));entry['initial_center_m']=p.tolist();entry['initial_quaternion_wxyz']=matrix_quat(rotation).tolist()
            for obj in env['objects']:
                if obj['id']==entry['id']+'_module':obj['initial_position_m']=p.tolist();obj['initial_quaternion_wxyz']=entry['initial_quaternion_wxyz']
        env['mechanism']['parking_fixture']='None; visible disk rim and blue handle tip rest directly on tabletop'
    env['physics']['material_scope']='Original dial assembly properties retained; initial rest reconfigured without parking blocks'
    return root,env


def rotation_to_z(direction):
    z=np.asarray(direction,dtype=float);z/=np.linalg.norm(z)
    trial=np.array([1.,0,0]) if abs(z[0])<.85 else np.array([0,1.,0])
    x=np.cross(trial,z);x/=np.linalg.norm(x);return np.column_stack((x,np.cross(z,x),z))


def slack_points(a,b,length,segments=8):
    """A chain of equal-length links, with an initially upward slack arch."""
    a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float);delta=b-a;distance=np.linalg.norm(delta)
    if not .005<distance<length:raise ValueError('Slack cable endpoints violate length')
    u=delta/distance;v=np.array([0.,0,1.]);v-=u*(u@v);v/=np.linalg.norm(v)
    lo,hi=1e-9,math.pi/segments-1e-9
    for _ in range(70):
        beta=(lo+hi)/2;ratio=math.sin(segments*beta)/(segments*math.sin(beta))
        if ratio>distance/length:lo=beta
        else:hi=beta
    beta=(lo+hi)/2;radius=(length/segments)/(2*math.sin(beta));angles=np.linspace(-segments*beta,segments*beta,segments+1)
    points=(a+b)/2+radius*np.sin(angles)[:,None]*u+radius*(np.cos(angles)-math.cos(segments*beta))[:,None]*v
    if np.max(abs(np.linalg.norm(np.diff(points,axis=0),axis=1)-length/segments))>1e-9:raise ValueError('Cable construction length mismatch')
    return points


def add_cable(root,parent,parent_p,parent_r,a,b,target_site,name,length=.20):
    points=slack_points(a,b,length);previous_p=np.asarray(parent_p);previous_r=np.asarray(parent_r)
    for i,(start,end) in enumerate(zip(points,points[1:])):
        r=rotation_to_z(end-start)
        body=ET.SubElement(parent,'body',name=name+f'_link{i}',pos=vector(previous_r.T@(start-previous_p)),quat=vector(matrix_quat(previous_r.T@r)))
        ET.SubElement(body,'joint',name=name+f'_joint{i}',type='ball',damping='.00015',armature='.000001')
        ET.SubElement(body,'geom',name=name+f'_geom{i}',type='capsule',fromto=vector([0,0,0,0,0,length/8]),size='.002',mass='.001',rgba='.22 .31 .38 1',contype='4',conaffinity='1',friction='.6 .003 .0001')
        previous_p=start;previous_r=r;parent=body
    site=name+'_end';ET.SubElement(parent,'site',name=site,pos=vector([0,0,length/8]),size='.002',rgba='0 0 0 0')
    equality=root.find('equality');ET.SubElement(equality,'connect',name=name+'_attachment',site1=site,site2=target_site,solref='.004 1',solimp='.95 .99 .001 .5 2')
    return parent.get('name')


def curved_beam(root,body,name):
    radius=1.2/math.sqrt(3);angles=np.linspace(-math.pi/3,math.pi/3,33);arc_length=radius*2*math.pi/3
    total=600*.045**2*arc_length
    vertices=[]
    for angle in angles:
        for rr,z in [(radius-.0225,-.0225),(radius+.0225,-.0225),(radius+.0225,.0225),(radius-.0225,.0225)]:
            vertices.append([rr*math.cos(angle)-radius/2,rr*math.sin(angle),z])
    faces=[]
    for i in range(32):
        for j in range(4):
            a=4*i+j;b=4*i+(j+1)%4;c=b+4;d=a+4;faces.extend([(a,b,c),(a,c,d)])
    faces.extend([(0,2,1),(0,3,2),(128,129,130),(128,130,131)])
    faces=[(a,c,b) for a,b,c in faces]
    ET.SubElement(root.find('asset'),'mesh',name=name+'_visual',vertex=vector(np.array(vertices).ravel()),face=' '.join(str(x) for f in faces for x in f))
    ET.SubElement(body,'geom',name=name+'_surface',type='mesh',mesh=name+'_visual',rgba='.84 .10 .08 1',mass='0',contype='0',conaffinity='0')
    for i,(a,b) in enumerate(zip(angles,angles[1:])):
        t=(a+b)/2;p=[radius*math.cos(t)-radius/2,radius*math.sin(t),0]
        box(body,name+'_part'+str(i),p,[.0225,(radius+.0225)*math.tan((b-a)/2),.0225],(.84,.10,.08,0),mass=total/32,
            quat=vector(matrix_quat(rz(t))),friction='1 .005 .0001',condim='4',group='3')
    return dict(chord_m=1.2,circular_angle_deg=120,radius_m=radius,arc_length_m=arc_length,mass_kg=total,cross_section_m=[.045,.045],collision='32 tangent convex segments; smooth visual sweep; analytic arc mass distributed over segments')


def relay(case,folder):
    n=case['arms'];level=case['level'];stations=[];centers=[];yaws=[]
    if n==8:
        centers=[np.array([(i-1.5)*.72,0]) for i in range(4)];yaws=[0.]*4
        for p in centers:
            for sign in (-1,1):stations.append(station(chr(65+len(stations)),p+[0,sign*.65],-sign*math.pi/2,.27,.98))
        tables=[rectangle(1.40,.92)];circle=None
    elif n==12:
        radius=2.6;angles=np.linspace(0,math.pi/2,6);raw=np.c_[radius*np.sin(angles),radius*(1-np.cos(angles))];offset=raw.mean(0)
        centers=[p-offset for p in raw];yaws=angles.tolist()
        for p,yaw in zip(centers,yaws):
            for sign in (-1,1):stations.append(station(chr(65+len(stations)),p+(rz(yaw)@np.array([0,sign*.65,0]))[:2],yaw-sign*math.pi/2,.27,.98))
        tables=[rectangle(2.2,2.2)];circle=dict(center=[-float(offset[0]),radius-float(offset[1])],radius=radius,start=-math.pi/2-.16,end=.16,half_width=.92)
    else:
        for radius,count,outside in [(2.5,12,True),(1.2,6,False)]:
            for t in np.linspace(math.pi,0,count):
                p=radius*np.array([math.cos(t),math.sin(t)]);yaw=t+math.pi if outside else t
                stations.append(station(chr(65+len(stations)),p,yaw,.27,.98))
        angles=np.linspace(math.pi,0,12);centers=[1.85*np.array([math.cos(t),math.sin(t)]) for t in angles];yaws=[t-math.pi/2 for t in angles]
        tables=[rectangle(2.78,1.53,0,1.25)];circle=dict(center=[0,0],radius=1.85,start=-.025,end=math.pi+.025,half_width=.90)
    env=env_base(case,stations,tables);env['gripper_profile']='standard-panda';root=base_xml(env,folder);world=root.find('worldbody')
    if circle:
        for g in list(world.findall('geom')):
            if (g.get('name') or '').startswith(('table_0','table_leg_')):world.remove(g)
        annular_table(root,circle['center'],circle['radius'],circle['half_width'],circle['start'],circle['end'])
    routes=[dict(region_id=f'zone_{i+1:02d}',center_m=[*p,.4],nominal_yaw_rad=float(yaw)) for i,(p,yaw) in enumerate(zip(centers,yaws))]
    for i,(p,yaw) in enumerate(zip(centers,yaws)):
        outline(root,f'route_zone_{i+1:02d}',[*p,.4002],[.24,.65],yaw,color=(.14,.42,.58,1))
        label(root,folder,f'route_label_{i+1:02d}',f'{i+1:02d}',[*p,.4006],half=(.04,.025),yaw=yaw)
        if i+1<len(centers):
            direction=centers[i+1]-p;mid=(centers[i+1]+p)/2;angle=math.atan2(direction[1],direction[0])
            arrow=ET.SubElement(world,'body',name=f'route_arrow_{i}',pos=vector([*mid,.4005]),quat=vector(matrix_quat(rz(angle))))
            box(arrow,f'route_arrow_shaft{i}',[-.025,0,0],[.045,.007,.0001],(.17,.41,.57,1),visual=True)
            mesh=f'route_arrow_tip{i}';prism_mesh(root,mesh,[[.02,-.026],[.065,0],[.02,.026]],-.0001,.0001)
            ET.SubElement(arrow,'geom',name=mesh,type='mesh',mesh=mesh,rgba='.17 .41 .57 1',contype='0',conaffinity='0',mass='0')
    rotation=rz(yaws[0]);position=np.r_[centers[0]+(rotation@np.array([.24,0,0]))[:2],.4225]
    body=ET.SubElement(world,'body',name='payload',pos=vector(position),quat=vector(matrix_quat(rotation)));ET.SubElement(body,'freejoint',name='payload_free')
    if level==1:
        box(body,'payload_beam',[0,0,0],[.0225,.6,.0225],(.84,.10,.08,1),mass=1.458,friction='1 .005 .0001',condim='4')
        mechanism=dict(chord_m=1.2,mass_kg=1.458,cross_section_m=[.045,.045],kind='straight_beam')
    else:mechanism=curved_beam(root,body,'payload_beam');mechanism['kind']='arc_cable_weight' if level==2 else 'arc_three_cable_tray'
    if level>1:
        radius=1.2/math.sqrt(3);t=math.radians(54)
        # A side attachment stays above the tabletop and outside the wood.
        # An underside anchor would be buried against the table at pickup.
        anchor_local=np.array([radius*math.cos(t)-radius/2,radius*math.sin(t),-.016])-np.array([math.cos(t),math.sin(t),0])*.026
        anchor=position+rotation@anchor_local;center=anchor+rotation@np.array([-.13,0,0]);center[2]=.401
        if level==2:
            center[2]=.425;target=ET.SubElement(world,'body',name='weight',pos=vector(center));ET.SubElement(target,'freejoint',name='weight_free')
            cylinder(target,'weight_geom',[0,0,0],[.025,.025],(.30,.39,.45,1),mass=.25,friction='.6 .003 .0001',condim='4')
            ET.SubElement(target,'site',name='weight_anchor',pos='0 0 .025',size='.002',rgba='0 0 0 0')
            tip=add_cable(root,body,position,rotation,anchor,center+[0,0,.025],'weight_anchor','cable')
            ET.SubElement(root.find('contact'),'exclude',body1=tip,body2='weight')
        else:
            tray=ET.SubElement(world,'body',name='tray',pos=vector(center));ET.SubElement(tray,'freejoint',name='tray_free')
            cylinder(tray,'tray_floor',[0,0,0],[.035,.001],(.91,.92,.85,1),density='1200',friction='.35 .003 .0001',condim='4')
            for i in range(24):
                a=2*math.pi*i/24;b=2*math.pi*(i+1)/24;poly=[[rr*math.cos(t),rr*math.sin(t)] for rr,t in [( .033,a),(.036,a),(.036,b),(.033,b)]]
                mesh=f'tray_rim_{i}';prism_mesh(root,mesh,poly,.001,.004)
                ET.SubElement(tray,'geom',name=mesh,type='mesh',mesh=mesh,rgba='.91 .92 .85 1',density='1200',friction='.35 .003 .0001',condim='4')
            for i,t in enumerate(np.arange(3)*2*math.pi/3):
                local=np.array([.034*math.cos(t),.034*math.sin(t),.004]);site='tray_anchor'+str(i)
                ET.SubElement(tray,'site',name=site,pos=vector(local),size='.002',rgba='0 0 0 0')
                tip=add_cable(root,body,position,rotation,anchor,center+local,site,'cable'+str(i))
                ET.SubElement(root.find('contact'),'exclude',body1=tip,body2='tray')
            weight=ET.SubElement(world,'body',name='weight',pos=vector(center+[0,0,.026]));ET.SubElement(weight,'freejoint',name='weight_free')
            cylinder(weight,'weight_geom',[0,0,0],[.025,.025],(.30,.39,.45,1),mass=.25,friction='.35 .003 .0001',condim='4')
            # Explicit material pair: same physical friction at floor and rim.
            for geom in ['tray_floor']+[f'tray_rim_{i}' for i in range(24)]:ET.SubElement(root.find('contact'),'pair',geom1='weight_geom',geom2=geom,friction='.25 .25 .003 .0001 .0001',condim='4')
            mechanism['tray']=dict(outer_diameter_m=.072,clear_diameter_m=.066,depth_m=.003,weight_diameter_m=.05,weight_mass_kg=.25,
                cable_attachments=3,attachment_angle_spacing_deg=120,free_weight=True,weight_tray_sliding_friction=.25)
        mechanism.update(cable_length_m=.20,cable_segments_each=8,cable_segment_mass_kg=.001,cable_radius_m=.002,
            cable_model='Ball-jointed rigid links with true end connections; equal-length initial slack arches',
            cable_self_collision=False,cable_environment_collision=True,initial_condition='All components table-supported; initially slack')
    env['objects']=[dict(id='payload',source=stations[0]['id'],shape='straight_beam' if level==1 else 'arc_beam',color='red',rgb=list(PALETTE['red']),initial_position_m=position.tolist())]
    env['route']=routes;env['support_groups']=None;env['pairs']=None
    env['mechanism']=dict(**mechanism,minimum_supporting_robots=2,flexible_teams=True,layout='straight' if n==8 else 'quarter_arc' if n==12 else 'semicircle',
        outer_arms=12 if n==18 else None,inner_arms=6 if n==18 else None,curve_radius_m=circle['radius'] if circle else None,
        finish='Tabletop destination region; no cradle; beam and suspended assembly must land stably')
    from ..cube_transport import _camera
    _camera(world,'payload_close',position+rotation@np.array([.9,-1.3,1.15]),position+np.array([0,0,.05]),48)
    return root,env


RULES={
 'transport':[
    'Deliver three source-identity items to each destination board; station IDs identify inventory, not exclusive manipulator ownership. Tabletop transfers are allowed.',
    'L1: release stably in marked white-board areas. L2: both flat blades must physically enter the rectangular blind slots, with the cube seated on the board.',
    'L3: the bottom crossbar must engage both opposing five-sided sleeves. Left sleeve is fixed; right sleeve is a passive spring-return slider with an accessible release tab. Board and latches respond to real load. Engagement, retention, stable release and correct identity are required; no snap or attachments.'
 ],
 'mosaic':[
    'Build the entire single right-pointing rocket reference, including background cells, one released cube per pixel.',
    'Cube side is 100 mm and common cell-center pitch is 126 mm. L1 has 125 mm clear square pockets, L2 printed lines, L3 no visible grid. The reference is north-up, centered on the declared assembly origin.',
    'Stock allocations do not restrict which robot can use an item. Preserve completed placements and account for finger release/withdrawal clearance.'
 ],
 'relay':[
    'Move the complete payload from the marked source through route regions to the destination. Regions indicate transport progress, not prescribed grasp points or fixed robot pairs.',
    'Airborne transport needs at least two real beam grasps. Establish receiving support before sender release. Teams may change dynamically; the 18-arm half-ring contains 12 outer and 6 inner robots.',
    'After pickup keep the whole transported assembly clear of table/floor until final landing. For L3 the free weight must remain in the tray. Rope and tray are passive; no stabilizing attachments or direct weight grasp.',
    'Drops are recorded. The last established sending team may recover; a receiver cannot retrieve a dropped object to bypass a handover. No reset or respawn.'
 ],
 'dial':[
    'Each private code digit is independently 1,2,3. Relative disk positions are zero, clockwise quarter-turn, half-turn and counterclockwise quarter-turn; -180 and +180 are the same half-turn symbol.',
    'All disks must stay within 4 degrees of a nonzero detent for 0.25 simulated seconds for a valid count. Only aggregate correct count is visible; no individual matches. Zero is invalid.',
    'Movable modules start naturally on the tabletop, without parking racks. Establish blue-handle support and full-module 20 mm clearance before red operation; sustain support and stop after loss.',
    'A crank contact point moves around its axle; rotation can require coordinated translation. Reach a valid count equal to the number of disks.'
 ]}


def task_for(case,env,folder):
    task=dict(kind='frontier_suite',suite_version=2,id=case['id'],family=case['family'],level=case['level'],scale=deepcopy(case),
        scene_asset=str(folder.relative_to(ROOT)),scene_sha256=hashlib.sha256((folder/'scene.xml').read_bytes()).hexdigest(),
        arms={s['id']:{k:s[k] for k in ('base_position_m','yaw_rad')} for s in env['stations']},objects=deepcopy(env['objects']),
        table_height_m=TABLE,cube_size_m=SIDE if case['family'] in ('transport','mosaic') else None,physics=deepcopy(env['physics']),mechanism=deepcopy(env['mechanism']),
        goals=deepcopy(env.get('goals',[])),grid=deepcopy(env.get('grid')),route=deepcopy(env.get('route')),pairs=None,dials=deepcopy(env.get('dials')),
        gripper_profile=env['gripper_profile'],coordinate_frame='Workstation metres, +X east, +Y north, +Z up; radians; wxyz quaternions',
        rules=RULES[case['family']]+['Only robot actuators and real contacts may affect the live task. No hand welds, object state writes, resets, hidden state tools or runtime physics edits.'],
        objective='Complete the declared assembly/transport task correctly and efficiently; allocation, local motion and collaboration are Agent choices.',
        observation_contract=dict(initial_geometry_public=True,live_object_truth_for_policy=False,policy=['Selected timestamped RGB','Robot proprioception and robot geometry','Action feedback','Visible dial count']),
        completion=dict(stable_duration_sim_s=1.,scope='Mechanism-specific commissioning required; no inherited v1 evaluator assumed'),
        execution_readiness='environment_preview_only',oracle_verified=False,
        cameras=[c.get('name') for c in ET.parse(folder/'scene.xml').findall('worldbody/camera')],view_camera='global',recording_camera='global')
    xy=np.array([s['base_position_m'][:2] for s in env['stations']]);task['workspace_world_m']=dict(lower=[*list(xy.min(0)-.85),.025],upper=[*list(xy.max(0)+.85),1.7])
    return task


def public_task(task):
    keys=('kind','id','suite_version','family','level','arms','rules','objective','observation_contract','gripper_profile','cameras','coordinate_frame','execution_readiness')
    result={k:deepcopy(task[k]) for k in keys}
    result['initial_supplies']=[dict(source=s,color=c,count=n) for (s,c),n in sorted(Counter((o.get('source'),o.get('color')) for o in task['objects']).items())]
    result['mechanism']=deepcopy(task['mechanism'])
    if task['family']=='mosaic':
        result['target']={k:deepcopy(task['grid'][k]) for k in ('rows','columns','colors','pitch_m','cube_side_m','pattern','pocket_opening_m','depth_m')}
        result['target'].update(center_world_m=[0,0,TABLE],row1='north',column1='west',reference_image='target-pattern.png')
    if task['family']=='transport':result['orders']=[{k:deepcopy(g[k]) for k in ('goal_id','robot','requires')} for g in task['goals']]
    if task['family']=='dial':result['dials']=[{k:deepcopy(g[k]) for k in ('id','index','nominal_robots')} for g in task['dials']]
    if task['family']=='relay':result['route_regions']=deepcopy(task['route'])
    return result


def validate_task(task):
    from ..scene_task import validate_scene_asset
    folder,env=validate_scene_asset(task);case=env['suite_case']
    if task.get('suite_version')!=2 or task['id']!=case['id'] or task['family']!=case['family']:raise ValueError('V2 identity mismatch')
    if len(task['arms'])!=case['arms']:raise ValueError('V2 arm count mismatch')
    for key in ('objects','mechanism','goals','grid','route','dials'):
        if task.get(key)!=env.get(key,[] if key=='goals' else None):raise ValueError('V2 geometry mismatch: '+key)
    if task['family'] in ('transport','mosaic') and len(task['objects'])!=len(task['goals']):raise ValueError('V2 inventory conservation')
    if task['family']=='mosaic':
        if task['grid']['pitch_m']!=.126 or len(task['goals'])!=case['rows']*case['columns']:raise ValueError('V2 mosaic dimensions')
        available=Counter(o['color'] for o in task['objects']);needed=Counter(g['requires']['color'] for g in task['goals'])
        if available!=needed:raise ValueError('V2 color conservation')
    if task['family']=='transport' and Counter(o['identity'] for o in task['objects'])!=Counter(g['requires']['identity'] for g in task['goals']):raise ValueError('V2 source identity conservation')
    return task


def self_contained(root,folder):
    compiler=root.find('compiler');old_mesh=(folder/compiler.get('meshdir','')).resolve();dest=folder/'assets/robot';dest.mkdir(parents=True,exist_ok=True)
    for asset in root.findall('asset/mesh[@file]'):
        path=Path(asset.get('file'));source=path if path.is_absolute() else old_mesh/path
        # MJCF derives unnamed mesh IDs from the original filename. Preserve
        # that identity before content-addressing the portable file.
        if not asset.get('name'):asset.set('name',path.stem)
        if not source.exists():source=folder/path
        if not source.is_file():raise FileNotFoundError(source)
        name=hashlib.sha256(source.read_bytes()).hexdigest()[:10]+'-'+source.name
        target=dest/name
        if not target.exists():shutil.copy2(source,target)
        asset.set('file',name)
    compiler.set('meshdir','assets/robot')


def physical_summary(model):
    attrs=['body_parentid','body_pos','body_quat','body_mass','body_inertia','body_ipos','body_iquat','jnt_type','jnt_pos','jnt_axis','jnt_range','jnt_stiffness',
           'dof_damping','dof_frictionloss','dof_armature','qpos_spring','actuator_gainprm','actuator_biasprm','actuator_ctrlrange','actuator_forcerange',
           'actuator_gear','tendon_stiffness','tendon_damping','eq_type','eq_data','eq_obj1id','eq_obj2id','eq_active0','eq_solref','eq_solimp','key_qpos','key_ctrl','numeric_data']
    result={name:np.asarray(getattr(model,name)).tolist() for name in attrs}
    result['body_names']=[model.body(i).name for i in range(model.nbody)];result['joint_names']=[model.joint(i).name for i in range(model.njnt)]
    mask=(model.geom_contype!=0)|(model.geom_conaffinity!=0)
    for name in ['geom_type','geom_bodyid','geom_pos','geom_quat','geom_size','geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp','geom_condim','geom_margin']:
        result[name]=np.asarray(getattr(model,name))[mask].tolist()
    meshes={}
    for g in np.flatnonzero(mask):
        if int(model.geom_type[g])!=mujoco.mjtGeom.mjGEOM_MESH:continue
        mid=int(model.geom_dataid[g]);start=int(model.mesh_vertadr[mid]);n=int(model.mesh_vertnum[mid]);face=int(model.mesh_faceadr[mid]);nf=int(model.mesh_facenum[mid])
        meshes[model.mesh(mid).name]=hashlib.sha256(model.mesh_vert[start:start+n].tobytes()+model.mesh_face[face:face+nf].tobytes()).hexdigest()
    result['physical_meshes']=meshes
    result['solver']={name:np.asarray(getattr(model.opt,name)).tolist() for name in ['timestep','gravity','integrator','solver','iterations','noslip_iterations','tolerance','impratio','enableflags','disableflags']}
    return result


def manifest(model,data,case,folder,env):
    physics=physical_summary(model);write_json(folder/'compiled-physics.json',physics)
    assets=[]
    for p in sorted((folder/'assets').rglob('*')):
        if p.is_file():assets.append(dict(path=str(p.relative_to(folder)),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size))
    bodies=[dict(id=model.body(i).name,parent=model.body(int(model.body_parentid[i])).name,position_parent_m=model.body_pos[i].tolist(),quaternion_parent_wxyz=model.body_quat[i].tolist(),initial_world_position_m=data.xpos[i].tolist(),initial_world_quaternion_wxyz=data.xquat[i].tolist(),mass_kg=float(model.body_mass[i]),com_body_m=model.body_ipos[i].tolist(),principal_inertia_kg_m2=model.body_inertia[i].tolist(),inertia_quaternion_wxyz=model.body_iquat[i].tolist()) for i in range(1,model.nbody)]
    joints=[dict(id=model.joint(i).name,body=model.body(int(model.jnt_bodyid[i])).name,type=int(model.jnt_type[i]),qpos_address=int(model.jnt_qposadr[i]),dof_address=int(model.jnt_dofadr[i]),axis=model.jnt_axis[i].tolist(),range=model.jnt_range[i].tolist()) for i in range(model.njnt)]
    moving=set()
    for i in range(1,model.nbody):
        if model.body(i).dofnum or int(model.body_parentid[i]) in moving:moving.add(i)
    geometries=[];local_indices=Counter()
    for i in range(model.ngeom):
        body=model.body(int(model.geom_bodyid[i])).name or 'world';ordinal=local_indices[body];local_indices[body]+=1
        mesh=int(model.geom_dataid[i]);material=int(model.geom_matid[i])
        geometries.append(dict(id=model.geom(i).name or f'{body}:geom{ordinal}',source_index=i,body=body,type=int(model.geom_type[i]),
            position_body_m=model.geom_pos[i].tolist(),quaternion_body_wxyz=model.geom_quat[i].tolist(),size=model.geom_size[i].tolist(),rgba=model.geom_rgba[i].tolist(),
            mesh=model.mesh(mesh).name if int(model.geom_type[i])==mujoco.mjtGeom.mjGEOM_MESH else None,
            material=model.mat(material).name if material>=0 else None,collision=bool(model.geom_contype[i] or model.geom_conaffinity[i])))
    state=dict(schema='robofrontier.workstation.v2',case=deepcopy(case),scene='scene.xml',units='metres',angle_units='radians',quaternion_order='wxyz',up_axis='Z',handedness='right',
        placement='Rigid translation and yaw only; no scale. Online control stays in canonical workstation frame.',
        assets=assets,bodies=bodies,joints=joints,geometries=geometries,actuator_ids=[model.actuator(i).name for i in range(model.nu)],
        cameras=[dict(id=model.camera(i).name,position_m=model.cam_pos[i].tolist(),quaternion_wxyz=model.cam_quat[i].tolist(),vertical_fov_deg=float(model.cam_fovy[i])) for i in range(model.ncam)],
        physics_sha256=hashlib.sha256(json.dumps(physics,sort_keys=True,separators=(',',':')).encode()).hexdigest(),scene_sha256=hashlib.sha256((folder/'scene.xml').read_bytes()).hexdigest(),
        support_surfaces=dict(tabletop_z_m=TABLE,ground_collision='floor',background_visual=['sky','floor_light']),
        gripper=dict(profile=env['gripper_profile'],travel_per_finger_m=.04,opening_range_m=[.04,.12] if env['gripper_profile'].startswith('offset') else [0,.08],
            wide_finger='Manufactured 18 mm wide mounting bridge + stem + elastomer pad, 54 mm reach; candidate, not hardware-certified' if env['gripper_profile'].startswith('offset') else None),
        motion_recording_contract=dict(visibility='Private only; no policy object truth',sampling='Physical substep time; target 60 Hz, preserve actual timestamps',
            required=['qpos','qvel','simulation_time','monotonic_wall_time_mapping','visual_events','source_assets_and_name_map'],
            dynamic_bodies=[model.body(i).name for i in sorted(moving)],
            visual_events=['Aggregate dial display'] if case['family']=='dial' else [],
            integration_status='Schema and assets preserved; unified live recorder hookup is separate from this environment build'),
        compatibility=dict(factory_composed=False,isaac_sim_verified=False,usd_export_verified=False,oracle_verified=False,policy_verified=False))
    write_json(folder/'workstation.json',state)
    return state


def build(case,replace_draft=False,render=True):
    folder=directory(case)
    if folder.exists():
        if not replace_draft:raise FileExistsError(folder)
        validation=folder/'validation.json'
        if validation.exists() and json.loads(validation.read_text()).get('oracle_verified'):raise ValueError('Verified inputs cannot be replaced')
        shutil.rmtree(folder)
    folder.mkdir(parents=True);(folder/'assets').mkdir()
    root,env={'transport':transport,'mosaic':mosaic,'relay':relay,'dial':dial}[case['family']](case,folder)
    # Preview-only global shape remains a pale background; visual metadata
    # never contributes dynamic body mass.
    for geom in root.findall('.//geom'):
        if geom.get('contype')=='0' and geom.get('conaffinity')=='0':geom.set('mass','0')
    self_contained(root,folder);path=folder/'scene.xml';ET.indent(root);ET.ElementTree(root).write(path,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(path));data=initialize(model,env)
    # Correct the tabletop rest from actual compiled geometry, not a hidden
    # parking fixture. Only the module's initial Z is adjusted before keyframe.
    if case['family']=='dial' and case['level']>1:
        for entry in env['dials']:
            bid=model.body(entry['id']+'_module').id;desc={bid}
            for i in range(bid+1,model.nbody):
                if int(model.body_parentid[i]) in desc:desc.add(i)
            minimum=1e9
            for g in range(model.ngeom):
                if int(model.geom_bodyid[g]) not in desc or not(model.geom_contype[g] or model.geom_conaffinity[g]):continue
                r=data.geom_xmat[g].reshape(3,3);typ=int(model.geom_type[g]);sz=model.geom_size[g]
                if typ==mujoco.mjtGeom.mjGEOM_CYLINDER:extent=sz[1]*abs(r[2,2])+sz[0]*math.sqrt(max(0.,1-r[2,2]**2))
                elif typ==mujoco.mjtGeom.mjGEOM_BOX:extent=abs(r[2])@sz
                else:extent=float(abs(r[2])@model.geom_aabb[g,3:])
                minimum=min(minimum,float(data.geom_xpos[g,2]-extent))
            shift=TABLE-minimum
            j=data.joint(entry['id']+'_module_free');j.qpos[2]+=shift
            entry['initial_center_m'][2]+=shift
            for obj in env['objects']:
                if obj['id']==entry['id']+'_module':obj['initial_position_m'][2]+=shift
        mujoco.mj_forward(model,data)
    # Fitting is presentation only; save the resulting declared cameras so
    # standalone and future factory views use an explicit, reproducible frame.
    from scripts.export_frontier_contact_sheets import fit_camera
    for view in ('global','global_rear','top'):
        fit_camera(model,data,view)
        root.find(f"worldbody/camera[@name='{view}']").set('pos',vector(model.cam_pos[model.camera(view).id]))
    add_initial_keyframe(root,model,data);ET.ElementTree(root).write(path,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
    write_json(folder/'environment.json',env);task=task_for(case,env,folder);write_json(folder/'task.json',task);write_json(folder/'public-task.json',public_task(task))
    write_json(folder/'parameters.json',dict(physics=env['physics'],mechanism=env['mechanism'],grid=env.get('grid'),scope='Declared geometry/material and engineering parameters; not a task success proof'))
    state=manifest(model,data,case,folder,env)
    validation=dict(status='generated',oracle_verified=False,policy_verified=False,scope='Compiled self-contained initial scene; no manipulation execution',
        robot_count=case['arms'],nq=model.nq,nv=model.nv,ngeom=model.ngeom,physics_sha256=state['physics_sha256'])
    write_json(folder/'validation.json',validation)
    if render:
        with mujoco.Renderer(model,width=1280,height=800) as renderer:
            for view in ('global','global_rear','top'):
                renderer.update_scene(data,camera=view);Image.fromarray(renderer.render()).save(folder/('preview-'+view+'.png'))
    print(json.dumps(dict(case=case['id'],**validation)),flush=True)
    return folder
