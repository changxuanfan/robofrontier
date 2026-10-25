"""Four scene generators sharing Panda assets and explicit physical specifications.

Generation never starts a policy or claims manipulation success. Dynamic
attachments are parts of the apparatus, never attachments to a robot.
"""
from collections import Counter
from copy import deepcopy
import hashlib,json,math,shutil
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco,numpy as np
from PIL import Image,ImageDraw

from ..scene import ROOT,vector
from ..cube_transport import build_xml,initialize,add_initial_keyframe,layout,make_task as transport_orders,route,xyz,_camera,_font
from ..robot_control import matrix_quat,quat_matrix
from ..run_setup import write_json

CATALOG=ROOT/'configs/suites/frontier-v1.json'
FOLDERS={'transport':'cube_transport','mosaic':'mosaic','relay':'beam_handoff','dial':'spring_dial'}
PAPER=(.945,.937,.910,1);BLUE=(.07,.39,.78,1);RED=(.83,.13,.11,1)
PALETTE={'red':[.85,.13,.13],'blue':[.07,.39,.78],'yellow':[.97,.75,.12],'white':[.945,.937,.910]}

def catalog():
    if not CATALOG.exists():
        source=json.loads((ROOT/'docs/plans/suite-v1/matrix.json').read_text())
        source.update(version=1,status='development; per-case evidence required')
        for c in source['cases']:c['directory']=str(case_directory(c).relative_to(ROOT))
        write_json(CATALOG,source)
    return json.loads(CATALOG.read_text())

def update_case(case_id,**fields):
    import fcntl
    catalog()
    with CATALOG.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        data=json.loads(CATALOG.read_text());case=next(c for c in data['cases'] if c['id']==case_id);case.update(fields);write_json(CATALOG,data)

def case_directory(case):
    revision=case.get('scene_revision',1);suffix='' if revision==1 else '-v'+str(revision)
    return ROOT/'configs/environments'/FOLDERS[case['family']]/('suite-'+case['id']+suffix)

def specification(case):return {k:deepcopy(case[k]) for k in ('id','family','level','arms','dials','pairs','rows','columns','layout','scene_revision') if k in case}
def rz(angle):
    c,s=math.cos(angle),math.sin(angle);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

def station(name,base,yaw,home_depth=.27,home_z=.80):
    s=dict(id=name,base_position_m=[*base,.4],yaw_rad=yaw,side='suite',source_positions_m=[],goal_positions_m=[],parking_positions_m=[])
    s['home_tcp_m']=xyz(s,home_depth,0,home_z);return s

def environment(case,stations,tables):
    return dict(schema='robofrontier.suite.environment.v1',id=case['id'],collection='frontier_suite',arm_count=len(stations),
        physics=dict(table_height_m=.4,cube_size_m=.04,object_mass_kg=.05,contact_friction=[1.,.005,.0001],
            noslip_iterations=20 if case['family']=='dial' else 10,timestep_s=.002,gravity_m_s2=[0,0,-9.81]),
        stations=stations,tables=tables,edges=[],objects=[],floor_rgba=[.84,.90,.95,1],table_rgba=[.604,.459,.345,1],
        initial_keyframe='initial',scene='scene.xml',suite_case=specification(case))

def rectangle(hx,hy,cx=0,cy=0):return dict(center_m=[cx,cy,.375],half_size_m=[hx,hy,.025])

def box(parent,name,pos,size,rgba=PAPER,mass=None,visual=False,**kw):
    attrs=dict(name=name,type='box',pos=vector(pos),size=vector(size),rgba=vector(rgba),**kw)
    if mass is not None:attrs['mass']=str(mass)
    if visual:attrs.update(contype='0',conaffinity='0',mass='0')
    return ET.SubElement(parent,'geom',**attrs)

def cylinder(parent,name,pos,size,rgba=PAPER,mass=None,visual=False,**kw):
    attrs=dict(name=name,type='cylinder',pos=vector(pos),size=vector(size),rgba=vector(rgba),**kw)
    if mass is not None:attrs['mass']=str(mass)
    if visual:attrs.update(contype='0',conaffinity='0',mass='0')
    return ET.SubElement(parent,'geom',**attrs)

def texture(root,directory,name,label,size=(256,256),font_size=150):
    image=Image.new('RGB',size,(244,241,233));ImageDraw.Draw(image).text((size[0]/2,size[1]/2),label,anchor='mm',font=_font(font_size),fill=(17,26,37))
    image.save(directory/'assets'/(name+'.png'));asset=root.find('asset')
    ET.SubElement(asset,'texture',name=name,type='2d',file='assets/'+name+'.png')
    ET.SubElement(asset,'material',name=name,texture=name,specular='0')
    return name

def finish_base(root,env):
    world=root.find('worldbody')
    ET.SubElement(root,'size',memory='64M')
    for g in list(world.findall('geom')):
        if (g.get('name') or '').startswith('table_support_'):world.remove(g)
        if g.get('contype')=='0':g.set('mass','0')
    for i,t in enumerate(env['tables']):
        cx,cy,_=t['center_m'];hx,hy,_=t['half_size_m']
        for j,(x,y) in enumerate(((cx-hx+.09,cy-hy+.09),(cx+hx-.09,cy-hy+.09),(cx-hx+.09,cy+hy-.09),(cx+hx-.09,cy+hy-.09))):
            box(world,f'table_leg_{i}_{j}',[x,y,.175],[.035,.035,.175],(.30,.32,.34,1))
    # Center the view on the actual mounting envelope, with true front/rear.
    xy=np.array([s['base_position_m'][:2] for s in env['stations']]);center=(xy.min(axis=0)+xy.max(axis=0))/2;radius=max(np.linalg.norm(p-center) for p in xy)+.40;target=np.r_[center,.50]
    for c in list(world.findall('camera')):
        if c.get('name') in ('global','global_side','top','global_rear'):world.remove(c)
    _camera(world,'global',np.r_[center+[0,-1.6*radius],.5+1.35*radius],target,43)
    _camera(world,'global_rear',np.r_[center+[0,1.6*radius],.5+1.35*radius],target,43)
    _camera(world,'global_side',np.r_[center+[-1.5*radius,-1.2*radius],.5+1.1*radius],target,48)
    ET.SubElement(world,'camera',name='top',pos=vector([*center,.4+2.7*radius]),xyaxes='1 0 0 0 1 0',fovy='48')
    root.set('model','RoboFrontier | '+env['id'])

def perch(world,name,position,yaw=0):
    bottom=position[2]-.05
    body=ET.SubElement(world,'body',name=name,pos=vector([*position[:2],bottom]),quat=vector(matrix_quat(rz(yaw))))
    if bottom>.401:box(world,name+'_stand',[*position[:2],(.4+bottom)/2],[.018,.018,(bottom-.4)/2],(.42,.49,.53,1))
    for i,(x,y) in enumerate(((-.016,-.016),(-.016,.016),(.016,-.016),(.016,.016))):box(body,name+'_corner'+str(i),[x,y,.015],[.004,.004,.015],(.79,.83,.86,1))

def prism_mesh(root,name,points,zlo,zhi):
    asset=root.find('asset')
    if asset.find(f"mesh[@name='{name}']") is not None:return
    points=np.asarray(points);n=len(points);vertices=np.vstack((np.c_[points,np.full(n,zlo)],np.c_[points,np.full(n,zhi)]));faces=[]
    for i in range(1,n-1):faces.extend(((0,i+1,i),(n,n+i,n+i+1)))
    for i in range(n):j=(i+1)%n;faces.extend(((i,j,n+j),(i,n+j,n+i)))
    ET.SubElement(asset,'mesh',name=name,vertex=vector(vertices.ravel()),face=' '.join(str(k) for f in faces for k in f))

def socket_meshes(root):
    """Six true polygonal bores; rectangular tiles meet without filling holes."""
    names=[]
    ys=[-.16,-.0525,.0525,.16]
    for slot,cy in enumerate((-.105,0,.105)):
        for side,cx in enumerate((-.010,.010)):
            xmin,xmax=((- .06,0) if side==0 else (0,.06));ymin,ymax=ys[slot:slot+2]
            angles=set(np.linspace(0,2*math.pi,25)[:-1].tolist())
            angles.update(math.atan2(y-cy,x-cx)%(2*math.pi) for x,y in ((xmin,ymin),(xmin,ymax),(xmax,ymin),(xmax,ymax)))
            angles=sorted(angles);angles.append(angles[0]+2*math.pi)
            def ray(t):
                dx,dy=math.cos(t),math.sin(t);dist=min((xmax-cx)/dx if dx>1e-9 else (xmin-cx)/dx if dx<-1e-9 else math.inf,(ymax-cy)/dy if dy>1e-9 else (ymin-cy)/dy if dy<-1e-9 else math.inf)
                return [cx+dist*dx,cy+dist*dy]
            for k,(t0,t1) in enumerate(zip(angles,angles[1:])):
                name=f'socket_{slot}_{side}_{k}';poly=[[cx+.005*math.cos(t0),cy+.005*math.sin(t0)],ray(t0),ray(t1),[cx+.005*math.cos(t1),cy+.005*math.sin(t1)]]
                prism_mesh(root,name,poly,-.012,0);names.append(name)
    return names

def add_goal_board(root,directory,s,level):
    center=xyz(s,.42,0,.52);yaw=s['yaw_rad'];world=root.find('worldbody');name='board_'+s['id']
    # A positive-down compression spring is preloaded for the declared board mass.
    if level==3:center[2]+=.15*9.81/300
    body=ET.SubElement(world,'body',name=name,pos=vector(center),quat=vector(matrix_quat(rz(yaw))))
    if level==3:
        ET.SubElement(body,'joint',name=name+'_z',type='slide',axis='0 0 -1',stiffness='300',damping='14',range='-.004 .018',limited='true')
        for axis,label in (('1 0 0','roll'),('0 1 0','pitch')):ET.SubElement(body,'joint',name=name+'_'+label,type='hinge',axis=axis,stiffness='.16',damping='.025',range='-.35 .35',limited='true')
    ET.SubElement(body,'inertial',pos='0 0 -.005',mass='.15',diaginertia='.001282 .000182 .00146')
    if level==1:box(body,name+'_flat',[0,0,-.0075],[.06,.16,.0075],mass=0)
    else:
        for i,mesh in enumerate(socket_meshes(root)):ET.SubElement(body,'geom',name=name+'_tile_'+str(i),type='mesh',mesh=mesh,mass='0',rgba=vector(PAPER),friction='1 .005 .0001',condim='4',contype='4',conaffinity='18')
        # A single top support manifold for bodies wider than the bores avoids
        # hundreds of redundant coplanar mesh contacts. Pegs and fingertips
        # collide with the actual hollow decomposition, never this manifold.
        box(body,name+'_body_support',[0,0,-.006],[.06,.16,.006],(0,0,0,0),mass=0,contype='8',conaffinity='1',group='3')
        box(body,name+'_floor',[0,0,-.0135],[.06,.16,.0015],mass=0)
    if level==3:
        cylinder(world,name+'_cup',[*center[:2],.413],[.045,.013],(.40,.46,.51,1))
        # Cosmetic coil inside a real central support envelope. The declared
        # z/roll/pitch joints are the equivalent passive physical support.
        points=[[.017*math.cos(t),.017*math.sin(t),-.095+.078*t/(10*math.pi)] for t in np.linspace(0,10*math.pi,81)]
        for i,(a,b) in enumerate(zip(points,points[1:])):ET.SubElement(body,'geom',name=name+'_coil_'+str(i),type='capsule',fromto=vector(a+b),size='.0018',rgba='.36 .43 .49 1',mass='0',contype='0',conaffinity='0')
    else:cylinder(world,name+'_stand',[*center[:2],.455],[.018,.055],(.42,.49,.53,1))
    goals=[]
    for slot,y in enumerate((-.105,0,.105)):
        material=texture(root,directory,name+'_label'+str(slot),str(slot+1),font_size=150)
        box(body,name+'_mark'+str(slot),[-.044,y,.0003],[.010,.010,.0002],visual=True,material=material)
        goals.append(dict(goal_id=s['id']+'_slot'+str(slot+1),robot=s['id'],board=name,local_position_m=[0,y,.02],position_m=xyz(s,.42,y,.54)))
    return goals

def build_transport(case,directory):
    n=case['arms'];level=case['level'];env=layout(n);env['collection']='frontier_suite';env['id']=case['id'];env['suite_case']=specification(case)
    env['floor_rgba']=[.84,.90,.95,1];env['table_rgba']=[.604,.459,.345,1]
    for s in env['stations']:
        s['home_tcp_m']=xyz(s,.25,0,.80)
        for p in s['source_positions_m']:p[2]=.57
    for o in env['objects']:
        o['shape']='cube';o['initial_position_m'][2]=.57;o['identity']=o['source']
    from ..cube_transport import SMALL_ORDERS,balanced_destinations
    if n<=6:wanted=[list(x) for x in SMALL_ORDERS[n]]
    else:
        destinations,_=balanced_destinations(n,0);wanted=[[chr(97+i//3) for i,d in enumerate(destinations) if d==j] for j in range(n)]
    by_id={s['id']:s for s in env['stations']}
    for edge in env['edges']:
        a,b=(by_id[r] for r in edge['stations'])
        if abs(math.sin(a['yaw_rad']-b['yaw_rad']))>.9:
            edge['position_m'][0]-=.080*np.sign(edge['position_m'][0]);edge['position_m'][1]-=.080*np.sign(edge['position_m'][1])
    # Build avoids printing old destination marks; the physical boards own them.
    render_env=deepcopy(env)
    for s in render_env['stations']:s['goal_positions_m']=[];s['parking_positions_m']=[]
    root=build_xml(render_env,directory);finish_base(root,env);world=root.find('worldbody');goals=[]
    if level>1:
        for s in env['stations']:
            for suffix in ('left_finger','right_finger'):
                for geom in world.find(f"body[@name='{s['id']}_link0']").findall(f".//body[@name='{s['id']}_{suffix}']/geom"):
                    if geom.get('class')!='visual':geom.set('contype','16');geom.set('conaffinity','23')
    for s,row in zip(env['stations'],wanted):
        board_goals=add_goal_board(root,directory,s,level)
        for g,code in zip(board_goals,row):g['requires']={'identity':code.upper()};goals.append(g)
        for i,p in enumerate(s['source_positions_m']):perch(world,'supply_'+s['id']+'_'+str(i),p)
        _camera(world,'board_'+s['id']+'_close',xyz(s,.62,-.40,.90),np.array(xyz(s,.42,0,.52)),38)
    for edge in env['edges']:
        edge['position_m'][2]=.57;perch(world,'buffer_'+edge['id'],edge['position_m'])
    for o in env['objects']:
        body=world.find(f"body[@name='{o['id']}']")
        label=texture(root,directory,'item_mark_'+o['id'],o['identity'],font_size=170)
        box(body,o['id']+'_mark',[0,0,.0203],[.014,.014,.0002],visual=True,material=label)
        if level>1:
            for j,x in enumerate((-.010,.010)):cylinder(body,o['id']+'_peg'+str(j),[x,0,-.025],[.004,.005],(.35,.39,.43,1),mass=.0006,friction='1 .005 .0001',contype='2',conaffinity='23')
    env['goals']=goals;env['mechanism']=dict(cube_side_m=.04,peg_diameter_m=.008 if level>1 else 0,peg_length_m=.010 if level>1 else 0,hole_diameter_m=.010 if level>1 else 0,peg_spacing_m=.020,board_mass_kg=.15,vertical_stiffness_n_m=300 if level==3 else None,angular_stiffness_nm_rad=.16 if level==3 else None,
        buffer_support='Four 8 mm corner pads on a 32 mm square; both pegs remain clear at in-plane quarter-turn orientations',contact_geometry='True hollow mesh bores for pegs/fingertips; consolidated coplanar support manifold for larger cube/arm bodies. This does not attach, align or insert objects.')
    return root,env

def mosaic_stations(n):
    if n==4:poses=[(0,-.68,math.pi/2),(.68,0,math.pi),(0,.68,-math.pi/2),(-.68,0,0)]
    elif n==6:poses=[(-.32,-.68,math.pi/2),(.32,-.68,math.pi/2),(.96,0,math.pi),(.32,.68,-math.pi/2),(-.32,.68,-math.pi/2),(-.96,0,0)]
    else:poses=[(-.55,-.68,math.pi/2),(0,-.68,math.pi/2),(.55,-.68,math.pi/2),(1.05,0,math.pi),(.55,.68,-math.pi/2),(0,.68,-math.pi/2),(-.55,.68,-math.pi/2),(-1.05,0,0)]
    return [station(chr(65+i),[x,y],a,.27) for i,(x,y,a) in enumerate(poses)]

def mosaic_buffers(env,half_width,half_height):
    hx,hy=env['tables'][0]['half_size_m'][:2];xx,yy=np.meshgrid(np.arange(-hx+.06,hx-.06,.03),np.arange(-hy+.06,hy-.06,.03));points=np.c_[xx.ravel(),yy.ravel()];stock=np.array([o['initial_position_m'][:2] for o in env['objects']])
    keep=(abs(points[:,0])>half_width+.045)|(abs(points[:,1])>half_height+.045)
    if len(stock):keep&=np.min(np.linalg.norm(points[:,None,:]-stock[None,:,:],axis=2),axis=1)>.09
    chosen=[];edges=[]
    stations=env['stations']
    for i,a in enumerate(stations):
        b=stations[(i+1)%len(stations)];valid=keep.copy();score=np.zeros(len(points))
        for s in (a,b):
            delta=points-np.asarray(s['base_position_m'][:2]);local=delta@rz(s['yaw_rad'])[:2,:2]
            valid&=(np.linalg.norm(delta,axis=1)<.72)&(local[:,0]>.06)&(abs(local[:,1])<.60);score+=np.linalg.norm(points-np.asarray(xyz(s,.30,0,.4)[:2]),axis=1)
        for p in chosen:valid&=np.linalg.norm(points-p,axis=1)>.10
        candidates=np.flatnonzero(valid)
        if not len(candidates):raise ValueError('No stock-clear geometric handoff candidate '+a['id']+'/'+b['id'])
        p=points[candidates[np.argmin(score[candidates])]];chosen.append(p);edges.append(dict(id=a['id']+'_'+b['id'],stations=[a['id'],b['id']],position_m=[*p,.60],capacity=1))
    return edges

def build_mosaic(case,directory):
    from scripts.preview_suite_rockets import pattern
    rows=pattern(case['columns']);count=Counter(''.join(rows));decode={'B':'blue','R':'red','Y':'yellow','W':'white'}
    stations=mosaic_stations(case['arms']);hx=max(abs(s['base_position_m'][0]) for s in stations)+.23;hy=max(abs(s['base_position_m'][1]) for s in stations)+.23
    env=environment(case,stations,[rectangle(hx,hy)]);level=case['level'];pitch=.051;side=.04;floor=.4-side/3 if level==1 else .4
    h,w=7*pitch/2,case['columns']*pitch/2
    # Reserve shared transfer space before allocating inventory. Otherwise an
    # apparently dense but valid stock layout can push all handoff buffers to
    # the very edge of the neighbor's kinematic workspace.
    env['edges']=mosaic_buffers(env,w,h)
    colors=['blue','red','yellow','white']
    if len(stations)==6:colors+=['white','red']
    elif len(stations)==8:colors*=2
    inventory=[]
    for color in sorted(set(colors)):
        owners=[s for s,c in zip(stations,colors) if c==color];amount=count[next(k for k,v in decode.items() if v==color)]
        for idx,s in enumerate(owners):
            total=amount//len(owners)+(idx<amount%len(owners))
            for k in range(total):
                inventory.append((s,color,k))
    # Exact stock counts are assigned to a common 80 mm lattice. This is a
    # scene-layout calculation only: the open finger shells need substantially
    # more clearance than the cube itself. Matching prevents overlapping piles
    # at neighboring stations without encoding an assembly solution.
    from scipy.optimize import linear_sum_assignment
    xx,yy=np.meshgrid(np.arange(math.ceil((-hx+.055)/.08),math.floor((hx-.055)/.08)+1)*.08,np.arange(math.ceil((-hy+.055)/.08),math.floor((hy-.055)/.08)+1)*.08);points=np.c_[xx.ravel(),yy.ravel()]
    assembly_x=case['columns']*.051/2;assembly_y=7*.051/2
    valid=(abs(points[:,0])>assembly_x+.055)|(abs(points[:,1])>assembly_y+.055)
    for s in stations:valid&=np.linalg.norm(points-np.asarray(s['base_position_m'][:2]),axis=1)>.19
    for edge in env['edges']:valid&=np.linalg.norm(points-np.asarray(edge['position_m'][:2]),axis=1)>.10
    points=points[valid];cost=[]
    for s,color,k in inventory:
        delta=points-np.asarray(s['base_position_m'][:2]);local=delta@rz(s['yaw_rad'])[:2,:2];distance=np.linalg.norm(delta,axis=1)
        admitted=(local[:,0]>=.30-1e-8)&(abs(local[:,1])<.65)&(distance<.78)
        value=np.linalg.norm(local-np.array([.35,0]),axis=1);cost.append(np.where(admitted,value,1e6))
    cost=np.asarray(cost);items,assigned=linear_sum_assignment(cost)
    if len(items)!=len(inventory) or np.any(cost[items,assigned]>1e5):raise ValueError('Insufficient robot-reachable stock clearance')
    for index,pindex in zip(items,assigned):
        s,color,k=inventory[index];p=[*points[pindex],.42];s['source_positions_m'].append(p)
        env['objects'].append(dict(id=s['id']+'_'+color+'_'+str(k+1),source=s['id'],color=color,rgb=PALETTE[color],shape='cube',initial_position_m=p))
    root=build_xml(env,directory);finish_base(root,env);world=root.find('worldbody')
    for edge in env['edges']:perch(world,'buffer_'+edge['id'],edge['position_m'])
    if level==1:
        world.remove(world.find("geom[@name='table_0']"))
        for i,(cx,cy,xx,yy) in enumerate(((0,-(hy+h)/2,hx,(hy-h)/2),(0,(hy+h)/2,hx,(hy-h)/2),(-(hx+w)/2,0,(hx-w)/2,h),((hx+w)/2,0,(hx-w)/2,h))):box(world,'table_frame_'+str(i),[cx,cy,.375],[xx,yy,.025],env['table_rgba'])
        box(world,'table_pocket_floor',[0,0,floor-.0125],[w+.001,h+.001,.0125],env['table_rgba'])
        for i in range(case['columns']+1):box(world,'pocket_column_'+str(i),[(i-case['columns']/2)*pitch,0,(floor+.4)/2],[.0005,h+.0005,(.4-floor)/2],(.77,.82,.85,1))
        for i in range(8):box(world,'pocket_row_'+str(i),[0,(i-3.5)*pitch,(floor+.4)/2],[w+.0005,.0005,(.4-floor)/2],(.77,.82,.85,1))
    elif level==2:
        for i in range(case['columns']+1):box(world,'printed_column_'+str(i),[(i-case['columns']/2)*pitch,0,.4001],[.0005,h,.00008],(.34,.41,.46,1),visual=True)
        for i in range(8):box(world,'printed_row_'+str(i),[0,(i-3.5)*pitch,.4001],[w,.0005,.00008],(.34,.41,.46,1),visual=True)
    goals=[]
    for r,line in enumerate(rows):
        for c,code in enumerate(line):goals.append(dict(goal_id=f'r{r+1}c{c+1}',row=r+1,column=c+1,requires={'color':decode[code]},position_m=[(c-(case['columns']-1)/2)*pitch,(3-r)*pitch,floor+.02]))
    env['goals']=goals;env['grid']=dict(rows=7,columns=case['columns'],pitch_m=pitch,cube_side_m=.04,pattern=rows,floor_z_m=floor,rim_z_m=.4,pocket_opening_m=.05 if level==1 else None)
    target=Image.new('RGB',(case['columns']*48,7*48),(240,244,247));draw=ImageDraw.Draw(target)
    for row,line in enumerate(rows):
        for col,code in enumerate(line):draw.rectangle((col*48,row*48,(col+1)*48-1,(row+1)*48-1),fill=tuple(round(255*v) for v in PALETTE[decode[code]]),outline=(120,130,140))
    target.save(directory/'target-pattern.png')
    _camera(world,'assembly_close',[0,-.4,1.35],np.array([0,0,.4]),48)
    return root,env

def annular_table(root,center,radius,half_width,start,end,name='table_route'):
    world=root.find('worldbody');segments=max(16,math.ceil(abs(end-start)/.12));angles=np.linspace(start,end,segments+1)
    for i,(a,b) in enumerate(zip(angles,angles[1:])):
        points=[np.asarray(center)+r*np.array([math.cos(t),math.sin(t)]) for r,t in ((radius-half_width,a),(radius+half_width,a),(radius+half_width,b),(radius-half_width,b))]
        mesh=name+'_mesh'+str(i);prism_mesh(root,mesh,points,.35,.4)
        ET.SubElement(world,'geom',name=name+'_'+str(i),type='mesh',mesh=mesh,material='table_oak',friction='1 .005 .0001')
        if i%3==0:
            p=np.asarray(center)+radius*np.array([math.cos(a),math.sin(a)]);box(world,name+'_leg'+str(i),[*p,.175],[.04,.04,.175],(.32,.36,.40,1))

def build_relay(case,directory):
    pairs=case['pairs'];level=case['level'];tangent=pairs==8 and case.get('scene_revision',1)>=2;distance=.50 if tangent else .65;stagger=(.15 if case.get('scene_revision',1)>=3 else .25) if tangent else 0.;rest_height=.72 if tangent else .90
    if pairs==4:centers=[[(i-1.5)*.72,0] for i in range(4)];yaws=[0.]*4;circle=None
    else:
        radius=2.6 if pairs==6 else (1.25 if case.get('scene_revision',1)>=5 else 1.1) if tangent else 1.4;angles=np.linspace(0,math.pi/2,pairs) if pairs==6 else np.arange(pairs)*2*math.pi/pairs
        points=np.array([[radius*math.sin(t),radius*(1-math.cos(t))] for t in angles]);offset=points.mean(axis=0);centers=(points-offset).tolist();yaws=angles.tolist();circle=dict(center=(-offset+np.array([0,radius])).tolist(),radius=radius,start=-math.pi/2,end=0 if pairs==6 else 3*math.pi/2)
    stations=[]
    for i,(p,yaw) in enumerate(zip(centers,yaws)):
        for j,sign in enumerate((-1,1)):
            base=np.asarray(p)+(rz(yaw)@np.array([sign*stagger,sign*distance,0]))[:2]
            facing=yaw+math.atan2(distance,stagger)+(math.pi if sign>0 else 0) if tangent else yaw-sign*math.pi/2
            stations.append(station(chr(65+2*i+j),base,facing,.27,1.0 if tangent else 1.08))
    hx=max(abs(s['base_position_m'][0]) for s in stations)+.24;hy=max(abs(s['base_position_m'][1]) for s in stations)+.24
    if tangent and pairs==4:hx=max(hx,max(abs(p[0]) for p in centers)+.70)
    env=environment(case,stations,[rectangle(hx,hy)]);root=build_xml(env,directory);finish_base(root,env);world=root.find('worldbody')
    if circle:
        for g in list(world.findall('geom')):
            if (g.get('name') or '').startswith(('table_0','table_leg_')):world.remove(g)
        annular_table(root,circle['center'],circle['radius'],.75 if tangent else .92,circle['start']-.15,circle['end']+.15 if pairs==6 else circle['end'])
    path=[dict(center_m=[*p,rest_height],yaw_rad=y-(math.pi/2 if tangent else 0)) for p,y in zip(centers,yaws)]
    if pairs==8:path.append(deepcopy(path[0]))
    env['route']=path;env['pairs']=[dict(robots=[chr(65+2*i),chr(66+2*i)],**path[i]) for i in range(pairs)]
    for which,index in (('start',0),('finish',len(path)-1)):
        frame=path[index];body=ET.SubElement(world,'body',name=which+'_cradle',pos=vector([*frame['center_m'][:2],.4]),quat=vector(matrix_quat(rz(frame['yaw_rad']))))
        # Closed-loop finish reuses the same physical cradle rather than
        # overlapping identical collision surfaces at the starting point.
        if which=='finish' and pairs==8:world.remove(body);continue
        half_height=(rest_height-.0225-.4)/2
        for j,y in enumerate((-.31,.31)):box(body,which+'_rail'+str(j),[0,y,half_height],[.06,.025,half_height],(.77,.82,.86,1))
    body=ET.SubElement(world,'body',name='payload',pos=vector(path[0]['center_m']),quat=vector(matrix_quat(rz(path[0]['yaw_rad']))));ET.SubElement(body,'freejoint',name='payload_free')
    box(body,'payload_beam',[0,0,0],[.0225,.6,.0225],(.52,.29,.16,1),mass=1.458,friction='1 .005 .0001',condim='4')
    if level>1:
        parent=body;segments=1 if level==2 else 8;length=.018 if level==2 else .20
        for i in range(segments):
            part=ET.SubElement(parent,'body',name='suspension_'+str(i),pos=vector([0,.52,-.0225] if i==0 else [0,0,-length/segments]))
            ET.SubElement(part,'joint',name='suspension_joint_'+str(i),type='ball',damping='.00015',armature='.000001')
            ET.SubElement(part,'geom',name='suspension_link_'+str(i),type='capsule',fromto=vector([0,0,0,0,0,-length/segments]),size='.0025',mass='.0015',rgba='.37 .43 .49 1',friction='.6 .005 .0001');parent=part
        weight=ET.SubElement(parent,'body',name='weight',pos=vector([0,0,-length/segments-.025]));cylinder(weight,'weight_geom',[0,0,0],[.025,.025],(.32,.38,.45,1),mass=.25,friction='.6 .005 .0001')
    env['objects']=[dict(id='payload',shape='beam',initial_position_m=path[0]['center_m'],color='wood',rgb=[.52,.29,.16])]
    env['mechanism']=dict(beam_length_m=1.2,beam_cross_section_m=[.045,.045],beam_mass_kg=1.458,weight_mass_kg=.25 if level>1 else 0,suspension_length_m=.018 if level==2 else .20 if level==3 else 0,suspension_segments=1 if level==2 else 8 if level==3 else 0,opposed_base_spacing_m=2*math.hypot(distance,stagger),cross_track_base_spacing_m=2*distance,longitudinal_pair_stagger_m=2*stagger,minimum_supporting_robots=2,loop_radius_m=circle['radius'] if pairs==8 else None,beam_alignment='tangent' if tangent else 'normal')
    for i,frame in enumerate(path[:pairs]):
        p=np.asarray(frame['center_m']);_camera(world,'pair_'+str(i+1),p+np.array([.6,-1,1]),p,45)
    return root,env

def dial_layout(count,level):
    if level==1:
        if count==4:positions=[(-.4,.36),(.4,.36),(-.4,-.36),(.4,-.36)]
        elif count==6:positions=[(-.45,.60),(.45,.60),(-.45,0),(.45,0),(-.45,-.60),(.45,-.60)]
        else:positions=[(-.72,.62),(0,.62),(.72,.62),(-.72,0),(.72,0),(-.72,-.62),(0,-.62),(.72,-.62)]
        stations=[]
        for i,p in enumerate(positions):
            outward=np.array([0,1 if p[1]>0 else -1]) if abs(p[1])>.01 else np.array([1 if p[0]>0 else -1,0])
            base=np.asarray(p)+outward*.56;yaw=math.atan2(-outward[1],-outward[0]);stations.append(station(chr(65+i),base,yaw,.24))
        return stations,[(np.array(p),0.) for p in positions]
    if count==4:modules=[(np.array([(i-1.5)*.7,0]),0.) for i in range(4)]
    elif count==6:modules=[(np.array([x,y]),0.) for y in (-1.05,1.05) for x in (-.7,0,.7)]
    else:modules=[(1.65*np.array([math.cos(t),math.sin(t)]),t-math.pi/2) for t in np.arange(8)*math.pi/4]
    stations=[]
    for i,(p,angle) in enumerate(modules):
        for j,(xy,yaw) in enumerate((([-.25,-.65],math.pi/2),([0,.65],-math.pi/2))):
            base=p+(rz(angle)@np.r_[xy,0])[:2];stations.append(station(chr(65+2*i+j),base,angle+yaw,.23,.82))
    return stations,modules

def add_display(root,directory,count,center):
    world=root.find('worldbody');x,y=center;box(world,'score_base',[x,y,.418],[.22,.17,.018])
    material=texture(root,directory,'score_title',f'CORRECT / {count:02d}',(1024,200),110)
    box(world,'score_title_geom',[x,y+.113,.4363],[.20,.033,.0002],visual=True,material=material)
    places=[(0,.055,.036,.005),(.041,.028,.005,.023),(.041,-.028,.005,.023),(0,-.055,.036,.005),(-.041,-.028,.005,.023),(-.041,.028,.005,.023),(0,0,.036,.005)]
    for digit,dx in enumerate((-.062,.062)):
        for i,(xx,yy,hx,hy) in enumerate(places):box(world,f'score_{digit}_{i}',[x+dx+xx,y+yy-.015,.437],[hx,hy,.0003],(.68,.13,.07,0) if i!=6 else (.68,.13,.07,1),visual=True)
    ET.SubElement(world,'camera',name='board',pos=vector([x,y,1.25]),xyaxes='1 0 0 0 1 0',fovy='40')

def build_dial(case,directory,layout_fn=None):
    count=case['dials'];level=case['level'];stations,modules=(layout_fn or dial_layout)(count,level)
    xy=np.array([s['base_position_m'][:2] for s in stations]);mid=(xy.min(axis=0)+xy.max(axis=0))/2;hx,hy=(xy.max(axis=0)-xy.min(axis=0))/2+.24
    env=environment(case,stations,[rectangle(hx,hy,*mid)]);root=build_xml(env,directory);finish_base(root,env);world=root.find('worldbody');asset=root.find('asset')
    if level>1 and count==8:
        for g in list(world.findall('geom')):
            if (g.get('name') or '').startswith(('table_0','table_leg_')):world.remove(g)
        annular_table(root,[0,0],case.get('ring_radius_m',1.65),.91,0,2*math.pi,'table_ring')
    # The established single-module mesh/texture is an asset template only.
    source=ROOT/'configs/environments/spring_dial'/('floating-crank-04-070-tight-noslip20' if level==3 else 'floating-04-060-tight-noslip20')
    template=ET.parse(source/'scene.xml').getroot();srcbody=template.find("worldbody/body[@name='dial_A_module']")
    needed={e.get(k) for e in srcbody.iter() for k in ('material','mesh') if e.get(k)}
    while needed:
        name=min(needed);needed.remove(name)
        if asset.find(f"*[@name='{name}']") is not None:continue
        e=template.find(f"asset/*[@name='{name}']")
        if e is None:raise ValueError('Missing module asset '+name)
        e=deepcopy(e)
        if e.get('texture'):needed.add(e.get('texture'))
        if e.get('file'):
            src=source/e.get('file');dest=directory/'assets'/src.name;shutil.copy2(src,dest);e.set('file','assets/'+src.name)
        asset.append(e)
    env['dials']=[]
    for i,(center,angle) in enumerate(modules):
        label=f'D{i+1:02d}';body=deepcopy(srcbody)
        for e in body.iter():
            for key,value in list(e.attrib.items()):
                if value.startswith('dial_A'):e.set(key,value.replace('dial_A',label,1))
            if e.tag=='geom' and e.get('contype')=='0':e.set('mass','0')
        if level==1:
            body.remove(body.find('freejoint'));body.set('pos',vector([*center,.405]));body.set('quat','1 0 0 0')
            stem=body.find(f"geom[@name='{label}_module_stem']");body.remove(stem)
        else:
            if case.get('scene_revision',1)>=3:
                body.set('pos',vector([*center,.515]));body.set('quat',vector(matrix_quat(rz(angle)@np.array([[1,0,0],[0,0,-1],[0,1,0]]))))
                body.find(f"geom[@name='{label}_module_stem']").set('pos','0 .02 -.154')
                parking=ET.SubElement(world,'body',name=label+'_parking',pos=vector(np.r_[center,.4]+rz(angle)@np.array([0,.27,0])),quat=vector(matrix_quat(rz(angle))))
                box(parking,label+'_parking_rest',[0,0,.0625],[.026,.018,.0625],(.64,.70,.73,1))
            else:
                sm=mujoco.MjModel.from_xml_path(str(source/'scene.xml'));sd=mujoco.MjData(sm);mujoco.mj_resetDataKeyframe(sm,sd,sm.key('initial').id)
                q=sd.joint('dial_A_module_free').qpos.copy();old_center=float(template.find("worldbody/body[@name='dial_A_module']").get('pos').split()[0]);q[0]-=old_center
                position=np.r_[center,0]+rz(angle)@q[:3];body.set('pos',vector(position));body.set('quat',vector(matrix_quat(rz(angle)@quat_matrix(q[3:]))))
        hinge=body.find(f"body/joint[@name='{label}_hinge']");hinge.set('stiffness','.03');hinge.set('damping','.003')
        if level>1:
            # 16 x 20 x 300 mm anodized aluminium handle, 2700 kg/m3.
            # The legacy steel-like handle remains in its original assets.
            body.find(f"geom[@name='{label}_module_stem']").set('mass','.2592')
            if case.get('scene_revision',1)>=3:
                for j,z in enumerate((-.078,-.050)):
                    box(body,f'{label}_handle_shoulder{j}',[0,.02,z],[.012,.028,.002],BLUE,mass=.0145152,friction='.8 .005 .0001',condim='4',solref='.004 1',solimp='.95 .99 .001 .5 2')
        if level==3:
            rotor=body.find(f"body[@name='{label}_rotor']")
            box(rotor,label+'_crank_balance_arm',[0,-.03,.021],[.008,.03,.0045],RED,mass=.015,friction='.8 .005 .0001',condim='4')
            cylinder(rotor,label+'_crank_balance',[0,-.06,.021],[.014,.0055],RED,mass=.04575,friction='.8 .005 .0001',condim='4')
            for geom in rotor.findall('geom'):
                if geom.get('contype')!='0':geom.set('priority','1');geom.set('solref','.004 1');geom.set('solimp','.95 .99 .001 .5 2')
        # A separate physical-looking identifier avoids confusing robot A with
        # a module A. It is a massless printed marking, not a state overlay.
        oldlabel=body.find(f"geom[@name='{label}_label']")
        if oldlabel is not None:
            material=texture(root,directory,label+'_identity',f'{i+1:02d}',font_size=170);oldlabel.set('material',material)
            if case.get('scene_revision',1)>=3:oldlabel.set('pos','-.04 .04 .0047');oldlabel.set('size','.018 .011 .0002')
        world.append(body)
        entry=dict(id=label,index=i,nominal_robots=[chr(65+i)] if level==1 else [chr(65+2*i),chr(66+2*i)],initial_center_m=[float(x) for x in body.get('pos').split()],layout_yaw_rad=angle)
        env['dials'].append(entry)
        if level>1:env['objects'].append(dict(id=label+'_module',source=entry['nominal_robots'][1],shape='dial_module',color='white',rgb=list(PAPER[:3]),initial_position_m=entry['initial_center_m']))
        for suffix,p in (('front',[0,-1.3,.95]),('side',[1.0,-.25,1.0]),('high',[.4,-.7,1.5])):
            point=np.r_[center,0]+rz(angle)@np.asarray(p);_camera(world,label+'_'+suffix,point,np.r_[center,.68 if level>1 else .45],38)
    if level>1 and count==8:score=((case.get('ring_radius_m',1.65)+.63)*np.array([math.cos(-3*math.pi/8),math.sin(-3*math.pi/8)])).tolist()
    else:
        front=sorted(s['base_position_m'][0] for s in stations if abs(s['base_position_m'][1]-xy[:,1].min())<.03)
        gaps=[(a+b)/2 for a,b in zip(front,front[1:]) if b-a>.55]
        score=[min(gaps,key=lambda x:abs(x-mid[0])) if gaps else float(mid[0]),float(mid[1]-hy+.20)]
    add_display(root,directory,count,score)
    env['mechanism']=dict(dial_count=count,interface='crank' if level==3 else 'gear',free_modules=level>1,handle_material='anodized aluminium' if level>1 else None,handle_mass_kg=.2592 if level>1 else 0,spring_nm_rad=.03,damping_nm_s_rad=.003,angle_tolerance_deg=4,minimum_airborne_clearance_m=.02,blue_support_dwell_s=.25,score_panel_xy_m=score,rotation_range_rad=[-math.pi,math.pi])
    env['mechanism'].update(feedback_rule_version=2,feedback_dwell_sim_s=.25,feedback_stability='same nonzero 4-degree detent band throughout the dwell; in-band vibration allowed')
    if level==3:env['mechanism'].update(rotor_counterbalanced=True,balance_mass_kg=.04575,balance_arm_mass_kg=.015,balance_radius_m=.06)
    if level>1 and case.get('scene_revision',1)>=3:env['mechanism'].update(handle_stem_mass_kg=.2592,handle_mass_kg=.2592+2*.0145152,parking_fixture='Passive rear rack; the complete module must leave all external supports before red operation',handle_cross_section_m=[.016,.020],handle_attachment_local_m=[0,.02,-.154],handle_neck_clear_length_m=.024,handle_shoulders='Two visible passive stop shoulders. They transmit load by real contact and release when the gripper opens; no hand attachment.')
    return root,env

BUILDERS={'transport':build_transport,'mosaic':build_mosaic,'relay':build_relay,'dial':build_dial}

def build_case(case,rebuild=False,render=True):
    directory=case_directory(case)
    if directory.exists():
        if not rebuild:raise ValueError('Scene exists; choose --rebuild for an unvalidated development asset')
        current=next(c for c in catalog()['cases'] if c['id']==case['id'])
        if current['status'] in ('oracle_witness_passed','repeatability_checked'):raise ValueError('Validated asset is immutable; make a new version')
        if (directory/'scene.xml').exists():
            previous_hash=hashlib.sha256((directory/'scene.xml').read_bytes()).hexdigest()
            for report in (ROOT/'runs/oracle/frontier-v1'/case['id']).glob('seed-*/attempt-*/report.json'):
                evidence=json.loads(report.read_text())
                if evidence.get('passed') and evidence.get('scene_sha256')==previous_hash:raise ValueError('A physical witness protects this asset even if the catalogue status changed; create a new revision')
    directory.mkdir(parents=True,exist_ok=True);(directory/'assets').mkdir(exist_ok=True)
    root,env=BUILDERS[case['family']](case,directory);path=directory/'scene.xml';ET.indent(root);ET.ElementTree(root).write(path,encoding='unicode')
    if case['family'] in ('transport','mosaic'):
        # Calibrated before benchmark freezing: the old 20 ms soft contact
        # admitted millimetre-scale pad/object drift, comparable to bore
        # clearance. A 4 ms pair model passed the physical grasp/insert probe.
        # This changes neither friction nor drive force and never runs in policy.
        env['physics']['cube_contact_time_constant_s']=.004
        for obj in env['objects']:
            body=root.find(f"worldbody/body[@name='{obj['id']}']")
            for geom in body.findall('geom'):
                if geom.get('contype')=='0':continue
                geom.set('priority','1');geom.set('solref','.004 1');geom.set('solimp','.95 .99 .001 .5 2')
        ET.ElementTree(root).write(path,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(path));data=initialize(model,env);add_initial_keyframe(root,model,data);ET.ElementTree(root).write(path,encoding='unicode')
    # Pure visual masses cannot contaminate the physical assembly specification.
    env['compiled_mass_kg']={model.body(i).name:float(model.body_mass[i]) for i in range(1,model.nbody) if (model.body(i).name or '').startswith(('board_','D','payload','weight','suspension_'))}
    write_json(directory/'environment.json',env);write_json(directory/'parameters.json',dict(case=case,physics=env['physics'],mechanism=env.get('mechanism'),grid=env.get('grid')))
    if render:
        with mujoco.Renderer(model,height=800,width=1280) as r:
            for camera in ('global','global_rear','top'):
                r.update_scene(data,camera=camera);Image.fromarray(r.render()).save(directory/('preview-'+camera+'.png'))
    from .tasks import make_task
    task=make_task(directory);write_json(directory/'task.json',task)
    report=dict(status='generated',passed=None,scope='Compiled scene and initial home configuration only; no full manipulation proof.',robot_count=model.nu//8,cameras=model.ncam,nq=model.nq,physical_geoms=model.ngeom,scene_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    write_json(directory/'validation.json',report);update_case(case['id'],status='generated',scene_sha256=report['scene_sha256']);print(json.dumps(dict(id=case['id'],**report)),flush=True)
    return directory
