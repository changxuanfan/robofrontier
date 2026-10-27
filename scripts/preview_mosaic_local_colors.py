"""Exact colour/layout diagrams and zero-step MuJoCo previews for Mosaic R14."""
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from robofrontier.suite.mosaic_policy_package import ROOT, read, write, checksum


def font(size):
    return ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Unicode.ttf',size)


def diagram(case, plan, old=False):
    folder=ROOT/case['directory'];env=read(folder/'environment.json')
    colours={a['object_id']:a['previous_colour'] if old else a['colour'] for a in plan['assignments']}
    image=Image.new('RGB',(1500,1500),'#F1F5F7');d=ImageDraw.Draw(image)
    ink='#19384C';muted='#526A78'
    d.text((42,27),f"{case['arms']}臂 · L{case['level']} · {'旧配色 R13' if old else '新版配色 R14'}",font=font(39),fill=ink)
    d.text((42,88),'只改初始供料颜色 · 位置、机械臂、目标图案均保持原样',font=font(25),fill=muted)
    d.text((42,132),'北向上；中间彩格是目标参考示意，不是初态已摆好的方块',font=font(24),fill=muted)
    hx,hy=env['tables'][0]['half_size_m'][:2]
    scale=min(1370/(2*hx),1010/(2*hy));cx,cy=750,730
    def point(x,y):return cx+x*scale,cy-y*scale
    def square(position,side,fill,outline,width=1):
        x,y=position[:2];a=point(x-side/2,y+side/2);b=point(x+side/2,y-side/2)
        d.rectangle((*a,*b),fill=fill,outline=outline,width=width)
    d.rectangle((*point(-hx,hy),*point(hx,-hy)),fill='#E7DCC7',outline='#BFB398',width=2)
    for g in env['goals']:
        square(g['position_m'],env['grid']['pitch_m']*.94,env['grid']['palette'][g['requires']['color']],'#FFFFFF')
    for o in env['objects']:
        square(o['initial_position_m'],env['grid']['cube_side_m'],env['grid']['palette'][colours[o['id']]],'#273D48',2)
    for station in env['stations']:
        x,y=station['base_position_m'][:2];px,py=point(x,y)
        tip=point(x+.13*math.cos(station['yaw_rad']),y+.13*math.sin(station['yaw_rad']))
        d.line((px,py,*tip),fill='#205E89',width=5)
        d.ellipse((px-17,py-17,px+17,py+17),fill='#205E89',outline='white',width=2)
        d.text((px,py),station['id'],anchor='mm',font=font(23),fill='white')
    y=1280
    for index,row in enumerate(plan['per_robot']):
        col=index%4;line=index//4
        text=f"{row['robot']}: {row['certified_local_pairs']}/{row['nearby_stock']} ({row['certified_local_fraction']:.0%})"
        d.text((45+col*365,y+line*42),text,font=font(27),fill=ink)
    if old:
        d.rectangle((30,1260,1470,1380),fill='#F1F5F7')
        d.text((45,1283),'旧版常出现邻近供料集中为一种颜色，需要大量跨区取料。',font=font(26),fill=ink)
    else:
        d.text((42,1390),'比例＝同一机械臂可独立取放的不同供料—目标格配对数 / 最近供料总数。',font=font(22),fill=muted)
        d.text((42,1427),'采样几何设计证据；尚未运行新版 Oracle / GPT，不代表并行路径已验证。',font=font(22),fill=muted)
    return image


def previews(cases,plans,output):
    import mujoco
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    cards=[];manifest=[]
    all_nine=Image.new('RGB',(3000,3000),'#F1F5F7')
    before_after=Image.new('RGB',(3600,2400),'#F1F5F7')
    for case in cases:
        dest=output/case['id'];dest.mkdir()
        plan=plans[case['arms']];layout=diagram(case,plan)
        layout.save(dest/'colour-layout.png')
        col=[4,6,8].index(case['arms']);row=case['level']-1
        all_nine.paste(layout.resize((1000,1000),Image.Resampling.LANCZOS),(col*1000,row*1000))
        if case['level']==1:
            before_after.paste(diagram(case,plan,True).resize((1200,1200),Image.Resampling.LANCZOS),(col*1200,0))
            before_after.paste(layout.resize((1200,1200),Image.Resampling.LANCZOS),(col*1200,1200))
        model=mujoco.MjModel.from_xml_path(str(ROOT/case['directory']/'scene.xml'));data=mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
        model.vis.global_.offwidth=1600;model.vis.global_.offheight=1200
        with mujoco.Renderer(model,width=1600,height=1200,max_geom=max(5000,model.ngeom+1000)) as renderer:
            for camera in ('top','global'):
                renderer.update_scene(data,camera=camera)
                Image.fromarray(renderer.render()).save(dest/(camera+'.png'))
        print('PREVIEW',case['id'],flush=True)
        assets={name:checksum(dest/name) for name in ('colour-layout.png','top.png','global.png')}
        manifest.append(dict(case_id=case['id'],scene_sha256=checksum(ROOT/case['directory']/'scene.xml'),files=assets,physics_steps=0))
        links=''.join(f'<a href="{case["id"]}/{file}"><span>{title}</span><img src="{case["id"]}/{file}"></a>' for file,title in [('colour-layout.png','精确配色图（中间为目标参考）'),('top.png','真实初态俯视'),('global.png','真实初态斜视')])
        cards.append(f'<article><h2>{case["arms"]}臂 · L{case["level"]} · {case["rows"]}×{case["columns"]}</h2><div>{links}</div></article>')
    all_nine.save(output/'all-nine-colour-layouts.jpg',quality=95)
    before_after.save(output/'before-after-colours.jpg',quality=95)
    write(output/'preview-manifest.json',dict(scene_revision=14,physics_steps=0,cases=manifest))
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Mosaic R14 方块配色</title><style>body{font:18px system-ui;color:#19384c;background:#f1f5f7;max-width:1800px;margin:30px auto;padding:0 20px}article{background:white;padding:22px;margin:20px 0}article div{display:grid;grid-template-columns:repeat(3,1fr);gap:15px}img{width:100%}span{display:block;margin:10px 0}p{line-height:1.7}a{color:#205e89}</style><h1>Mosaic R14：九项任务的新供料配色</h1><p>只改初始方块颜色。每种颜色的总数量、目标图案、方块位置和尺寸、桌面、机械臂及物理参数保持原样。各等级同尺度的供料配色一致。中间彩格是目标参考示意，真实初态请看俯视/斜视截图。</p><p>每臂约七成附近供料具有不同目标格的同臂取放几何证据；其余留给跨区协作。设计分配不是 GPT 的固定路线，GPT 可以找到更少交接的方案。未执行新版 Oracle 或 GPT。</p><p><a href="all-nine-colour-layouts.jpg">全部九项总图</a> · <a href="before-after-colours.jpg">三种规模旧/新对比</a></p>'+''.join(cards),encoding='utf-8')
