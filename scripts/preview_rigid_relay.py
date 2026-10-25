"""Render current cable-free tabletop initial states; no physics steps."""
from html import escape
import json
import mujoco
from PIL import Image, ImageDraw, ImageFont

from robofrontier.suite.v2 import ROOT,cases,directory

REVISION=max(c['scene_revision'] for c in cases() if c['family']=='relay')
OUT=ROOT/f'docs/generated/frontier-v2/relay-r{REVISION}'
FONT='/System/Library/Fonts/Supplemental/Arial Unicode.ttf'
DESCRIPTIONS={2:'L2 · 90°弯杆＋闭合环悬挂砝码',3:'L3 · 弹性折杆＋短硬摆杆＋砝码'}


def sheet(selected,view,name,title):
    width,height=1600,1000;gap=24;margin=28;head=120;rowheight=1100
    image=Image.new('RGB',(3*width+2*gap+2*margin,head+2*rowheight+gap+margin),'#eef3f6')
    draw=ImageDraw.Draw(image)
    draw.text((margin,16),title,font=ImageFont.truetype(FONT,46),fill='#183345')
    draw.text((margin,77),'真实 MuJoCo 静态初态 · 600 g 配重 · 未运行 Oracle',font=ImageFont.truetype(FONT,28),fill='#526979')
    for row,level in enumerate((2,3)):
        for col,arms in enumerate((8,12,16)):
            case=next(c for c in selected if c['arms']==arms and c['level']==level)
            x=margin+col*(width+gap);y=head+row*(rowheight+gap)
            draw.rounded_rectangle((x,y,x+width,y+rowheight),18,fill='white')
            draw.text((x+15,y+15),f'{DESCRIPTIONS[level]} · {arms} 臂 / {arms//4} 根',font=ImageFont.truetype(FONT,32),fill='#183345')
            with Image.open(OUT/case['id']/(view+'.png')) as source:image.paste(source,(x,y+80))
    image.save(OUT/(name+'.png'))
    image.thumbnail((2000,1300));image.save(OUT/(name+'-preview.jpg'),quality=94)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    selected=[c for c in cases() if c['family']=='relay' and c['level']>1]
    cards=[]
    for case in selected:
        folder=directory(case);target=OUT/case['id'];target.mkdir(exist_ok=True)
        m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));d=mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id);mujoco.mj_forward(m,d)
        m.vis.global_.offwidth=1600;m.vis.global_.offheight=1000
        views=['top','global','global_rear','payload_01_close','payload_01_weight_close']
        labels=['俯视','正面','背面','杆体近景','闭合环与砝码近景']
        task=json.loads((folder/'task.json').read_text());legend=[]
        if case['level']==3:
            for a in task['mechanism']['payload_assemblies']:
                mode=a.get('elastic_hinge',{}).get('mode','out_of_plane')
                name='平面内开合' if mode=='in_plane' else '垂直杆面弯折'
                views.append(a['id']+'_hinge_close');labels.append(a['id']+' · '+name)
                legend.append(a['id']+'：'+name)
        with mujoco.Renderer(m,width=1600,height=1000,max_geom=max(5000,m.ngeom+1000)) as renderer:
            for view in views:
                renderer.update_scene(d,camera=view);image=Image.fromarray(renderer.render())
                image.save(target/(view+'.png'));image.save(folder/('preview-'+view+'.png'))
        pictures=''.join(f'<a href="{case["id"]}/{v}.png"><strong>{label}</strong><img loading="lazy" src="{case["id"]}/{v}.png"></a>' for v,label in zip(views,labels))
        cards.append(f'<article><h2>{escape(DESCRIPTIONS[case["level"]])} · {case["arms"]} 臂 · {case["arms"]//4} 根</h2><p>{"；".join(legend)}</p><div class="views">{pictures}</div></article>')
        print(case['id'],flush=True)
    sheet(selected,'top','top-all','无绳空中接力 · 六个新版场景俯视图')
    sheet(selected,'payload_01_weight_close','weights-all','无绳空中接力 · 闭合环与配重近景')
    sheet(selected,'payload_01_close','assemblies-all','无绳空中接力 · 搬运杆近景')
    comparison=Image.new('RGB',(3200,1130),'#eef3f6');draw=ImageDraw.Draw(comparison)
    for col,level in enumerate((2,3)):
        draw.text((col*1600+24,24),DESCRIPTIONS[level],font=ImageFont.truetype(FONT,40),fill='#183345')
        with Image.open(OUT/f'relay-a08-d{level}'/'payload_01_weight_close.png') as src:comparison.paste(src,(col*1600,110))
    comparison.save(OUT/'l2-l3-detail.png')
    hinges=Image.new('RGB',(3200,1130),'#eef3f6');draw=ImageDraw.Draw(hinges)
    for col,(ident,name) in enumerate((('payload_01','杆 1：平面内开合'),('payload_02','杆 2：垂直杆面弯折'))):
        draw.text((col*1600+24,24),name,font=ImageFont.truetype(FONT,42),fill='#183345')
        with Image.open(OUT/'relay-a08-d3'/(ident+'_hinge_close.png')) as src:hinges.paste(src,(col*1600,110))
    hinges.save(OUT/'hinge-directions.png')
    (OUT/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>无绳空中接力 R8</title><style>body{font:18px system-ui;color:#183345;background:#eef3f6;max-width:1700px;margin:30px auto;padding:20px}a{color:#14618a}.views{display:grid;grid-template-columns:1fr 1fr;gap:20px}img{width:100%;display:block}article{background:white;padding:24px;margin:30px 0;border-radius:16px}strong{display:block;margin:12px 0}</style><h1>无绳空中接力 · R8</h1><p>L1 保留。L2：90°弯杆、互扣闭合金属环、600 g 砝码。L3：135°弹性铰接折杆、互扣闭合环、约 120 mm 短硬摆杆和 600 g 砝码。8／12／16 臂分别配 2／3／4 根独立搬运杆。原 Cable 版本保留。</p><p>以下全部是桌面初态的实际 MuJoCo 渲染，未执行机械臂搬运或 Oracle。图片中的弹簧线圈用于表现真实的等效回正铰链，不是柔性 Cable。旧工厂组合图尚未重新生成；每个新场景已包含离线状态回放清单。</p><p><a href="top-all.png">高清俯视合集</a> · <a href="weights-all.png">高清砝码近景合集</a> · <a href="checks.json">被动检查与计时记录</a></p><a href="top-all.png"><img src="top-all-preview.jpg"></a>'+''.join(cards),encoding='utf-8')
    print(OUT/'index.html',flush=True)
    page=OUT/'index.html'
    page.write_text(page.read_text().replace('R8',f'R{REVISION}').replace('<p>L1 保留。','<p>L3 铰链混合配置：8 臂各一根；12 臂平面内一根、垂直两根；16 臂各两根。<a href="hinge-directions.png">查看两种铰链近景对比</a>。</p><p>L1 保留。'),encoding='utf-8')


if __name__=='__main__':main()
