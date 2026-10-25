"""Actual static MuJoCo views and a four-family gallery; no physics stepping."""
import argparse,html,json
from pathlib import Path
import mujoco
from PIL import Image,ImageDraw,ImageFont
from robofrontier.suite.v2 import ROOT,cases,directory

OUT=ROOT/'docs/generated/frontier-v2'
LABELS={'transport':('物品传递',['普通放置','固定白板插座','弹簧白板插座']),
        'mosaic':('郁金香拼图',['实体孔槽','画线网格','无线桌面']),
        'relay':('空中接力',['红色直杆','90°弯杆＋闭合环砝码','弹性折杆＋短硬摆杆砝码']),
        'dial':('旋钮密码锁',['固定齿轮','浮空齿轮','浮空手摇柄'])}

def font(n):return ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Unicode.ttf',n)

def render(case,top_only=False):
    folder=directory(case);m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id);mujoco.mj_forward(m,d)
    m.vis.global_.offwidth=1920;m.vis.global_.offheight=1200
    close='board_A_close' if case['family']=='transport' else 'assembly_01' if case['family']=='mosaic' else 'payload_01_close' if case['family']=='relay' else 'D01_face'
    views=['top'] if top_only else ['top','global','global_rear',close];(OUT/'images'/case['id']).mkdir(parents=True,exist_ok=True)
    if not top_only and case['family']=='relay':
        views+=['payload_'+str(i).zfill(2)+'_close' for i in range(2,case['arms']//4+1)]
        if case['level']==3:views+=['payload_01_hinge_close' if case.get('scene_revision',1)>=8 else 'tray_close']
        if case.get('scene_revision',1)>=8 and case['level']>1:views+=['payload_01_weight_close']
    if not top_only and case['family']=='transport' and case['level']>=2:views+=['plug_close','socket_close']
    with mujoco.Renderer(m,width=1920,height=1200,max_geom=max(5000,m.ngeom+1000)) as r:
        for view in views:
            r.update_scene(d,camera=view);im=Image.fromarray(r.render());im.save(folder/('preview-'+view+'.png'))
            im.save(OUT/'images'/case['id']/(view+'.jpg'),quality=94,subsampling=0)
    if (folder/'target-pattern.png').exists():
        with Image.open(folder/'target-pattern.png') as im:im.save(OUT/'images'/case['id']/'target.png')
    print(case['id'],flush=True)
    return close


def top_sheet(family,group,detail_views=None):
    cw,ch=1952,1344;gap=24;margin=36;header=160
    im=Image.new('RGB',(2*margin+3*cw+2*gap,header+3*ch+2*gap+36),'#edf3f6');draw=ImageDraw.Draw(im)
    draw.text((margin,20),LABELS[family][0]+(' · 近距离观察' if detail_views else ' · 俯视图'),font=font(60),fill='#173044')
    draw.text((margin,100),'行：L1 → L3    列：规模递增',font=font(34),fill='#3b5969')
    scales=sorted({c.get('dials',c['arms']) for c in group})
    for row in range(3):
        for col,scale in enumerate(scales):
            c=next(c for c in group if c['level']==row+1 and c.get('dials',c['arms'])==scale)
            x=margin+col*(cw+gap);y=header+row*(ch+gap)
            draw.rounded_rectangle((x,y,x+cw,y+ch),radius=18,fill='white')
            desc=f'{c["arms"]} 臂'
            if family=='mosaic':desc+=f' · {c["rows"]}×{c["columns"]} · 四色'
            if family=='dial':desc+=f' · {c["dials"]} 盘'
            if family=='relay':desc+=f' · {c["arms"]//4} 根'
            draw.text((x+16,y+10),f'L{row+1} {LABELS[family][1][row]} | {desc}',font=font(34),fill='#173044')
            draw.text((x+16,y+58),c['id'],font=font(28),fill='#536b7a')
            view=detail_views[c['id']] if detail_views else 'top'
            with Image.open(directory(c)/('preview-'+view+'.png')) as src:im.paste(src,(x+16,y+112))
    suffix='close' if detail_views else 'top'
    im.save(OUT/(family+('-close-only.png' if detail_views else '-top-only.png')),optimize=True)
    im.thumbnail((1800,1500));im.save(OUT/(family+'-'+suffix+'-overview.jpg'),quality=94)
    print('TOP SHEET',family,flush=True)

def sheet(family,group):
    cw,ch=1952,2656;gap=24;margin=36;header=160;size=(3*cw+2*gap+2*margin,3*ch+2*gap+header+60)
    im=Image.new('RGB',size,'#edf3f6');draw=ImageDraw.Draw(im)
    draw.text((margin,20),LABELS[family][0]+' · 新版九项环境',font=font(60),fill='#173044')
    draw.text((margin,100),'行：L1 → L3    列：规模递增    每项上方俯视、下方正面 · 静态初态，未运行 Oracle',font=font(30),fill='#3b5969')
    scales=sorted(set(c.get('dials',c['arms']) for c in group))
    for row in range(3):
        for col,scale in enumerate(scales):
            c=next(c for c in group if c['level']==row+1 and c.get('dials',c['arms'])==scale);x=margin+col*(cw+gap);y=header+row*(ch+gap)
            draw.rounded_rectangle((x,y,x+cw,y+ch),radius=18,fill='white')
            desc=f'{c["arms"]} 臂'+(f' · {c["rows"]}×{c["columns"]} · {c["colors"]} 色' if family=='mosaic' else f' · {c["dials"]} 盘' if family=='dial' else '')
            draw.text((x+16,y+10),f'L{row+1} {LABELS[family][1][row]} | {desc}',font=font(34),fill='#173044')
            draw.text((x+16,y+60),c['id'],font=font(28),fill='#536b7a')
            for k,view in enumerate(['top','global']):
                yy=y+112+k*1260;draw.text((x+16,yy),'TOP 俯视' if view=='top' else 'FRONT 正面',font=font(28),fill='#173044')
                with Image.open(directory(c)/('preview-'+view+'.png')) as src:im.paste(src,(x+16,yy+40))
    im.save(OUT/(family+'-top-front.png'),optimize=True);thumb=im.copy();thumb.thumbnail((1500,2100));thumb.save(OUT/(family+'-overview.jpg'),quality=92)
    print('SHEET',family,im.size,flush=True)

def main(top_only=False):
    OUT.mkdir(parents=True,exist_ok=True);data=cases();details={}
    for c in data:details[c['id']]=render(c,top_only)
    sections=[]
    for family in LABELS:
        group=[c for c in data if c['family']==family]
        top_sheet(family,group)
        if not top_only and family=='relay':top_sheet(family,group,details)
        if not top_only:sheet(family,group)
        cards=[]
        for c in group:
            prefix='images/'+c['id']+'/';views=[('top','俯视')] if top_only else [('top','俯视'),('global','正面'),('global_rear','背面'),(details[c['id']],'装置近景')]
            if not top_only and family=='relay':views += [('payload_'+str(i).zfill(2)+'_close',f'木杆 {i} 近景') for i in range(2,c['arms']//4+1)]
            if not top_only and family=='transport' and c['level']>=2:views += [('plug_close','插片近景'),('socket_close','插孔近景')]
            pictures=''.join(f'<a href="{prefix}{v}.jpg" target="_blank"><span>{label}</span><img loading="lazy" src="{prefix}{v}.jpg"></a>' for v,label in views)
            target=f'<a href="{prefix}target.png">郁金香目标图</a>' if family=='mosaic' else ''
            cards.append(f'<article><h3>{c["id"]} · {c["arms"]} 臂 · L{c["level"]}</h3><p>工程初态预览，尚未进行完整操作验证。{target}</p><div class="views">{pictures}</div></article>')
        targets=''
        if family=='mosaic':
            targets='<div class="targets">'+''.join(f'<a href="images/{c["id"]}/target.png"><strong>{c["arms"]} 臂 · {c["rows"]}×{c["columns"]} · 四色目标图</strong><img src="images/{c["id"]}/target.png"></a>' for c in group if c['level']==1)+'</div>'
        sections.append(f'<section id="{family}"><h2>{LABELS[family][0]}</h2>{targets}<p><a href="{family}-top-only.png">查看九项环境俯视高清大图</a></p><a href="{family}-top-only.png"><img class="overview" src="{family}-top-overview.jpg"></a><details><summary>展开九个环境的独立高清视图</summary>{"".join(cards)}</details></section>')
    (OUT/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>RoboFrontier v2 · 36 个简单背景环境</title><style>body{font:17px system-ui;background:#edf3f6;color:#173044;max-width:1500px;margin:30px auto;padding:0 24px}nav{position:sticky;top:0;background:#edf3f6;padding:16px 0}a{color:#185e8c}nav a{margin-right:24px}.overview{max-width:100%;max-height:1100px;object-fit:contain;object-position:left}article{padding:20px;background:white;margin:20px 0;border-radius:12px}.views{display:grid;grid-template-columns:1fr 1fr;gap:16px}.views img,.targets img{width:100%}.views span{display:block}.targets{display:grid;grid-template-columns:1fr;gap:24px;max-width:1150px}.targets strong{display:block;padding:12px 0}summary{cursor:pointer;font-size:22px;padding:20px}section{margin-top:40px}</style><h1>RoboFrontier v2 · 36 个简单背景环境</h1><p>60 mm 方块／圆柱直径，80 mm 拼图孔口，81 mm 节距。拼图统一四色：6×12／9×18／12×24。接力 8／12／16 臂，最高档外 10 内 6。全部为真实 MuJoCo 静态渲染；未启动 GPT、完整 Oracle 或工厂迁移。点击大图放大，展开查看每项独立视图。</p><nav>'+''.join(f'<a href="#{f}">{v[0]}</a>' for f,v in LABELS.items())+'</nav>'+''.join(sections))
    print(OUT/'index.html',flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--top-only',action='store_true');args=parser.parse_args()
    main(args.top_only)
