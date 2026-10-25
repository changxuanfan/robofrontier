"""Static MuJoCo layout gallery for an explicitly selected mosaic catalog."""
import argparse
import json
from pathlib import Path

import mujoco
from PIL import Image,ImageDraw,ImageFont

from robofrontier.suite.v2 import ROOT
from robofrontier.run_setup import write_json


def main(catalog,output):
    cases=json.loads(catalog.read_text())['cases'];assert cases and all(c['family']=='mosaic' for c in cases)
    output.mkdir(parents=True,exist_ok=True);revisions={c['scene_revision'] for c in cases};assert len(revisions)==1
    rev=next(iter(revisions));summary=[]
    for case in cases:
        folder=ROOT/case['directory'];env=json.loads((folder/'environment.json').read_text())
        m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));d=mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id);mujoco.mj_forward(m,d)
        m.vis.global_.offwidth=1920;m.vis.global_.offheight=1440
        target=output/case['id'];target.mkdir(exist_ok=True)
        with mujoco.Renderer(m,width=1920,height=1440,max_geom=max(5000,m.ngeom+1000)) as renderer:
            for view in ('top','global','global_rear'):
                renderer.update_scene(d,camera=view);image=Image.fromarray(renderer.render())
                image.save(target/(view+'.png'));image.save(folder/('preview-'+view+'.png'))
        summary.append(dict(case_id=case['id'],moved_stock_count=env['mechanism'].get('stock_layout_revision',{}).get('changed_stock_count',0)))
        print('RENDERED',case['id'],flush=True)
    font=lambda n:ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Unicode.ttf',n)
    scales=sorted({c['arms'] for c in cases})
    for view,title in [('top','俯视布局'),('global','正面斜视'),('global_rear','反面斜视')]:
        sheet=Image.new('RGB',(3000,950),'#edf3f6');draw=ImageDraw.Draw(sheet)
        draw.text((30,18),f'拼图 R{rev} · 供料调整 · {title}',font=font(35),fill='#183245')
        for i,n in enumerate(scales):
            case=next(c for c in cases if c['arms']==n and c['level']==1)
            moved=next(s['moved_stock_count'] for s in summary if s['case_id']==case['id'])
            draw.text((i*1000+25,78),f'{n} 臂 · {case["rows"]}×{case["columns"]} · 调整 {moved} 个供料位置',font=font(27),fill='#183245')
            with Image.open(output/case['id']/(view+'.png')) as im:im.thumbnail((980,770));sheet.paste(im,(i*1000+10,135))
        sheet.save(output/(view+'-overview.png'))
    cards=[]
    for case in cases:
        views=''.join(f'<a href="{case["id"]}/{view}.png"><span>{label}</span><img src="{case["id"]}/{view}.png"></a>' for view,label in [('top','俯视'),('global','正面'),('global_rear','反面')])
        cards.append(f'<article><h2>{case["arms"]} 臂 · L{case["level"]} · {case["rows"]}×{case["columns"]}</h2><div class="views">{views}</div></article>')
    page='<!doctype html><meta charset="utf-8"><title>拼图 R'+str(rev)+' · 供料调整</title><style>body{font:18px system-ui;background:#edf3f6;color:#183245;max-width:1800px;margin:30px auto;padding:0 24px}article{padding:20px;background:white;margin:24px 0;border-radius:12px}.views{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}img{width:100%}a{color:#17678c}span{display:block;padding:10px}</style><h1>拼图 R'+str(rev)+' · 供料调整</h1><p>保留旧版。机械臂布局、桌面、方块颜色/数量/尺寸、目标图案和物理参数保持不变，仅调整部分初始供料位置。静态布局预览，未执行Oracle或GPT操作。</p><p><a href="top-overview.png">高清俯视总图</a> · <a href="global-overview.png">高清正面总图</a> · <a href="coverage/index.html">全量几何覆盖检查</a></p>'+''.join(cards)
    (output/'index.html').write_text(page);write_json(output/'preview-manifest.json',dict(catalog=str(catalog),physics_steps=0,cases=summary))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--catalog',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();main(a.catalog,a.output)
