"""Static scene gallery and explicitly illustrative BEST/READY states."""
from pathlib import Path
import html
import json
import shutil
from types import SimpleNamespace


def render_display_examples():
    """Visual-only state samples, never a physical puzzle success record."""
    import mujoco
    from PIL import Image,ImageDraw
    from robofrontier.suite import v2
    from robofrontier.suite.dial_feedback import paint_display
    src=ROOT/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n08-d1'
    model=mujoco.MjModel.from_xml_path(str(src/'scene.xml'));data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
    sheet=Image.new('RGB',(1800,1000),'#edf1f3');draw=ImageDraw.Draw(sheet)
    draw.text((30,18),'BEST 0-8 / DIM vs READY - visual states only',font=v2.font(30),fill='#172d3f')
    with mujoco.Renderer(model,width=1280,height=800) as renderer:
        for value in range(9):
            x=(value%3)*600;y=65+(value//3)*310
            for ready in (False,True):
                paint_display(model,SimpleNamespace(display=value,ready=ready,indicator='ready' if ready else 'invalid'))
                renderer.update_scene(data,camera='board')
                panel=Image.fromarray(renderer.render()).crop((340,168,940,634)).resize((280,218),Image.Resampling.LANCZOS)
                px=x+10+int(ready)*295
                draw.text((px,y),f'{value} / '+('READY' if ready else 'NOT READY'),font=v2.font(22),fill='#172d3f')
                sheet.paste(panel,(px,y+36))
    sheet.save(OUT/'digit-states.png')


def render_task_overviews(cases):
    """Keep every original view available; montages only arrange existing RGB."""
    from PIL import Image,ImageDraw
    from robofrontier.suite import v2
    for view in ('top','global','global_rear'):
        sheet=Image.new('RGB',(3840,2700),'#f1f4f5');draw=ImageDraw.Draw(sheet)
        draw.text((28,12),f'BEST5 R11 / {view.upper()} / INITIAL SCENES - NO ORACLE',font=v2.font(38),fill='#172d3f')
        for case in cases:
            x=(case['level']-1)*1280;y=80+case['scale_index']*870
            label=f"{case['dials']} DIALS / {case['arms']} ARMS / L{case['level']}"
            draw.text((x+24,y+6),label,font=v2.font(34),fill='#172d3f')
            with Image.open(ROOT/case['directory']/f'preview-{view}.png') as frame:
                if frame.size!=(1280,800):raise ValueError('Unexpected preview size')
                sheet.paste(frame,(x,y+55))
        sheet.save(OUT/f'all-tasks-{view}.png')

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'docs/generated/frontier-v2/best5-dials-r11';OUT.mkdir(parents=True,exist_ok=True)
render_display_examples()
catalog=json.loads((ROOT/'configs/suites/dial-best5-r11.json').read_text());cards=[]
render_task_overviews(catalog['cases'])
for c in catalog['cases']:
    src=ROOT/c['directory'];dest=OUT/c['id'];dest.mkdir(exist_ok=True)
    for name in ['preview-global.png','preview-global_rear.png','preview-top.png','preview-board.png','preview-D01_face.png','public-task.json']:
        shutil.copyfile(src/name,dest/name)
    links=''.join(f'<a href="{c["id"]}/preview-{view}.png"><b>{label}</b><img loading="lazy" src="{c["id"]}/preview-{view}.png"></a>' for view,label in [('global','全景'),('global_rear','背面'),('top','俯视'),('board','BEST / READY 面板'),('D01_face','0～5刻度近景')])
    cards.append(f'<details><summary>{c["dials"]}盘 · L{c["level"]} · {c["arms"]}臂</summary><div class="views">{links}</div><a href="{c["id"]}/public-task.json">公开任务说明</a></details>')
style='''body{font:17px system-ui;background:#f1f4f5;color:#172d3f;line-height:1.7}main{max-width:1450px;margin:30px auto;padding:0 24px}h1{font-size:34px}.examples{display:grid;grid-template-columns:repeat(3,1fr);gap:20px}.device{background:#fffdf6;border:2px solid #d4d0c2;border-radius:14px;padding:24px;text-align:center}.device>strong{font-size:30px}.ready{font-size:27px;font-weight:700}.number{font:96px ui-monospace,monospace;color:#aa2316;line-height:1.4}.number.dim{color:#514540}.lamp{display:inline-block;width:36px;height:36px;border-radius:50%;margin-right:12px;vertical-align:middle}.off{background:#78838a}.amber{background:#f6a222}.green{background:#0bb64a}.note{background:#fff3d8;padding:18px;border-left:4px solid #d39a35}.views{display:grid;grid-template-columns:1fr 1fr;gap:18px}.views img{width:100%}details{background:white;margin:20px 0;padding:20px;border-radius:12px}summary{cursor:pointer;font-size:22px}a{color:#08638c}b{display:block}figure img{max-width:100%}@media(max-width:800px){.examples,.views{grid-template-columns:1fr}}'''
examples=''
for color,title,body in [('off','当前无效','存在0位、偏离有效刻度或未保持；纪录仍是3。'),('amber','正在稳定','全部满足输入条件，尚未保持满0.25秒。'),('green','输入就绪','当前已形成有效输入；不代表有3个正确。')]:
    examples+=f'<div class="device"><strong>BEST / 6</strong><div class="number {"" if color=="green" else "dim"}">3</div><div class="ready"><span class="lamp {color}"></span>READY</div><h3>{title}</h3><p>{body}</p></div>'
overview=''.join(f'<figure><a href="all-tasks-{view}.png"><img loading="lazy" src="all-tasks-{view}.png"></a><figcaption>{label}，3840×2700，点击原图。行：4／6／8盘；列：L1／L2／L3。均为初始场景。</figcaption></figure>' for view,label in [('top','九任务俯视'),('global','九任务正面全景'),('global_rear','九任务背面全景')])
page=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>五档有效 · BEST密码锁</title><style>{style}</style><main><h1>0～5刻度 · BEST密码锁新版本</h1><p>九个环境：4／6／8盘 × L1／L2／L3。六个刻度间隔60°，1～5为密码选项，0无效。BEST初始0，之后保留本轮最高纪录；READY单独显示当前输入是否有效。未就绪时数字变暗但仍可读，绿灯就绪后数字亮起，历史值不会因亮度变化而降低。</p>{overview}<p class="note">以下三个面板是显示逻辑示意，不是物理操作结果。下方九个场景是实际MuJoCo初态静态渲染；旧版保留。本轮没有运行Oracle，未证明五个新角度全部可操作。</p><div class="examples">{examples}</div><figure><img src="digit-states.png"><figcaption>0～8 实际 MuJoCo 数字段的明暗显示检查；仅展示面板状态，不是物理解谜结果。</figcaption></figure><p>READY是整组就绪提示，不提供哪位密码正确，也不指出哪只指针需要调整。绿灯仍可能对应0个正确；找出未对齐或未保持的部分要依据指针和机器人观测。有效中间组合也会自动更新纪录，不能把历史BEST当作当前组合的得分。</p><figure><img src="dial-best5-n04-d1/preview-board.png"><figcaption>真实静态初态：BEST0，READY灰；指针均在无效0位。</figcaption></figure><figure><img src="dial-best5-n04-d1/preview-D01_face.png"><figcaption>新面盘：0、1、2、3、4、5。</figcaption></figure>{''.join(cards)}</main></html>'''
(OUT/'index.html').write_text(page)
print(OUT/'index.html')
