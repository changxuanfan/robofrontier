"""Render conceptual sketches of three long-horizon robot task families."""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'runs/task-family-sketch-20261023/task-families.png'
W, H = 1800, 1450
BG = '#F5F1EA'; INK = '#24313A'; MUTED = '#66727A'; CARD = '#FFFCF6'
BLUE = '#3E83C4'; RED = '#D94C45'; GREEN = '#328A70'; ORANGE = '#E98B3C'
PURPLE = '#7F66A9'; LINE = '#CFC8BD'; PALE = '#E8E1D6'

font_path = '/System/Library/Fonts/STHeiti Medium.ttc'
def font(size): return ImageFont.truetype(font_path, size)

im = Image.new('RGB', (W, H), BG); d = ImageDraw.Draw(im)

def text(x, y, s, size, fill=INK, anchor=None):
    d.text((x,y), s, font=font(size), fill=fill, anchor=anchor)

def rounded(box, radius=18, fill=CARD, outline=LINE, width=2):
    d.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)

def arrow(a, b, color=INK, width=5, head=14):
    d.line([a,b], fill=color, width=width)
    import math
    ang=math.atan2(b[1]-a[1],b[0]-a[0])
    p1=(b[0]-head*math.cos(ang-.55),b[1]-head*math.sin(ang-.55))
    p2=(b[0]-head*math.cos(ang+.55),b[1]-head*math.sin(ang+.55))
    d.polygon([b,p1,p2], fill=color)

def cube(cx, cy, label, color, size=64):
    d.rounded_rectangle((cx-size//2,cy-size//2,cx+size//2,cy+size//2),
                        radius=10, fill=color, outline='#FFFFFF', width=3)
    text(cx,cy,label,30,'white','mm')

def slot(cx,cy,label=None,filled=False):
    d.rounded_rectangle((cx-40,cy-38,cx+40,cy+38), radius=9,
                        fill='#EDE7DD' if not filled else '#FFF9EF', outline='#AFA79A', width=3)
    if label: text(cx,cy,label,28,MUTED,'mm')

# Title
text(70,42,'长程推理任务：动作简单，计划有依赖',54,INK)
text(74,112,'示意图是任务概念，不是已搭建或已验证的 MuJoCo 场景。',26,MUTED)

rows=[(175,550),(590,960),(1000,1370)]

# 1. One-buffer rearrangement
y0,y1=rows[0]
rounded((55,y0,1745,y1))
text(90,y0+22,'1  有限缓冲位重排',36,BLUE)
text(90,y0+75,'原子动作：抓取 / 移动 / 放置      长程难点：先移谁，才不会把目标位堵住？',25,MUTED)
text(115,y0+135,'初始',25,INK)
for i,(lab,col) in enumerate([('B',BLUE),('C',GREEN),('A',RED)]): cube(205+i*98,y0+222,lab,col,62)
text(160,y0+278,'目标位  1     2     3',20,MUTED)
text(510,y0+135,'只有一个临时缓冲位',23,INK)
slot(575,y0+222,'空')
text(535,y0+278,'BUFFER',18,MUTED)
arrow((680,y0+222),(820,y0+222),BLUE,4,13)
text(750,y0+188,'规划暂存顺序',20,BLUE,'mm')
text(865,y0+135,'可行步骤',25,INK)
steps=['1  A → 缓冲位','2  C → 目标位 3','3  B → 目标位 2','4  A → 目标位 1']
for i,s in enumerate(steps):
    rounded((860+i*205,y0+185,1048+i*205,y0+270),12,'#F1F5F8','#D5DEE4',1)
    text(954+i*205,y0+227,s,19,INK,'mm')
text(870,y0+326,'目标：',25,GREEN)
for i,(lab,col) in enumerate([('A',RED),('B',BLUE),('C',GREEN)]): cube(1015+i*95,y0+328,lab,col,50)

# 2. Rotate gates then push along a constrained route
y0,y1=rows[1]
rounded((55,y0,1745,y1))
text(90,y0+22,'2  旋转通道，再推送物体',36,PURPLE)
text(90,y0+75,'原子动作：旋转 / 推      长程难点：先把通道片转到连通状态，再决定推送路线。',25,MUTED)
text(115,y0+140,'起点',23,INK,'mm'); text(1590,y0+140,'终点',23,INK,'mm')
# Track cells and gates
cy=y0+245
for i,x in enumerate([245,475,705,935,1165,1395]):
    rounded((x-70,cy-56,x+70,cy+56),12,'#F3EEE5','#B9B0A2',3)
    if i<5: arrow((x+72,cy),(x+152,cy),LINE,5,13)
# gates with rotating bar symbols
for x,angle in [(475,0),(935,90),(1165,0)]:
    import math
    rad=math.radians(angle)
    dx,dy=34*math.cos(rad),34*math.sin(rad)
    d.line((x-dx,cy-dy,x+dx,cy+dy), fill=PURPLE, width=10)
    d.ellipse((x-8,cy-8,x+8,cy+8),fill=INK)
# Mark the currently closed middle gate. The grey arrows show the desired route
# after rotation, not an already-passable path.
d.line((912,cy-21,958,cy+21),fill=RED,width=7)
d.line((958,cy-21,912,cy+21),fill=RED,width=7)
# puck
d.ellipse((190,cy-25,240,cy+25),fill=ORANGE,outline='white',width=3)
text(210,cy,'●',23,'white','mm')
text(475,y0+321,'通道片可转',22,PURPLE,'mm')
text(935,y0+321,'当前未连通 ×',22,RED,'mm')
text(1175,y0+321,'转正后沿槽推至终点',22,PURPLE,'mm')

# 3. One shared tool across ordered jobs / arms
y0,y1=rows[2]
rounded((55,y0,1745,y1))
text(90,y0+22,'3  共享工具与多订单',36,GREEN)
text(90,y0+75,'原子动作：抓取 / 移动 / 旋转      长程难点：一个工具服务多个目标，顺序和交接会互相影响。',25,MUTED)
# Three stations / robots
stations=[340,875,1410]
for name,x in zip('ABC',stations):
    rounded((x-105,y0+142,x+105,y0+294),18,'#EFF3F4','#BCC8CC',2)
    text(x,y0+166,'机械臂 '+name,24,INK,'mm')
    d.arc((x-38,y0+205,x+38,y0+281),180,360,fill=INK,width=8)
    d.line((x,y0+243,x,y0+210),fill=INK,width=8)
    d.ellipse((x-10,y0+197,x+10,y0+217),fill=INK)
# Shared tool and handoff path
rounded((stations[0]-32,y0+228,stations[0]+32,y0+272),12,'#F4D494','#B68A39',2)
text(stations[0],y0+250,'钥匙',20,INK,'mm')
arrow((stations[0]+125,y0+220),(stations[1]-125,y0+220),ORANGE,5,14)
arrow((stations[1]+125,y0+220),(stations[2]-125,y0+220),ORANGE,5,14)
text(610,y0+195,'交接',18,ORANGE,'mm'); text(1145,y0+195,'交接',18,ORANGE,'mm')
# Order dependencies
rounded((215,y0+320,750,y0+354),12,'#F8F4EB','#E1D8C8',1)
text(482,y0+337,'订单 1：A 使用钥匙后交给 B',20,INK,'mm')
rounded((795,y0+320,1340,y0+354),12,'#F8F4EB','#E1D8C8',1)
text(1067,y0+337,'订单 2：B 完成后，C 才能使用',20,INK,'mm')

im.save(OUT)
print(OUT)
