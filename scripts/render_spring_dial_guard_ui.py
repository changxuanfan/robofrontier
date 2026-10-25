"""Draw a UI concept for a spring dial with one guarded gripper entrance."""
from pathlib import Path
import math
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/spring-dial-guard-ui-20261023/guard-concept.png'
W,H=1800,1120
BG='#F3F0E9'; CARD='#FFFCF6'; INK='#26343D'; MUTED='#65727A'; LINE='#CEC7BC'
RED='#D94E45'; ORANGE='#ED7C32'; BLUE='#3784BF'; GREEN='#37886F'; METAL='#59636B'; PALE='#E9E4DA'
fp='/System/Library/Fonts/STHeiti Medium.ttc'
def f(n): return ImageFont.truetype(fp,n)
im=Image.new('RGB',(W,H),BG); d=ImageDraw.Draw(im)
def txt(x,y,s,n=24,c=INK,a='la'): d.text((x,y),s,font=f(n),fill=c,anchor=a)
def rr(b,fill=CARD,out=LINE,r=20,w=2): d.rounded_rectangle(b,radius=r,fill=fill,outline=out,width=w)
def arrow(a,b,c=INK,w=6,head=16):
    t=math.atan2(b[1]-a[1],b[0]-a[0]); d.line((a,b),fill=c,width=w)
    d.polygon([b,(b[0]-head*math.cos(t-.55),b[1]-head*math.sin(t-.55)),
               (b[0]-head*math.cos(t+.55),b[1]-head*math.sin(t+.55))],fill=c)

txt(62,38,'单侧开口护圈：只从开放侧接近旋钮',46)
txt(66,102,'任务规则不变；护圈遮住其他方向，夹爪需判断入口、对准并从侧面抓取。',24,MUTED)
rr((52,158,1125,1005)); rr((1152,158,1748,1005))
txt(90,185,'俯视图',30)

# Main top-down scene
cx,cy,R=585,580,300
# Work surface / board
d.rounded_rectangle((150,260,1020,920),radius=34,fill='#B98F70',outline='#927057',width=4)
# Smooth guard arc with a 84-degree opening on the right.
ring=[(cx+R*math.cos(math.radians(deg)),cy+R*math.sin(math.radians(deg)))
      for deg in range(42,319)]
d.line(ring,fill=METAL,width=22,joint='curve')
# Ring end caps
for deg in (42,318):
    a=math.radians(deg); px=cx+R*math.cos(a); py=cy+R*math.sin(a)
    d.ellipse((px-13,py-13,px+13,py+13),fill='#424A50',outline='white',width=2)
# Dial stationary base/face
d.ellipse((cx-175,cy-175,cx+175,cy+175),fill='#DDCBBB',outline='#846A57',width=5)
d.ellipse((cx-151,cy-151,cx+151,cy+151),fill='#FFFDF8',outline='#C9BDAE',width=3)
for deg,label in [(270,'0°'),(90,'180°'),(180,'270°')]:
    a=math.radians(deg)
    x1,y1=cx+120*math.cos(a),cy+120*math.sin(a)
    x2,y2=cx+140*math.cos(a),cy+140*math.sin(a)
    d.line((x1,y1,x2,y2),fill=INK,width=5)
    lx,ly=cx+185*math.cos(a),cy+185*math.sin(a)
    txt(lx,ly,label,25,INK,'mm')
# Place the east-facing 90-degree numeral below the jaw path.
a=math.radians(0)
d.line((cx+120,cy,cx+140,cy),fill=INK,width=5)
txt(cx+133,cy+92,'90°',25,INK,'mm')
# red knob and orange pointer
d.rounded_rectangle((cx-35,cy-35,cx+35,cy+35),radius=14,fill=RED,outline='white',width=3)
d.rounded_rectangle((cx-8,cy-115,cx+8,cy-34),radius=7,fill=ORANGE)
# allowed approach: gripper fingers straddling knob from right through opening
rr((875,cy-62,994,cy+62),fill='#E9F0F5',out=BLUE,r=16,w=4)
txt(935,cy,'夹爪',23,BLUE,'mm')
d.rounded_rectangle((cx+70,cy-43,875,cy-27),radius=7,fill=BLUE)
d.rounded_rectangle((cx+70,cy+27,875,cy+43),radius=7,fill=BLUE)
arrow((870,cy),(cx+78,cy),BLUE,7,18)
txt(840,cy-108,'唯一入口',25,BLUE,'mm')
# blocked approaches and X marks
for a in (90,180,270):
    t=math.radians(a); start=(cx+(R+85)*math.cos(t),cy+(R+85)*math.sin(t))
    end=(cx+(R+18)*math.cos(t),cy+(R+18)*math.sin(t))
    arrow(start,end,RED,5,13)
    mx,my=cx+(R+5)*math.cos(t),cy+(R+5)*math.sin(t)
    d.line((mx-12,my-12,mx+12,my+12),fill=RED,width=6)
    d.line((mx-12,my+12,mx+12,my-12),fill=RED,width=6)
txt(360,270,'不可从上方压下',21,RED)
txt(186,585,'侧面被挡',20,RED)
# Side / entrance cross-section
txt(1190,185,'从入口看的侧视图',30)
sx,sy=1450,470
d.rectangle((1225,sy+140,1690,sy+184),fill='#B98F70',outline='#927057',width=3)
# dial disk / knob
d.rectangle((sx-125,sy+112,sx+125,sy+140),fill='#DDCBBB',outline='#846A57',width=3)
d.rectangle((sx-32,sy+15,sx+32,sy+112),fill=RED,outline='white',width=3)
d.rectangle((sx-8,sy-18,sx+8,sy+52),fill=ORANGE)
# surrounding walls higher than knob, with entrance on near/front side shown by gap
d.rectangle((sx-170,sy-28,sx-144,sy+142),fill=METAL)
d.rectangle((sx+144,sy-28,sx+170,sy+142),fill=METAL)
d.rectangle((sx-170,sy-28,sx+170,sy-3),fill=METAL)
# side entering jaw tips, shown through front gap
d.rounded_rectangle((sx-85,sy+47,sx-20,sy+62),radius=6,fill=BLUE)
d.rounded_rectangle((sx+20,sy+47,sx+85,sy+62),radius=6,fill=BLUE)
arrow((sx,sy+260),(sx,sy+75),BLUE,7,18)
txt(1460,sy+276,'从开放侧水平进入',22,BLUE,'mm')
txt(1200,sy+360,'侧墙高过旋钮顶部，限制从上方抓取。',22,INK)
txt(1200,sy+404,'夹爪必须穿过入口，再闭合夹持。',22,INK)
rr((1190,sy+390,1710,sy+530),fill='#F4EFE5',out='#DDD4C7',r=14,w=2)
txt(1220,sy+405,'主要测量',22,INK)
txt(1220,sy+445,'抓取成功 · 重试次数 · 目标角度误差',19,MUTED)
txt(1220,sy+484,'到位耗时 · 接近路径 · 滑脱',19,MUTED)

txt(62,1040,'示意 UI：护圈为概念几何。入口宽度、夹爪尺寸、全臂可达性及旋转行程仍需在 MuJoCo 中验证。',20,MUTED)
im.save(OUT); print(OUT)
