"""Draw a conventional multi-arm pick-and-deliver tabletop task."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/task-family-sketch-20261023/multiarm-delivery.png'
W,H=1700,1050
BG='#F3F0E9'; CARD='#FFFCF6'; INK='#26343D'; MUTED='#65727A'; LINE='#CEC7BC'
COLORS={'Blue':'#3784BF','Red':'#D94E45','Green':'#37886F','Purple':'#7962A1'}
fp='/System/Library/Fonts/STHeiti Medium.ttc'
def f(s):return ImageFont.truetype(fp,s)
im=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(im)
def text(x,y,s,n=24,c=INK,a='la'):d.text((x,y),s,font=f(n),fill=c,anchor=a)
def roundbox(b,fill=CARD,out=LINE,r=18,w=2):d.rounded_rectangle(b,radius=r,fill=fill,outline=out,width=w)
def arrow(a,b,c=INK,w=6,head=15,dash=False):
 import math
 if dash:
  dx,dy=b[0]-a[0],b[1]-a[1]; length=math.hypot(dx,dy); ux,uy=dx/length,dy/length
  t=0
  while t<length-head:
   end=min(t+18,length-head)
   d.line((a[0]+ux*t,a[1]+uy*t,a[0]+ux*end,a[1]+uy*end),fill=c,width=w)
   t+=30
 else:d.line((a,b),fill=c,width=w)
 q=math.atan2(b[1]-a[1],b[0]-a[0])
 d.polygon([b,(b[0]-head*math.cos(q-.55),b[1]-head*math.sin(q-.55)),
              (b[0]-head*math.cos(q+.55),b[1]-head*math.sin(q+.55))],fill=c)

text(64,38,'典型多臂任务：多订单取放（MAPD）',46)
text(68,101,'一条清楚的目标：把桌上的每个方块送到对应颜色的目标区。',25,MUTED)
roundbox((55,155,1195,975))
roundbox((1230,155,1645,975))

# Shared table
tx,ty,tw,th=190,280,850,560
d.rounded_rectangle((tx,ty,tx+tw,ty+th),radius=28,fill='#B78A69',outline='#8E674D',width=5)
# Four fixed robot stations around the same table; no task-imposed color ownership.
stations=[('A',(tx+tw/2,ty-45),COLORS['Blue']),
          ('B',(tx+tw+48,ty+th/2),COLORS['Red']),
          ('C',(tx+tw/2,ty+th+48),COLORS['Green']),
          ('D',(tx-48,ty+th/2),COLORS['Purple'])]
for label,(x,y),col in stations:
 d.ellipse((x-29,y-29,x+29,y+29),fill=col,outline='white',width=4)
 text(x,y,label,26,'white','mm')

items=[
 ('Blue',(tx+140,ty+130),(tx+560,ty+480)),
 ('Red',(tx+680,ty+140),(tx+250,ty+475)),
 ('Green',(tx+145,ty+435),(tx+540,ty+290)),
 ('Purple',(tx+675,ty+430),(tx+320,ty+145)),
]
for name,(sx,sy),(gx,gy) in items:
 col=COLORS[name]
 # Candidate routes are conceptual; intersections represent ordinary path planning.
 mid=((sx+gx)//2,(sy+gy)//2)
 arrow((sx,sy),mid,col,4,12,True); arrow(mid,(gx,gy),col,4,12,True)
 # Source cube
 d.rounded_rectangle((sx-27,sy-27,sx+27,sy+27),radius=8,fill=col,outline='white',width=3)
 # Matching marked destination
 d.ellipse((gx-30,gy-30,gx+30,gy+30),fill='#FFF9EF',outline=col,width=5)
 text(gx,gy,'G',20,col,'mm')

# Small rule / decision / architecture panel
text(1270,197,'任务规则',28)
text(1270,247,'每个方块放入同色目标区。',21)
text(1270,300,'不指定哪个机械臂搬哪个方块。',19,MUTED)
text(1270,365,'Agent 决定',28)
for i,s in enumerate(['任务分配与执行次序',
                       '哪些路线可并行',
                       '路径相遇时如何错开',
                       '是否需要彼此发消息']):
 text(1280,410+i*42,'• '+s,20)
text(1270,610,'可比较的组织方式',26)
for i,s in enumerate(['一个 Agent 调度四臂',
                       'Coordinator + Workers',
                       'Worker 之间协商',
                       '按需 Adaptive fork']):
 text(1280,656+i*42,'• '+s,19)
text(1270,850,'指标：成功率、完成时间、',20,'#38765F')
text(1270,887,'动作数、并行重叠、等待与通信',19,'#38765F')
text(62,1000,'示意草图。虚线是候选移动路径，不表示已验证的可达性或无碰撞轨迹。',20,MUTED)

im.save(OUT);print(OUT)
