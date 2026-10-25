"""Draw four canonical puzzle templates as possible robot task families."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/task-family-sketch-20261023/classic-task-families.png'
W,H=1800,1260
BG='#F3F0E9'; CARD='#FFFCF6'; INK='#26343D'; MUTED='#65727A'
BLUE='#3784BF'; RED='#D94E45'; GREEN='#37886F'; GOLD='#E5A53A'; PURPLE='#7962A1'; LINE='#CFC8BC'
fp='/System/Library/Fonts/STHeiti Medium.ttc'
def f(n): return ImageFont.truetype(fp,n)
im=Image.new('RGB',(W,H),BG); d=ImageDraw.Draw(im)

def txt(x,y,s,n=24,fill=INK,anchor='la'):
    d.text((x,y),s,font=f(n),fill=fill,anchor=anchor)
def box(x1,y1,x2,y2,fill=CARD,outline=LINE,r=20,w=2):
    d.rounded_rectangle((x1,y1,x2,y2),radius=r,fill=fill,outline=outline,width=w)
def arrow(a,b,color=INK,width=5,head=13):
    import math
    d.line((a,b),fill=color,width=width)
    q=math.atan2(b[1]-a[1],b[0]-a[0])
    d.polygon([b,(b[0]-head*math.cos(q-.55),b[1]-head*math.sin(q-.55)),
               (b[0]-head*math.cos(q+.55),b[1]-head*math.sin(q+.55))],fill=color)

txt(66,45,'经典任务模板：动作原语简单，长程结构清楚',50)
txt(70,110,'可映射到抓取、放置、移动、旋转或推移；图示是概念草图，不是已搭建的仿真。',24,MUTED)
cards=[(55,165,875,660),(925,165,1745,660),(55,695,875,1190),(925,695,1745,1190)]
for c in cards: box(*c)

# Sokoban
x1,y1,x2,y2=cards[0]
txt(x1+32,y1+24,'1  推箱子 · Sokoban',34,BLUE)
txt(x1+34,y1+72,'只推不拉；推错位置可能形成死局。',23,MUTED)
gx,gy=115,290; cell=74
grid=[['#','#','#','#','#','#'],['#','.','.','.','.','#'],['#','.','@','$','.','#'],['#','.','.','.','G','#'],['#','#','#','#','#','#']]
for r,row in enumerate(grid):
  for c,ch in enumerate(row):
    xx,yy=gx+c*cell,gy+r*cell
    fill='#D9D5CE' if ch=='#' else '#F0ECE4'
    d.rectangle((xx,yy,xx+cell-3,yy+cell-3),fill=fill,outline='#C7C0B4',width=2)
    if ch=='G':
      d.ellipse((xx+18,yy+18,xx+54,yy+54),fill='#D5EADD',outline=GREEN,width=4)
      txt(xx+36,yy+36,'G',19,GREEN,'mm')
    elif ch=='$':
      d.rounded_rectangle((xx+14,yy+14,xx+57,yy+57),radius=7,fill=GOLD,outline='white',width=3)
    elif ch=='@':
      d.ellipse((xx+17,yy+17,xx+55,yy+55),fill=BLUE,outline='white',width=3)
txt(x1+555,gy+4,'任务',22,INK)
txt(x1+555,gy+45,'把箱子推到目标。',22)
txt(x1+555,gy+104,'规划重点',22,BLUE)
txt(x1+555,gy+145,'先站到箱子哪一侧？',20)
txt(x1+555,gy+181,'这一步会不会把箱子',20)
txt(x1+555,gy+217,'推入无法恢复的角落？',20)
txt(x1+555,gy+285,'动作：走 / 推',22,MUTED)

# Blocks World
x1,y1,x2,y2=cards[1]
txt(x1+32,y1+24,'2  积木世界 · Blocks World',34,PURPLE)
txt(x1+34,y1+72,'只能拿起最上面的积木，再放到桌面或另一块上。',22,MUTED)
def tower(cx,base,blocks):
  bw,bh=78,55
  for i,(label,color) in enumerate(blocks):
    yy=base-(i+1)*bh
    d.rounded_rectangle((cx-bw//2,yy,cx+bw//2,yy+bh-3),radius=7,fill=color,outline='white',width=3)
    txt(cx,yy+bh//2,label,23,'white','mm')
  d.line((cx-95,base,cx+95,base),fill='#7E756B',width=4)
txt(x1+88,y1+145,'当前',22,INK)
tower(x1+205,y1+383,[('A',RED),('B',BLUE),('C',GREEN)])
arrow((x1+285,y1+286),(x1+416,y1+286),PURPLE,5)
txt(x1+352,y1+255,'先清上层',20,PURPLE,'mm')
txt(x1+464,y1+145,'目标',22,INK)
tower(x1+580,y1+383,[('C',GREEN),('A',RED),('B',BLUE)])
txt(x1+664,y1+231,'一个移动会改变',19,MUTED)
txt(x1+664,y1+266,'哪些块可拿、可放。',19,MUTED)
txt(x1+664,y1+325,'动作：抓 / 放',21,MUTED)

# Tower of Hanoi
x1,y1,x2,y2=cards[2]
txt(x1+32,y1+24,'3  汉诺塔 · Tower of Hanoi',34,GREEN)
txt(x1+34,y1+72,'一次移动一片；大盘不能压在小盘上。',23,MUTED)
base=y1+380; pegs=[x1+180,x1+425,x1+670]
for cx in pegs:
  d.line((cx,base-210,cx,base),fill='#8C8378',width=8)
  d.line((cx-85,base,cx+85,base),fill='#8C8378',width=6)
sizes=[150,112,74]; colors=[RED,BLUE,GREEN]
for i in range(3):
  width=sizes[i]; yy=base-(i+1)*45
  d.rounded_rectangle((pegs[0]-width//2,yy,pegs[0]+width//2,yy+42),radius=13,fill=colors[i],outline='white',width=3)
arrow((pegs[0]+95,base-125),(pegs[2]-95,base-125),GREEN,5,16)
txt(pegs[0],base+18,'起点',20,MUTED,'ma')
txt(pegs[1],base+18,'辅助柱',20,MUTED,'ma')
txt(pegs[2],base+18,'目标',20,MUTED,'ma')
txt(x1+45,y1+438,'动作简单；合法顺序形成递归依赖。',21,GREEN)

# Rush Hour
x1,y1,x2,y2=cards[3]
txt(x1+32,y1+24,'4  华容道 / 交通堵塞 · Rush Hour',33,RED)
txt(x1+34,y1+72,'车辆只能沿车身方向滑动；清出目标车辆的出口通道。',22,MUTED)
gx,gy,cell=1000,850,48
for r in range(6):
  for c in range(6):
    xx,yy=gx+c*cell,gy+r*cell
    d.rectangle((xx,yy,xx+cell,yy+cell),fill='#F1EDE5',outline='#CEC7BB',width=2)
def car(c,r,length,horizontal,color,label):
  if horizontal: x,y=gx+c*cell+3,gy+r*cell+5; w,h=length*cell-6,cell-10
  else: x,y=gx+c*cell+5,gy+r*cell+3; w,h=cell-10,length*cell-6
  d.rounded_rectangle((x,y,x+w,y+h),radius=11,fill=color,outline='white',width=3)
  txt(x+w/2,y+h/2,label,20,'white','mm')
car(0,2,2,True,RED,'目标')
car(2,0,2,False,BLUE,'1')
car(3,1,2,False,GREEN,'2')
car(1,4,2,True,PURPLE,'3')
car(4,3,2,False,GOLD,'4')
arrow((gx+6*cell+10,gy+2*cell+cell//2),(gx+6*cell+65,gy+2*cell+cell//2),RED,5,14)
txt(1340,gy+25,'先移动挡路车辆',22,INK)
txt(1340,gy+70,'有些车先挪开后，',20,MUTED)
txt(1340,gy+104,'可能又会堵住别的车。',20,MUTED)
txt(1340,gy+164,'动作：推 / 滑动',21,MUTED)
txt(1340,gy+221,'共享窄道可增加',20,RED)
txt(1340,gy+255,'多臂时序与冲突。',20,RED)

im.save(OUT)
print(OUT)
