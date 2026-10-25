"""Draw a canonical Sokoban task extended with four distinct robot workspaces."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/task-family-sketch-20261023/multiarm-sokoban.png'
W,H=1800,1120
BG='#F3F0E9'; INK='#26343D'; MUTED='#65727A'; CARD='#FFFCF6'; LINE='#CFC8BC'
BLUE='#3784BF'; RED='#D94E45'; GREEN='#37886F'; GOLD='#E5A53A'; PURPLE='#7962A1'
fp='/System/Library/Fonts/STHeiti Medium.ttc'
def f(s): return ImageFont.truetype(fp,s)
im=Image.new('RGB',(W,H),BG); d=ImageDraw.Draw(im)
def text(x,y,s,size=24,color=INK,anchor='la'): d.text((x,y),s,font=f(size),fill=color,anchor=anchor)
def rounded(b,fill=CARD,outline=LINE,r=18,w=2): d.rounded_rectangle(b,radius=r,fill=fill,outline=outline,width=w)
def arrow(a,b,color,width=6,head=16):
    import math
    d.line((a,b),fill=color,width=width)
    t=math.atan2(b[1]-a[1],b[0]-a[0])
    d.polygon([b,(b[0]-head*math.cos(t-.55),b[1]-head*math.sin(t-.55)),
               (b[0]-head*math.cos(t+.55),b[1]-head*math.sin(t+.55))],fill=color)

text(64,42,'四机械臂 Sokoban：并行工作 + 共享通道协调',48)
text(68,107,'规则仍是推箱子；新增的难度来自机械臂工作区和共享通道，不增加单次操作复杂度。',24,MUTED)
rounded((52,160,1250,1045))
gx,gy,cell,n=110,245,80,8
colors=[[BLUE,PURPLE],[GREEN,RED]]
for r in range(n):
    for c in range(n):
        zone=(0 if r<4 else 1,0 if c<4 else 1)
        fill=['#E7EFF5','#EEEAF3','#E8F1EC','#F5E9E7'][zone[0]*2+zone[1]]
        x,y=gx+c*cell,gy+r*cell
        d.rectangle((x,y,x+cell,y+cell),fill=fill,outline='#CDC6BA',width=2)
# The one shared cell lies at the four workspace boundaries.
cx,cy=gx+4*cell,gy+4*cell
d.rectangle((cx,cy,cx+cell,cy+cell),fill='#F7D878',outline='#A97C1B',width=5)
text(cx+cell/2,cy+cell/2,'共享\n通道',20,'#5A4311','mm')

def robot_label(cx,cy,label,color):
    d.ellipse((cx-34,cy-34,cx+34,cy+34),fill=color,outline='white',width=4)
    text(cx,cy,label,30,'white','mm')
robot_label(gx+32,gy+32,'A',BLUE); robot_label(gx+7*cell+48,gy+32,'B',PURPLE)
robot_label(gx+32,gy+7*cell+48,'C',GREEN); robot_label(gx+7*cell+48,gy+7*cell+48,'D',RED)

def crate(c,r,label,color):
    x,y=gx+c*cell,gy+r*cell
    d.rounded_rectangle((x+11,y+11,x+cell-11,y+cell-11),radius=9,fill=color,outline='white',width=3)
    text(x+cell/2,y+cell/2,label,26,'white','mm')
def goal(c,r):
    x,y=gx+c*cell,gy+r*cell
    d.ellipse((x+19,y+19,x+cell-19,y+cell-19),fill='#D2E5D6',outline='#407A53',width=4)
    text(x+cell/2,y+cell/2,'G',22,'#316344','mm')

# Two long jobs approach the same one-cell bottleneck from opposite directions.
crate(1,2,'A',BLUE); goal(6,1)
crate(6,6,'D',RED); goal(1,6)
# Two short local jobs can be assigned to B and C while the bottleneck is occupied.
crate(6,2,'B',PURPLE); goal(7,2)
crate(1,5,'C',GREEN); goal(2,5)
arrow((gx+2.5*cell,gy+2.5*cell),(cx+cell/2,cy+cell/2),BLUE,7,18)
arrow((gx+6.5*cell,gy+6.5*cell),(cx+cell/2,cy+cell/2),RED,7,18)
arrow((gx+6.5*cell,gy+2.5*cell),(gx+7.0*cell,gy+2.5*cell),PURPLE,6,15)
arrow((gx+1.5*cell,gy+5.5*cell),(gx+2.0*cell,gy+5.5*cell),GREEN,6,15)

# Workspace boundaries: dashed lines are reach partitions, not physical walls.
for x1,y1,x2,y2 in [(gx+4*cell,gy,gx+4*cell,gy+4*cell),
                     (gx+4*cell,gy+5*cell,gx+4*cell,gy+8*cell),
                     (gx,gy+4*cell,gx+4*cell,gy+4*cell),
                     (gx+5*cell,gy+4*cell,gx+8*cell,gy+4*cell)]:
    d.line((x1,y1,x2,y2),fill='#69747A',width=3)

# Right explanatory panel
rounded((1290,160,1748,1045))
text(1330,200,'为什么需要多臂？',30)
text(1330,270,'A → 右上目标',26,BLUE)
text(1330,308,'D → 左下目标',26,RED)
text(1330,360,'两条路线都要经过',22,MUTED)
text(1330,394,'中间唯一的共享格。',22,MUTED)
rounded((1322,442,1718,580),fill='#F8F0D7',outline='#D7C27D',r=14,w=2)
text(1345,463,'通道被占用时',22,'#725A1F')
text(1345,503,'B、C 仍可做本地目标。',21,INK)
text(1345,540,'避免全队一起空等。',21,INK)
text(1330,622,'Agent 结构可比较：',23)
for i,s in enumerate(['一个 Agent 同时调度四臂',
                       'Coordinator 分配，Workers 执行',
                       '四个 Worker 协商通道次序',
                       'Adaptive 按收益临时 fork']):
    text(1338,666+i*50,'• '+s,19,INK)
text(1330,890,'指标：完成 / 等待 / 并行重叠',20,GREEN)
text(1330,930,'动作：移动 / 推箱；不许隔空搬运',19,MUTED)
text(70,1065,'示意：虚线为机械臂可达分区；黄色格为共享通道。具体可达性和不碰撞路径须用 MuJoCo 验证。',20,MUTED)

im.save(OUT)
print(OUT)
