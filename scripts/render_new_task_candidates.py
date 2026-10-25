"""Render concrete candidate task cards for a multi-arm tabletop benchmark."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/new-task-ideas-20261023/task-candidates.png'
W,H=1800,1650
BG='#F3F0E9'; CARD='#FFFCF6'; INK='#26343D'; MUTED='#65727A'; LINE='#CFC8BC'
BLUE='#3784BF'; RED='#D94E45'; GREEN='#37886F'; GOLD='#E5A53A'; PURPLE='#7962A1'
fp='/System/Library/Fonts/STHeiti Medium.ttc'
def f(n): return ImageFont.truetype(fp,n)
im=Image.new('RGB',(W,H),BG); d=ImageDraw.Draw(im)
def text(x,y,s,n=23,c=INK,a='la'): d.text((x,y),s,font=f(n),fill=c,anchor=a)
def card(x,y,w,h,num,title,tag,color,goal,actions,reason,arms,complexity):
    d.rounded_rectangle((x,y,x+w,y+h),radius=22,fill=CARD,outline=LINE,width=2)
    d.rounded_rectangle((x+22,y+22,x+74,y+74),radius=14,fill=color)
    text(x+48,y+48,str(num),25,'white','mm')
    text(x+94,y+24,title,30,INK)
    text(x+w-24,y+38,tag,19,color,'ra')
    d.line((x+24,y+88,x+w-24,y+88),fill='#E4DED4',width=2)
    text(x+28,y+109,'任务指令',19,MUTED)
    # Wrap the quoted, concrete goal over two lines.
    goal_lines=goal.split('|')
    for i,line in enumerate(goal_lines): text(x+28,y+142+i*34,'“'+line+'”' if i==0 else ' '+line,22,INK)
    yy=y+142+len(goal_lines)*34+15
    text(x+28,yy,'动作',18,MUTED); text(x+93,yy,actions,20,INK)
    text(x+28,yy+43,'长程难点',18,MUTED); text(x+125,yy+43,reason,20,INK)
    text(x+28,yy+85,'多臂参与',18,MUTED); text(x+125,yy+85,arms,20,INK)
    d.rounded_rectangle((x+w-180,y+h-43,x+w-24,y+h-12),radius=12,fill='#F0ECE5')
    text(x+w-102,y+h-27,'仿真复杂度：'+complexity,16,MUTED,'mm')

text(60,38,'比现有四项不同的长程操作任务',47)
text(65,102,'保持抓、移动、旋转、推等原子动作；用任务状态依赖产生长程规划。',24,MUTED)

cw,ch=810,415
card(55,155,cw,ch,1,'承重桥搭建','结构稳定',BLUE,
     '用6块支柱和2块桥面跨过桌面缺口。|小球必须能从桥上滚到另一侧。',
     '抓取 · 放置 · 推',
     '先搭两侧支撑，再铺桥面；放错顺序会挡住支点或导致桥面塌落。',
     '两侧机械臂可同时搭支柱；其他臂随后铺设桥面并测试。','中')
card(925,155,cw,ch,2,'齿轮传动','因果配置',PURPLE,
     '从托盘选齿轮，把输入轴连到输出轴。|转动输入轴一圈，输出须反向转两圈。',
     '抓取 · 放置 · 旋转',
     '推理齿数与转向；中间齿轮选择会改变输出方向与圈数。',
     '多臂可并行布置不同齿轮，最后由一臂转动输入轴验证。','中高')
card(55,600,cw,ch,3,'天平配重','数量与物理',GREEN,
     '用桌上的砝码让天平指针进入绿色区并稳定2秒。|随后取下钥匙。',
     '抓取 · 放置 · 移动',
     '根据支点距离和砝码重量调整两侧力矩，观察结果后决定增减。',
     '两臂可同时放置两侧砝码；其他臂准备或移走挡路砝码。','中')
card(925,600,cw,ch,4,'限容积装箱','空间与顺序',GOLD,
     '把8个不同形状的积木装进箱子并盖上箱盖。|每块都必须完全放入。',
     '抓取 · 旋转 · 放置 · 推',
     '先放哪些形状、朝向如何，会决定剩余空间和后续可放入的积木。',
     '机械臂可并行准备互不遮挡的物件；箱口附近需要安排放置次序。','中')
card(55,1045,1680,415,5,'骨牌连锁','传播与并行',RED,
     '把骨牌从起点排到铃铛前。|轻推第一块，整条链必须倒下并敲响铃。',
     '抓取 · 旋转 · 放置 · 推',
     '间距和朝向的小误差会沿链传播；先规划路线，再逐段核实连接。',
     '多臂可同时摆不同路段；合拢路段时核对连接，最后再触发。','中高')

text(66,1500,'区分方式：桥搭建测结构支撑；齿轮测因果；天平测物理反馈；装箱测空间规划；骨牌测误差传播。',21,MUTED)
im.save(OUT);print(OUT)
