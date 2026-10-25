"""Draw a visual affordance concept for a press-to-turn spring dial."""
from pathlib import Path
import math
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/spring-dial-press-turn-ui-20261023/press-turn-concept.png'
W,H=1800,1120
BG='#F3F0E9'; CARD='#FFFCF6'; INK='#17232B'; MUTED='#66737B'; LINE='#CEC7BC'
RED='#D94E45'; ORANGE='#ED7C32'; BLUE='#3784BF'; GREEN='#37886F'; METAL='#5B656D'; CREAM='#F8F4E9'
fp='/System/Library/Fonts/STHeiti Medium.ttc'
def f(n): return ImageFont.truetype(fp,n)
im=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(im)
def text(x,y,s,n=24,c=INK,a='la'): d.text((x,y),s,font=f(n),fill=c,anchor=a)
def rr(b,fill=CARD,out=LINE,r=18,w=2): d.rounded_rectangle(b,radius=r,fill=fill,outline=out,width=w)
def arrow(a,b,c,w=7,head=18):
    t=math.atan2(b[1]-a[1],b[0]-a[0]); d.line((a,b),fill=c,width=w)
    d.polygon([b,(b[0]-head*math.cos(t-.55),b[1]-head*math.sin(t-.55)),
               (b[0]-head*math.cos(t+.55),b[1]-head*math.sin(t+.55))],fill=c)

text(60,38,'按压解锁旋钮：先按入，再旋转，松开后卡位',44)
text(65,102,'同一组角度与计分规则；新增可见的轴向按钮和真实机械定位结构。',24,MUTED)
rr((52,155,1050,1030));rr((1080,155,1748,1030))
text(90,180,'正面：看角度并操作中心按钮',28)

# Main top-down dial
cx,cy,R=535,570,290
d.rounded_rectangle((150,260,1020,920),radius=34,fill='#B98F70',outline='#927057',width=4)
d.ellipse((cx-R,cy-R,cx+R,cy+R),fill='#D8CEC0',outline='#857566',width=4)
d.ellipse((cx-210,cy-210,cx+210,cy+210),fill=CREAM,outline='#AFA394',width=3)
for deg,label in [(270,'0°'),(0,'90°'),(90,'180°'),(180,'270°')]:
    a=math.radians(deg)
    p1=(cx+170*math.cos(a),cy+170*math.sin(a));p2=(cx+195*math.cos(a),cy+195*math.sin(a))
    d.line((p1,p2),fill=INK,width=6)
    lx,ly=cx+255*math.cos(a),cy+255*math.sin(a)
    text(lx,ly,label,26,INK,'mm')
# Red rotating hub with an axial push cap and orange pointer
d.ellipse((cx-92,cy-92,cx+92,cy+92),fill='#B8AEA0',outline='#73695F',width=4)
d.ellipse((cx-72,cy-72,cx+72,cy+72),fill=RED,outline='#8F2D29',width=4)
d.rounded_rectangle((cx-13,cy-174,cx+13,cy-66),radius=10,fill=ORANGE)
d.ellipse((cx-48,cy-48,cx+48,cy+48),fill='#EF6258',outline='white',width=4)
d.ellipse((cx-25,cy-25,cx+25,cy+25),fill='#C93C35')
# Press-in and turn cues
arrow((cx,cy-275),(cx,cy-112),BLUE,8,20)
text(cx+36,cy-278,'按入',26,BLUE)
d.arc((cx-126,cy-126,cx+126,cy+126),195,325,fill=ORANGE,width=9)
arrow((cx+103,cy-73),(cx+88,cy-58),ORANGE,6,16)
text(cx+145,cy-96,'按住旋转',23,ORANGE)

# Side-section panel
text(1120,180,'侧剖面与动作顺序',28)
text(1120,224,'按压行程会让定位销退出凹槽。',20,MUTED)

def section(y,pressed,title,subtitle):
    text(1120,y,title,22,INK)
    d.rectangle((1225,y+108,1690,y+130),fill='#B98F70',outline='#8E674D',width=2)
    d.rectangle((1170,y+29,1200,y+108),fill=METAL)
    d.rectangle((1335,y+29,1365,y+108),fill=METAL)
    d.rectangle((1170,y+29,1365,y+43),fill=METAL)
    shift=22 if pressed else 0
    d.rectangle((1222,y+59+shift,1290,y+99+shift),fill=RED,outline='#8F2D29',width=2)
    d.rectangle((1245,y+31+shift,1267,y+63+shift),fill=ORANGE)
    if pressed:
        d.rectangle((1198,y+46,1225,y+54),fill=BLUE)
        arrow((1208,y+32),(1208,y+43),BLUE,4,9)
    else:
        d.rectangle((1198,y+76,1225,y+84),fill=METAL)
        d.line((1222,y+76,1227,y+76),fill=INK,width=5)
    text(1405,y+72,subtitle,18,BLUE if pressed else MUTED)

section(265,False,'1  未按入','销卡在槽内')
section(420,True,'2  按入','销退出凹槽')
text(1120,575,'3  按住旋转 → 松开卡入刻度槽',21,INK)
rr((1110,625,1718,810),fill='#F5F1E8',out='#DDD5C8',r=14,w=2)
text(1140,650,'视觉线索',23,INK)
text(1140,692,'中心按钮有明显轴向行程。',20,MUTED)
text(1140,729,'按下时按钮缩入，松手后卡位。',20,MUTED)
text(1140,766,'指针和刻度始终保持清晰可见。',20,MUTED)
rr((1110,835,1718,960),fill='#EEF3F5',out='#D0DDE2',r=14,w=2)
text(1140,860,'使用原有动作接口',22,INK)
text(1140,900,'移动 / 夹爪 / 观察；顺序由 Agent 决定。',19,MUTED)

text(58,1060,'结构示意，尚未验证推力、行程、销槽接触和旋转稳定性。',20,MUTED)
im.save(OUT);print(OUT)
