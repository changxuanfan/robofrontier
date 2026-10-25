"""Draft pixel targets only: no MuJoCo scene, robot motion or benchmark run."""
import argparse,json
from collections import Counter
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
from robofrontier.mosaic import PATTERN_7X7

ROOT=Path(__file__).resolve().parents[1]
COLORS={'R':'#d73331','B':'#1263bc','Y':'#f8c62e','W':'#f3f0e9'}

def pattern(width):
    if width not in (7,14,21):raise ValueError('Supported draft widths: 7, 14, 21')
    # Preserve the supplied seven-by-seven topology, turned to face right.
    base=[''.join(row) for row in zip(*PATTERN_7X7[::-1])]
    rows=[list(row[:3]+row[3]*(width-6)+row[4:]) for row in base]
    if width>7:
        for col in range(5,width-3,4):rows[3][col]='B'
    return [''.join(row) for row in rows]

def build(directory):
    directory.mkdir(parents=True,exist_ok=True)
    fontfile='/System/Library/Fonts/Helvetica.ttc'
    title=ImageFont.truetype(fontfile,25,index=1);small=ImageFont.truetype(fontfile,17)
    image=Image.new('RGB',(1100,1060),'#edf3f7');draw=ImageDraw.Draw(image)
    draw.text((30,20),'ROCKET TARGETS | DESIGN DRAFT',font=title,fill='#192c3e')
    draw.text((30,56),'All cells require cubes, including white. These are target images, not simulated scenes.',font=small,fill='#40556a')
    specs=[]
    for i,(arms,width) in enumerate(((4,7),(6,14),(8,21))):
        rows=pattern(width);counts=dict(Counter(''.join(rows)));y=105+i*305;cell=31;x=48
        draw.text((30,y),f'{arms} ARMS  |  7 x {width}  |  {7*width} CUBES',font=title,fill='#192c3e')
        for c in range(width):draw.text((x+c*cell+cell/2,y+41),str(c+1),font=small,anchor='mm',fill='#40556a')
        for r,row in enumerate(rows):
            draw.text((29,y+66+r*cell+cell/2),str(r+1),font=small,anchor='mm',fill='#40556a')
            for c,code in enumerate(row):
                xx=x+c*cell;yy=y+66+r*cell;draw.rectangle((xx,yy,xx+cell-2,yy+cell-2),fill=COLORS[code],outline='#9ea9b0')
        for j,code in enumerate(('R','B','Y','W')):
            xx=780;yy=y+67+j*43;draw.rectangle((xx,yy,xx+25,yy+25),fill=COLORS[code],outline='#9ea9b0');draw.text((xx+38,yy+2),f'{code}: {counts[code]}',font=small,fill='#192c3e')
        specs.append(dict(arms=arms,rows=7,columns=width,pattern=rows,inventory_counts=counts,status='design draft',physical_pitch_m=.051))
    draw.text((30,1030),'Right-facing rocket: extended fuselage and repeated blue windows; no color added.',font=small,fill='#40556a')
    image.save(directory/'rocket-targets.png')
    (directory/'rocket-targets.json').write_text(json.dumps(dict(status='design draft, not a built environment',scale_pairing='4 arms:7x7, 6 arms:7x14, 8 arms:7x21',shared_pitch_m=.051,targets=specs),indent=2)+'\n')
    print(json.dumps({'preview':str(directory/'rocket-targets.png'),'targets':specs}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=ROOT/'docs/plans/suite-v1');build(p.parse_args().output)
