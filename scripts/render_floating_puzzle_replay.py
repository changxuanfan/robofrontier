"""Render recorded Oracle qpos only; no stepping, no policy input generation."""
import json
from pathlib import Path
import subprocess
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scripts.build_floating_dial_puzzle import DIRECTORY


def render(output,scene_directory=DIRECTORY):
    output=Path(output);states=np.load(output/'frame-states.npz')
    model=mujoco.MjModel.from_xml_path(str(Path(scene_directory)/'scene.xml'));data=mujoco.MjData(model)
    video=output/'oracle-eight-arm-detail.mp4'
    process=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','1600x640','-r','4','-i','-','-c:v','libx264','-preset','veryfast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],stdin=subprocess.PIPE)
    try:font=ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc',22,index=1)
    except OSError:font=ImageFont.load_default(size=22)
    try:
        with mujoco.Renderer(model,height=580,width=800) as r:
            for q,t in zip(states['qpos'],states['time_s']):
                data.qpos[:]=q;mujoco.mj_forward(model,data)
                canvas=Image.new('RGB',(1600,640),'#f3f6fa');draw=ImageDraw.Draw(canvas)
                for x,camera in ((0,'global'),(800,'dial_A_high')):
                    r.update_scene(data,camera=camera);canvas.paste(Image.fromarray(r.render()),(x,60))
                angles=[float(np.degrees(data.joint('dial_'+a+'_hinge').qpos[0])) for a in 'ABCD']
                draw.text((16,12),f'ORACLE · 8 ARMS / 4 FREE MODULES  |  t={t:.1f}s',font=font,fill='#182638')
                draw.text((816,12),'MODULE A DETAIL  |  '+' / '.join(f'{x:+.1f}' for x in angles)+' deg',font=font,fill='#182638')
                process.stdin.write(canvas.tobytes())
    finally:process.stdin.close();code=process.wait()
    if code:raise RuntimeError('encoder failed')
    print(json.dumps({'video':str(video),'frames':len(states['qpos']),'physics_advanced':False}))

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--scene-directory',type=Path,default=DIRECTORY);a=p.parse_args()
    render(a.output,a.scene_directory)
