"""Inspect or export the static Robo Factory without task simulation."""
import argparse
from pathlib import Path
from robofrontier.factory_scene import ROOT
from robofrontier.factory_view import render_views,serve
from robofrontier.factory_playback import current_scene

def main():
    p=argparse.ArgumentParser(description=__doc__)
    current=current_scene()
    p.add_argument('--scene',type=Path,default=current if current.exists() else ROOT/'outputs/robo-factory/scene/scene.xml')
    p.add_argument('--renderer',choices=['classic'],default='classic')
    p.add_argument('--render',action='store_true',help='Export fixed-camera PNGs and measurements')
    p.add_argument('--output',type=Path,default=ROOT/'outputs/robo-factory/previews')
    p.add_argument('--views',nargs='+');p.add_argument('--width',type=int,default=1280);p.add_argument('--height',type=int,default=720)
    p.add_argument('--benchmark-frames',type=int,default=8);p.add_argument('--port',type=int,default=0)
    a=p.parse_args()
    if a.render:render_views(a.scene,a.output,a.renderer,a.width,a.height,a.views,a.benchmark_frames)
    else:serve(a.scene,a.renderer,a.width,a.height,a.port)

if __name__=='__main__':main()
