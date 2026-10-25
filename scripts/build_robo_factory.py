"""Generate the empty Robo Factory shell and portable assets."""
import argparse,json
from pathlib import Path
from robofrontier.factory_scene import build_factory,DEFAULT_CONFIG

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=DEFAULT_CONFIG);p.add_argument('--output',type=Path)
    a=p.parse_args();scene,manifest=build_factory(a.config,a.output)
    print(json.dumps({'scene':str(scene),'dimensions_m':manifest['dimensions_m'],'regions':len(manifest['regions']),'task_workstations':0,'mesh_definition_triangles':manifest['mesh_definition_triangles'],'cameras':len(manifest['cameras'])}))

if __name__=='__main__':main()
