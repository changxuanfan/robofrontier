"""Compose frozen workstations for display; never run physics or an Oracle."""
import argparse,json
from pathlib import Path
from robofrontier.factory_workstations import compose
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--shell',type=Path,default=Path('outputs/robo-factory/v5-shell/scene.xml'));p.add_argument('--catalog',type=Path,default=Path('configs/suites/frontier-v2.json'));p.add_argument('--output',type=Path,default=Path('outputs/robo-factory/v5-workstations'));p.add_argument('--placements',type=Path,help='Optional case_id to translation_m/yaw_rad mapping; no scale or geometry change');a=p.parse_args()
    print(compose(a.shell,a.catalog,a.output,json.loads(a.placements.read_text()) if a.placements else None))
