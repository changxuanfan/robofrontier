"""Compose frozen workstations for display; never run physics or an Oracle."""
import argparse
from pathlib import Path
from robofrontier.factory_workstations import compose
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--shell',type=Path,default=Path('outputs/robo-factory/v5-shell/scene.xml'));p.add_argument('--catalog',type=Path,default=Path('configs/suites/frontier-v2.json'));p.add_argument('--output',type=Path,default=Path('outputs/robo-factory/v5-workstations'));a=p.parse_args()
    print(compose(a.shell,a.catalog,a.output))
