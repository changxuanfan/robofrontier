"""Prepare a run only: no GUI, model calls, session creation or robot actions."""
import argparse
from pathlib import Path
from robofrontier.run_setup import prepare_run

if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run_dir',type=Path)
    p.add_argument('--task',type=Path)
    p.add_argument('--harness',type=Path)
    p.add_argument('--private-puzzle',type=Path,help='Engineering-only code_symbols JSON for paired spring-dial trials; omitted samples a fresh private code')
    args=p.parse_args()
    print(prepare_run(args.run_dir,args.task,args.harness,args.private_puzzle))
