"""Build/check a Mosaic family review bundle; never prepare or start a run."""
import argparse
import json
from pathlib import Path
from robofrontier.suite.mosaic_policy_package import build,check,verify_registry

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--check',action='store_true')
    p.add_argument('--verify-recorded-chunks',action='store_true')
    args=p.parse_args()
    if args.check and args.verify_recorded_chunks:verify_registry(verify_recorded_chunks=True)
    result=check(args.output) if args.check else build(args.output,verify_recorded_chunks=args.verify_recorded_chunks)
    print(json.dumps({key:value for key,value in result.items() if key!='public_files'},ensure_ascii=False,indent=2))
