"""Build Transport R14 static scenes and reviewable family inputs; no GPT run."""
import argparse
from pathlib import Path
from robofrontier.suite.transport_policy_package import load_json,resolve,verify_oracle,build_half_scene,build_package,materialize_scene_assets

ROOT=Path(__file__).resolve().parents[1]

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/task_families/transport/half-opening-r14.json')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--assets-only',action='store_true',help='Restore ignored assets of existing R14 scenes; no package or previews')
    parser.add_argument('--no-previews',action='store_true')
    args=parser.parse_args();cfg=load_json(args.config)
    if not args.assets_only and args.output is None:parser.error('--output is required unless --assets-only is selected')
    if args.assets_only and args.output is not None:parser.error('--assets-only does not write a package; omit --output')
    if args.output is not None and args.output.exists():parser.error('Output already exists')
    restored=0
    for row in cfg['cases']:
        _,_,source=verify_oracle(row,ROOT);destination=resolve(ROOT,row['physical_scene'])
        if not destination.exists():
            if args.assets_only:parser.error('R14 scene metadata is missing: '+str(destination))
            build_half_scene(source,destination,ROOT,not args.no_previews)
        restored+=materialize_scene_assets(source,destination)
    if args.assets_only:print('ASSETS_VERIFIED',len(cfg['cases']),'cases;',restored,'restored',flush=True)
    else:
        result=build_package(args.config,args.output,ROOT)
        print(result['status'],result['case_count'],'cases;',args.output,flush=True)
