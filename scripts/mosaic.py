"""Check or export the current Mosaic delivery; recover ignored scene assets."""
import argparse
import json
from pathlib import Path

from robofrontier.suite.mosaic_delivery import (check_delivery,export_package,
    materialize_assets,asset_inventory,verify_protected)
from robofrontier.suite.mosaic_policy_package import write


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command')
    check=commands.add_parser('check',help='Check nine GPT tasks by default; creates no run')
    check.add_argument('--purpose',choices=('gpt','oracle'),default='gpt')
    check.add_argument('--package',type=Path)
    check.add_argument('--verify-recorded-chunks',action='store_true')
    check.add_argument('--verify-protected',action='store_true')
    check.add_argument('--report',type=Path)
    package=commands.add_parser('package',help='Copy the verified frozen review bundle to a new directory')
    package.add_argument('--purpose',choices=('gpt','oracle'),default='gpt')
    package.add_argument('--output',type=Path,required=True)
    assets=commands.add_parser('assets',help='Materialize ignored meshes/textures, including after merge')
    assets.add_argument('--purpose',choices=('gpt','oracle','all'),default='all')
    assets.add_argument('--destination',type=Path,required=True)
    assets.add_argument('--dry-run',action='store_true')
    assets.add_argument('--inventory',type=Path,help='Write the asset transfer inventory')
    args=parser.parse_args(['check'] if argv==[] else argv)
    if args.command is None:args=parser.parse_args(['check'])
    if args.command=='check':
        result=check_delivery(args.purpose,args.package,args.verify_recorded_chunks)
        if args.verify_protected:result['protection']=verify_protected()
        if args.report:
            if args.report.exists():raise FileExistsError(args.report)
            write(args.report,result)
    elif args.command=='package':result=export_package(args.output,args.purpose)
    else:
        if args.inventory and args.inventory.exists():raise FileExistsError(args.inventory)
        result=materialize_assets(args.destination,args.purpose,dry_run=args.dry_run)
        if args.inventory:
            write(args.inventory,asset_inventory(args.purpose))
    summary=dict(result)
    if isinstance(summary.get('cases'),list):summary['cases']=len(summary['cases'])
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    return result


if __name__=='__main__':main()
