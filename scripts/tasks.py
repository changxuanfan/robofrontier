"""List released tasks, audit their inputs, or prepare isolated policy runs."""
import argparse
import json
from pathlib import Path
import hashlib
from robofrontier.task_families import ROOT, RELEASE, read, resolve_case, prepare
from robofrontier.run_setup import write_json


def check_case_assets(case, root=ROOT):
    """Verify released workstation assets without creating a world."""
    root=Path(root); directory=root/case['directory']
    manifest=read(directory/'workstation.json')
    missing=[]; changed=[]
    for asset in manifest.get('assets', []):
        relative=Path(asset['path'])
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Invalid asset path in '+case['id'])
        path=directory/relative
        if not path.is_file():missing.append(str(path.relative_to(root)))
        elif hashlib.sha256(path.read_bytes()).hexdigest()!=asset['sha256']:
            changed.append(str(path.relative_to(root)))
    return {'checked':len(manifest.get('assets', [])),'missing':missing,'changed':changed}


def prepare_batch(output, families=None, harness=None):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    cases=[c for name,f in read(ROOT/RELEASE)['families'].items()
           if families is None or name in families for c in f['gpt']]
    jobs=[dict(case_id=c['id'],run_directory=str(output/c['id']),status='pending')for c in cases]
    manifest=dict(status='preparing',simulator_started=False,model_calls=0,
        isolation='One run directory, runtime database, connection set and independent service per task',jobs=jobs)
    write_json(output/'batch.json',manifest)
    try:
        for job in jobs:
            prepare(job['case_id'],job['run_directory'],harness)
            job['status']='prepared'
            write_json(output/'batch.json',manifest)
    except Exception as error:
        manifest.update(status='incomplete',error=str(error))
        write_json(output/'batch.json',manifest)
        raise
    manifest['status']='prepared_not_started'
    write_json(output/'batch.json',manifest)
    return output/'batch.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    listing = commands.add_parser('list')
    listing.add_argument('--purpose', choices=('gpt', 'oracle'), default='gpt')
    check = commands.add_parser('check')
    check.add_argument('--purpose', choices=('gpt', 'oracle'), default='gpt')
    check.add_argument('--assets', action='store_true', help='Verify every released workstation asset hash')
    setup = commands.add_parser('prepare')
    setup.add_argument('case_id')
    setup.add_argument('--output', type=Path, required=True)
    setup.add_argument('--harness', type=Path)
    batch = commands.add_parser('prepare-batch',help='Freeze isolated parallel jobs; no model or simulator launch')
    batch.add_argument('--output',type=Path,required=True)
    batch.add_argument('--families',nargs='+',choices=('transport','mosaic','relay','dial'))
    batch.add_argument('--harness',type=Path)
    args = parser.parse_args()
    if args.command == 'prepare-batch':
        print(prepare_batch(args.output,args.families,args.harness))
        return
    if args.command == 'prepare':
        print(prepare(args.case_id, args.output, args.harness))
        return
    rows = []; failed = False
    for family in read(ROOT/RELEASE)['families'].values():
        for case in family[args.purpose]:
            if args.command == 'check':
                resolve_case(case['id'], args.purpose)
            row=dict(id=case['id'], directory=case['directory'],qualification=case['qualification'])
            if args.command=='check' and args.assets:
                row['assets']=check_case_assets(case)
                failed = failed or bool(row['assets']['missing'] or row['assets']['changed'])
            rows.append(row)
    print(json.dumps(dict(purpose=args.purpose, cases=rows, count=len(rows)), indent=2))
    if failed:raise SystemExit(1)


if __name__ == '__main__':
    main()
