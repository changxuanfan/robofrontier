"""Build or check the dial-only policy review; starts no simulation or model."""
import argparse
from pathlib import Path

from robofrontier.run_setup import write_json
from robofrontier.suite.dial_policy_package import REGISTRY, build_review, read, registry, verify_review


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--write-registry',action='store_true',help='Create the family evidence registry; never overwrites one')
    parser.add_argument('--check-output',type=Path,help='Verify an existing family review against its source inputs')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    path = root/REGISTRY
    current = registry(root)
    if args.write_registry:
        if path.exists():
            raise FileExistsError(path)
        write_json(path,current)
    if read(path) != current:
        raise ValueError('Registry differs from current inputs')
    if args.output:
        build_review(root,args.output,current)
        verify_review(root,args.output)
    if args.check_output:
        verify_review(root,args.check_output)
    print(f"9 dial cases verified; review only; formal GPT readiness remains pending")
