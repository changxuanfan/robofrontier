"""Create reusable source-time idle edits; run no robot commands or physics."""
import argparse
from pathlib import Path

from robofrontier.motion_edit import IdleSettings, build_edit
from robofrontier.run_setup import write_json


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    for name, value in vars(IdleSettings()).items():
        parser.add_argument('--'+name.replace('_', '-'), type=float, default=value)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    settings = IdleSettings(**{name:getattr(args, name) for name in vars(IdleSettings())})
    edit = build_edit(args.recording, settings)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.output, edit)
    print(f"{edit['case_id']}: {edit['source_duration_s']:.2f}s -> {edit['edited_duration_s']:.2f}s; {len(edit['removed_spans'])} idle intervals")
