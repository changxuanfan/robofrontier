"""Build dial-owned progress-aware edits; no render, physics or inference."""
import argparse
from pathlib import Path

from robofrontier.run_setup import write_json
from robofrontier.suite.dial_motion_edit import build_dial_edit


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    edit = build_dial_edit(args.recording)
    write_json(args.output,edit)
    print(f"{edit['case_id']}: removed {edit['removed_frames']/edit['sample_hz']:.2f} source seconds; "
        f"oscillation {edit['dial_profile']['removed_oscillation_frames']/edit['sample_hz']:.2f}s")
