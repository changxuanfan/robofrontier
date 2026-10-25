"""RGB + robot-proprioception client; no object-state or depth endpoint."""
import argparse
import json
import urllib.request
import uuid

from .scene import ROOT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--request-id", default=None)
    commands = parser.add_subparsers(dest="type", required=True)
    observe = commands.add_parser("observe")
    observe.add_argument("--camera", choices=["front", "side", "both"], default="both")
    move = commands.add_parser("move")
    for axis in "xyz":
        move.add_argument(f"--d{axis}", type=float, default=0)
    move.add_argument("--yaw-deg", type=float, default=0)
    move.add_argument("--duration", type=float, default=1,
                      help="Move duration, 0.1..4 seconds; maximum distance = speed x duration")
    move.add_argument("--speed", choices=["slow", "normal", "fast"], default="normal",
                      help="TCP/joint speed profile; duration and acceleration limits still apply")
    gripper = commands.add_parser("gripper")
    gripper.add_argument("state", choices=["open", "close"])
    wait = commands.add_parser("wait")
    wait.add_argument("--duration", type=float, default=1)
    record = commands.add_parser("record")
    record.add_argument("state", choices=["start", "stop"])
    commands.add_parser("stop")
    args = vars(parser.parse_args())
    args["request_id"] = args["request_id"] or uuid.uuid4().hex
    runtime = json.loads((ROOT / ".robot-bridge.json").read_text())
    request = urllib.request.Request(runtime["url"], data=json.dumps(args).encode(),
        headers={"Content-Type": "application/json", "X-Robot-Token": runtime["token"]})
    with urllib.request.urlopen(request, timeout=18) as response:
        print(response.read().decode())


if __name__ == "__main__":
    main()
