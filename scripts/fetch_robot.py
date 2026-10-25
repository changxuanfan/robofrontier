"""Download only the official Panda model and its referenced meshes."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "vendor" / "panda"
REPO = "google-deepmind/mujoco_menagerie"


def get(url):
    request = urllib.request.Request(url, headers={"User-Agent": "RoboFrontier-mini"})
    with urllib.request.urlopen(request, timeout=90) as response:
        return response.read()


def main():
    manifest = DEST / "SOURCE.json"
    if manifest.exists():
        source = json.loads(manifest.read_text())
        if all((DEST / name).exists() and hashlib.sha256((DEST / name).read_bytes()).hexdigest() == digest
               for name, digest in source["sha256"].items()):
            print(f"Panda assets verified ({source['commit'][:12]})")
            return
        commit = source["commit"]
    else:
        commit = json.loads(get(f"https://api.github.com/repos/{REPO}/commits/main"))["sha"]
    base = f"https://raw.githubusercontent.com/{REPO}/{commit}/franka_emika_panda"
    xml = get(f"{base}/panda.xml")
    tree = ET.fromstring(xml)
    meshes = sorted({node.attrib["file"] for node in tree.findall("asset/mesh")})
    DEST.mkdir(parents=True, exist_ok=True)
    (DEST / "assets").mkdir(exist_ok=True)
    files = {"panda.xml": xml}

    def fetch(name):
        path = f"assets/{name}"
        return path, get(f"{base}/{path}")

    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        for path, content in pool.map(fetch, meshes):
            files[path] = content
    for name in ("LICENSE", "README.md"):
        files[name] = get(f"{base}/{name}")
    for path, content in files.items():
        (DEST / path).write_bytes(content)
    manifest.write_text(json.dumps({"repository": f"https://github.com/{REPO}", "commit": commit,
                                   "sha256": {name: hashlib.sha256(data).hexdigest()
                                              for name, data in files.items()}}, indent=2) + "\n")
    print(f"Downloaded Panda and {len(meshes)} meshes ({sum(map(len, files.values())) / 1e6:.1f} MB), commit {commit}")


if __name__ == "__main__":
    main()
