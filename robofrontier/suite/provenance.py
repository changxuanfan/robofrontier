"""Content fingerprints for physical assets, independent of catalogue status."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

def bundle_manifest(directory):
    directory=Path(directory).resolve();xml=directory/'scene.xml'
    root=ET.parse(xml).getroot();compiler=root.find('compiler');records=[]
    records.append(dict(kind='scene',name='scene.xml',sha256=hashlib.sha256(xml.read_bytes()).hexdigest()))
    for asset in root.findall('asset/*[@file]'):
        file=Path(asset.get('file'))
        prefix=compiler.get('meshdir','') if asset.tag=='mesh' else compiler.get('texturedir','') if asset.tag=='texture' else ''
        resolved=file if file.is_absolute() else directory/prefix/file
        records.append(dict(kind=asset.tag,name=asset.get('name') or file.stem,file=str(file),sha256=hashlib.sha256(resolved.read_bytes()).hexdigest()))
    for name in ('task.json','environment.json'):
        records.append(dict(kind='contract',name=name,sha256=hashlib.sha256((directory/name).read_bytes()).hexdigest()))
    if (directory/'target-pattern.png').exists():records.append(dict(kind='task_image',name='target-pattern.png',sha256=hashlib.sha256((directory/'target-pattern.png').read_bytes()).hexdigest()))
    records.sort(key=lambda r:(r['kind'],r['name']))
    digest=hashlib.sha256()
    for record in records:digest.update((record['kind']+'\0'+record['name']+'\0'+record['sha256']+'\n').encode())
    return dict(sha256=digest.hexdigest(),files=records)


def environment_logic_manifest(source_hashes,task=None):
    """Fingerprint both shared process rules and the v2 geometry evaluator."""
    names=('robofrontier/suite/evaluate.py','robofrontier/suite/evaluate_v2.py')
    if task and task.get('family')=='dial' and task.get('mechanism',{}).get('feedback'):
        names+=('robofrontier/suite/dial_feedback.py',)
    code={name:source_hashes[name] for name in names}
    payload=json.dumps(code,sort_keys=True,separators=(',',':')).encode()
    return dict(schema='robofrontier.environment-logic.v2',sha256=hashlib.sha256(payload).hexdigest(),sources=code)
