"""R17 task clarification only: byte-identical r16 physics and canonical pose."""
import hashlib,json,os,shutil
from pathlib import Path
from . import v2
from ..run_setup import write_json

RULE_REVISION='relay-goal-low-release-20261026'
RULE=('Within GOAL, original qualified hands may lower an assembly while its '
      'components contact the table separately. The final team may open with '
      'the lowest timber at most20mm above the table, without prior table '
      'contact. Every complete assembly must finally be released, actually '
      'supported, stable and entirely inside GOAL. Real airborne handover '
      'remains required; unsupported loss outside GOAL or from a high pose '
      'does not receive this low-release permission.')


def build(case,replace_draft=False,render=False):
    if case['family']!='relay' or case['level']==1:raise ValueError('R17 changes only L2/L3 release rules')
    source=v2.directory(dict(case,scene_revision=16));dest=v2.directory(dict(case,scene_revision=17))
    if dest.exists():raise FileExistsError(dest)
    def copy(path,target):
        if Path(path).suffix=='.json':return shutil.copy2(path,target)
        try:os.link(path,target)
        except OSError:shutil.copy2(path,target)
        return target
    shutil.copytree(source,dest,copy_function=copy,ignore=shutil.ignore_patterns('FROZEN'))
    newcase=dict(case,scene_revision=17,source_scene_revision=16)
    newcase['directory']=str(dest.relative_to(v2.ROOT))
    task=json.loads((dest/'task.json').read_text());before=hashlib.sha256((source/'task.json').read_bytes()).hexdigest()
    task.update(scale=newcase,scene_asset=newcase['directory'],oracle_verified=False,execution_readiness='environment_preview_only')
    task['completion'].update(allow_goal_partial_support=True,allow_near_goal_release=True,
                              max_timber_release_gap_m=.020,rule_revision=RULE_REVISION)
    task['rules'].append(RULE)
    write_json(dest/'task.json',task);write_json(dest/'public-task.json',v2.public_task(task))
    env=json.loads((dest/'environment.json').read_text());env['suite_case']=newcase;write_json(dest/'environment.json',env)
    manifest=json.loads((dest/'workstation.json').read_text());manifest['case']=newcase;write_json(dest/'workstation.json',manifest)
    unchanged=[]
    for path in source.rglob('*'):
        if not path.is_file() or path.suffix=='.json' or path.name=='FROZEN':continue
        relative=path.relative_to(source)
        if hashlib.sha256(path.read_bytes()).digest()!=hashlib.sha256((dest/relative).read_bytes()).digest():raise ValueError('Physics input changed: '+str(relative))
        unchanged.append(str(relative))
    write_json(dest/'rule-revision.json',dict(user_authorized=True,rule_revision=RULE_REVISION,
        source_directory=str(source.relative_to(v2.ROOT)),source_task_sha256=before,
        scene_sha256=task['scene_sha256'],physics_files_unchanged=unchanged,
        geometry_revision=16,scope='New canonical task contract only; original frozen scenes/episodes retained; L1 unchanged'))
    return dest
