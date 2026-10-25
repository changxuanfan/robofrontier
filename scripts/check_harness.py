"""Offline checks: fake services, robot/contact physics and real encoder recovery."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import py_compile
import re
import subprocess
import sys
import unittest
from unittest.mock import patch
from urllib.parse import unquote
from zoneinfo import ZoneInfo

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'tests'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path)
    p.add_argument('--skill-validator',type=Path)
    p.add_argument('--skill-python',default=sys.executable,help='Interpreter with the optional validator dependencies installed')
    p.add_argument('--protocol-only',action='store_true',help='Run explicit non-physics contracts; prohibit physics-world creation/stepping')
    args=p.parse_args()
    guard=None
    if args.protocol_only:
        import mujoco
        def forbidden(*args,**kwargs):raise AssertionError('Physics execution prohibited by --protocol-only')
        class NoPhysicsModel:
            from_xml_path=staticmethod(forbidden)
            from_xml_string=staticmethod(forbidden)
            from_binary_path=staticmethod(forbidden)
        guard=patch.multiple(mujoco,MjModel=NoPhysicsModel,MjData=forbidden,mj_step=forbidden,mj_forward=forbidden)
        guard.start()
        modules=('test_action_batch','test_sync_groups','test_beam_policy','test_spring_dial','test_spring_dial_policy','test_pipeline','test_pipeline_client',
                 'test_runtime_store','test_run_setup','test_scheduling','test_centralized','test_delegation',
                 'test_delegate_callbacks','test_private_evaluation','test_run_seal','test_run_report',
                 'test_session_mailbox_demo','test_transport_schedule','test_recording')
        suite=unittest.defaultTestLoader.loadTestsFromNames(modules)
    else:suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'),pattern='test_*.py')
    try:result=unittest.TextTestRunner(verbosity=2).run(suite)
    finally:
        if guard:guard.stop()
    node=subprocess.run(['node',str(ROOT/'tests/test_meta_dispatcher.cjs')],capture_output=True,text=True,cwd=ROOT)
    print(node.stdout,end='');print(node.stderr,end='',file=sys.stderr)
    bridge=subprocess.run(['node',str(ROOT/'tests/test_role_tools.cjs')],capture_output=True,text=True,cwd=ROOT)
    print(bridge.stdout,end='');print(bridge.stderr,end='',file=sys.stderr)
    sequence=subprocess.run(['node',str(ROOT/'tests/test_robot_sequence.cjs')],capture_output=True,text=True,cwd=ROOT)
    print(sequence.stdout,end='');print(sequence.stderr,end='',file=sys.stderr)
    local_step=subprocess.run(['node',str(ROOT/'tests/test_role_step.cjs')],capture_output=True,text=True,cwd=ROOT)
    print(local_step.stdout,end='');print(local_step.stderr,end='',file=sys.stderr)
    compilation=[]
    for source in sorted(p for folder in ('robofrontier','scripts','tests') for p in (ROOT/folder).glob('*.py')):
        py_compile.compile(str(source),doraise=True);compilation.append(str(source.relative_to(ROOT)))
    # Parse the inline UI scripts without executing a browser or opening a port.
    ui_script="const fs=require('fs'),vm=require('vm');for(const f of process.argv.slice(1)){const s=fs.readFileSync(f,'utf8');for(const m of s.matchAll(/<script>([\\s\\S]*?)<\\/script>/g))new vm.Script(m[1]);}"
    ui=subprocess.run(['node','-e',ui_script,str(ROOT/'assets/pipeline_dashboard.html'),str(ROOT/'assets/pipeline_cameras.html')],capture_output=True,text=True)
    # Validate runnable documentation as syntax, without executing its tools.
    doc_js="const fs=require('fs');const AsyncFunction=Object.getPrototypeOf(async function(){}).constructor;const s=fs.readFileSync(process.argv[1],'utf8');for(const m of s.matchAll(/```javascript\\n([\\s\\S]*?)```/g))new AsyncFunction(m[1]);"
    runbook=subprocess.run(['node','-e',doc_js,str(ROOT/'docs/meta-runbook.md')],capture_output=True,text=True)
    tool_docs=subprocess.run(['node','-e',doc_js,str(ROOT/'.agents/skills/robot-collaboration/references/tools.md')],capture_output=True,text=True)
    broken_links=[]
    documents=[ROOT/'README.md',*(ROOT/'docs').rglob('*.md'),*(ROOT/'.agents/skills/robot-collaboration').rglob('*.md')]
    for document in documents:
        for target in re.findall(r'\[[^\]]*\]\(([^)]+)\)',document.read_text()):
            if '://' in target or target.startswith(('#','mailto:')):continue
            path=unquote(target.split('#')[0].strip('<>'))
            if path and not (document.parent/path).exists():broken_links.append({'file':str(document.relative_to(ROOT)),'target':target})
    validator=None
    if args.skill_validator:
        validator=subprocess.run([args.skill_python,str(args.skill_validator),str(ROOT/'.agents/skills/robot-collaboration')],capture_output=True,text=True)
        print(validator.stdout,end='');print(validator.stderr,end='',file=sys.stderr)
    passed=result.wasSuccessful() and node.returncode==0 and bridge.returncode==0 and sequence.returncode==0 and local_step.returncode==0 and ui.returncode==0 and runbook.returncode==0 and tool_docs.returncode==0 and not broken_links and (validator is None or validator.returncode==0)
    report={'checked_at':datetime.now(ZoneInfo('Asia/Hong_Kong')).isoformat(),'passed':passed,
            'scope':('Protocol-only engineering: FakeWorld/HTTP, private scoring on synthetic records, frozen inputs and encoder recovery. Physics creation/stepping prohibited; no native sessions, model inference or GUI.' if args.protocol_only else 'Offline engineering; synthetic sessions, FakeWorld/HTTP, robot kinematics/contacts and FFmpeg encode/decode/crash regressions when tools are installed. No live model inference, native session wakeups, GUI or multi-agent policy trial.'),
            'physics_entrypoints_blocked':args.protocol_only,
            'python_tests':{'run':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),'skipped':len(result.skipped)},
            'dispatcher_js':{'passed':node.returncode==0,'scenarios':23},
            'role_tool_bridge_js':{'passed':bridge.returncode==0,'scenarios':15,'error':bridge.stderr or None},
            'finite_sequence_js':{'passed':sequence.returncode==0,'error':sequence.stderr or None},
            'local_role_step_js':{'passed':local_step.returncode==0,'error':local_step.stderr or None},
            'ui_script_syntax':{'passed':ui.returncode==0,'error':ui.stderr or None},
            'runbook_script_syntax':{'passed':runbook.returncode==0,'error':runbook.stderr or None},
            'tool_example_script_syntax':{'passed':tool_docs.returncode==0,'error':tool_docs.stderr or None},
            'documentation_links':{'passed':not broken_links,'broken':broken_links},
            'skill_format':{'checked':validator is not None,'passed':validator.returncode==0 if validator else None,'interpreter':args.skill_python if validator else None},
            'compiled_modules':compilation,
            'source_sha256':{str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (ROOT/'robofrontier').glob('*.py')},
            'not_verified':['new live multi-agent MuJoCo policy trial','new live Codex adapter roundtrip','coordination/parallelism improvement']}
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('compiled_modules','source_sha256')},ensure_ascii=False,indent=2))
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
