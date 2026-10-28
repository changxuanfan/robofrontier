"""Opt-in signal shutdown check with real CLI/MuJoCo and a loopback model."""
import hashlib
import http.server
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from test_claude_physics_smoke import MotionModel
from robofrontier.claude.engineering_batch import status_batch
from scripts.claude_benchmark import prepare_draft


@unittest.skipUnless(os.environ.get('ROBOFRONTIER_PHYSICS_SIGNAL') == '1' and shutil.which('claude'),
                     'Opt-in signal smoke requires Claude CLI and scene assets')
class SignalPhysicsSmokeTests(unittest.TestCase):
    def test_sigterm_stops_and_seals_actual_owner(self):
        self.check_sigterm(four_cases=False)

    def test_sigterm_stops_and_seals_four_actual_owners(self):
        self.check_sigterm(four_cases=True)

    def check_sigterm(self, four_cases):
        directory=Path(tempfile.mkdtemp(prefix='claude-signal-physics-'))
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),MotionModel)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        MotionModel.requests.clear()
        process=None; safely_closed=False
        try:
            batch=directory/'batch'
            if four_cases:
                prepare_draft(batch,['opus'],level=1,min_arms=True,topology='adaptive')
            else:
                prepare_draft(batch,['opus'],cases=['transport-a04-d1'],topology='adaptive')
            jobs=json.loads((batch/'batch.json').read_text())['jobs']
            self.assertEqual(len(jobs),4 if four_cases else 1)
            for job in jobs:
                run=Path(job['run_directory'])
                launch=run/'private/inputs/claude.json'; data=json.loads(launch.read_text())
                data['profile'].update(base_url=f'http://127.0.0.1:{server.server_port}',env_key='FAKE_KEY')
                launch.write_text(json.dumps(data))
                manifest=run/'private/inputs/manifest.json'; frozen=json.loads(manifest.read_text())
                frozen['input_sha256']['private/inputs/claude.json']=hashlib.sha256(launch.read_bytes()).hexdigest()
                manifest.write_text(json.dumps(frozen))
            env={key:os.environ[key] for key in ('PATH','HOME','LANG','LC_ALL','TMPDIR','DISPLAY','MUJOCO_GL') if key in os.environ}
            env['FAKE_KEY']='synthetic'
            cmd=[sys.executable,'-m','scripts.claude_benchmark','run-engineering',str(batch),
                 '--parallel',str(len(jobs)),'--turn-budget-usd','.5','--task-budget-usd','2',
                 '--wall-limit-s','120','--confirm-engineering-spend']
            with (directory/'supervisor.log').open('w') as log:
                process=subprocess.Popen(cmd,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log)
                deadline=time.monotonic()+90
                while time.monotonic()<deadline and process.poll() is None:
                    state=status_batch(batch)
                    details=[item['detail'] or {} for item in state['runs']]
                    if all(item.get('status')=='running' for item in details) and MotionModel.requests: break
                    time.sleep(.1)
                else:
                    self.fail('Owner or synthetic model did not start; retained '+str(directory))
                owner_pids=[item['owner_pid'] for item in details]
                self.assertEqual(len(set(owner_pids)),len(jobs))
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=60)
                self.assertEqual(process.returncode,0,(directory/'supervisor.log').read_text()[-3000:])
            state=status_batch(batch)
            self.assertEqual(state['status'],'stopped',state)
            self.assertFalse(state['supervisor_live'],state)
            for item,owner_pid in zip(state['runs'],owner_pids):
                detail=item['detail']
                self.assertEqual(detail['reason'],'stop_requested',detail)
                self.assertEqual(detail['status'],'finished',detail)
                self.assertTrue(detail['seal']['sealed'],detail)
                self.assertFalse(detail['owner_alive'],detail)
                with self.assertRaises(ProcessLookupError): os.kill(owner_pid,0)
            safely_closed=True
        finally:
            if process is not None and process.poll() is None:
                # Preserve the world and ask the operator to inspect it.
                print('Supervisor still running; retained for review:',directory)
            server.shutdown(); server.server_close()
            if safely_closed: shutil.rmtree(directory)
            else: print('Uncertain signal smoke retained for review:',directory)


if __name__=='__main__': unittest.main()
