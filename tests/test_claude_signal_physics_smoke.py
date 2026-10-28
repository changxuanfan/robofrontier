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
        directory=Path(tempfile.mkdtemp(prefix='claude-signal-physics-'))
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),MotionModel)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        MotionModel.requests.clear()
        process=None; safely_closed=False
        try:
            batch=directory/'batch'; case='transport-a04-d1'
            prepare_draft(batch,['opus'],cases=[case],topology='adaptive')
            run=batch/'opus'/case
            launch=run/'private/inputs/claude.json'; data=json.loads(launch.read_text())
            data['profile'].update(base_url=f'http://127.0.0.1:{server.server_port}',env_key='FAKE_KEY')
            launch.write_text(json.dumps(data))
            manifest=run/'private/inputs/manifest.json'; frozen=json.loads(manifest.read_text())
            frozen['input_sha256']['private/inputs/claude.json']=hashlib.sha256(launch.read_bytes()).hexdigest()
            manifest.write_text(json.dumps(frozen))
            env={key:os.environ[key] for key in ('PATH','HOME','LANG','LC_ALL','TMPDIR','DISPLAY','MUJOCO_GL') if key in os.environ}
            env['FAKE_KEY']='synthetic'
            cmd=[sys.executable,'-m','scripts.claude_benchmark','run-engineering',str(batch),
                 '--parallel','1','--turn-budget-usd','.5','--task-budget-usd','2',
                 '--wall-limit-s','120','--confirm-engineering-spend']
            with (directory/'supervisor.log').open('w') as log:
                process=subprocess.Popen(cmd,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log)
                deadline=time.monotonic()+90
                while time.monotonic()<deadline and process.poll() is None:
                    state=status_batch(batch)
                    detail=state['runs'][0]['detail'] or {}
                    if detail.get('status')=='running' and MotionModel.requests: break
                    time.sleep(.1)
                else:
                    self.fail('Owner or synthetic model did not start; retained '+str(directory))
                owner_pid=detail['owner_pid']
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=60)
                self.assertEqual(process.returncode,0,(directory/'supervisor.log').read_text()[-3000:])
            state=status_batch(batch); detail=state['runs'][0]['detail']
            self.assertEqual(state['status'],'stopped',state)
            self.assertFalse(state['supervisor_live'],state)
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
