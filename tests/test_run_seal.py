"""Engineering seal uses the existing owner or records loss; never rebuilds it."""
import json
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from fake_pipeline import FakeWorld,fake_capture
from robofrontier.run_setup import prepare_run,write_json
from robofrontier.runtime_cli import execute
from robofrontier.runtime_service import RuntimeService
from robofrontier.runtime_store import RuntimeStore


class SealTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.run=Path(self.tmp.name)/'run';prepare_run(self.run)
        self.store=RuntimeStore(self.run/'private/state/runtime.sqlite')
    def tearDown(self):self.store.close();self.tmp.cleanup()
    def seal(self,timeout=.1):return execute(self.store,'seal',{'timeout_s':timeout},'test-meta',time.time())

    def test_lost_owner_seal_marks_accepted_motion_unknown_without_replay(self):
        self.store.put('execution_owner_attached',True);self.store.put('lifecycle','running')
        self.store.begin_call('lost','A','move',{'target_m':[.4,0,.6]})
        result=self.seal()
        self.assertEqual(result['lifecycle'],'interrupted');self.assertFalse(result['fresh_observation'])
        self.assertEqual(self.store.actions()['lost']['status'],'uncertain')
        self.assertEqual(self.store.call('lost')['status'],'uncertain')
        self.assertTrue(self.store.get('dispatch_paused'))
        before=len(self.store.events());self.seal()
        self.assertEqual(before,len(self.store.events()))

    def test_live_pending_motion_remains_paused_and_owner_is_not_closed(self):
        service=RuntimeService(self.run,FakeWorld,fake_capture)
        try:
            self.store.put('lifecycle','running');self.store.begin_call('pending','A','move',{'target_m':[.4,0,.6]})
            with self.assertRaisesRegex(ValueError,'execution'):self.seal(.02)
            self.assertTrue(self.store.get('dispatch_paused'));self.assertFalse(service.shutdown)
            self.assertEqual(self.store.call('pending')['status'],'accepted')
        finally:service.close()

    def test_prepared_run_does_not_start_or_seal_an_unattached_world(self):
        with self.assertRaisesRegex(ValueError,'owner'):self.seal()
        self.assertFalse(self.store.get('execution_owner_attached'))

    def test_seal_checks_the_declared_recording_camera_file(self):
        path=self.run/'private/inputs/task.json';task=json.loads(path.read_text())
        task['recording_camera']='board';write_json(path,task)
        self.store.put('execution_owner_attached',True);self.store.put('lifecycle','ended')
        with patch('robofrontier.run_seal.probe_video',return_value={'readable':True}) as probe:
            result=self.seal()
        expected=(self.run/'private/outputs/board-realtime.mp4').resolve()
        probe.assert_called_once_with(expected)
        self.assertEqual(result['video']['path'],str(expected));self.assertTrue(result['video']['readable'])

    def test_live_idle_owner_gets_only_observe_and_shutdown(self):
        service=RuntimeService(self.run,FakeWorld,fake_capture);world=service.world
        self.store.put('lifecycle','running');ops=[]
        source=self.run/'public/observations/captured-global.png';source.write_bytes(b'fake capture bytes')
        service.capture=lambda cameras:{'images':{'global':str(source)},'scope':'fake capture, not rendered pixels'}
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_POST(self):
                b=json.loads(self.rfile.read(int(self.headers['Content-Length'])));ops.append(b['op'])
                if self.headers['X-Token']!=service.tokens['Meta']:self.send_error(403);return
                j=service.accept(b['actor'],b['op'],b['data'],b['call_id'],b.get('ack_delivery'));service.pump()
                raw=json.dumps(j['result']).encode();self.send_response(200);self.end_headers();self.wfile.write(raw)
                if service.shutdown:service.close()
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        connection=self.run/'private/state/connections/Meta.json';cfg=json.loads(connection.read_text())
        cfg['url']=f'http://127.0.0.1:{server.server_port}';write_json(connection,cfg)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            result=self.seal(3)
            self.assertEqual(ops,['observe','shutdown'])
            self.assertEqual(result['lifecycle'],'interrupted');self.assertTrue(result['fresh_observation'])
            self.assertEqual((self.run/'private/outputs/final-global.png').read_bytes(),source.read_bytes())
            self.assertEqual(world.executed,[])
            with self.assertRaisesRegex(ValueError,'Physical restart'):
                RuntimeService(self.run,FakeWorld,fake_capture)
        finally:
            server.shutdown();server.server_close()
            if service.store:service.close()


    def test_live_idle_owner_preserves_jpeg_final(self):
        service=RuntimeService(self.run,FakeWorld,fake_capture);world=service.world
        self.store.put('lifecycle','running');ops=[]
        source=self.run/'public/observations/captured-global.jpg';source.write_bytes(b'fake capture bytes')
        service.capture=lambda cameras:{'images':{'global':str(source)},'scope':'fake capture, not rendered pixels'}
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_POST(self):
                b=json.loads(self.rfile.read(int(self.headers['Content-Length'])));ops.append(b['op'])
                if self.headers['X-Token']!=service.tokens['Meta']:self.send_error(403);return
                j=service.accept(b['actor'],b['op'],b['data'],b['call_id'],b.get('ack_delivery'));service.pump()
                raw=json.dumps(j['result']).encode();self.send_response(200);self.end_headers();self.wfile.write(raw)
                if service.shutdown:service.close()
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        connection=self.run/'private/state/connections/Meta.json';cfg=json.loads(connection.read_text())
        cfg['url']=f'http://127.0.0.1:{server.server_port}';write_json(connection,cfg)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            result=self.seal(3)
            self.assertEqual(ops,['observe','shutdown'])
            self.assertEqual(result['lifecycle'],'interrupted');self.assertTrue(result['fresh_observation'])
            self.assertEqual((self.run/'private/outputs/final-global.jpg').read_bytes(),source.read_bytes())
            self.assertEqual(world.executed,[])
            with self.assertRaisesRegex(ValueError,'Physical restart'):
                RuntimeService(self.run,FakeWorld,fake_capture)
        finally:
            server.shutdown();server.server_close()
            if service.store:service.close()


if __name__=='__main__':unittest.main()
