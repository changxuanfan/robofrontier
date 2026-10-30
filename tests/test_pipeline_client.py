import json
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading
import subprocess
import sys
import unittest
from unittest.mock import patch
from robofrontier.pipeline_client import client_call

class ClientTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.path=self.root/'private/state/connections/A.json';self.path.parent.mkdir(parents=True)
        self.bodies=[];test=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_POST(self):
                body=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                test.bodies.append(body)
                if body['op']=='fake_blocking':
                    test.blocking_entered.set()
                    test.release_blocking.wait(2)
                n=sum(b['call_id']==body['call_id'] for b in test.bodies)
                if body['op'] in ('fake_pending','fake_async') and n==1:
                    result={'call_id':body['call_id'],'execution_status':'running' if body['op']=='fake_async' else 'pending'}
                elif body['op']=='wait_events' and n<3:
                    result={'call_id':body['call_id'],'execution_status':'pending'}
                elif body['op'] in ('move','gripper','action_batch') and body['data'].get('wait',True) and n<3:
                    result={'call_id':body['call_id'],'execution_status':'pending'}
                elif body['op']=='wait_events':
                    result={'call_id':body['call_id'],'wait_status':'events','messages':[], 'delivery_id':'delivery-1'}
                else:result={'call_id':body['call_id'],'delivery_id':'delivery-1','messages':[],'scope':'fake HTTP, no executor'}
                raw=json.dumps(result).encode();self.send_response(200);self.end_headers();self.wfile.write(raw)
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.blocking_entered=threading.Event();self.release_blocking=threading.Event()
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
        self.path.write_text(json.dumps({'schema_version':3,'run_id':'fake','actor':'A','url':f'http://127.0.0.1:{self.server.server_port}','token':'fake-test-token'}))
    def tearDown(self):self.server.shutdown();self.server.server_close();self.tmp.cleanup()
    def test_default_http_timeout_covers_bounded_database_wait(self):
        from robofrontier.pipeline_client import DEFAULT_HTTP_TIMEOUT_SECONDS
        from robofrontier.sqlite_busy import MAX_BEGIN_WAIT_SECONDS
        self.assertGreaterEqual(DEFAULT_HTTP_TIMEOUT_SECONDS, MAX_BEGIN_WAIT_SECONDS + 40)
        import urllib.request
        original = urllib.request.urlopen
        with patch('robofrontier.pipeline_client.urllib.request.urlopen', wraps=original) as call:
            client_call(self.path,'observe',{},'default-timeout',log_call_id=False)
            self.assertEqual(call.call_args.kwargs['timeout'],DEFAULT_HTTP_TIMEOUT_SECONDS)
            client_call(self.path,'observe',{},'explicit-timeout',timeout=2,log_call_id=False)
            self.assertEqual(call.call_args.kwargs['timeout'],2)

    def test_cached_retry_does_not_send_twice_and_next_call_acknowledges_receipt(self):
        one=client_call(self.path,'observe',{},'one')
        self.assertEqual(client_call(self.path,'observe',{},'one'),one)
        self.assertEqual(len(self.bodies),1)
        client_call(self.path,'work',{},'two')
        self.assertEqual(self.bodies[-1]['ack_delivery'],'delivery-1')
        with self.assertRaises(ValueError):client_call(self.path,'move',{'target_m':[1,2,3]},'one')
    def test_pending_request_keeps_same_id_and_payload(self):
        self.assertEqual(client_call(self.path,'fake_pending',{},'p')['execution_status'],'pending')
        self.assertEqual(client_call(self.path,'pending',{})['pending'][0]['call_id'],'p')
        client_call(self.path,'fake_pending',{},'p')
        self.assertEqual(self.bodies[0],self.bodies[1])

    def test_slow_blocking_actions_cross_http_slices_under_original_id(self):
        for op,data in [('move',{'robot':'A','delta_m':[0,0,.1],'speed_m_s':.005}),
                        ('gripper',{'robot':'A','width_m':.08,'speed_m_s':.005}),
                        ('action_batch',{'actions':[{'robot':'A','op':'move','parameters':{'delta_m':[0,0,.01]}}]})]:
            result=client_call(self.path,op,data,'slow-'+op,log_call_id=False)
            self.assertNotEqual(result.get('execution_status'),'pending')
            requests=[b for b in self.bodies if b['call_id']=='slow-'+op]
            self.assertEqual(len(requests),3)
            self.assertTrue(all(b==requests[0] for b in requests))
            before=len(self.bodies);self.assertEqual(client_call(self.path,op,data,'slow-'+op,log_call_id=False),result)
            self.assertEqual(before,len(self.bodies))
        self.assertEqual(client_call(self.path,'pending',{})['pending'],[])
    def test_offline_pending_and_invalid_file_identifier(self):
        with self.assertRaises(ValueError):client_call(self.path,'observe',{},'../../outside')
        cfg=json.loads(self.path.read_text());cfg['url']=None;self.path.write_text(json.dumps(cfg))
        self.assertEqual(client_call(self.path,'pending',{})['pending'],[])
        with self.assertRaisesRegex(ValueError,'never starts'):client_call(self.path,'observe',{})
        self.assertEqual(self.bodies,[])

    def test_async_acceptance_is_not_cached_as_terminal(self):
        result=client_call(self.path,'fake_async',{},'async')
        self.assertEqual(result['execution_status'],'running')
        self.assertEqual(client_call(self.path,'pending',{})['pending'][0]['call_id'],'async')
        client_call(self.path,'fake_async',{},'async')
        self.assertEqual(len(self.bodies),2)
        self.assertEqual(client_call(self.path,'pending',{})['pending'],[])

    def test_wait_transport_pending_does_not_force_a_new_model_decision_or_call_id(self):
        result=client_call(self.path,'wait_events',{'timeout_s':120},'wait-original')
        self.assertEqual(result['wait_status'],'events')
        self.assertEqual(len(self.bodies),3)
        self.assertTrue(all(b==self.bodies[0] for b in self.bodies))
        self.assertEqual(client_call(self.path,'pending',{})['pending'],[])

    def test_quiet_cli_error_keeps_original_id_and_pending_request(self):
        cfg=json.loads(self.path.read_text());cfg['url']='http://127.0.0.1:0'
        self.path.write_text(json.dumps(cfg))
        result=subprocess.run([sys.executable,'-m','robofrontier.pipeline_client',
                               '--connection',str(self.path),'--quiet','--call-id','held-id','observe'],
                              capture_output=True,text=True)
        self.assertEqual(result.returncode,1)
        self.assertEqual(json.loads(result.stdout)['call_id'],'held-id')
        self.assertEqual(result.stderr,'')
        self.assertEqual(client_call(self.path,'pending',{})['pending'][0]['call_id'],'held-id')

    def test_one_blocking_call_does_not_serialize_other_robot_or_mail_calls(self):
        errors=[];finished=threading.Event()
        def blocked():
            try:client_call(self.path,'fake_blocking',{},'long',log_call_id=False)
            except Exception as error:errors.append(error)
        def other():
            try:client_call(self.path,'work',{},'short',log_call_id=False)
            except Exception as error:errors.append(error)
            finally:finished.set()
        first=threading.Thread(target=blocked);first.start()
        self.assertTrue(self.blocking_entered.wait(1))
        second=threading.Thread(target=other);second.start()
        try:self.assertTrue(finished.wait(.5),'Client file lock serialized independent tool calls')
        finally:
            self.release_blocking.set();first.join();second.join()
        self.assertEqual(errors,[])

if __name__=='__main__':unittest.main()
