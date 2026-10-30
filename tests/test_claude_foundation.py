import base64
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.claude_benchmark import doctor, plan, profile
from robofrontier.claude.robot_mcp import RobotBridge, handle


class ClaudeFoundationTests(unittest.TestCase):
    def test_doctor_loads_env_without_disclosing_credentials_and_environment_wins(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'.env').write_text('FAKE_KEY=file-secret\nFAKE_BASE=https://file.example/api\n')
            catalog=root/'catalog.json'
            catalog.write_text(json.dumps({'schema':'robofrontier.claude-models.v1','profiles':{
                'fake':{'provider':'fake','model':'model','env_key':'FAKE_KEY',
                        'base_url_env':'FAKE_BASE','effort':'high'}}}))
            from robofrontier.task_families import RELEASE
            release=root/RELEASE; release.parent.mkdir(parents=True)
            release.write_text(json.dumps({'families':{}}))
            with patch.dict('os.environ',{'FAKE_KEY':'environment-secret'},clear=True), \
                 patch('scripts.claude_benchmark.shutil.which',return_value=None):
                result=doctor(['fake'],root=root,catalog=catalog)
                self.assertTrue(result['profiles']['fake']['credential_set'])
                self.assertTrue(result['profiles']['fake']['base_url_configured'])
                import os
                self.assertEqual(os.environ['FAKE_KEY'],'environment-secret')
                self.assertEqual(os.environ['FAKE_BASE'],'https://file.example/api')
                self.assertNotIn('environment-secret',json.dumps(result))
                self.assertNotIn('file-secret',json.dumps(result))
                self.assertFalse(result['profiles']['fake']['live_verified'])
                self.assertEqual(result['recording_tools'],{'ffmpeg':False,'ffprobe':False})

    def test_plan_four_minimum_level_one_tasks_without_execution(self):
        result = plan(['kimi'], level=1, min_arms=True, parallel=4)
        self.assertEqual(result['task_count'], 4)
        self.assertEqual([j['family'] for j in result['jobs']], ['transport', 'mosaic', 'relay', 'dial'])
        self.assertEqual([j['arms'] for j in result['jobs']], [4, 4, 8, 4])
        self.assertEqual(result['model_calls'], 0)
        self.assertFalse(result['execution_available'])

    def test_precheck_uses_haiku_and_bench_keeps_opus(self):
        self.assertEqual(profile('haiku')['model'], 'claude-haiku-5-5')
        self.assertEqual(profile('opus')['model'], 'claude-opus-5-5')
        self.assertEqual(profile('haiku')['env_key'], 'ANTHROPIC_API_KEY')

    def test_kimi_code_is_separate_from_moonshot_profile(self):
        code = profile('kimi_code')
        self.assertEqual(code['model'], 'k3.1-pro')
        self.assertEqual(code['base_url'], 'https://api.kimi.com/coding/')
        self.assertEqual(code['env_key'], 'KIMI_CODE_API_KEY')
        self.assertNotEqual(code['env_key'], profile('kimi')['env_key'])
        self.assertEqual(plan(['kimi_code'], level=1, min_arms=True)['task_count'], 4)

    def test_invalid_provider_address_and_parallel_are_rejected(self):
        with patch.dict('os.environ', {'DASHSCOPE_ANTHROPIC_BASE_URL': 'https://secret:pass@example.com'}):
            with self.assertRaises(ValueError): profile('qwen')
        with self.assertRaises(ValueError): plan(['kimi'], parallel=0)
        with self.assertRaises(ValueError): plan(['kimi', 'kimi'])

    def test_role_binding_image_and_stable_call_id(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'; connection = run / 'private/state/connections/Controller.json'
            connection.parent.mkdir(parents=True)
            connection.write_text(json.dumps({'run_id': 'run', 'actor': 'Controller', 'token': 'secret'}))
            image = run / 'public/observations/scene.png'; image.parent.mkdir(parents=True)
            png = b'\x89PNG\r\n\x1a\nimage'; image.write_bytes(png)
            calls = []
            def invoke(conn, op, data, call_id, **kwargs):
                calls.append((conn, op, data, call_id))
                return {'observation': {'images': {'top': str(image)}}}
            bridge = RobotBridge(run, connection, invoke)
            with self.assertRaisesRegex(ValueError, 'stable call_id'):
                bridge.call({'op': 'move', 'data': {}})
            response = bridge.call({'op': 'move', 'data': {}, 'call_id': 'move-1', 'views': ['top']})
            self.assertEqual(calls[0][1:], ('move', {'cameras': ['top']}, 'move-1'))
            self.assertEqual(base64.b64decode(response['content'][1]['data']), png)
            self.assertNotIn('secret', json.dumps(response))
            bridge.call({'op':'stop','call_id':'stop-1',
                         'data':{'robot':'A','call_id':'move-1'}})
            self.assertEqual(calls[-1][1:], ('stop', {'robot':'A','call_id':'move-1'}, 'stop-1'))
            bridge.call({'op':'job','data':{'call_id':'move-1'}})
            self.assertEqual(calls[-1][1:3], ('job', {'call_id':'move-1'}))
            bridge.call({'op':'observe','data':{'cameras':['top']},'views':['top']})
            self.assertEqual(calls[-1][1:3], ('observe', {'cameras':['top']}))
            with self.assertRaises(ValueError): bridge.call({'op': 'start', 'data': {}})
            self.assertEqual(handle({'method': 'tools/list'}, bridge)['tools'][0]['name'], 'robot')
            outside = Path(tmp) / 'outside.png'; outside.write_bytes(png)
            image.unlink(); image.symlink_to(outside)
            response = bridge.call({'op': 'observe', 'data': {}, 'views': ['top']})
            self.assertEqual(json.loads(response['content'][0]['text'])['observation']['images']['top'], str(image))
            self.assertIn('could not be loaded', response['content'][1]['text'])
            self.assertEqual(len(response['content']), 2)

    def test_observation_jpeg_uses_actual_bytes_and_mime(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp)/'run'; conn=run/'private/state/connections/Controller.json'
            conn.parent.mkdir(parents=True)
            conn.write_text(json.dumps({'run_id':'run','actor':'Controller'}))
            image=run/'public/observations/scene.png'; image.parent.mkdir(parents=True)
            from PIL import Image
            Image.new('RGB',(896,560),(30,100,180)).save(image,format='JPEG',quality=85)
            def invoke(conn,op,data,call_id,**kwargs):
                return {'call_id':call_id,'observation':{'images':{'top':str(image)}}}
            response=RobotBridge(run,conn,invoke).call(
                {'op':'move','data':{},'call_id':'move-1','views':['top']})
            self.assertEqual(response['content'][1]['mimeType'],'image/jpeg')
            self.assertEqual(base64.b64decode(response['content'][1]['data']),image.read_bytes())
            self.assertEqual(json.loads(response['content'][0]['text'])['call_id'],'move-1')

    def test_failed_action_image_keeps_execution_result_and_call_id(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp)/'run'; conn=run/'private/state/connections/Controller.json'
            conn.parent.mkdir(parents=True)
            conn.write_text(json.dumps({'run_id':'run','actor':'Controller'}))
            image=run/'public/observations/bad.png'; image.parent.mkdir(parents=True)
            image.write_bytes(b'not a png')
            def invoke(conn, op, data, call_id, **kwargs):
                return {'call_id':call_id,'execution':{'status':'finished'},
                        'observation':{'images':{'top':str(image)}}}
            bridge=RobotBridge(run,conn,invoke)
            response=bridge.call({'op':'move','data':{},'call_id':'motion-1','views':['top']})
            result=json.loads(response['content'][0]['text'])
            self.assertEqual(result['call_id'],'motion-1')
            self.assertEqual(result['execution']['status'],'finished')
            self.assertIn('could not be loaded',response['content'][1]['text'])

    def test_public_images_are_limited_to_frozen_task_allowlist(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'; connection = run / 'private/state/connections/Controller.json'
            connection.parent.mkdir(parents=True)
            connection.write_text(json.dumps({'run_id': 'run', 'actor': 'Controller'}))
            public = run / 'public/inputs'; public.mkdir(parents=True)
            (public / 'task.json').write_text(json.dumps({'initial_reference_images': ['family/view.png'],
                                                         'robot_layout_image': 'public/inputs/robot-layout.png',
                                                         'target': {'reference_image': 'target-pattern.png'}}))
            image = public / 'family/view.png'; image.parent.mkdir()
            png = b'\x89PNG\r\n\x1a\nimage'; image.write_bytes(png)
            (public / 'robot-layout.png').write_bytes(png)
            (public / 'target-pattern.png').write_bytes(png)
            bridge = RobotBridge(run, connection)
            response = bridge.call({'op': 'public_image', 'data': {'names': ['family/view.png']}})
            self.assertEqual(base64.b64decode(response['content'][1]['data']), png)
            layout = bridge.call({'op': 'public_image', 'data': {'names': ['public/inputs/robot-layout.png']}})
            self.assertEqual(base64.b64decode(layout['content'][1]['data']), png)
            target = bridge.call({'op': 'public_image', 'data': {'names': ['target-pattern.png']}})
            self.assertEqual(base64.b64decode(target['content'][1]['data']), png)
            with self.assertRaises(ValueError):
                bridge.call({'op': 'public_image', 'data': {'names': ['../private/secret.png']}})
            image.unlink(); image.symlink_to(Path(tmp) / 'outside.png')
            with self.assertRaises(ValueError):
                bridge.call({'op': 'public_image', 'data': {'names': ['family/view.png']}})

    def test_connection_cannot_be_manager(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'r'; conn = run / 'private/state/connections/Meta.json'; conn.parent.mkdir(parents=True)
            conn.write_text(json.dumps({'run_id': 'r', 'actor': 'Meta'}))
            with self.assertRaises(ValueError): RobotBridge(run, conn)

    def test_controller_can_read_only_runtime_authorized_delegate_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp)/'run'; conn=run/'private/state/connections/Controller.json'
            conn.parent.mkdir(parents=True)
            conn.write_text(json.dumps({'run_id':'run','actor':'Controller'}))
            actor='Delegate_123456abcdef'; reports=run/'public/results'; reports.mkdir(parents=True)
            report=reports/(actor+'.json')
            report.write_text(json.dumps({'actor':actor,'delegation_id':'batch1','result':'handback evidence'}))
            state={'state':'finished','delegation_id':'batch1','result_ref':str(report)}
            def invoke(*args, **kwargs): return {'delegate_status':{actor:state}}
            bridge=RobotBridge(run,conn,invoke)
            def read(selected=actor):
                return json.loads(bridge.call({'op':'work','data':{'delegate_id':selected}})['content'][0]['text'])
            self.assertEqual(read()['delegate_report']['result'],'handback evidence')
            self.assertNotIn('delegate_report',read('Delegate_000000000000'))
            state['state']='active'
            self.assertNotIn('delegate_report',read())
            state['state']='finished'; state['delegation_id']='other'
            self.assertTrue(read()['delegate_report_unavailable'])
            state['delegation_id']='batch1'
            report.write_text('x' * 128_001)
            self.assertTrue(read()['delegate_report_unavailable'])
            state['delegation_id']=None
            self.assertTrue(read()['delegate_report_unavailable'])
            state['delegation_id']='batch1'
            outside=Path(tmp)/'private.json'; outside.write_text('{"secret":"private"}')
            report.unlink(); report.symlink_to(outside)
            self.assertTrue(read()['delegate_report_unavailable'])


if __name__ == '__main__': unittest.main()
