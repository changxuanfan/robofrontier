"""MCP failure semantics without Claude CLI, paid inference or physics."""
import json
from pathlib import Path
import tempfile
import unittest

from robofrontier.claude.robot_mcp import RobotBridge
from robofrontier.tool_errors import CallIdConflict


class ClaudeToolErrorTests(unittest.TestCase):
    def test_conflict_and_unknown_failures_are_distinct_and_never_retried(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp)/'run';connection=run/'private/state/connections/Controller.json'
            connection.parent.mkdir(parents=True)
            connection.write_text(json.dumps({'run_id':'run','actor':'Controller'}))
            for error,code,status in ((CallIdConflict('original-id'),'CALL_ID_CONFLICT','rejected'),
                                      (TimeoutError('sk-fake-secret'),'TRANSPORT_UNCERTAIN','unknown'),
                                      (ValueError('not a typed rejection'),'TRANSPORT_UNCERTAIN','unknown')):
                calls=[]
                def invoke(*args,**kwargs):
                    calls.append(args)
                    raise error
                reply=RobotBridge(run,connection,invoke).call({'op':'move','data':{},'call_id':'original-id'})
                self.assertTrue(reply['isError'])
                value=json.loads(reply['content'][0]['text'])
                self.assertEqual(value['error_code'],code)
                self.assertEqual(value['request_status'],status)
                self.assertEqual(value['call_id'],'original-id')
                self.assertEqual(value['original_call_status'],'unknown')
                self.assertEqual(len(calls),1)
                self.assertNotIn('sk-fake-secret',json.dumps(reply))


if __name__=='__main__':unittest.main()
