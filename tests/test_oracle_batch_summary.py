"""Batch success must not combine distinct driver profiles or partial samples."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from scripts.summarize_oracle_batch import summarize


class OracleBatchSummaryTests(unittest.TestCase):
    def test_profile_mismatch_and_partial_goal_do_not_pass_complete_case(self):
        with TemporaryDirectory() as folder:
            root=Path(folder);catalog=root/'catalog.json'
            catalog.write_text(json.dumps(dict(cases=[dict(id='case',family='mosaic')])))
            (root/'batch.json').write_text(json.dumps(dict(batch_id='test',catalog=str(catalog),families=['mosaic'],driver_profile='pd1000-v1')))
            attempt=root/'mosaic/case/seed-0/attempt-001';(attempt/'executor-source').mkdir(parents=True)
            harness=attempt/'executor-source/effective-harness.json';report=attempt/'report.json'
            record=dict(passed=True,evaluation=dict(task_success=True),reference_result={})
            report.write_text(json.dumps(record));harness.write_text(json.dumps(dict(gripper_position_servo='legacy')))
            self.assertFalse(summarize(root)['all_36_passed'])
            harness.write_text(json.dumps(dict(gripper_position_servo='pd1000-v1')))
            self.assertTrue(summarize(root)['all_36_passed'])
            record['reference_result']['commissioning_only']=True;report.write_text(json.dumps(record))
            result=summarize(root);self.assertFalse(result['all_36_passed'])
            self.assertTrue(result['cases'][0]['latest_result']['commissioning_only'])


if __name__=='__main__':unittest.main()
