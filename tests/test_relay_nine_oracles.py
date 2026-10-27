import unittest
from scripts.run_relay_nine_oracles import full_task_passed

class FullCaseTests(unittest.TestCase):
    def report(self):return dict(passed=True,commissioning_only=False,unresolved_robot_actions=[],evaluation=dict(task_success=True,completed_payloads=2,total_payloads=2,violations=[]))
    def test_only_complete_native_success_counts(self):self.assertTrue(full_task_passed(self.report(),0))
    def test_unit_and_partial_results_do_not_count(self):
        report=self.report();report['commissioning_only']=True;self.assertFalse(full_task_passed(report,0))
        report=self.report();report['evaluation']['completed_payloads']=1;self.assertFalse(full_task_passed(report,0))
    def test_unresolved_jobs_and_process_failures_do_not_count(self):
        report=self.report();report['unresolved_robot_actions']=['H'];self.assertFalse(full_task_passed(report,0))
        self.assertFalse(full_task_passed(self.report(),1))


class L1ReuseTests(unittest.TestCase):
    def test_reuse_requires_same_task_and_complete_original_recording(self):
        import hashlib,json,tempfile
        from pathlib import Path
        from scripts.run_relay_nine_oracles import reusable_l1_records
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);source=root/'current';run=root/'old';(run/'motion').mkdir(parents=True);source.mkdir()
            payload=b'{"original_task":true}';digest=hashlib.sha256(payload).hexdigest()
            (source/'task.json').write_bytes(payload);(run/'task.json').write_bytes(payload)
            report=FullCaseTests().report();report['task_contract_sha256']=digest;(run/'report.json').write_text(json.dumps(report))
            (run/'RECORDING-VERIFIED.json').write_text(json.dumps(dict(status='complete',frames_verified=12)));(run/'motion/stream.json').write_text(json.dumps(dict(status='complete',frames=12)))
            (root/'batch-status.json').write_text(json.dumps(dict(results=[dict(case_id='relay-a08-d1',passed=True,returncode=0,report=str(run/'report.json'))])))
            case=dict(id='relay-a08-d1',level=1,directory='current')
            self.assertTrue(reusable_l1_records(root,[case],root=root)[0]['imported'])
            (source/'task.json').write_text('changed')
            with self.assertRaisesRegex(ValueError,'contract changed'):reusable_l1_records(root,[case],root=root)

if __name__=='__main__':unittest.main()
