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

if __name__=='__main__':unittest.main()
