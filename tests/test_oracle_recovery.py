"""Public recovery seam, using real Trial/PandaArm rather than a mock controller."""
import json
import hashlib
from pathlib import Path
import tempfile
import unittest

from robofrontier.suite.oracle import Trial
from robofrontier.suite.oracle_recovery import OracleRecovery, RecoverablePlanError


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'configs/environments/cube_transport/frontier-v2-r10-transport-a04-d1'
HARNESS = ROOT / 'configs/harness-frontier-v2.json'


class OracleRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.trial = Trial(SCENE, Path(self.tmp.name)/'world', frames=False,
                           controller_cfg=json.loads(HARNESS.read_text()))
        self.extra_observation = {}
        self.closed = False
        self.adapter = OracleRecovery(
            self.trial, owner='owner', world_id='world-1',
            observe=lambda: {'robots': {r:self.trial.arms[r].observe() for r in ('A','B')},
                             **self.extra_observation})

    def tearDown(self):
        if not self.closed:
            self.adapter.pause('owner')
            self.adapter.advance(.5)
            self.adapter.close({'scope':'unit harness test', 'passed':False})
        self.tmp.cleanup()

    def request(self, ident, robot, sequence, **extra):
        return dict(owner='owner', world_id='world-1', id=ident, robot=robot,
                    sequence=sequence, review='Independent empty-hand gripper test', **extra)

    def test_repeated_known_idle_handover_reuses_one_execution_owner(self):
        rec=self.adapter.recover(RecoverablePlanError('Review A'),['A'])
        self.adapter.acknowledge('owner',rec['id'],'No action needed after review')
        self.adapter.handoff('owner')
        with self.assertRaises(ValueError):self.adapter.reattach('other')
        self.adapter.reattach('owner')
        rec=self.adapter.recover(RecoverablePlanError('Review B'),['B'])
        self.adapter.submit(self.request('cycle-2','B',[
            {'op':'gripper','parameters':{'width_m':.06,'speed_m_s':.05}},
        ],recovery_id=rec['id']))
        self.adapter.advance(1.5)
        self.assertEqual(self.adapter.status('cycle-2')['status'],'completed')
        self.adapter.handoff('owner');self.adapter.reattach('owner')
        self.assertEqual(len(self.adapter.recoveries),2)

    def test_other_admitted_sequence_advances_during_scoped_plan_recovery(self):
        self.adapter.submit(self.request('b-sequence', 'B', [
            {'op':'gripper','parameters':{'width_m':.02,'speed_m_s':.01}},
            {'op':'gripper','parameters':{'width_m':.08,'speed_m_s':.01}},
        ]))
        before = self.trial.arms['B'].observe()['gripper_width_m']
        try:
            raise RecoverablePlanError('Explicit harness-only A branch fault')
        except RecoverablePlanError as error:
            recovery = self.adapter.recover(error, ['A'], evidence={'task':{'scope':'harness test'}})
        self.adapter.advance(.5)
        self.assertLess(self.trial.arms['B'].observe()['gripper_width_m'], before-.002)
        self.assertEqual(self.adapter.status('b-sequence')['status'], 'running')
        self.adapter.submit(self.request('a-repair', 'A', [
            {'op':'gripper','parameters':{'width_m':.06,'speed_m_s':.05}},
        ], recovery_id=recovery['id']))
        self.adapter.advance(1.5)
        self.assertEqual(self.adapter.status('a-repair')['status'], 'completed')
        self.assertEqual(self.adapter.status('b-sequence')['status'], 'running')
        self.assertGreater(self.trial.time, 1.9)

    def test_only_matching_owner_world_and_recovery_can_resume_affected_robot(self):
        recovery = self.adapter.recover(RecoverablePlanError('A needs owner review'), ['A'])
        steps = [{'op':'gripper','parameters':{'width_m':.06,'speed_m_s':.05}}]
        for change in ({'owner':'other'}, {'world_id':'other'}, {'recovery_id':'missing'}):
            request = self.request('bad-repair', 'A', steps, recovery_id=recovery['id'])
            request.update(change)
            with self.assertRaises(ValueError):
                self.adapter.submit(request)
        with self.assertRaises(ValueError):
            self.adapter.submit(self.request('uncorrelated', 'A', steps))
        self.adapter.submit(self.request('matching', 'A', steps, recovery_id=recovery['id']))
        self.adapter.advance(1.5)
        self.assertEqual(self.adapter.status('matching')['status'], 'completed')

    def test_owner_file_source_hash_and_assisted_final_report_are_preserved(self):
        recovery = self.adapter.recover(RecoverablePlanError('Owner repair required'), ['A'])
        source = self.adapter.directory/'owner-repair.py'
        source.write_text("# SHA-versioned external owner recipe, never hot-executed\n")
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        request = self.request('repair-file', 'A', [
            {'op':'gripper','parameters':{'width_m':.06,'speed_m_s':.05}},
        ], recovery_id=recovery['id'], assisted=True,
            repair_source={'path':str(source),'sha256':'0'*64})
        with self.assertRaises(ValueError):
            self.adapter.submit(request)
        request['repair_source']['sha256'] = digest
        (self.adapter.directory/'requests'/'owner.json').write_text(json.dumps(request))
        self.adapter.advance(1.5)
        self.assertEqual(self.adapter.status('repair-file')['status'], 'completed')
        before = self.trial.time
        self.assertEqual(self.adapter.submit(request)['status'], 'completed')
        self.assertEqual(self.trial.time, before)
        changed = json.loads(json.dumps(request))
        changed['sequence'][0]['parameters']['width_m'] = .04
        with self.assertRaises(ValueError):
            self.adapter.submit(changed)
        original_executor = self.trial.executor_sha256
        self.adapter.close({'scope':'assistance metadata unit test', 'passed':False})
        self.closed = True
        report = json.loads((self.trial.directory/'report.json').read_text())
        self.assertTrue(report['llm_used'])
        self.assertEqual(report['oracle_mode'], 'privileged_assisted')
        self.assertEqual(report['executor_sha256'], original_executor)
        self.assertEqual(report['recovery']['sources'][0]['sha256'], digest)
        event = json.loads((self.trial.directory/'trace.jsonl').read_text().splitlines()[-1])
        self.assertTrue(event['result']['llm_used'])

    def test_world_cannot_be_attached_by_a_second_adapter(self):
        with self.assertRaises(ValueError):
            OracleRecovery(self.trial, owner='other', world_id='other', observe=lambda:{})

    def test_fenced_execution_is_not_replayed_after_terminal_feedback(self):
        steps = [{'op':'gripper','parameters':{'width_m':.02,'speed_m_s':.01}}]
        request = self.request('a-original', 'A', steps)
        self.adapter.submit(request)
        self.adapter.submit(self.request('b-independent', 'B', [
            {'op':'gripper','parameters':{'width_m':.06,'speed_m_s':.05}},
        ]))
        self.adapter.fence('owner', 'a-original', 'Owner acknowledgement is uncertain')
        self.adapter.advance(7.)
        self.assertEqual(self.adapter.status('b-independent')['status'], 'completed')
        status = self.adapter.status('a-original')
        self.assertEqual(status['status'], 'uncertain')
        self.assertEqual(status['terminal_feedback']['status'], 'completed')
        self.assertEqual(self.adapter.submit(request)['status'], 'uncertain')
        with self.assertRaises(ValueError):
            self.adapter.submit(self.request('a-new-id', 'A', steps))
        with self.assertRaises(ValueError):
            self.adapter.handoff('owner')
        self.adapter.close({'scope':'uncertain reporting unit test', 'passed':True})
        self.closed = True
        final = json.loads((self.trial.directory/'report.json').read_text())
        self.assertFalse(final['passed'])
        self.assertIn('A', final['unresolved_robot_actions'])

    def test_hidden_or_non_json_observation_is_not_exposed_or_a_physics_barrier(self):
        self.adapter.submit(self.request('b-observe', 'B', [
            {'op':'gripper','parameters':{'width_m':.02,'speed_m_s':.01}},
        ]))
        self.adapter.service()
        before = self.trial.arms['B'].observe()['gripper_width_m']
        self.extra_observation['game'] = {'secret':'must-never-be-written'}
        with self.assertRaises(ValueError):
            self.adapter.observation()
        self.adapter.advance(.7)
        self.assertLess(self.trial.arms['B'].observe()['gripper_width_m'], before-.002)
        self.assertNotIn('must-never-be-written',
                         (self.adapter.directory/'observation.json').read_text())
        self.extra_observation.clear()
        self.extra_observation['objects'] = self.trial
        with self.assertRaises(ValueError):
            self.adapter.observation()
        self.extra_observation.clear()

    def test_known_failed_grasp_does_not_cancel_other_independent_sequence(self):
        self.adapter.submit(self.request('b-long', 'B', [
            {'op':'gripper','parameters':{'width_m':.02,'speed_m_s':.005}},
        ]))
        self.adapter.submit(self.request('a-no-object', 'A', [
            {'op':'gripper','parameters':{'mode':'grasp','width_m':.04,
                                        'force_n':30.,'speed_m_s':.1}},
            {'op':'gripper','parameters':{'width_m':.08,'speed_m_s':.05}},
        ]))
        self.adapter.advance(3.)
        self.assertEqual(self.adapter.status('a-no-object')['status'], 'failed')
        self.assertEqual(len(self.adapter.status('a-no-object')['feedback']), 1)
        self.assertLess(self.trial.arms['A'].observe()['gripper_width_m'], .005)
        self.assertEqual(self.adapter.status('b-long')['status'], 'running')
        self.adapter.pause('owner')
        self.adapter.advance(1.)
        self.adapter.close({'scope':'pure known failure unit test', 'passed':False})
        self.closed = True
        final = json.loads((self.trial.directory/'report.json').read_text())
        self.assertFalse(final['llm_used'])
        self.assertEqual(final['oracle_mode'], 'pure_program')
        self.assertGreaterEqual(final['failed_robot_actions'], 1)

    def test_foreign_inflight_action_is_not_polled_by_adapter(self):
        arm = self.trial.arms['C']
        prepared = arm.validate('gripper', {'width_m':.02,'speed_m_s':.01},
            self.trial.cfg, self.trial.task['workspace_world_m']['lower'],
            self.trial.task['workspace_world_m']['upper'])
        arm.start('foreign-original-id', prepared, self.trial.time)
        before = self.trial.time
        with self.assertRaises(ValueError):
            self.adapter.advance(.02)
        self.assertEqual(self.trial.time, before)
        self.assertEqual(arm.job, 'foreign-original-id')
        # Only the original external execution owner resolves that ID.
        arm.request_stop(self.trial.time, 'External owner test cleanup')
        result = None
        for _ in range(500):
            self.trial.step()
            if self.trial.tick % self.trial.stride == 0:
                result = arm.poll(self.trial.time, 15)
                if result:
                    break
        self.assertIsNotNone(result)
        self.assertEqual(result['call_id'], 'foreign-original-id')

    def test_recovery_does_not_clear_existing_process_violations(self):
        self.trial.monitor.violation('injected_harness_marker', key='retained-test')
        before = json.loads(json.dumps(self.trial.monitor.violations))
        with self.assertRaises(ValueError):
            self.adapter.recover(RuntimeError('Not an approved recoverable fault'), ['A'])
        self.adapter.recover(RecoverablePlanError('Owner may inspect, not erase'), ['A'])
        self.adapter.close({'scope':'retained violation unit test', 'passed':True})
        self.closed = True
        report = json.loads((self.trial.directory/'report.json').read_text())
        self.assertFalse(report['passed'])
        self.assertEqual(report['recovery']['retained_process_violations'], before)

    def test_owner_pause_stops_original_execution_without_starting_queued_steps(self):
        started = self.adapter.submit(self.request('b-pause', 'B', [
            {'op':'gripper','parameters':{'width_m':.02,'speed_m_s':.005}},
            {'op':'gripper','parameters':{'width_m':.08,'speed_m_s':.05}},
        ]))
        original = started['active_call_id']
        self.adapter.advance(.3)
        self.adapter.pause('owner')
        self.adapter.advance(1.)
        final = self.adapter.status('b-pause')
        self.assertEqual(final['status'], 'failed')
        self.assertEqual(len(final['feedback']), 1)
        self.assertEqual(final['feedback'][0]['call_id'], original)
        self.assertEqual(final['feedback'][0]['reason'], 'Paused by Oracle owner')
        with self.assertRaises(ValueError):
            self.adapter.submit(self.request('after-pause', 'A', [
                {'op':'gripper','parameters':{'width_m':.06,'speed_m_s':.05}},
            ]))

    def test_legacy_execute_requires_drained_explicit_handoff(self):
        self.adapter.submit(self.request('b-before-handoff', 'B', [
            {'op':'gripper','parameters':{'width_m':.06,'speed_m_s':.05}},
        ]))
        with self.assertRaises(ValueError):
            self.trial.execute('legacy-conflict', {'A':('gripper',{'width_m':.06})})
        with self.assertRaises(ValueError):
            self.adapter.handoff('owner')
        self.assertEqual(self.adapter.status('b-before-handoff')['status'], 'running')
        self.adapter.advance(1.5)
        self.adapter.handoff('owner')
        result = self.trial.execute('legacy-after-handoff',
            {'A':('gripper',{'width_m':.06,'speed_m_s':.05})})
        self.assertEqual(result['A']['status'], 'completed')
        with self.assertRaises(ValueError):
            self.adapter.advance(.02)
        self.adapter.close({'scope':'handoff unit test', 'passed':False})
        self.closed = True


if __name__ == '__main__':
    unittest.main()
