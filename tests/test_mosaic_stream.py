import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import json
import tempfile
from robofrontier.suite.mosaic_stream import completed_prefix,review_deliberation


class CompletedPrefixTests(unittest.TestCase):
    def test_closed_partial_source_hands_off_to_actual_target_progress_without_reparking(self):
        from robofrontier.suite.mosaic_stream import execute
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'source';source.mkdir();target=root/'target';target.mkdir();accepted={}
            (source/'executor-launch.json').write_text('immutable')
            (source/'runner-status.json').write_text(json.dumps(dict(world_id='w',phase='failed')))
            (source/'report.json').write_text(json.dumps(dict(passed=False,error='KeyError: old graph',failed_robot_actions=0,
                unresolved_robot_actions=[],evaluation=dict(violations=[]))))
            t=SimpleNamespace(directory=target,source_bundle={'sha256':'target'},task={'goals':[{},{}]},held={},actions=[],
                arms={'A':SimpleNamespace(job=None,grasp_force=None)},mosaic_motion_profile={'loaded_joint_speed_rad_s':.35},
                check_stop=Mock(),event=Mock(),monitor=SimpleNamespace(result=lambda:{'assignments':dict(accepted)},violations=[]))
            calls=[]
            def snapshot(*args,**kwargs):
                calls.append(1)
                if len(calls)>1:return None
                path=args[1];path.mkdir(parents=True);(path/'trace.jsonl').write_text('')
                return dict(cut_sim_s=1.,source_assignments={'g':'cube'})
            def native(*args,**kwargs):
                accepted['g']='cube';t.mosaic_protected_assignments=dict(accepted)
                return dict(concurrent_operation_peak=1)
            with patch('robofrontier.suite.mosaic_stream.snapshot',side_effect=snapshot), \
                 patch('robofrontier.suite.mosaic_stream.build_plan',return_value=dict(jobs=[],source_assisted=True,total_goals=1)), \
                 patch('robofrontier.suite.mosaic_async.execute_plan',side_effect=native), \
                 patch('robofrontier.suite.mosaic_oracle_v2.solve',return_value={'reference_placed':2}) as finish:
                result=execute(t,source,{'equivalent_for_command_candidate':True},finish_after_source=True)
            self.assertEqual(result['reference_placed'],2);self.assertTrue(finish.call_args.kwargs['resume'])
            self.assertEqual(t.mosaic_stream_summary['source_terminal']['target_progress_at_handoff'],1)
            self.assertFalse(t.mosaic_stream_summary['source_terminal']['source_passed'])
            self.assertEqual(accepted,{'g':'cube'})

    def test_only_long_resolved_review_idle_is_replanned(self):
        jobs=[dict(actions=[dict(end_sim_s=a)],end_sim_s=b) for a,b in [(1,10),(11,20),(21,21.15),(31,40)]]
        p=dict(jobs=jobs);events=[dict(event='mosaic_review_opened',review_id=1,sim_time_s=11),
            dict(event='mosaic_review_resumed',review_id=1,sim_time_s=30)]
        review_deliberation(p,events)
        self.assertNotIn('wait_after_s',jobs[0]);self.assertNotIn('wait_after_s',jobs[2]);self.assertNotIn('wait_after_s',jobs[3])
        self.assertAlmostEqual(jobs[1]['wait_after_s'],2.4);self.assertEqual(jobs[1]['end_sim_s'],20)
        self.assertAlmostEqual(p['review_deliberation_schedule']['saved_source_idle_s'],6.6)

    def action(self,label,start,end,status='completed'):
        return [dict(event='action_start',label=label,sim_time_s=start,commands={'A':['move',{}]}),
            dict(event='action_end',label=label,sim_time_s=end,results={'A':dict(status=status)})]

    def events(self):
        return self.action('clear-idle-arms',0,1)+self.action('pick-place',1,2)+[
            dict(event='goal_attempt',sim_time_s=2,goal='g1',object='x',path=['A'])]+self.action('pick-place',2,3)+[
            dict(event='goal_attempt',sim_time_s=3,goal='g2',object='y',path=['A'])]

    def test_unrecorded_motion_tail_is_not_admitted(self):
        p=completed_prefix(self.events(),2.8)
        self.assertEqual(p['assignments'],{'g1':'x'});self.assertEqual(len(p['actions']),2)

    def test_boundary_uses_event_order_when_next_command_starts_at_same_time(self):
        p=completed_prefix(self.events(),4,max_new_goals=1)
        self.assertEqual(p['events'][-1]['event'],'goal_attempt');self.assertEqual(p['cut'],2)
        self.assertEqual(p['actions'][-1]['label'],'pick-place')

    def test_continuation_consumes_new_scopes_once(self):
        p=completed_prefix(self.events(),4,after_sim_s=2,previous_pairs={'g1':'x'},max_new_goals=1)
        self.assertEqual(p['cut'],3);self.assertEqual(p['assignments'],{'g1':'x','g2':'y'})
        self.assertIsNone(completed_prefix(self.events(),4,after_sim_s=3,previous_pairs=p['assignments']))

    def test_repair_marker_keeps_native_commands_without_inventing_a_goal_attempt(self):
        events=self.events()[:5]+self.action('repair',2,3)+[dict(event='mosaic_resume_from_actual_progress',sim_time_s=3,protected_assignments={'g1':'x','g2':'y'})]
        p=completed_prefix(events,4);self.assertEqual(p['assignments'],{'g1':'x','g2':'y'})
        self.assertEqual(p['actions'][-1]['label'],'repair')

    def test_source_failure_is_never_blindly_replayed(self):
        events=self.events();events[3]['results']['A']['status']='uncertain'
        with self.assertRaisesRegex(ValueError,'uncertain'):completed_prefix(events,4)

    def test_no_complete_marker_is_waiting_not_a_pass(self):
        self.assertIsNone(completed_prefix(self.events()[:3],100))


if __name__=='__main__':unittest.main()
