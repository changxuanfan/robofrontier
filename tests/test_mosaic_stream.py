import unittest
from robofrontier.suite.mosaic_stream import completed_prefix


class CompletedPrefixTests(unittest.TestCase):
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
