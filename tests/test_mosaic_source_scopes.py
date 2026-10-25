import unittest
from robofrontier.suite.mosaic_source_scopes import source_scopes
from robofrontier.suite.mosaic_concurrency import dependencies


class SourceScopesTests(unittest.TestCase):
    def action(self,label,start,end,robot='A',status='completed'):
        return dict(label=label,start_sim_s=start,end_sim_s=end,commands={robot:['move',{}]},results={robot:dict(status=status)})

    def test_repaired_goal_commands_remain_in_an_explicit_barrier(self):
        actions=[self.action('clear-idle-arms',0,1),self.action('ordinary',1,2),
            self.action('partial-failed-plan',2,3),self.action('review-repair',100,101,'B'),self.action('ordinary',101,102)]
        events=[dict(event='goal_attempt',sim_time_s=2,goal='g1',object='x',path=['A']),
            dict(event='mosaic_resume_from_actual_progress',sim_time_s=101,protected_assignments={'g1':'x','g2':'y'}),
            dict(event='goal_attempt',sim_time_s=102,goal='g3',object='z',path=['A'])]
        initial,scopes=source_scopes(actions,events,{'g1':'x','g2':'y','g3':'z'},['x','y','z'])
        self.assertEqual(initial,actions[0]);self.assertEqual([len(s['actions']) for s in scopes],[1,2,1])
        self.assertEqual(scopes[1]['expected_assignments'],{'g2':'y'})
        self.assertTrue(scopes[1]['barrier']);self.assertIsNone(scopes[1]['object'])
        self.assertEqual(scopes[1]['objects'],['x','y','z'])

    def test_repair_without_newly_completed_goal_still_retains_its_actions(self):
        actions=[self.action('clear-idle-arms',0,1),self.action('recovery',1,2),self.action('finish',2,3)]
        events=[dict(event='mosaic_resume_from_actual_progress',sim_time_s=2,protected_assignments={}),
            dict(event='goal_attempt',sim_time_s=3,goal='g',object='x',path=['A'])]
        _,scopes=source_scopes(actions,events,{'g':'x'},['x']);self.assertEqual(len(scopes),2)
        self.assertEqual(scopes[0]['expected_assignments'],{})

    def test_missing_manual_goal_marker_is_not_counted_as_success(self):
        actions=[self.action('clear-idle-arms',0,1),self.action('ordinary',1,2)]
        events=[dict(event='goal_attempt',sim_time_s=2,goal='g1',object='x',path=['A'])]
        with self.assertRaisesRegex(ValueError,'every final goal'):
            source_scopes(actions,events,{'g1':'x','g2':'y'},['x','y'])

    def test_failed_native_action_is_retained_as_a_rejection(self):
        actions=[self.action('clear-idle-arms',0,1),self.action('failed',1,2,status='failed')]
        with self.assertRaisesRegex(ValueError,'failed actions'):source_scopes(actions,[],{},[])

    def test_barrier_prevents_otherwise_disjoint_work_crossing_the_repair(self):
        jobs=[dict(robots=[r],object=r,bounds_m=[[i*4,0,0],[i*4+1,1,1]],barrier=i==1)
            for i,r in enumerate(['A','B','C'])]
        self.assertEqual(dependencies(jobs),[[],[0],[1]])


if __name__=='__main__':unittest.main()
