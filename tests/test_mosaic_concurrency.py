import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from robofrontier.suite.mosaic_concurrency import dependencies,ready_scopes


class ConcurrencyPlanningTests(unittest.TestCase):
    def job(self,robot,lo,hi):return dict(robots=[robot],bounds_m=[lo,hi])

    def test_four_or_more_ready_scopes_have_no_artificial_pair_limit(self):
        jobs=[self.job(str(i),[i*2,0,0],[i*2+1,1,1]) for i in range(8)]
        pred=dependencies(jobs)
        self.assertEqual(ready_scopes(pred,set()),list(range(8)))

    def test_shared_robot_and_payload_space_preserve_source_dependencies(self):
        jobs=[self.job('A',[0,0,0],[1,1,1]),self.job('B',[2,0,0],[3,1,1]),
              self.job('C',[.5,0,0],[1.5,1,1]),self.job('A',[4,0,0],[5,1,1])]
        pred=dependencies(jobs)
        self.assertEqual(ready_scopes(pred,set()),[0,1])
        self.assertEqual(ready_scopes(pred,{0},active={1}),[2,3])

    def test_clearance_reserve_is_retained(self):
        jobs=[self.job('A',[0,0,0],[1,1,1]),self.job('B',[1.03,0,0],[2,1,1])]
        self.assertEqual(dependencies(jobs),[[],[0]])

    def test_object_custody_keeps_order_even_between_different_robots(self):
        jobs=[dict(self.job('A',[0,0,0],[1,1,1]),object='cube'),
              dict(self.job('B',[3,0,0],[4,1,1]),object='cube')]
        self.assertEqual(dependencies(jobs),[[],[0]])

    def test_operation_completion_is_not_premature_goal_completion(self):
        from robofrontier.suite.mosaic_concurrency import execute_plan
        accepted={};batches=[]
        t=SimpleNamespace(physical_control_sha256='control',environment_logic_sha256='logic',
            source_bundle={'sha256':'scene'},task={'id':'test','goals':[{},{}]},held={},
            arms={r:SimpleNamespace(grasp_force=None) for r in ('A','B')},
            monitor=SimpleNamespace(result=lambda:{'assignments':dict(accepted)}),wait=Mock(),event=Mock())
        def execute(label,commands):
            batches.append(commands)
            for robot,(_,parameters) in commands.items():
                if parameters['joint_rad'][0]==2:accepted[robot]=robot+'-cube'
        t.execute=execute
        jobs=[]
        for robot in ('A','B'):
            for step in (1,2):
                jobs.append(dict(goal_id=robot,object=robot+'-cube',path=[robot],
                    actions=[dict(commands={robot:('move',{'joint_rad':[step]*7})},start_sim_s=step-1,end_sim_s=step)],end_sim_s=step))
        plan=dict(source_physical_control_sha256='control',source_environment_logic_sha256='logic',
            source_scene_bundle_sha256='scene',jobs=jobs,predecessors=[[],[0],[],[2]],
            critical_path_weights=[2,1,2,1],last_goal_nodes={1:'A',3:'B'},total_goals=2,
            initial_commands={r:('move',{'joint_rad':[0]*7}) for r in ('A','B')})
        result=execute_plan(t,plan)
        self.assertEqual(result['reference_placed'],2)
        self.assertEqual(result['concurrent_scope_peak'],2)
        self.assertEqual(len(batches),3)
        self.assertEqual(accepted,{'A':'A-cube','B':'B-cube'})


if __name__=='__main__':unittest.main()
