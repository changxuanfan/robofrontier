from copy import deepcopy
from types import SimpleNamespace
import unittest

from robofrontier.suite.mosaic_async import execute_plan


class TimedArm:
    def __init__(self,trial,robot,failed=False,unknown=False):
        self.trial=trial;self.robot=robot;self.job=None;self.grasp_force=None
        self.failed=failed;self.unknown=unknown;self.started=[]

    def validate(self,op,parameters,*args):return dict(parameters)

    def start(self,ident,parameters,now):
        self.job=ident;self.parameters=parameters;self.end=now+parameters['duration']
        self.started.append((ident,now))

    def poll(self,now,timeout):
        if now+1e-8<self.end:return None
        result=dict(call_id=self.job,status='uncertain' if self.unknown else 'failed' if self.failed else 'completed')
        if self.unknown:return result
        self.job=None
        if not self.failed and self.parameters['joint_rad'][0]==2:
            self.trial.accepted[self.robot]=self.robot+'-cube'
        return result


class Trial:
    def __init__(self,failed=False,unknown=False):
        self.physical_control_sha256='control';self.environment_logic_sha256='logic'
        self.source_bundle={'sha256':'scene'};self.time=0.;self.tick=0;self.stride=1
        self.task=dict(id='async-test',goals=[{},{}],workspace_world_m=dict(lower=[-1]*3,upper=[1]*3))
        self.cfg={};self.held={};self.actions=[];self.accepted={}
        self.arms={'A':TimedArm(self,'A',failed,unknown),'B':TimedArm(self,'B')}
        self.monitor=SimpleNamespace(result=lambda:{'assignments':dict(self.accepted)})
    def execute(self,*args):pass
    def event(self,*args,**kwargs):pass
    def check_stop(self):pass
    def wait(self,seconds):self.time+=seconds
    def step(self):self.time+=.1;self.tick+=1


def plan():
    jobs=[]
    for robot,durations in [('A',[1,1]),('B',[5,1])]:
        for step,duration in enumerate(durations,1):
            jobs.append(dict(goal_id=robot,object=robot+'-cube',path=[robot],
                actions=[dict(commands={robot:('move',dict(joint_rad=[step]*7,duration=duration))},end_sim_s=duration)],end_sim_s=duration))
    return dict(source_physical_control_sha256='control',source_environment_logic_sha256='logic',
        source_scene_bundle_sha256='scene',jobs=jobs,predecessors=[[],[0],[],[2]],critical_path_weights=[2,1,6,1],
        last_goal_nodes={1:'A',3:'B'},total_goals=2,initial_commands={r:('move',dict(joint_rad=[0]*7)) for r in ('A','B')})


class AsyncExecutionTests(unittest.TestCase):
    def test_continuation_neither_reparks_nor_reuses_previous_native_action_ids(self):
        t=Trial();p=plan();p['operation_prefix']='prefix-zero';execute_plan(t,p)
        first_ids={r['call_id'] for a in t.actions for r in a['results'].values()}
        t.execute=lambda *args: self.fail('Continuation must not repeat initial parking')
        p=plan();p['operation_prefix']='prefix-one';execute_plan(t,p,initialize=False)
        second_ids={r['call_id'] for a in t.actions[4:] for r in a['results'].values()}
        self.assertFalse(first_ids&second_ids);self.assertEqual(len(t.actions),8)

    def test_continuation_checks_existing_goals_before_new_work(self):
        t=Trial();execute_plan(t,plan());t.accepted.pop('A')
        with self.assertRaisesRegex(RuntimeError,'disturbed completed pairing A'):
            execute_plan(t,plan(),initialize=False)

    def test_repair_barrier_counts_only_new_original_evaluator_assignments(self):
        t=Trial();p=plan()
        for j in p['jobs'][:2]:j.update(goal_id='@repair',object=None)
        p['last_goal_nodes'][1]='@repair';p['predecessors'][2]=[1]
        p['last_goal_assignments']={'1':{'A':'A-cube'},'3':{'B':'B-cube'}}
        result=execute_plan(t,p)
        self.assertEqual(result['reference_placed'],2)
        self.assertGreaterEqual(t.arms['B'].started[0][1],t.arms['A'].started[1][1]+1-1e-8)
        self.assertEqual(t.held,{})

    def test_repair_marker_does_not_copy_an_unobserved_goal_success(self):
        t=Trial();p=plan();p['last_goal_assignments']={1:{'A':'wrong-object'},3:{'B':'B-cube'}}
        with self.assertRaisesRegex(RuntimeError,'original evaluator A'):execute_plan(t,p)

    def test_short_arm_continues_while_unrelated_long_move_is_active(self):
        t=Trial();result=execute_plan(t,plan())
        self.assertEqual(result['reference_placed'],2)
        self.assertLess(t.arms['A'].started[1][1],t.arms['B'].started[0][1]+5)
        self.assertEqual(len(t.actions),4)
        self.assertEqual(result['concurrent_operation_peak'],2)

    def test_known_failure_drains_admitted_peer_without_replaying_or_new_admission(self):
        t=Trial(failed=True)
        with self.assertRaisesRegex(RuntimeError,'Known async failure'):execute_plan(t,plan())
        self.assertIsNone(t.arms['A'].job);self.assertIsNone(t.arms['B'].job)
        self.assertEqual(len(t.arms['A'].started),1);self.assertEqual(len(t.arms['B'].started),1)
        self.assertEqual(sorted(r['status'] for a in t.actions for r in a['results'].values()),['completed','failed'])

    def test_unknown_is_preserved_and_never_replayed(self):
        t=Trial(unknown=True)
        with self.assertRaisesRegex(RuntimeError,'Unknown'):execute_plan(t,plan())
        self.assertIsNotNone(t.arms['A'].job)
        self.assertEqual(len(t.arms['A'].started),1)
        self.assertEqual(t.actions[-1]['results']['A']['status'],'uncertain')


if __name__=='__main__':unittest.main()
