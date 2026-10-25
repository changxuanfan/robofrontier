"""Execution ownership and scheduling tests; physics is separately witnessed."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from robofrontier.suite.transport_runtime import TransportRuntime


class Arm:
    job=None
    grasp_force=None
    uncertainty_reported=False
    def __init__(self):self.starts=[]
    def validate(self,op,p,*_):return p['delta_m'][0]
    def start(self,ident,duration,now):
        self.job=(ident,now+duration);self.starts.append((ident,now))
    def poll(self,now,*_):
        if self.job is None or now+1e-9<self.job[1]:return None
        ident=self.job[0];self.job=None
        return dict(status='completed',call_id=ident)


def trial():
    t=SimpleNamespace(time=0.,tick=0,stride=1,cfg={},held={},actions=[],check_stop=lambda:None,
        event=Mock(),arms={'A':Arm(),'B':Arm()},task={'workspace_world_m':{'lower':[0]*3,'upper':[1]*3}})
    def step():t.tick+=1;t.time=round(t.time+.02,8)
    t.step=step;t.execute=Mock(side_effect=RuntimeError('Legacy executor used'));t.wait=Mock()
    return t


def command(duration):return ('move',dict(delta_m=[duration,0,0]))


class TransportRuntimeTests(unittest.TestCase):
    def drain(self,runtime):
        for _ in range(1000):
            if not runtime.active:return
            runtime.cycle()
        self.fail('Cooperative runtime did not drain')

    def test_fast_program_and_wait_continue_without_slow_program_barrier(self):
        t=trial();runtime=TransportRuntime(t)
        def a():
            t.execute('a1',{'A':command(.1)});t.wait(.06);t.execute('a2',{'A':command(.1)})
        runtime.add('a',['A'],a);runtime.add('b',['B'],lambda:t.execute('b1',{'B':command(.5)}))
        self.drain(runtime)
        self.assertEqual(t.arms['A'].starts[1][1],.16)
        self.assertLess(t.arms['A'].starts[1][1],.5)
        self.assertEqual(len(t.actions),3);self.assertFalse(runtime.leases);runtime.close()

    def test_explicit_robot_lease_and_single_physics_writer(self):
        t=trial();runtime=TransportRuntime(t)
        runtime.add('a',['A'],lambda:t.step())
        self.assertFalse(runtime.add('overlap',['A'],lambda:None))
        self.drain(runtime)
        self.assertEqual(t.tick,0)
        self.assertIn('owner',str(runtime.jobs['a']['error']))
        self.assertTrue(runtime.add('next',['A'],lambda:None));self.drain(runtime);runtime.close()

    def test_unleased_robot_never_starts(self):
        t=trial();runtime=TransportRuntime(t)
        runtime.add('a',['A'],lambda:t.execute('wrong',{'B':command(.1)}));self.drain(runtime)
        self.assertEqual(runtime.jobs['a']['status'],'failed');self.assertFalse(t.arms['B'].starts)

    def test_retained_force_load_keeps_failed_job_lease(self):
        t=trial();runtime=TransportRuntime(t)
        def fail():t.arms['A'].grasp_force=70.;raise RuntimeError('Placement search failed')
        runtime.add('a',['A'],fail);self.drain(runtime)
        self.assertEqual(runtime.leases,{'A':'a'})
        self.assertFalse(runtime.add('new',['A'],lambda:None))

    def test_nominal_conflict_delays_only_new_motion(self):
        t=trial()
        runtime=TransportRuntime(t,admission=lambda req,rt: req['job']!='b' or not rt.running)
        runtime.add('a',['A'],lambda:t.execute('a',{'A':command(.2)}))
        runtime.add('b',['B'],lambda:t.execute('b',{'B':command(.1)}));self.drain(runtime)
        self.assertEqual(t.arms['B'].starts[0][1],.2)
        self.assertEqual(t.time,.3)

    def test_unknown_action_result_does_not_release_control_or_retry(self):
        t=trial();runtime=TransportRuntime(t)
        original=t.arms['A'].poll
        def uncertain(now,*args):
            result=original(now,*args)
            if result:t.arms['A'].uncertainty_reported=True;result['status']='uncertain'
            return result
        t.arms['A'].poll=uncertain
        runtime.add('a',['A'],lambda:t.execute('a',{'A':command(.1)}));self.drain(runtime)
        self.assertEqual(runtime.jobs['a']['status'],'failed')
        self.assertEqual(runtime.leases,{'A':'a'});self.assertEqual(len(t.arms['A'].starts),1)


if __name__=='__main__':unittest.main()
