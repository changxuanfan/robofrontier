"""Execution ownership and scheduling tests; physics is separately witnessed."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from robofrontier.suite.transport_runtime import TransportRuntime, PreparationComplete


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
    def test_fault_review_gate_resumes_the_same_independent_controller_job(self):
        import threading
        from pathlib import Path
        from tempfile import TemporaryDirectory
        from robofrontier.suite.transport_recovery import EngineeringRecovery
        from robofrontier.suite.oracle_budget import ExplicitPauseBudget
        from robofrontier.run_setup import write_json
        with TemporaryDirectory() as tmp:
            paused=threading.Event();state={};errors=[]
            def worker():
                try:
                    t=trial();state['trial']=t;budget=ExplicitPauseBudget(tmp)
                    def review(_t,_rt,_notice):
                        request=dict(paused=True,reason='Known fault review')
                        write_json(Path(tmp)/'control.json',request);budget.update(request)
                        paused.set();budget.gate(lambda:False)
                    handler=EngineeringRecovery(tmp,on_review=review)
                    handler.observation=Mock(return_value=dict(failed_jobs={}))
                    runtime=TransportRuntime(t,on_terminal=lambda i,j,k:handler.on_terminal(t,runtime,i,j,k))
                    def failed():
                        t.execute('probe',{'A':command(.1)})
                        raise RuntimeError('Grasp stopped outside expected object-width tolerance')
                    runtime.add('a',['A'],failed)
                    runtime.add('b',['B'],lambda:t.execute('independent',{'B':command(.5)}))
                    self.drain(runtime);runtime.close()
                except Exception as error:errors.append(error);paused.set()
            thread=threading.Thread(target=worker);thread.start()
            self.assertTrue(paused.wait(2.))
            t=state['trial'];self.assertFalse(errors)
            self.assertEqual(t.time,.1);self.assertIsNotNone(t.arms['B'].job)
            before=t.arms['B'].job
            write_json(Path(tmp)/'control.json',dict(paused=False,reason='Measured repair queued'))
            thread.join(2.);self.assertFalse(thread.is_alive());self.assertFalse(errors)
            self.assertEqual(t.time,.5);self.assertEqual(len(t.arms['B'].starts),1)
            self.assertEqual(t.arms['B'].starts[0][0],before[0])

    def test_passive_payload_contact_retains_a_known_failed_receiver(self):
        t=trial();touching={'A'}
        runtime=TransportRuntime(t,payload_contacts=lambda r,n:r in touching)
        runtime.add('grip',['A'],lambda:(_ for _ in ()).throw(RuntimeError('Known close failure')))
        runtime.jobs['grip'].update(object='part')
        self.drain(runtime)
        self.assertEqual(runtime.leases,{'A':'grip'})
        self.assertFalse(runtime.add('other-part',['A'],lambda:None))
        touching.clear()
        self.assertTrue(runtime.recover('grip','clear',['A'],lambda:None))
        self.drain(runtime);self.assertFalse(runtime.leases)

    def test_prepared_empty_reservation_is_explicit_and_recoverable(self):
        t=trial();runtime=TransportRuntime(t)
        def prepare():raise PreparationComplete(retain_robots={'A'},pause_after_prepare=True)
        runtime.add('prepare',['A'],prepare);self.drain(runtime)
        self.assertEqual(runtime.leases,{'A':'prepare'})
        self.assertTrue(runtime.jobs['prepare']['pause_after_prepare'])
        self.assertTrue(runtime.recover('prepare','release',['A'],lambda:None))
        self.drain(runtime);self.assertFalse(runtime.leases)

    def test_terminal_notice_preserves_independent_inflight_action(self):
        t=trial();notices=[]
        runtime=TransportRuntime(t,on_terminal=lambda ident,job,kind:notices.append((ident,kind,t.arms['B'].job)))
        def fail_after_probe():
            t.execute('probe',{'A':command(.1)})
            raise RuntimeError('Known support loss')
        runtime.add('a',['A'],fail_after_probe)
        runtime.add('b',['B'],lambda:t.execute('independent',{'B':command(.5)}))
        while runtime.jobs['a']['status']!='failed':runtime.cycle()
        notice=next(row for row in notices if row[0]=='a')
        self.assertEqual(notice[1],'failed');self.assertIsNotNone(notice[2])
        self.assertEqual(len(t.arms['B'].starts),1)
        self.drain(runtime);runtime.close()

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

    def test_prepared_frontier_retains_load_and_can_be_recovered_again(self):
        t=trial();runtime=TransportRuntime(t)
        def prepare():
            t.execute('prepare',{'A':command(.1)})
            t.arms['A'].grasp_force=70.;t.held['A']='part'
            raise PreparationComplete()
        runtime.add('prepare',['A'],prepare);self.drain(runtime)
        self.assertEqual(runtime.jobs['prepare']['status'],'prepared')
        self.assertEqual(runtime.leases,{'A':'prepare'})
        def finish():
            t.execute('finish',{'A':command(.1)})
            t.arms['A'].grasp_force=None;t.held.pop('A')
        self.assertTrue(runtime.recover('prepare','finish',['A'],finish));self.drain(runtime)
        self.assertEqual(runtime.jobs['finish']['status'],'done')
        self.assertFalse(runtime.leases)

    def test_nominal_conflict_delays_only_new_motion(self):
        t=trial()
        checks=[]
        def admission(req,rt):
            checks.append(req['job']);return req['job']!='b' or not rt.running
        runtime=TransportRuntime(t,admission=admission)
        runtime.add('a',['A'],lambda:t.execute('a',{'A':command(.2)}))
        runtime.add('b',['B'],lambda:t.execute('b',{'B':command(.1)}));self.drain(runtime)
        self.assertEqual(t.arms['B'].starts[0][1],.2)
        self.assertEqual(t.time,.3)
        self.assertEqual(checks.count('b'),2)

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

    def test_live_repair_continues_while_independent_job_keeps_running(self):
        t=trial();runtime=TransportRuntime(t)
        def first():
            t.execute('a-first',{'A':command(.1)})
            t.arms['A'].grasp_force=70.;t.held['A']='part'
            raise RuntimeError('Needs a different placement plan')
        runtime.add('a',['A'],first);runtime.add('b',['B'],lambda:t.execute('b',{'B':command(.5)}))
        while runtime.jobs['a']['status']!='failed':runtime.cycle()
        self.assertEqual(runtime.jobs['b']['status'],'executing')
        def repair():
            t.execute('a-repair',{'A':command(.1)})
            t.arms['A'].grasp_force=None;t.held.pop('A')
        self.assertTrue(runtime.recover('a','a-repair',['A'],repair));self.drain(runtime)
        self.assertEqual(runtime.jobs['a']['status'],'failed')
        self.assertEqual(runtime.jobs['a-repair']['status'],'done')
        self.assertEqual(runtime.jobs['a']['recovered_by'],'a-repair')
        self.assertEqual(len(t.arms['B'].starts),1);self.assertEqual(t.time,.5)
        self.assertFalse(runtime.leases)

    def test_rejected_command_fails_only_its_own_job(self):
        t=trial();runtime=TransportRuntime(t)
        t.arms['A'].validate=Mock(side_effect=ValueError('Invalid target'))
        runtime.add('a',['A'],lambda:t.execute('bad',{'A':command(.1)}))
        runtime.add('b',['B'],lambda:t.execute('good',{'B':command(.2)}));self.drain(runtime)
        self.assertEqual(runtime.jobs['a']['status'],'failed')
        self.assertEqual(runtime.jobs['b']['status'],'done');self.assertEqual(t.time,.2)
        self.assertFalse(t.arms['A'].starts)


if __name__=='__main__':unittest.main()
