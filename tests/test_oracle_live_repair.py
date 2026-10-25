import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import time
import unittest
import numpy as np
from robofrontier.suite.dial_live_repair import LiveRepair
from robofrontier.suite.dial_best_oracle import BestManipulator
from robofrontier.suite.oracle import Trial


class LiveRepairTests(unittest.TestCase):
    def test_red_turn_stops_and_requalifies_idle_blue_approach_phase(self):
        from robofrontier.suite.oracle import Trial
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(__file__).resolve().parents[1]
            scene=root/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n06-d3'
            cfg=json.loads((root/'runs/oracle/suite-v2-r10-closure-20261025/inputs/harness.json').read_text())
            t=Trial(scene,Path(tmp)/'world',frames=False,controller_cfg=cfg)
            m=BestManipulator(t);entry=m.dials[3];m.lift([entry])
            m.grasp([entry],[1]*len(m.dials))
            self.assertTrue(t.monitor.dial_ready[3])
            step=t.step;injected=[False]
            def inject_idle_driver_phase():
                if t.arms['G'].job is not None and not injected[0]:
                    # Declared driver fault injection, no object/robot pose edits.
                    t.arms['H'].grasp_phase='approach';t.arms['H'].grasp_confirmed=False
                    t.arms['H'].grasp_drive_force=0.;t.arms['H'].grasp_force_started=None
                    t.arms['H'].grasp_contact_width=None;t.arms['H'].grasp_settling.clear()
                    injected[0]=True
                step()
            t.step=inject_idle_driver_phase
            target=t.data.site_xpos[t.arms['G'].site]+np.array([0,0,.005])
            rotation=t.data.site_xmat[t.arms['G'].site].reshape(3,3).copy()
            try:
                result=m.execute_turn('guard-regression',{'G':t.pose_command('G',target,rotation,.02,.2)})
                self.assertEqual(result['G']['status'],'failed')
                self.assertTrue(t.monitor.dial_ready[3]);self.assertEqual(t.arms['H'].grasp_phase,'hold')
                self.assertFalse(t.monitor.violations)
                self.assertTrue(any(e['event']=='reference_blue_pressure_requalified' for e in t.events))
            finally:t.close(dict(passed=False,scope='Engineering blue-pressure fault injection'))

    def test_production_close_drains_recovery_and_preserves_assistance(self):
        from scripts.run_blind_dial_oracle import close_trial
        from robofrontier.suite.oracle_recovery import OracleRecovery, RecoverablePlanError
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            scene=Path(__file__).resolve().parents[1]/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n04-d1'
            t=Trial(scene,root/'world',frames=False)
            adapter=OracleRecovery(t,owner='test',world_id='one',observe=lambda:{})
            recovery=adapter.recover(RecoverablePlanError('Aperture recovery'),['A'])
            adapter.submit(dict(owner='test',world_id='one',id='repair',robot='A',
                recovery_id=recovery['id'],assisted=True,review='Close during real recovery action',
                sequence=[dict(op='gripper',parameters=dict(mode='move',width_m=.02,speed_m_s=.01))]))
            t.stop_requested=lambda:True
            report=dict(passed=False)
            close_trial(t,report)
            saved=json.loads((t.directory/'report.json').read_text())
            self.assertTrue(adapter.closed)
            self.assertTrue(saved['assisted']);self.assertTrue(saved['llm_used'])
            self.assertEqual(saved['oracle_mode'],'privileged_assisted')
            self.assertFalse(saved['unresolved_robot_actions'])
            self.assertEqual(len(saved['recovery']['requests']),1)
            self.assertNotEqual(saved['recovery']['requests']['repair']['status'],'running')

    def test_unverified_shoulder_capture_is_not_the_production_default(self):
        self.assertEqual(BestManipulator.blue_capture_mode,'stem')

    def test_production_close_retains_uncertain_call_after_stop_feedback(self):
        from scripts.run_blind_dial_oracle import close_trial
        from robofrontier.suite.oracle_recovery import OracleRecovery
        with tempfile.TemporaryDirectory() as tmp:
            scene=Path(__file__).resolve().parents[1]/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n04-d1'
            t=Trial(scene,Path(tmp)/'world',frames=False)
            adapter=OracleRecovery(t,owner='test',world_id='one',observe=lambda:{})
            adapter.submit(dict(owner='test',world_id='one',id='unknown',robot='A',review='Test uncertain acknowledgement',
                sequence=[dict(op='gripper',parameters=dict(width_m=.02,speed_m_s=.01))]))
            adapter.fence('test','unknown','Owner cannot establish original acknowledgement')
            report=dict(passed=True)
            close_trial(t,report)
            self.assertFalse(report['passed']);self.assertEqual(report['unresolved_robot_actions'],['A'])
            self.assertEqual(report['recovery']['requests']['unknown']['status'],'uncertain')

    def test_failed_plan_keeps_world_and_accepts_a_later_repair(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);events=[];calls=[];deadline=time.monotonic()+10
            scene=Path(__file__).resolve().parents[1]/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n04-d1'
            t=Trial(scene,root/'world',frames=False)
            t.stop_requested=lambda:time.monotonic()>deadline
            m=SimpleNamespace(dials=[]);repair=LiveRepair(t,m)
            def submit():
                state=repair.path/'awaiting.json'
                while not state.exists():time.sleep(.01)
                folder=Path(json.loads(state.read_text())['folder'])
                (folder/'plan.json').write_text(json.dumps({'source':'research_gpt','qpos':[9]}))
                while not (folder/'result-001.json').exists():time.sleep(.01)
                (folder/'plan.json').write_text(json.dumps({'source':'research_gpt','batches':[{'A':['gripper',{'mode':'move','width_m':.08,'speed_m_s':.02}]}],'resume':True}))
            thread=threading.Thread(target=submit,daemon=True);thread.start()
            repair.recover(RuntimeError('Recoverable aperture problem'),[1]*4);thread.join()
            self.assertEqual(repair.interventions,2);self.assertEqual(repair.adapter.submitted_actions,1)
            self.assertTrue((repair.path/'pause-001/result-002.json').exists())
            self.assertFalse(json.loads((repair.path/'pause-001/state.json').read_text())['private_password_included'])
            self.assertFalse(t.monitor.violations)
            repair.adapter.close(dict(passed=False,scope='unit live repair'))

    def test_shoulders_are_grasped_with_realistic_span_and_rotation(self):
        m=object.__new__(BestManipulator)
        geom=lambda name:SimpleNamespace(id=int(name[-1]),size=np.array([.012,.028,.002]))
        poses=[SimpleNamespace(xpos=np.array([0.,.02,-.078])),SimpleNamespace(xpos=np.array([0.,.02,-.050]))]
        m.t=SimpleNamespace(task={'level':3},model=SimpleNamespace(geom=geom),data=SimpleNamespace(geom=lambda i:poses[i]),
            arms={'B':SimpleNamespace(grip_max=.08)},event=lambda *a,**kw:None)
        m.blue_capture_mode='shoulder'
        options=m.blue_capture_options({'id':'D01','nominal_robots':['A','B']},np.eye(3),np.zeros(3))
        self.assertEqual(len(options),2)
        for rotation,point,width in options:
            self.assertAlmostEqual(width,.032);self.assertAlmostEqual(np.linalg.det(rotation),1.)
            np.testing.assert_allclose(point,[0,.02,-.064]);np.testing.assert_allclose(rotation[:,1],[0,0,1])

    def test_interrupted_action_is_recorded_before_next_call_id(self):
        t=object.__new__(Trial);starts=[];t.data=SimpleNamespace(time=0.);t._live_data=t.data
        t.task={'workspace_world_m':{'lower':[-1]*3,'upper':[1]*3}};t.cfg={};t.actions=[];t.events=[];t.tick=0;t.stride=1
        arm=SimpleNamespace(validate=lambda *a:{},start=lambda call,*a:starts.append(call),poll=lambda *a:{'status':'completed','call_id':starts[-1]})
        t.arms={'A':arm};t.check_stop=lambda:None;t.event=lambda *a,**kw:None
        def fail():raise RuntimeError('temporary step failure')
        t.step=fail
        with self.assertRaises(RuntimeError):t.execute('first',{'A':('move',{})})
        self.assertEqual(t.actions[0]['results']['A']['status'],'uncertain')
        t.step=lambda:None
        t.execute('second',{'A':('move',{})})
        self.assertEqual(starts,['oracle-0-A','oracle-1-A'])


if __name__=='__main__':unittest.main()
