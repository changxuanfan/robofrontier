"""BEST5 actuator reference. Mechanical geometry is explicitly privileged.

The isolated solver receives observed complete inputs and the public record.
The evaluator alone loads the secret; no private trial history is relayed.
"""
import math
from pathlib import Path
import numpy as np
import mujoco
from .dial_hold_oracle import Channel,HoldingManipulator
from .evaluate import Monitor

WORKER=Path(__file__).with_name('dial_best_worker.py')


class BestDecoder:
    def __init__(self,n,directory):
        self.channel=Channel('solver',[n,5],Path(directory)/'solver-io.jsonl',worker=WORKER)
        self.candidates=range(5**n);self.best=0;self.closed=False
    def choose(self,current):
        result=self.channel.call(dict(op='choose',current=list(current)))
        self.candidates=range(result['candidates']);return result['guess']
    def observe(self,t,inputs,best):
        result=self.channel.call(dict(op='observe',t=float(t),inputs=inputs,best=int(best)))
        self.candidates=range(result['candidates']);self.best=result['best']
    def close(self):
        if not self.closed:self.closed=True;self.channel.close()


class RemoteBest:
    def __init__(self,trial,puzzle):
        self.trial=trial;self.observer=None;self.last_observation=None;self.mode='best_so_far'
        self.display=0;self.won=False;self.queries=0;self.since=None;self.ready=False;self.input_valid=False;self.indicator='invalid'
        self.channel=Channel('evaluator',[Path(puzzle).resolve(),trial.scene/'task.json',Path(__file__).with_name('dial_feedback.py'),trial.directory/'private-evaluator-input.json'],trial.directory/'evaluator-io.jsonl',worker=WORKER)
        self.closed=False
    def update(self,t,angles,speeds,eligible):
        result=self.channel.call(dict(op='sample',t=float(t),angles=np.asarray(angles).tolist(),eligible=bool(eligible)))
        for name,value in result.items():setattr(self,name,value)
        if self.ready:
            step=self.trial.task['mechanism']['feedback']['step_deg']
            inputs=[int(round(math.degrees(float(a))/step))%6 for a in angles]
            key=(tuple(inputs),self.display)
            if key!=self.last_observation:
                # Same-sample apparatus measurement, not a requested target or
                # evaluator candidate. This privilege belongs to Oracle only.
                event=dict(observed_inputs=inputs,best=self.display,
                    scope='Oracle measured angles and visible BEST sampled together; no private current score')
                self.trial.event('observed_best_input',**event)
                if self.observer:self.observer(float(t),[inputs],self.display)
                self.last_observation=key
        else:self.last_observation=None
        return self.display
    def close(self):
        if not self.closed:self.closed=True;self.channel.close()


class BestMonitor(Monitor):
    def __init__(self,trial,puzzle):
        super().__init__(trial,secret=[1]*len(trial.task['dials']))
        self.game=RemoteBest(trial,puzzle)
    def close(self):
        self.game.close();super().close()


class MechanicalDecoder:
    candidates=()
    def close(self):pass


class BestManipulator(HoldingManipulator):
    # Release requires measured separation and a clear spring sweep, not an
    # expensive return to an unrelated historical arm posture.
    home_after_release=False
    def __init__(self,trial,decoder=None):
        if trial.task['mechanism']['feedback']['symbol_count']!=5:raise ValueError('BEST5 reference requires five symbols')
        super().__init__(trial,decoder=decoder if decoder is not None else MechanicalDecoder())
        # Start from a centered capture at a normal, declared grasp force;
        # do not inherit layout-specific offsets from the old three-mark run.
        self.red_capture_offset_m=0.;self.red_grasp_force_n=30.
        self.robot_geometry=None
        self.hold_recoveries={}
    def set_symbols(self,symbols,finish_when_solved=False):
        self.hold_recoveries={}
        return super().set_symbols(symbols,finish_when_solved)
    def maintain_holds(self,symbols):
        # An unchanged angle is not proof that its gripper is still holding.
        t=self.t;lost=[]
        for a in self.dials:
            grip=self.grips.get(a['id'])
            if grip is None:lost.append(a);continue
            robot=grip['robot']
            if robot not in t.monitor.bilateral(self.red_geoms(a)) or t.arms[robot].grasp_force is None:lost.append(a)
        if not lost:return
        if any(self.hold_recoveries.get(a['id'],0)>=2 for a in lost):raise RuntimeError('Held-input recovery budget exhausted')
        for a in lost:self.hold_recoveries[a['id']]=self.hold_recoveries.get(a['id'],0)+1
        t.event('reference_lost_unchanged_hold',devices=[a['id'] for a in lost],scope='Measured grip checked independently of angle error')
        self.prepare_symbols(lost,symbols)
    def arc_joint_clear(self,robot,start,goal):
        from ..motion_check import RobotGeometry
        t=self.t
        if self.robot_geometry is None:self.robot_geometry=RobotGeometry(t.model,t.arms)
        scratch=t.screen_data;scratch.qpos[:]=t.data.qpos
        for f in np.linspace(0,1,max(2,math.ceil(float(np.max(abs(goal-start)))/.025))+1)[1:]:
            t.check_stop();scratch.qpos[t.arms[robot].qids]=start+(goal-start)*f
            mujoco.mj_kinematics(t.model,scratch)
            if self.robot_geometry.distance(scratch,.02)['distance_m']<.003:return False
        return True
    def batch_arc_clear(self,commands):
        from ..motion_check import RobotGeometry
        t=self.t
        if self.robot_geometry is None:self.robot_geometry=RobotGeometry(t.model,t.arms)
        goals={}
        for robot,(op,p) in commands.items():
            if op!='move' or 'target_m' not in p:raise ValueError('Reference turn requires Cartesian move candidates')
            from ..robot_control import quat_matrix
            q=t.screen_ik(robot,p['target_m'],quat_matrix(p['quaternion_wxyz']))
            if q is None:
                q=t.screen_ik(robot,p['target_m'],quat_matrix(p['quaternion_wxyz']),seed=t.home_q[robot])
            if q is None:
                t.event('validation_rejected',robot=robot,label='sampled-turn-admission',reason='No verified IK for next measured held arc')
                raise ValueError('no verified IK solution for next measured held arc')
            goals[robot]=(t.data.qpos[t.arms[robot].qids].copy(),q)
        steps=max(2,math.ceil(max(float(np.max(abs(q-start))) for start,q in goals.values())/.025))
        scratch=t.screen_data;scratch.qpos[:]=t.data.qpos
        for f in np.linspace(0,1,steps+1)[1:]:
            t.check_stop()
            for robot,(start,q) in goals.items():scratch.qpos[t.arms[robot].qids]=start+(q-start)*f
            mujoco.mj_kinematics(t.model,scratch)
            if self.robot_geometry.distance(scratch,.02)['distance_m']<.003:return False
        return True
    def execute_turn(self,label,commands):
        t=self.t
        if self.batch_arc_clear(commands):return t.execute(label,commands)
        # A bounded reference scheduling fallback, not a harness restriction.
        t.event('reference_turn_batch_conflict',robots=list(commands),scope='Sampled nominal whole-arm joint interpolation; no dynamics guarantee')
        pending=dict(commands);results={}
        while pending:
            robot=next((r for r,c in pending.items() if self.batch_arc_clear({r:c})),None)
            if robot is None:raise RuntimeError('No screened turn batch or single-arm ordering')
            results.update(t.execute(label+'-separate-'+robot,{robot:pending.pop(robot)}))
        return results
    def update_decoder(self,guess):
        # Updates occur on every actual observed ready combination, including
        # intermediate ones. Never attribute historical BEST to a target.
        pass
    def input_settled(self,symbols):
        t=self.t
        if not t.monitor.game.ready:return False
        angles=np.degrees(t.data.qpos[t.monitor.qids])
        return bool(np.all(abs((angles-np.asarray(symbols)*self.angle_step_deg+180)%360-180)<=4))


def solve(trial,max_guesses=48):
    """Available after commissioning; not invoked by the mechanical runner."""
    decoder=BestDecoder(len(trial.task['dials']),trial.directory)
    trial.monitor.game.observer=decoder.observe
    m=BestManipulator(trial,decoder)
    try:
        m.lift()
        for index in range(max_guesses):
            if trial.monitor.game.won:break
            current=[round(math.degrees(float(trial.data.joint(a['id']+'_hinge').qpos[0]))/60)%6 for a in m.dials]
            guess=decoder.choose(current)
            trial.event('blind_best_guess',attempt=index+1,requested=guess,candidates=len(decoder.candidates))
            m.set_symbols(guess,finish_when_solved=True)
        if not trial.monitor.game.won:raise RuntimeError('BEST-only guess budget exhausted')
        trial.wait(.3)
        trial.event('blind_best_solved',trials=trial.monitor.game.queries)
    finally:
        trial.monitor.game.observer=None;decoder.close()
