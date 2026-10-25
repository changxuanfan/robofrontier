"""Continuous-grasp actuator reference; code inference and answer are separated."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
from .dial_oracle_v2 import Manipulator
from .evaluate import Monitor
from scripts.verify_floating_dial_oracle import axis_rotation

WORKER=Path(__file__).with_name('dial_blind_worker.py')


class Channel:
    def __init__(self,mode,args,log,worker=WORKER):
        self.tmp=tempfile.TemporaryDirectory(prefix='dial-count-only-');self.log=Path(log).open('x')
        self.process=subprocess.Popen([sys.executable,'-I','-S',str(worker),mode,*map(str,args)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1,
            cwd=self.tmp.name,env={'PATH':os.defpath,'LC_ALL':'C'})
    def call(self,request):
        self.process.stdin.write(json.dumps(request)+'\n');self.process.stdin.flush()
        line=self.process.stdout.readline()
        if not line:raise RuntimeError('Isolated worker stopped: '+self.process.stderr.read()[-600:])
        result=json.loads(line);self.log.write(json.dumps(dict(input=request,output=result))+'\n');self.log.flush();return result
    def close(self):
        if self.process.poll() is None:
            self.process.stdin.write('{"op":"close"}\n');self.process.stdin.flush();self.process.stdin.close()
            self.process.wait(timeout=10)
        code=self.process.returncode
        for pipe in (self.process.stdin,self.process.stdout,self.process.stderr):
            if pipe and not pipe.closed:pipe.close()
        self.log.close();self.tmp.cleanup()
        if code:raise RuntimeError('Isolated worker failed '+str(code))


class BlindDecoder:
    def __init__(self,n,directory):
        self.channel=Channel('solver',[n],Path(directory)/'solver-io.jsonl');self.candidates=range(3**n)
    def choose(self,current):
        result=self.channel.call(dict(op='choose',current=current));self.candidates=range(result['candidates']);return result['guess']
    def update(self,guess,count):
        result=self.channel.call(dict(op='feedback',guess=list(guess),count=int(count)));self.candidates=range(result['candidates'])
    def close(self):self.channel.close()


class EvaluatedHold:
    def __init__(self,monitor,codebook):
        self.monitor=monitor;self.channel=Channel('evaluator',[codebook,monitor.task['id']],monitor.w.directory/'evaluator-io.jsonl')
        self.display=None;self.won=False;self.queries=0;self.since=None;self.red_ready=[]
    def update(self,t,angles,speeds,eligible):
        mon=self.monitor;self.red_ready=[]
        for a in mon.dials:
            geoms=[mon.m.geom(a['id']+'_hand_grip').id] if mon.task['level']==3 else [g for g,i in mon.red.items() if i==a['index']]
            rotating={r for r in mon.bilateral(geoms) if mon.w.arms[r].grasp_force is not None}
            if mon.task['level']>1:
                supporting=mon.bilateral([g for g,i in mon.blue.items() if i==a['index']])
                rotating={r for r in rotating if supporting-{r}}
            self.red_ready.append(bool(rotating))
        result=self.channel.call(dict(op='sample',t=float(t),angles=np.asarray(angles).tolist(),eligible=bool(eligible and all(self.red_ready))))
        for k,v in result.items():setattr(self,k,v)
        return self.display
    def close(self):self.channel.close()


class HoldMonitor(Monitor):
    def __init__(self,trial,codebook):
        super().__init__(trial,secret=[1]*len(trial.task['dials']))
        self.game=EvaluatedHold(self,codebook)
    def sample_dial(self,dt):
        super().sample_dial(dt);self.result_cache['private_red_hold_ready']=self.game.red_ready


class HoldingManipulator(Manipulator):
    # A held input has no mandatory post-turn withdrawal. Actual release,
    # when needed later, retains the existing measured-separation checks.
    require_grasp_exit=False
    def __init__(self,trial,decoder=None):
        if trial.task.get('mechanism',{}).get('feedback') and decoder is None:
            raise ValueError('Use a BEST/alphabet-aware reference; the current-count three-symbol decoder is incompatible')
        super().__init__(trial);self.decoder=decoder if decoder is not None else BlindDecoder(len(self.dials),trial.directory)
        if trial.task['level']==3 and len(self.dials)>4:
            self.red_capture_offset_m=0. if len(self.dials)==8 else .004
            self.red_grasp_force_n=30. if len(self.dials)==8 else 50.
    def prepare_symbols(self,entries,symbols):
        t=self.t;fresh=[];regrasp=[]
        for a in entries:
            name=a['id'];held=self.grips.get(name)
            if held is None:fresh.append(a);continue
            robot=held['robot'];arm=t.arms[robot]
            geoms=[t.model.geom(name+'_hand_grip').id] if t.task['level']==3 else self.red_geoms(a)
            if robot not in t.monitor.bilateral(geoms) or arm.grasp_force is None:regrasp.append(a);continue
            now=float(t.data.joint(name+'_hinge').qpos[0]);symbol=symbols[a['index']]
            options=self.target_angles(symbol)
            jid=t.model.joint(name+'_hinge').id;anchor=t.data.xanchor[jid].copy();axis=t.data.xaxis[jid].copy()
            point=t.data.site_xpos[arm.site].copy();rotation=t.data.site_xmat[arm.site].reshape(3,3).copy();chosen=None
            for target in sorted(options,key=lambda x:abs(x-now)):
                q=arm.qref.copy()
                for delta in np.linspace(0,target-now,max(2,math.ceil(abs(target-now)/math.radians(6))))[1:]:
                    previous=q.copy();dr=axis_rotation(axis,float(delta));q=t.screen_ik(robot,anchor+dr@(point-anchor),dr@rotation,seed=q)
                    if q is None:break
                    if not self.arc_joint_clear(robot,previous,q):break
                else:chosen=target;break
            if chosen is None:regrasp.append(a);continue
            held['target_angle_rad']=chosen
            t.event('continuous_grasp_target',dial=name,robot=robot,from_angle_rad=now,target_angle_rad=chosen,
                    scope='IK-screened held arc; real stepping and whole-scene monitor still required')
        if regrasp:
            t.event('necessary_regrasp_admission',devices=[a['id'] for a in regrasp],reason='Current contact lost or no sampled held IK arc; not an automatic reset for every change')
            self.release(regrasp);fresh+=regrasp
        if fresh:self.grasp(fresh,symbols)


def solve(trial):
    m=HoldingManipulator(trial)
    try:
        m.lift()
        for attempt in range(24):
            if trial.monitor.game.won:break
            current=[round(math.degrees(float(trial.data.joint(a['id']+'_hinge').qpos[0]))/90)%4 for a in m.dials]
            guess=m.decoder.choose(current)
            trial.event('blind_guess',attempt=attempt+1,requested=guess,candidates=len(m.decoder.candidates))
            m.set_symbols(guess,finish_when_solved=True)
        if not trial.monitor.game.won:raise RuntimeError('Count-only decoder query budget exhausted')
        trial.event('blind_decoder_solved',trials=trial.monitor.game.queries,scope='Visible counts only; no mechanical warmup and no private-answer solver inputs')
        trial.wait(.3)
    finally:m.decoder.close()
