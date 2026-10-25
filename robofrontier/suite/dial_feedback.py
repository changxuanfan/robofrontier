"""Versioned dial feedback; public readings never contain the private score.

The state machine consumes apparatus samples, not robot commands. BEST is
episode history; readiness describes only the current fully held input.
"""
import math

DIGIT_BRIGHT_RGB=(.68,.13,.07)
DIGIT_DIM_RGB=(.32,.27,.25)


def settings(task):
    return task.get('mechanism',{}).get('feedback')


def alphabet(task):
    spec=settings(task)
    return tuple(range(1,int(spec['symbol_count'])+1)) if spec else (1,2,3)


class DialFeedback:
    def __init__(self,secret,*,symbol_count=5,mode='best_so_far',tolerance_deg=4.,dwell_s=.25):
        if type(symbol_count)!=int or not 1<=symbol_count<=9:raise ValueError('Unsupported dial alphabet')
        if mode not in ('current','best_so_far'):raise ValueError('Unknown feedback mode')
        if not 0<tolerance_deg<180/(symbol_count+1) or not 0<dwell_s<=10:raise ValueError('Invalid detent qualification')
        self.__secret=tuple(secret)
        if not self.__secret or any(type(x)!=int or not 1<=x<=symbol_count for x in self.__secret):raise ValueError('Secret outside configured alphabet')
        self.n=len(secret);self.symbol_count=symbol_count;self.mode=mode
        self.step_deg=360/(symbol_count+1);self.tolerance_deg=tolerance_deg;self.dwell_s=dwell_s
        self.best=0;self.display=0 if mode=='best_so_far' else None
        self.ready=False;self.input_valid=False;self.candidate=None;self.since=None;self.last=None
        self.submitted=False;self.won=False;self.queries=0;self.trials=[]

    def update(self,t,angles,speeds,eligible):
        if not math.isfinite(t) or (self.last is not None and t<self.last-1e-9):raise ValueError('Feedback clock must be monotone')
        if len(angles)!=self.n or not all(math.isfinite(float(x)) for x in angles):raise ValueError('Bad dial sample')
        degrees=[math.degrees(float(x))%360 for x in angles]
        marks=tuple(round(x/self.step_deg)%(self.symbol_count+1) for x in degrees)
        self.input_valid=bool(eligible and all(marks) and all(abs((x-m*self.step_deg+180)%360-180)<=self.tolerance_deg+1e-9 for x,m in zip(degrees,marks)))
        guess=marks if self.input_valid else None
        if guess!=self.candidate or not self.input_valid:
            self.candidate=guess;self.since=t if self.input_valid else None;self.submitted=False
        if self.last is not None and t-self.last>.030001:self.since=t if self.input_valid else None
        self.last=t;self.ready=bool(self.input_valid and t-self.since>=self.dwell_s-1e-8)
        current=sum(a==b for a,b in zip(guess,self.__secret)) if self.ready else None
        if self.ready:
            self.best=max(self.best,current)
            if not self.submitted and not self.won:
                self.queries+=1
                self.trials.append(dict(index=self.queries,sim_time_s=t,input_symbols=list(guess),
                    **({'best_count':self.best} if self.mode=='best_so_far' else {'correct_count':current})))
                self.submitted=True
            if current==self.n:self.won=True
        self.display=self.best if self.mode=='best_so_far' else self.n if self.won else current
        return self.display

    @property
    def indicator(self):
        return 'ready' if self.ready else 'qualifying' if self.input_valid else 'invalid'


def from_task(task,secret):
    spec=settings(task)
    return DialFeedback(secret,symbol_count=spec['symbol_count'],mode=spec['mode'],
        tolerance_deg=spec.get('angle_tolerance_deg',4.),dwell_s=spec.get('dwell_sim_s',.25))


def paint_ready(model,game):
    # A visual-only device lamp. Its colors are recorded by existing RGBA events.
    import mujoco
    gid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'score_ready_light')
    if gid<0:return
    colors={'invalid':(.47,.51,.53,1.),'qualifying':(1.,.59,.08,1.),'ready':(.04,.72,.28,1.)}
    model.geom_rgba[gid]=colors[game.indicator]


def paint_display(model,game,digits=1):
    """Retain the record's shape when unready; brightness follows readiness only.

    RGBA changes use the existing motion recorder's visual events, so replay
    preserves the same appearance without reconstructing puzzle feedback.
    """
    from .evaluate import DIGITS
    paint_ready(model,game)
    score=game.display
    if score is not None and not 0<=score<10**digits:
        raise ValueError('Score exceeds configured display width')
    show='-'*digits if score is None else f'{score:0{digits}d}'
    color=DIGIT_BRIGHT_RGB if game.ready else DIGIT_DIM_RGB
    for digit,char in enumerate(show):
        active=DIGITS[int(char)] if char!='-' else 'g'
        for k,segment in enumerate('abcdefg'):
            model.geom_rgba[model.geom(f'score_{digit}_{k}').id]=[*color,float(segment in active)]
