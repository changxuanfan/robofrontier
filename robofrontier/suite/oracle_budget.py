"""Explicit engineering pauses; unacknowledged process stalls still count."""
import json
from pathlib import Path
import time

from ..run_setup import write_json


class ExplicitPauseBudget:
    def __init__(self,directory,clock=time.monotonic):
        self.directory=Path(directory);self.clock=clock;self.started=clock()
        self.paused_at=None;self.paused_seconds=0.;self.next_poll=0.

    def times(self):
        now=self.clock();paused=self.paused_seconds
        if self.paused_at is not None:paused+=now-self.paused_at
        total=now-self.started
        return dict(total_wall_s=total,paused_wall_s=paused,active_wall_s=total-paused)

    def update(self,request):
        if set(request)-{'paused','reason'} or type(request.get('paused')) is not bool:
            raise ValueError('Control accepts only paused:boolean and optional reason')
        paused=request['paused']
        if paused==(self.paused_at is not None):return
        now=self.clock()
        if paused:self.paused_at=now
        else:self.paused_seconds+=now-self.paused_at;self.paused_at=None
        record=dict(paused=paused,reason=str(request.get('reason',''))[:500],
                    acknowledged_unix_s=time.time(),**self.times())
        with (self.directory/'pause-events.jsonl').open('a') as stream:
            stream.write(json.dumps(record)+'\n')
        write_json(self.directory/'pause-status.json',record)

    def gate(self,stop):
        """Explicitly freeze the engineering world at executor boundaries.

        Stop remains effective while paused. No inferred suspension intervals.
        """
        while True:
            if stop():return True
            now=self.clock()
            if now>=self.next_poll:
                self.next_poll=now+.1;control=self.directory/'control.json'
                if control.exists():self.update(json.loads(control.read_text()))
            if self.paused_at is None:return False
            time.sleep(.05)

    def pause_for_review(self,trial,notice,has_pending_input):
        """Preserve known controller jobs at an engineering review boundary."""
        if trial.motion is not None:
            trial.motion.capture(trial.data,force=True);trial.motion.flush()
        trial.event('transport_engineering_review_paused',review=notice,
            scope='Known affected actions; independent controller jobs preserved, never adopted or replayed')
        request=dict(paused=True,reason='Measured grasp/preparation review: '+str(notice['reason'])[:300])
        write_json(self.directory/'control.json',request);self.update(request)
        if has_pending_input():
            request=dict(paused=False,reason='Owner recovery input already queued at review acknowledgement')
            write_json(self.directory/'control.json',request);self.update(request)
        trial.check_stop()
