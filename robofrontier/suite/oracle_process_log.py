"""Compact Oracle process events; full visual state lives in the motion stream."""
from copy import deepcopy
import json
from pathlib import Path


class OracleProcessLog:
    schema='robofrontier.oracle-process-events.v1'
    def __init__(self,path,motion_ref='motion/stream.json'):
        self.stream=Path(path).open('x');self.last_state=None;self.last_cache=None;self.violation_count=-1;self.last_time=-float('inf')
        self.stream.write(json.dumps(dict(schema=self.schema,pose_stream=motion_ref,scope='Acceptance/violation changes and one-simulation-second heartbeat; poses are in the complete recorded-state stream'))+'\n')
        self.stream.flush()
    def record(self,time_s,result,violations):
        heartbeat=time_s-self.last_time>=1.-1e-9
        if result is self.last_cache and len(violations)==self.violation_count and not heartbeat:return
        self.last_cache=result;self.violation_count=len(violations)
        state={k:v for k,v in result.items() if not k.startswith('private_') or k=='private_support_ready'}
        state['violations']=violations
        if state!=self.last_state or heartbeat:
            self.stream.write(json.dumps(dict(sim_time_s=float(time_s),result=state),allow_nan=False)+'\n');self.stream.flush()
            self.last_state=deepcopy(state);self.last_time=float(time_s)
    def close(self):self.stream.close()
