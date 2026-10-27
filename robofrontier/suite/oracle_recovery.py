"""Forward-only Oracle recovery at an explicitly idle ownership handover.

The owner reviews independent single-robot sequences; this module never plans
tasks. RuntimeService owns a separate prepared-world lifecycle, so this adapter
reuses PandaArm and Trial's execution/recording primitives without another DB.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import threading

from ..run_setup import write_json


class RecoverablePlanError(RuntimeError):
    """Owner-declared planning fault, never an automatic physics exception."""


class OracleRecovery:
    def __init__(self, trial, *, owner, world_id, observe):
        if getattr(trial, '_oracle_recovery_attachment', None) is not None:
            raise ValueError('This world already has a recovery execution owner')
        if any(arm.job is not None for arm in trial.arms.values()):
            raise ValueError('Attach only at an idle, exclusive execution handover')
        self.trial, self.owner, self.world_id, self.observe = trial, owner, world_id, observe
        self.requests, self.running, self.reserved, self.unknown = {}, {}, {}, {}
        self.recoveries, self.paused = {}, False
        self.dt = float(trial.model.opt.timestep)
        self.directory = Path(trial.directory)/'recovery'
        for folder in ('requests', 'results', 'sources', 'rejected'):
            (self.directory/folder).mkdir(parents=True, exist_ok=True)
        self.files, self.sources, self.assisted = {}, {}, False
        self.last_observation = -math.inf
        self.thread, self.closed = threading.get_ident(), False
        self.attached, self.submitted_actions = True, 0
        self.original_execute = trial.execute
        trial._oracle_recovery_attachment = self
        write_json(self.directory/'world.json', dict(owner=owner, world_id=world_id,
            scope='Trusted owner identity, not an OS security sandbox'))
        trial.execute = self._blocked_legacy_execute

    def _thread(self, execution=True):
        if threading.get_ident() != self.thread or self.closed:
            raise ValueError('Use the single live physics owner thread')
        if execution and not self.attached:
            raise ValueError('Execution was handed back; this adapter cannot step or submit')

    def _blocked_legacy_execute(self, *args, **kwargs):
        raise ValueError('Recovery owns execution; drain and explicitly handoff before Trial.execute')

    @staticmethod
    def _evidence(value):
        try:
            value = json.loads(json.dumps(value, allow_nan=False))
        except (TypeError, ValueError) as error:
            raise ValueError('Owner observation must be finite JSON, never Trial/Monitor') from error
        if not isinstance(value, dict) or set(value)-{'task', 'robots', 'objects', 'actions', 'images'}:
            raise ValueError('Only owner-selected observation fields are allowed')
        def inspect(node):
            if isinstance(node, dict):
                for key, child in node.items():
                    if key.lower() in ('game', 'secret', 'decoder', 'unobserved_feedback') or key.lower().startswith('private'):
                        raise ValueError('Hidden game/feedback fields are not repair observations')
                    inspect(child)
            elif isinstance(node, list):
                for child in node:
                    inspect(child)
        inspect(value)
        return value

    def _source(self, request):
        source = request.get('repair_source')
        if source is None:
            return
        path = Path(source['path'])
        path = (self.directory/path).resolve() if not path.is_absolute() else path.resolve()
        if not path.is_relative_to(self.directory.resolve()) or path.suffix != '.py':
            raise ValueError('Repair source must be an owner-directory Python file')
        payload = path.read_bytes(); digest = hashlib.sha256(payload).hexdigest()
        if source['sha256'] != digest:
            raise ValueError('Repair source SHA mismatch')
        target = self.directory/'sources'/(digest+'.py')
        if target.exists() and target.read_bytes() != payload:
            raise ValueError('Archived repair source changed')
        if not target.exists():
            target.write_bytes(payload)
        # Owner evaluates this module externally. The physics thread never execs it.
        self.sources[digest] = dict(sha256=digest, archived_path=str(target), execution='external_owner')

    def _publish(self, ident):
        write_json(self.directory/'results'/(ident+'.json'), self.status(ident))

    def submit(self, request):
        self._thread()
        request = json.loads(json.dumps(request, allow_nan=False))
        if set(request)-{'owner', 'world_id', 'id', 'robot', 'sequence', 'review',
                        'recovery_id', 'assisted', 'repair_source'}:
            raise ValueError('Unsupported recovery request field')
        if request['owner'] != self.owner or request['world_id'] != self.world_id:
            raise ValueError('Request does not belong to this world owner')
        ident, robot, steps = request['id'], request['robot'], request['sequence']
        if not isinstance(ident, str) or re.fullmatch(r'[A-Za-z0-9_-]{1,96}', ident) is None:
            raise ValueError('Use a bounded plain request ID')
        if ident in self.requests:
            if self.requests[ident]['request'] != request:
                raise ValueError('Request ID reused with different input')
            return self.status(ident)
        if self.paused or robot in self.reserved or robot in self.unknown:
            raise ValueError('Robot paused, reserved, or outcome uncertain')
        if robot not in self.trial.arms or self.trial.arms[robot].job is not None:
            raise ValueError('Unknown or externally busy robot')
        recovery = self.recoveries.get(request.get('recovery_id'))
        affected = [r for r in self.recoveries.values()
                    if robot in r['robots'] and r['status'] != 'feedback_ready']
        if ('recovery_id' in request and (recovery is None or robot not in recovery['robots'])):
            raise ValueError('Recovery does not match this robot')
        if affected and recovery not in affected:
            raise ValueError('Affected robot requires the matching owner recovery input')
        if not request.get('review') or not isinstance(steps, list) or not 1 <= len(steps) <= 64:
            raise ValueError('Owner review and a finite sequence are required')
        if any(set(s) != {'op', 'parameters'} or s['op'] not in ('move', 'gripper')
               or not isinstance(s['parameters'], dict) for s in steps):
            raise ValueError('Only ordinary finite robot commands are supported')
        if not isinstance(request.get('assisted', False), bool):
            raise ValueError('Assistance is an explicit owner attestation')
        self._source(request)
        self.assisted |= request.get('assisted', False)
        rec = dict(request=deepcopy(request), status='queued', next_step=0, feedback=[])
        rec['input_sha256'] = hashlib.sha256(json.dumps(request, sort_keys=True).encode()).hexdigest()
        self.requests[ident] = rec; self.reserved[robot] = ident
        if recovery is not None:
            recovery['status'] = 'repair_running'
        self._publish(ident)
        self.trial.event('oracle_recovery_input', request_id=ident, input_sha256=rec['input_sha256'],
                         assisted=request.get('assisted', False), repair_source=request.get('repair_source'))
        self._start(ident)
        return self.status(ident)

    def _start(self, ident):
        rec = self.requests[ident]; req = rec['request']; robot = req['robot']
        if self.paused:
            self._finish(ident, 'failed', 'Paused before next command'); return
        step = req['sequence'][rec['next_step']]; t = self.trial
        try:
            prepared = t.arms[robot].validate(step['op'], step['parameters'], t.cfg,
                t.task['workspace_world_m']['lower'], t.task['workspace_world_m']['upper'])
        except (ValueError, KeyError, TypeError) as error:
            self._finish(ident, 'failed', 'Admission rejected: '+str(error)); return
        call_id = f'recovery-{ident}-{rec["next_step"]}'
        self.running[robot] = dict(id=ident, call_id=call_id, start=t.time, step=step)
        rec.update(status='running', active_call_id=call_id)
        self._publish(ident)
        t.event('action_start', label=call_id, commands={robot:(step['op'], step['parameters'])})
        self.submitted_actions += 1
        try:
            t.arms[robot].start(call_id, prepared, t.time)
        except Exception as error:
            self.unknown[robot] = call_id
            self._finish(ident, 'uncertain', 'Executor acknowledgement failed: '+str(error))

    def _finish(self, ident, status, reason=None):
        rec = self.requests[ident]; robot = rec['request']['robot']
        rec.update(status=status, reason=reason, finished_sim_s=self.trial.time)
        if status != 'uncertain':
            rec['active_call_id'] = None
        self.reserved.pop(robot, None)
        recovery = self.recoveries.get(rec['request'].get('recovery_id'))
        if recovery is not None:
            recovery['status'] = 'needs_engineering_review' if status == 'uncertain' else 'feedback_ready'
        self._publish(ident)

    def recover(self, error, robots, *, evidence=None):
        self._thread()
        if not isinstance(error, RecoverablePlanError):
            raise ValueError('Only an explicitly marked planning fault is recoverable here')
        if not robots or any(r not in self.trial.arms or self.trial.arms[r].job is not None
                             or r in self.reserved or r in self.unknown for r in robots):
            raise ValueError('Recovery requires known idle affected robots')
        ident = f'plan-{len(self.recoveries)+1}'
        rec = dict(id=ident, robots=list(robots), status='awaiting_owner',
                   error=str(error), evidence=self._evidence(evidence or {}), sim_time_s=self.trial.time)
        self.recoveries[ident] = rec
        self.trial.event('oracle_recovery_requested', recovery=rec)
        self.service()
        return deepcopy(rec)

    def service(self):
        self._thread()
        for path in sorted((self.directory/'requests').glob('*.json')):
            payload = path.read_bytes(); digest = hashlib.sha256(payload).hexdigest()
            if self.files.get(str(path)) == digest:
                continue
            self.files[str(path)] = digest
            try:
                self.submit(json.loads(payload))
            except (ValueError, KeyError, TypeError, OSError) as error:
                write_json(self.directory/'rejected'/(digest+'.json'), dict(error=str(error), input_sha256=digest))
                self.trial.event('oracle_recovery_input_rejected', input_sha256=digest, reason=str(error))
        if self.trial.time-self.last_observation >= .5:
            try:
                write_json(self.directory/'observation.json', self.observation())
            except (ValueError, TypeError) as error:
                # Bad observation selection cannot expose secrets or stop other work.
                write_json(self.directory/'observation-error.json', dict(error=str(error)))
                self.trial.event('oracle_recovery_observation_rejected', reason=str(error))
            self.last_observation = self.trial.time

    def advance(self, seconds):
        self._thread()
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError('Finite forward duration required')
        if float(self.trial.model.opt.timestep) != self.dt:
            raise ValueError('Physics timestep changed')
        for _ in range(round(seconds/self.dt)):
            if self.trial.tick % self.trial.stride == 0:
                self.service()
            for robot, arm in self.trial.arms.items():
                expected = self.running.get(robot, {}).get('call_id', self.unknown.get(robot))
                if arm.job is not None and arm.job != expected:
                    raise ValueError('Foreign in-flight action; adapter must not steal its poll')
            self.trial.step()
            if self.trial.tick % self.trial.stride:
                continue
            for robot, active in list(self.running.items()):
                result = self.trial.arms[robot].poll(self.trial.time, 15)
                if result is None:
                    continue
                ident = active['id']; rec = self.requests[ident]
                rec['feedback'].append(deepcopy(result)); self.running.pop(robot)
                if result['status'] == 'uncertain':
                    self.unknown[robot] = active['call_id']
                    self._finish(ident, 'uncertain', result.get('reason')); continue
                self.trial.actions.append(dict(label=active['call_id'],
                    commands={robot:(active['step']['op'], active['step']['parameters'])},
                    results={robot:result}, start_sim_s=active['start'], end_sim_s=self.trial.time))
                self.trial.event('action_end', label=active['call_id'], results={robot:result})
                if rec['status'] == 'uncertain':
                    rec.update(terminal_feedback=deepcopy(result), active_call_id=None)
                    self._publish(ident); continue
                rec['next_step'] += 1
                if result['status'] != 'completed':
                    self._finish(ident, 'failed', result.get('reason'))
                elif rec['next_step'] == len(rec['request']['sequence']):
                    self._finish(ident, 'completed')
                else:
                    self._start(ident)
                self._publish(ident)

    def status(self, ident):
        self._thread(False)
        return deepcopy(self.requests[ident])

    def observation(self):
        self._thread(False)
        return dict(world_id=self.world_id, sim_time_s=self.trial.time,
                    requests=deepcopy(self.requests), recoveries=deepcopy(self.recoveries),
                    evidence=self._evidence(self.observe()))

    def handoff(self, owner):
        """Return the live world to legacy code only after a known drained boundary."""
        self._thread()
        if (owner != self.owner or self.running or self.reserved or self.unknown
            or any(a.job is not None for a in self.trial.arms.values())
            or any(r['status'] != 'feedback_ready' for r in self.recoveries.values())):
            raise ValueError('Resolve all executions/recovery feedback before legacy handoff')
        self.trial.execute = self.original_execute
        self.attached = False
        self.trial.event('oracle_recovery_legacy_handoff', owner=owner)

    def reattach(self, owner):
        """Allow a later repair boundary without creating a second owner."""
        self._thread(False)
        if (owner != self.owner or self.attached or self.running or self.reserved or self.unknown
            or any(a.job is not None for a in self.trial.arms.values())
            or self.trial.execute != self.original_execute):
            raise ValueError('Reattach only from a known idle legacy handover')
        self.attached=True;self.paused=False
        self.trial.execute=self._blocked_legacy_execute
        self.trial.event('oracle_recovery_reattached', owner=owner)

    def acknowledge(self, owner, ident, review, assisted=False):
        """Owner closes a reviewed boundary only after all feedback is known."""
        self._thread()
        if (owner != self.owner or ident not in self.recoveries or not review
            or self.running or self.reserved or self.unknown
            or any(a.job is not None for a in self.trial.arms.values())):
            raise ValueError('Known idle reviewed recovery required')
        if not isinstance(assisted,bool):raise ValueError('Assistance must be explicit')
        self.assisted |= assisted
        self.recoveries[ident].update(status='feedback_ready',review=str(review))
        self.trial.event('oracle_recovery_reviewed', recovery_id=ident,review=str(review))

    def pause(self, owner):
        self._thread()
        if owner != self.owner:
            raise ValueError('Only the world owner may pause')
        self.paused = True
        for robot in self.running:
            self.trial.arms[robot].request_stop(self.trial.time, 'Paused by Oracle owner')

    def resume(self, owner):
        self._thread()
        if owner != self.owner or self.running or self.reserved or self.unknown:
            raise ValueError('Resume only after known terminal feedback')
        self.paused=False
        self.trial.event('oracle_recovery_resumed', owner=owner)

    def fence(self, owner, ident, reason):
        """Preserve an uncertain acknowledgement; terminal feedback cannot auto-clear it."""
        self._thread()
        if owner != self.owner or self.requests[ident]['status'] != 'running':
            raise ValueError('Only the owner may fence its active execution')
        robot = self.requests[ident]['request']['robot']
        self.unknown[robot] = self.running[robot]['call_id']
        self._finish(ident, 'uncertain', reason)
        self.trial.event('oracle_recovery_execution_fenced', request_id=ident, reason=reason)

    def close(self, report):
        if self.closed:
            return
        self._thread(False)
        report['unresolved_robot_actions'] = sorted(set(self.unknown) |
            {r for r, a in self.trial.arms.items() if a.job is not None})
        if report['unresolved_robot_actions'] or self.trial.monitor.violations:
            report['passed'] = False
        metadata = dict(llm_used=self.assisted, assisted=self.assisted,
            oracle_mode='privileged_assisted' if self.assisted else 'pure_program',
            recovery=dict(world_id=self.world_id, owner=self.owner, sources=list(self.sources.values()),
                inputs=[dict(id=i, input_sha256=r['input_sha256']) for i,r in self.requests.items()],
                requests=deepcopy(self.requests), boundaries=deepcopy(self.recoveries),
                retained_process_violations=deepcopy(self.trial.monitor.violations),
                submitted_robot_actions=self.submitted_actions, legacy_handoff_supported=True,
                restart_supported=False, source_modules_hot_executed=False))
        report.update(metadata)  # trial_end must already carry the correct classification.
        self.trial.close(report)
        report.update(metadata)  # Preserve adapter classification after core close.
        write_json(self.trial.directory/'report.json', report)
        self.trial.execute = self.original_execute
        self.closed = True
