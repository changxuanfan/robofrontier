"""Claude session observations and deterministic continuation, under development.

This dispatcher does not launch a world, start a benchmark, create sessions, or
implement adaptive delegation. It operates only on an already-bound Controller.
"""
import time
import uuid

from robofrontier.runtime_cli import execute


class ClaudeDispatcher:
    def __init__(self, store, host, owner=None):
        self.store = store
        self.host = host
        self.owner = owner or 'claude-meta-' + uuid.uuid4().hex
        cfg = store.get('config')
        if cfg.get('engine') != 'claude' or cfg.get('topology') != 'centralized':
            raise ValueError('Only an engine=claude centralized run is supported; adaptive is not qualified')
        if host.actor != 'Controller':
            raise ValueError('Expected the bound Controller host')
        if store.agent('Controller')['thread_id'] != host.session_id:
            raise ValueError('Host session is not bound to this Controller')

    def admin(self, op, data=None):
        return execute(self.store, op, data or {}, self.owner, time.time())

    def observe(self):
        observation = self.host.observe()
        if observation['thread_id'] != self.store.agent('Controller')['thread_id']:
            raise ValueError('Observed session does not match runtime binding')
        state = observation['status']
        if state not in ('active', 'idle', 'error'):
            self.admin('session-unavailable', {'actor': 'Controller', 'thread_id': observation['thread_id'],
                                                'reason': 'Claude turn outcome unknown'})
            self.admin('pause', {'reason': 'Claude session state requires engineering review'})
            return None
        item = {'actor': 'Controller', 'thread_id': observation['thread_id'],
                'status': state, 'turn_id': observation['turn_id'], 'observed_at': time.time(),
                'source': 'claude.host', 'turn_status': 'failed' if state == 'error' else None}
        self.admin('observe-sessions', {'observations': [item]})
        return item

    def cycle(self):
        """Observe and dispatch eligible scheduler wakes; START remains explicit."""
        snapshot = self.admin('snapshot')
        if snapshot['paused'] or snapshot['lifecycle'] in ('ended', 'interrupted', 'recovery_required'):
            return {'dispatched': [], 'lifecycle': snapshot['lifecycle'], 'paused': snapshot['paused']}
        if not self.observe():
            return {'dispatched': [], 'lifecycle': snapshot['lifecycle'], 'paused': True}
        if snapshot['lifecycle'] != 'running':
            return {'dispatched': [], 'lifecycle': snapshot['lifecycle'], 'paused': False}
        sent = []
        for command in self.admin('plan')['commands']:
            # Reobserve before claiming; the runtime checks turn, lease and pending actions.
            if not self.observe(): break
            claimed = self.admin('claim', {'wake_id': command['wake_id']})['command']
            if not claimed: continue
            try:
                ack = self.host.send(claimed['prompt'])
                if ack.get('threadId') != claimed['thread_id']:
                    raise ValueError('Claude turn acknowledgement has wrong session')
            except Exception:
                self.admin('dispatch-result', {'wake_id': claimed['wake_id'], 'outcome': 'uncertain',
                                               'result': {'error': 'Claude dispatch outcome requires review'}})
                self.admin('pause', {'reason': 'Claude dispatch outcome uncertain'})
                return {'dispatched': sent, 'lifecycle': snapshot['lifecycle'], 'paused': True}
            self.admin('dispatch-result', {'wake_id': claimed['wake_id'], 'outcome': 'sent',
                                           'result': {'accepted': True}})
            sent.append(claimed['wake_id'])
        return {'dispatched': sent, 'lifecycle': snapshot['lifecycle'], 'paused': False}
