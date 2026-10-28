"""Experimental centralized policy orchestration for an existing world.

This module deliberately does not launch worlds, admit scored batches, or
support adaptive delegation. A caller owns the world process and final seal.
"""
from pathlib import Path
import time
import uuid

from robofrontier.claude.bootstrap import bootstrap_prompt
from robofrontier.claude.dispatch import ClaudeDispatcher
from robofrontier.claude.session_host import ClaudeSessionHost
from robofrontier.pipeline_client import client_call
from robofrontier.run_setup import verify_inputs
from robofrontier.runtime_store import RuntimeStore


class CentralizedPolicyHost:
    def __init__(self, run_dir, profile, base_env, *, host_factory=ClaudeSessionHost,
                 client=client_call, verifier=verify_inputs):
        self.run = Path(run_dir).resolve()
        self.store = RuntimeStore(self.run / 'private/state/runtime.sqlite')
        try:
            config = self.store.get('config')
            if not config or config.get('engine') != 'claude' or config.get('topology') != 'centralized':
                raise ValueError('Only centralized Claude engineering runs are supported')
            if config.get('model') != profile.get('model') or config.get('effort') != profile.get('effort'):
                raise ValueError('Profile differs from frozen runtime configuration')
            if self.store.get('lifecycle') != 'prepared' or self.store.agent('Controller')['thread_id']:
                raise ValueError('A previously bound or started world cannot be restarted')
            changed = verifier(self.run)
            if changed:
                raise ValueError('Frozen inputs changed: ' + ', '.join(changed))
            self.prompt = bootstrap_prompt(self.run, startup_protocol=config.get('startup_protocol', 'ready_barrier_v1'))
            self.host = host_factory(self.run, 'Controller', profile, base_env)
            self.client = client
            self.bound = False
            self.bind_attempted = False
            self.dispatcher = None
        except Exception:
            self.store.close()
            raise

    def bind_and_bootstrap(self):
        if self.bind_attempted or self.store.agent('Controller')['thread_id']:
            raise ValueError('Never repeat a role binding')
        # An acknowledgement can be lost even if the owner accepted the bind.
        self.bind_attempted = True
        connection = self.run / 'private/state/connections/Meta.json'
        result = self.client(connection, 'bind', {'actor': 'Controller', 'thread_id': self.host.session_id},
                             'bind-' + uuid.uuid4().hex, log_call_id=False)
        if not result.get('bound') or self.store.agent('Controller')['thread_id'] != self.host.session_id:
            raise ValueError('Role binding outcome unknown; review instead of retrying')
        self.bound = True
        self.dispatcher = ClaudeDispatcher(self.store, self.host)
        if self.store.get('config').get('startup_protocol') == 'direct_controller_v2':
            self.started_monotonic = time.monotonic()
            reply = self.dispatcher.admin('request-start')
            if not reply.get('started'): raise ValueError('Direct START was not confirmed')
        return self.host.send(self.prompt)

    def start_when_ready(self):
        """Explicitly start after a completed bootstrap and fresh READY evidence."""
        if not self.bound or not self.dispatcher:
            raise ValueError('Controller has not been bound')
        observation = self.dispatcher.observe()
        if not observation or observation['status'] != 'idle' or not self.store.agent('Controller')['ready']:
            raise ValueError('Controller must be READY and its startup turn idle')
        answer = self.dispatcher.admin('request-start')
        if not answer.get('started'):
            raise ValueError('World did not confirm START')
        return answer

    def cycle(self):
        if not self.dispatcher: raise ValueError('Controller has not been bound')
        return self.dispatcher.cycle()

    def stop(self):
        """Stop model turn; caller must still seal the existing world."""
        return self.host.stop()

    def close(self):
        if self.host.observe()['status'] == 'active':
            raise ValueError('Stop and wait for the active policy turn before closing')
        self.store.close()
