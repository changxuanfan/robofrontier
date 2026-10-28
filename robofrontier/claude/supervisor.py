"""Engineering lifecycle for an existing centralized Claude policy/world.

This does not admit benchmark batches or create worlds. The caller owns the
physical process; seal is the only normal shutdown path and never replays work.
"""
import math
import time


class CentralizedSupervisor:
    def __init__(self, policy, owner_alive, *, stop_requested=lambda: False,
                 startup_timeout_s=300, wall_limit_s=86400, poll_s=1,
                 seal_timeout_s=30, clock=time.monotonic, sleep=time.sleep):
        for name, value, maximum in (('startup_timeout_s', startup_timeout_s, 86400),
                                     ('wall_limit_s', wall_limit_s, 86400),
                                     ('poll_s', poll_s, 60)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= maximum:
                raise ValueError('Invalid ' + name)
        if isinstance(seal_timeout_s, bool) or not isinstance(seal_timeout_s, (int, float)) or not math.isfinite(seal_timeout_s) or not 0 <= seal_timeout_s <= 120:
            raise ValueError('Invalid seal timeout')
        self.policy = policy
        self.owner_alive = owner_alive
        self.stop_requested = stop_requested
        self.startup_timeout_s = startup_timeout_s
        self.wall_limit_s = wall_limit_s
        self.poll_s = poll_s
        self.seal_timeout_s = seal_timeout_s
        self.clock = clock
        self.sleep = sleep
        self.used = False

    def run(self):
        if self.used:
            raise ValueError('Supervisor cannot restart a world')
        self.used = True
        reason = 'supervisor_error'
        error_type = None
        sealed = None
        seal_error_type = None
        started_at = None
        session_evidence = None
        began = self.clock()
        try:
            if self.stop_requested():
                reason = 'stop_requested'
            elif not self.owner_alive():
                reason = 'owner_unavailable'
            else:
                self.policy.bind_and_bootstrap()
                if self.policy.store.get('lifecycle') == 'running':
                    # The direct host starts before the first model turn.
                    started_at = getattr(self.policy, 'started_monotonic', self.clock())
                while True:
                    if self.stop_requested():
                        reason = 'stop_requested'; break
                    if not self.owner_alive():
                        reason = 'owner_unavailable'; break
                    lifecycle = self.policy.store.get('lifecycle')
                    if lifecycle in ('ended', 'interrupted', 'recovery_required'):
                        reason = 'world_' + lifecycle; break
                    if self.policy.store.get('dispatch_paused', False):
                        reason = 'dispatch_paused'; break
                    observation = self.policy.host.observe()
                    state = observation['status']
                    if state in ('unknown', 'error'):
                        session_evidence = {key: observation.get(key) for key in
                                            ('budget_status', 'last_result_subtype', 'last_turn_reported_cost_usd')}
                        reason = 'session_' + state; break
                    now = self.clock()
                    if started_at is None:
                        if now - began >= self.startup_timeout_s:
                            reason = 'startup_timeout'; break
                        if state == 'idle' and self.policy.store.agent('Controller')['ready']:
                            self.policy.start_when_ready()
                            started_at = self.clock()
                    else:
                        if now - started_at >= self.wall_limit_s:
                            reason = 'wall_limit'; break
                        result = self.policy.cycle()
                        if result.get('paused'):
                            reason = 'dispatch_paused'; break
                    self.sleep(self.poll_s)
        except Exception as error:
            error_type = type(error).__name__
        finally:
            # Prevent further model turns before asking the physical owner to seal.
            # A killed CLI turn has an unknown outcome, never an automatic retry.
            try:
                self.policy.stop()
                if hasattr(self.policy.host, 'wait'):
                    self.policy.host.wait(timeout=5)
            except Exception as error:
                error_type = error_type or type(error).__name__
            try:
                # The shared seal leaves an unresolved live world paused rather
                # than force-closing or replaying uncertain physical actions.
                sealed = self.policy.dispatcher.admin('seal', {'timeout_s': self.seal_timeout_s}) if self.policy.dispatcher else None
                if sealed is None:
                    from robofrontier.run_seal import seal_run
                    sealed = seal_run(self.policy.store, self.seal_timeout_s)
            except Exception as error:
                seal_error_type = type(error).__name__
        return {'reason': reason, 'error_type': error_type, 'seal': sealed,
                'seal_error_type': seal_error_type, 'started': started_at is not None,
                'session_evidence': session_evidence,
                'scope': 'engineering lifecycle; task success requires runtime evaluation'}
