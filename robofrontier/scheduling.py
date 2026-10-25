"""Deterministic continuation planning; contains no task planner or model calls."""
import time
import uuid
from .runtime_store import packed


class Scheduler:
    def __init__(self, store):
        self.store = store

    def execution_blocks_wake(self, actor):
        # Planning/mail may continue during acknowledged nonblocking execution.
        # A blocking robot action or unknown outcome still needs its existing result.
        from .action_batch import external_wait
        return any(v['status']=='uncertain' or
                   v['status']=='running' and external_wait(self.store,v)
                   for v in self.store.actions(actor).values())

    def stopped_delegate_without_new_mail(self, actor, value):
        return (value['kind']=='delegate' and value.get('last_stop_notice_turn')==value['session_turn']
                and self.store.pending(actor)['message_seq']<=value.get('stop_notice_message_seq',0))

    def failed_delegate_can_resume(self, actor, value):
        # Process a new parent message after a proven failed turn, not a blind
        # retry driven by old work/timers. No physical command is replayed here.
        return (value['kind']=='delegate' and value['session_status']=='error'
                and value.get('session_turn_status')=='failed'
                and value.get('last_stop_notice_turn')==value['session_turn']
                and not self.stopped_delegate_without_new_mail(actor,value)
                and not any(v['status'] in ('running','uncertain') for v in self.store.actions(actor).values())
                and not self.store.db.execute("SELECT 1 FROM calls WHERE actor=? AND status='accepted' AND op IN ('move','gripper','action_batch') LIMIT 1",(actor,)).fetchone())

    def can_wake(self, actor, value, now):
        if value['kind']=='manager' or value['recovery_required'] or not value['thread_id']:
            return False
        if value['kind']=='delegate' and (value.get('delegation_state')!='active' or not value.get('delegation_started')):
            return False
        observed=value['session_observed_at']
        return ((value['session_status']=='idle' or self.failed_delegate_can_resume(actor,value))
                and bool(value['session_turn']) and observed is not None
                and 0 <= now-observed <= self.store.get('config')['session_observation_ttl_s']
                and not self.execution_blocks_wake(actor))

    @staticmethod
    def explicit_wait_due(value, now):
        return (value['work_state'] in ('waiting','available') and value['next_check'] is not None
                and value['next_check']<=now)

    def plan(self, owner, now=None):
        now = time.time() if now is None else now
        s = self.store
        cfg = s.get('config')
        with s.transaction():
            s.lease(owner, now, cfg['manager_lease_s'])
            if s.get('lifecycle') != 'running' or s.get('dispatch_paused', False):
                return []
            wakes = s.wake_rows()
            commands = []
            for actor, a in s.agents().items():
                if not self.can_wake(actor,a,now):
                    continue
                inflight = [w for w in wakes if w['actor'] == actor and w['status'] in ('pending', 'dispatching', 'sent', 'uncertain')]
                if inflight:
                    for w in inflight:
                        if w['status'] == 'pending':
                            commands.append(w['value'])
                        elif w['status'] == 'dispatching' and w['value'].get('claimed_by') != owner:
                            s.db.execute("UPDATE wakes SET status='uncertain' WHERE id=?", (w['id'],))
                            s.event('WAKE_OUTCOME_UNCERTAIN', cfg['manager'], {'wake_id': w['id'], 'reason': 'Dispatcher ownership changed during send'})
                    continue
                pending = s.pending(actor)
                mail_token = f"{pending['message_seq']}:{pending['signal_seq']}"
                has_mail = pending['messages'] + pending['signals'] > 0
                timer_seen = any(w['actor'] == actor and w['status'] != 'cancelled' and w['value'].get('timer_revision') == a['work_revision'] for w in wakes)
                due = a['next_check'] is not None and a['next_check'] <= now and not timer_seen
                continuing = a['work_state'] in ('continue', 'working')
                if self.stopped_delegate_without_new_mail(actor,a):
                    # An explicit saved wait continues this active scope once.
                    # Old mail or an inherited continue state alone does not.
                    has_mail=False;continuing=False
                    due=due and self.explicit_wait_due(a,now)
                if not (has_mail or due or continuing):
                    continue
                previous_mail = [w for w in wakes if w['actor'] == actor and w['status'] != 'cancelled' and w['value'].get('mail_token') == mail_token]
                if has_mail and len(previous_mail) >= cfg['max_unacknowledged_wakes']:
                    s.update_agent(actor, recovery_required=True)
                    s.event('RECIPIENT_RECOVERY_REQUIRED', cfg['manager'], {'actor': actor, 'reason': 'Repeated continuations without client receipt; no automatic interpretation of message content'})
                    continue
                terminal_count = sum(v['status'] in ('completed', 'failed') for v in s.actions(actor).values())
                work_token = f"{a['work_revision']}:{terminal_count}"
                previous_work = [w for w in wakes if w['actor'] == actor and w['status'] != 'cancelled' and w['value'].get('work_token') == work_token]
                if continuing and not has_mail and not due and len(previous_work) >= cfg['max_unacknowledged_wakes']:
                    s.update_agent(actor, recovery_required=True)
                    s.event('RECIPIENT_RECOVERY_REQUIRED', cfg['manager'], {'actor': actor, 'reason': 'Pending work had no saved or execution progress'})
                    continue
                reasons = []
                if has_mail:
                    reasons.append('有发给你的未确认收件或系统状态事件')
                if due:
                    reasons.append('你保存的检查期限已到')
                if continuing:
                    reasons.append('你保存的工作状态仍需继续')
                key = packed([actor, a['session_turn'], mail_token if has_mail else None,
                              a['work_revision'] if due else None, work_token if continuing else None])
                existing=s.db.execute('SELECT id,status FROM wakes WHERE trigger_key=?',(key,)).fetchone()
                if existing:
                    if existing['status']!='cancelled':
                        continue
                    s.db.execute('UPDATE wakes SET trigger_key=? WHERE id=?',(key+'|cancelled:'+existing['id'],existing['id']))
                wake_id = uuid.uuid4().hex
                prompt = (f"运行通知〔wake_id={wake_id}〕：平台状态观察显示你的上一轮已结束；"
                          + '；'.join(reasons) + '。请合并新增事件，复用有效上下文；需要时恢复已保存工作，自行决定继续、等待或结束。'
                          '本通知不包含任务分工或动作建议。结束前按当前角色协议保存状态或交回委派；交付/收件不代表接受或任务完成。')
                value = {'wake_id': wake_id, 'actor': actor, 'thread_id': a['thread_id'],
                         'expected_turn': a['session_turn'], 'reasons': reasons, 'prompt': prompt,
                         'created_at': now, 'mail_token': mail_token if has_mail else None,
                         'timer_revision': a['work_revision'] if due else None,
                         'work_token': work_token if continuing else None}
                s.db.execute('INSERT INTO wakes VALUES (?,?,?,?,?)', (wake_id, actor, key, 'pending', packed(value)))
                s.event('WAKE_PLANNED', cfg['manager'], value, now)
                commands.append(value)
            return commands

    def claim(self, wake_id, owner, now=None):
        now = time.time() if now is None else now
        s = self.store
        cfg = s.get('config')
        with s.transaction():
            lease = s.db.execute("SELECT * FROM leases WHERE name='scheduler'").fetchone()
            if not lease or lease['owner'] != owner or lease['expires'] < now:
                raise ValueError('Refresh scheduler lease before dispatch')
            row = s.db.execute('SELECT * FROM wakes WHERE id=?', (wake_id,)).fetchone()
            if not row or row['status'] != 'pending':
                raise ValueError('Wake is not pending')
            import json
            value = json.loads(row['value'])
            actor = s.agent(row['actor'])
            scheduled_review=(value.get('timer_revision')==actor['work_revision'] and self.explicit_wait_due(actor,now))
            stopped_without_intent=self.stopped_delegate_without_new_mail(row['actor'],actor) and not scheduled_review
            if (s.get('lifecycle') != 'running' or s.get('dispatch_paused', False)
                or not self.can_wake(row['actor'],actor,now) or actor['session_turn'] != value['expected_turn']
                or actor['thread_id']!=value['thread_id'] or stopped_without_intent):
                s.db.execute("UPDATE wakes SET status='cancelled' WHERE id=?", (wake_id,))
                s.event('WAKE_CANCELLED', cfg['manager'], {'wake_id': wake_id, 'reason': 'Session/run/execution state changed before dispatch'}, now)
                return None
            value.update(claimed_by=owner, claimed_at=now)
            s.db.execute("UPDATE wakes SET status='dispatching',value=? WHERE id=?", (packed(value), wake_id))
            return value


class FakeSessionAdapter:
    """Offline contract adapter. No Codex connections, inference or robot actions."""
    def __init__(self, observations):
        self.observations = observations
        self.sent = []

    def observe(self, actor):
        return self.observations[actor]

    def send(self, command):
        if self.observations[command['actor']]['status'] != 'idle':
            raise ValueError('Fake receiver became active')
        self.sent.append(command)
        return {'threadId': command['thread_id']}


def dispatch_once(store, adapter, owner, now):
    """Used by offline tests; live Meta uses the same plan/claim/record sequence."""
    for actor, a in store.agents().items():
        if a['kind'] != 'manager' and a['thread_id']:
            observed = adapter.observe(actor)
            store.observe_session(actor, observed['status'], observed['turn_id'], now, 'codex.wait_threads')
    scheduler = Scheduler(store)
    results = []
    for command in scheduler.plan(owner, now):
        o = adapter.observe(command['actor'])
        store.observe_session(command['actor'], o['status'], o['turn_id'], now, 'codex.wait_threads')
        claimed = scheduler.claim(command['wake_id'], owner, now)
        if claimed:
            try:
                result = adapter.send(claimed)
            except Exception as exc:
                store.record_dispatch(command['wake_id'], 'uncertain', {'error': str(exc)})
            else:
                store.record_dispatch(command['wake_id'], 'sent', result)
            results.append(command['wake_id'])
    return results
