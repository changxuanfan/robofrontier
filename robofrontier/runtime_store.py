"""Transactional run state. Natural-language task decisions belong to agents."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import sqlite3
import threading
import time
import uuid
from .tool_errors import CallIdConflict


def packed(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)


class RuntimeStore:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.db = sqlite3.connect(self.path, isolation_level=None, check_same_thread=False, timeout=10)
        self.db.row_factory = sqlite3.Row
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('PRAGMA foreign_keys=ON')
        self.db.executescript('''
        CREATE TABLE IF NOT EXISTS meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS agents(id TEXT PRIMARY KEY, value TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS messages(seq INTEGER PRIMARY KEY AUTOINCREMENT, id TEXT UNIQUE,
          sender TEXT NOT NULL, recipients TEXT NOT NULL, text TEXT NOT NULL, reply_to TEXT,
          send_key TEXT UNIQUE NOT NULL, created REAL NOT NULL);
        CREATE TABLE IF NOT EXISTS receipts(seq INTEGER REFERENCES messages(seq), actor TEXT,
          acked INTEGER NOT NULL DEFAULT 0, PRIMARY KEY(seq,actor));
        CREATE TABLE IF NOT EXISTS signals(seq INTEGER PRIMARY KEY AUTOINCREMENT, actor TEXT,
          value TEXT NOT NULL, acked INTEGER NOT NULL DEFAULT 0);
        CREATE TABLE IF NOT EXISTS batches(id TEXT PRIMARY KEY, actor TEXT NOT NULL,
          value TEXT NOT NULL, acked INTEGER NOT NULL DEFAULT 0);
        CREATE TABLE IF NOT EXISTS calls(id TEXT PRIMARY KEY, actor TEXT NOT NULL, op TEXT NOT NULL,
          parameters TEXT NOT NULL, status TEXT NOT NULL, result TEXT, updated REAL NOT NULL);
        CREATE TABLE IF NOT EXISTS actions(id TEXT PRIMARY KEY REFERENCES calls(id), actor TEXT,
          status TEXT NOT NULL, value TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS wakes(id TEXT PRIMARY KEY, actor TEXT NOT NULL,
          trigger_key TEXT UNIQUE NOT NULL, status TEXT NOT NULL, value TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS leases(name TEXT PRIMARY KEY, owner TEXT, expires REAL);
        CREATE TABLE IF NOT EXISTS events(seq INTEGER PRIMARY KEY AUTOINCREMENT, created REAL,
          kind TEXT, actor TEXT, value TEXT);
        ''')
        os.chmod(self.path, 0o600)

    @contextmanager
    def transaction(self):
        with self.lock:
            nested = self.db.in_transaction
            savepoint = 's' + uuid.uuid4().hex
            self.db.execute('SAVEPOINT ' + savepoint if nested else 'BEGIN IMMEDIATE')
            try:
                yield self
                self.db.execute('RELEASE ' + savepoint if nested else 'COMMIT')
            except BaseException:
                if nested:
                    self.db.execute('ROLLBACK TO ' + savepoint)
                    self.db.execute('RELEASE ' + savepoint)
                else:
                    self.db.execute('ROLLBACK')
                raise

    def close(self):
        self.db.close()

    def get(self, key, default=None):
        with self.lock:
            row = self.db.execute('SELECT value FROM meta WHERE key=?', (key,)).fetchone()
            return json.loads(row[0]) if row else default

    def put(self, key, value):
        with self.transaction():
            self.db.execute('INSERT OR REPLACE INTO meta VALUES (?,?)', (key, packed(value)))

    def event(self, kind, actor, value=None, now=None):
        with self.transaction():
            cur = self.db.execute('INSERT INTO events(created,kind,actor,value) VALUES (?,?,?,?)',
                                  (time.time() if now is None else now, kind, actor, packed(value or {})))
            return cur.lastrowid

    def initialize(self, run_id, config, workers):
        with self.transaction():
            if self.get('run_id') is not None:
                raise ValueError('Runtime already initialized; never overwrite a run')
            self.put('run_id', run_id)
            self.put('schema_version', 1)
            self.put('config', config)
            self.put('robots', list(workers))
            self.put('lifecycle', 'prepared')
            self.put('physics_state', 'not_started')
            self.put('integrity_valid', True)
            roles = [(config['manager'], 'manager')]
            if config['topology'] == 'hierarchical':
                roles.append((config['coordinator'], 'coordinator'))
            roles += [('Controller', 'controller')] if config['topology'] in ('centralized','adaptive') else [(w, 'worker') for w in workers]
            for actor, kind in roles:
                self.db.execute('INSERT INTO agents VALUES (?,?)', (actor, packed({
                    'kind': kind, 'thread_id': None, 'requested_model': config['model'] if kind!='manager' else None, 'requested_effort': config['effort'] if kind!='manager' else None,
                    'ready': False, 'work_state': 'waiting', 'note': '', 'work_saved_at': None, 'next_check': None,
                    'work_revision': 0, 'session_status': 'unknown', 'session_turn': None,
                    'session_observed_at': None, 'recovery_required': False})))
            self.event('RUN_PREPARED', config['manager'])

    def agents(self):
        with self.lock:
            return {r['id']: json.loads(r['value']) for r in self.db.execute('SELECT * FROM agents ORDER BY id')}

    def agent(self, actor):
        with self.lock:
            row = self.db.execute('SELECT value FROM agents WHERE id=?', (actor,)).fetchone()
            if row is None:
                raise ValueError('Unknown role: ' + actor)
            return json.loads(row[0])

    def update_agent(self, actor, **changes):
        with self.transaction():
            value = self.agent(actor)
            value.update(changes)
            self.db.execute('UPDATE agents SET value=? WHERE id=?', (packed(value), actor))
            return value

    def start(self):
        with self.transaction():
            if self.get('lifecycle') != 'prepared':
                raise ValueError('Run is not prepared')
            policy = {a: v for a, v in self.agents().items() if v['kind'] != 'manager'}
            direct = self.get('config').get('startup_protocol') == 'direct_controller_v2'
            if not policy or (direct and (set(policy) != {'Controller'} or not policy['Controller']['thread_id'])) or (not direct and not all(v['ready'] for v in policy.values())):
                raise ValueError('All policy roles must be READY')
            self.put('lifecycle', 'running')
            self.put('physics_state', 'running')
            self.put('start_at', time.time())
            for a, v in policy.items():
                self.update_agent(a, work_state='waiting' if direct else 'continue', note='' if direct else 'START; form your own plan within role authority',
                                  work_saved_at=time.time(), work_revision=v['work_revision'] + 1)
            self.event('START', self.get('config')['manager'])

    def checkpoint_work(self, actor, state, note, after_s=None, now=None):
        cfg = self.get('config')
        if state not in ('working', 'waiting', 'available', 'continue'):
            raise ValueError('Invalid work state')
        if not isinstance(note, str) or len(note) > 16000:
            raise ValueError('Invalid note')
        if state in ('working', 'continue') and not note.strip():
            raise ValueError('Save pending work before requesting continuation')
        if after_s is not None:
            after_s = float(after_s)
            if not cfg['min_check_interval_s'] <= after_s <= cfg['max_check_interval_s']:
                raise ValueError('Check interval outside run limits')
        now = time.time() if now is None else now
        with self.transaction():
            old = self.agent(actor)
            value = self.update_agent(actor, work_state=state, note=note,
                                      work_saved_at=now,
                                      next_check=now + after_s if after_s is not None else None,
                                      work_revision=old['work_revision'] + 1)
            self.event('WORK_SAVED', actor, {'state': state, 'note': note, 'next_check': value['next_check']}, now)
            if state == 'available' and old['work_state'] != 'available' and old['kind'] == 'worker' and self.get('lifecycle') == 'running' and cfg['topology'] == 'hierarchical':
                self.signal(cfg['coordinator'], {'event': 'worker_self_reported_available', 'worker': actor,
                                                'work_revision':value['work_revision'], 'verified_task_completion': False})
            return value

    @staticmethod
    def _delivery_value(messages, events, remaining_messages=0, remaining_events=0, delivery_id=None):
        return {'delivery_id':delivery_id,'messages':messages,'system_events':events,
                'remaining_messages':remaining_messages,'remaining_events':remaining_events,'redelivered':False}

    def _notice_fits(self, message=None, event=None):
        # Include envelope overhead and conservative sequence/count widths.
        value=self._delivery_value([message] if message else [],[event] if event else [],
                                   10**20,10**20,'f'*32)
        return len(packed(value).encode())<=self.get('config')['max_batch_bytes']

    def signal(self, actor, value):
        with self.transaction():
            self.agent(actor)
            if not isinstance(value,dict) or not self._notice_fits(event={**value,'seq':10**20}):
                raise ValueError('System event cannot fit delivery budget; use a compact notice/reference')
            self.db.execute('INSERT INTO signals(actor,value) VALUES (?,?)', (actor, packed(value)))

    def send(self, actor, to, text, send_key, reply_to=None):
        cfg = self.get('config')
        to = [to] if isinstance(to, str) else to
        if not isinstance(to, list) or not to or len(set(to)) != len(to):
            raise ValueError('Explicit unique recipients required')
        if not isinstance(text, str) or not 0 < len(text) <= cfg['max_message_chars']:
            raise ValueError('Message text exceeds run limit')
        probe={'message_id':'m'+'9'*20,'seq':10**20,'from':actor,'to':to,'text':text,'reply_to':reply_to,'created_at':time.time()}
        if not self._notice_fits(message=probe):
            raise ValueError('Message cannot fit the configured delivery-byte limit')
        with self.transaction():
            sender = self.agent(actor)
            if sender['kind'] == 'manager':
                raise ValueError('Meta cannot send business messages')
            if self.get('lifecycle') in ('interrupted', 'recovery_required'):
                raise ValueError('Run interrupted; no new business work')
            for target in to:
                recipient = self.agent(target)
                if target == actor or recipient['kind'] == 'manager':
                    raise ValueError('Invalid business recipient')
            if cfg['topology'] == 'hierarchical' and not (
                sender['kind'] == 'coordinator' and all(self.agent(t)['kind'] == 'worker' for t in to)
                or sender['kind'] == 'worker' and to == [cfg['coordinator']]
            ):
                raise ValueError('Tree topology requires Coordinator <-> Worker')
            if cfg['topology']=='adaptive' and not (
                actor=='Controller' and all(self.agent(t)['kind']=='delegate' for t in to)
                or sender['kind']=='delegate' and to==['Controller']):
                raise ValueError('Adaptive messages use Controller <-> its delegates')
            if cfg['topology']=='adaptive' and (
                sender.get('delegation_state')=='finished' or
                any(self.agent(t).get('delegation_state')=='finished' for t in to)):
                raise ValueError('Delegate scope already returned; read its result or create a new scope')
            if reply_to:
                ref = self.db.execute('SELECT * FROM messages WHERE id=?', (reply_to,)).fetchone()
                if not ref or actor not in [ref['sender'], *json.loads(ref['recipients'])]:
                    raise ValueError('Reply must reference an accessible message')
            row = self.db.execute('SELECT * FROM messages WHERE send_key=?', (send_key,)).fetchone()
            if row:
                if (row['sender'], json.loads(row['recipients']), row['text'], row['reply_to']) != (actor, to, text, reply_to):
                    raise ValueError('Send key reused with different content')
                return self._message(row)
            cur = self.db.execute('INSERT INTO messages(sender,recipients,text,reply_to,send_key,created) VALUES (?,?,?,?,?,?)',
                                  (actor, packed(to), text, reply_to, send_key, time.time()))
            seq = cur.lastrowid
            self.db.execute('UPDATE messages SET id=? WHERE seq=?', ('m'+str(seq), seq))
            self.db.executemany('INSERT INTO receipts(seq,actor) VALUES (?,?)', [(seq, t) for t in to])
            result = self._message(self.db.execute('SELECT * FROM messages WHERE seq=?', (seq,)).fetchone())
            self.event('MESSAGE_PERSISTED', actor, result)
            if self.get('lifecycle') == 'ended':
                self.event('LATE_MESSAGE_AFTER_FINISH', actor, {'message_id':result['message_id'],
                           'note':'Audit only; does not reopen the run or authorize motion'})
            return result

    @staticmethod
    def _message(row):
        return {'message_id': row['id'], 'seq': row['seq'], 'from': row['sender'],
                'to': json.loads(row['recipients']), 'text': row['text'],
                'reply_to': row['reply_to'], 'created_at': row['created']}

    def history(self, actor):
        with self.lock:
            return [self._message(r) for r in self.db.execute('SELECT * FROM messages ORDER BY seq')
                    if actor == r['sender'] or actor in json.loads(r['recipients'])]

    def pending(self, actor):
        with self.lock:
            messages = self.db.execute('SELECT COUNT(*),COALESCE(MAX(seq),0) FROM receipts WHERE actor=? AND acked=0', (actor,)).fetchone()
            signals = self.db.execute('SELECT COUNT(*),COALESCE(MAX(seq),0) FROM signals WHERE actor=? AND acked=0', (actor,)).fetchone()
            return {'messages': messages[0], 'signals': signals[0], 'message_seq': messages[1], 'signal_seq': signals[1]}

    def delivery(self, actor):
        """Reserve a bounded batch; only a later explicit receipt acknowledges it."""
        cfg = self.get('config')
        with self.transaction():
            old = self.db.execute('SELECT id,value FROM batches WHERE actor=? AND acked=0 ORDER BY rowid LIMIT 1', (actor,)).fetchone()
            if old:
                result = json.loads(old['value'])
                result['redelivered'] = True
                return result
            rows = self.db.execute('SELECT m.* FROM messages m JOIN receipts r USING(seq) WHERE r.actor=? AND r.acked=0 ORDER BY seq', (actor,)).fetchall()
            signals = self.db.execute('SELECT seq,value FROM signals WHERE actor=? AND acked=0 ORDER BY seq', (actor,)).fetchall()
            messages=[];events=[];blocked=set()
            delivery_id=uuid.uuid4().hex
            from itertools import zip_longest
            # Alternate queues, preserving FIFO within each: neither repeated
            # progress events nor ordinary mail can starve the other queue.
            for signal_row, message_row in zip_longest(signals,rows):
                for kind,row in (('event',signal_row),('message',message_row)):
                    if row is None or kind in blocked:continue
                    if len(messages)+len(events)>=cfg['max_batch_messages']:
                        blocked.update(('event','message'));break
                    item={'seq':row['seq'],**json.loads(row['value'])} if kind=='event' else self._message(row)
                    ms=messages+([item] if kind=='message' else [])
                    es=events+([item] if kind=='event' else [])
                    probe=self._delivery_value(ms,es,len(rows)-len(ms),len(signals)-len(es),delivery_id)
                    if len(packed(probe).encode())>cfg['max_batch_bytes']:
                        blocked.add(kind);continue
                    messages,events=ms,es
                if len(blocked)==2:break
            result=self._delivery_value(messages,events,len(rows)-len(messages),len(signals)-len(events),
                                        delivery_id if messages or events else None)
            if (rows or signals) and not (messages or events):
                raise ValueError('Queued notice cannot fit delivery budget; engineering review required')
            if len(packed(result).encode())>cfg['max_batch_bytes']:
                raise ValueError('Delivery budget cannot fit envelope metadata')
            if messages or events:
                self.db.execute('INSERT INTO batches(id,actor,value) VALUES (?,?,?)', (result['delivery_id'], actor, packed(result)))
                self.event('DELIVERY_PREPARED', actor, result)
            return result

    def acknowledge(self, actor, delivery_id):
        with self.transaction():
            row = self.db.execute('SELECT * FROM batches WHERE id=? AND actor=?', (delivery_id, actor)).fetchone()
            if row is None:
                raise ValueError('Unknown delivery receipt for this role')
            if row['acked']:
                return
            batch = json.loads(row['value'])
            for m in batch['messages']:
                self.db.execute('UPDATE receipts SET acked=1 WHERE seq=? AND actor=?', (m['seq'], actor))
            for signal in batch['system_events']:
                self.db.execute('UPDATE signals SET acked=1 WHERE seq=? AND actor=?', (signal['seq'], actor))
            self.db.execute('UPDATE batches SET acked=1 WHERE id=?', (delivery_id,))
            self.event('CLIENT_RECEIPT', actor, {'delivery_id': delivery_id, 'business_acceptance': False})

    def begin_call(self, call_id, actor, op, parameters):
        with self.transaction():
            self.agent(actor)
            row = self.db.execute('SELECT * FROM calls WHERE id=?', (call_id,)).fetchone()
            if row:
                if (row['actor'], row['op'], row['parameters']) != (actor, op, packed(parameters)):
                    raise CallIdConflict(call_id)
                return dict(row)
            self.db.execute('INSERT INTO calls VALUES (?,?,?,?,?,?,?)',
                            (call_id, actor, op, packed(parameters), 'accepted', None, time.time()))
            return {'id': call_id, 'actor': actor, 'op': op, 'status': 'new', 'result': None}

    def finish_call(self, call_id, result):
        with self.transaction():
            changed = self.db.execute('UPDATE calls SET status=?, result=?, updated=? WHERE id=?',
                                     (result['execution_status'] if result.get('execution_status') in ('uncertain','running','pending') else 'failed' if result.get('error') else 'completed', packed(result), time.time(), call_id)).rowcount
            if not changed:
                raise ValueError('Call was not accepted')

    def call(self, call_id, actor=None):
        with self.lock:
            row = self.db.execute('SELECT * FROM calls WHERE id=?', (call_id,)).fetchone()
            if not row or actor is not None and row['actor'] != actor:
                raise ValueError('Unknown call for role')
            result = dict(row)
            result['result'] = json.loads(row['result']) if row['result'] else None
            return result

    def action_robot(self, actor, requested=None):
        kind = self.agent(actor)['kind']
        if self.get('config')['topology']=='adaptive':
            from .delegation import Delegations
            if kind not in ('controller','delegate') or requested not in self.get('robots',[]):
                raise ValueError('Select a robot controlled by this role')
            return Delegations(self).authorize(actor,requested)
        if kind == 'worker' and (requested is None or requested == actor):
            return actor
        if kind == 'controller' and requested in self.get('robots', []):
            return requested
        raise ValueError('Select a robot controlled by this role')

    def start_action(self, call_id, actor, op, parameters, sim_time, timing=None, batch_id=None, pending=False, sync_group_id=None):
        with self.transaction():
            robot = self.action_robot(actor, parameters.get('robot'))
            if self.get('lifecycle') != 'running':
                raise ValueError('Actions require a running trial')
            if self.get('physics_state') == 'paused_by_user' or self.get('dispatch_paused',False):
                raise ValueError('Run paused by experiment manager; stop new actions and save waiting state')
            if self.get('physics_state') != 'running':
                raise ValueError('Physics recovery required')
            if any(v['arm']==robot and v['status'] in ('pending','running','uncertain') and v['action_id']!=call_id for v in self.actions().values()):
                raise ValueError('Arm busy or action result uncertain; query existing action')
            old_action=self.actions().get(call_id)
            if old_action and (old_action['status']!='pending' or pending):raise ValueError('Action already admitted; never replay')
            if old_action and old_action['parameters']!=parameters:raise ValueError('Reserved command cannot be replaced')
            value = {'action_id':call_id,'arm':robot,'type':op,'status':'pending' if pending else 'running','parameters':parameters}
            value.update({'submitted_sim_s':sim_time} if pending else {'started_sim_s':sim_time,'started_at':time.time()})
            if old_action and 'submitted_sim_s' in old_action:value['submitted_sim_s']=old_action['submitted_sim_s']
            if timing is not None:value['execution_timing']=timing
            if batch_id is not None:value['batch_id']=batch_id
            if sync_group_id is not None:value['sync_group_id']=sync_group_id
            self.db.execute('INSERT OR REPLACE INTO actions VALUES (?,?,?,?)', (call_id,actor,value['status'],packed(value)))
            self.db.execute('UPDATE calls SET status=? WHERE id=?',(value['status'],call_id))
            old = self.agent(actor)
            self.update_agent(actor, work_state='working', note=old['note'] or 'Operation in progress; save work before ending turn')
            self.event('ACTION_RESERVED' if pending else 'ACTION_STARTED', actor, value)
            return value

    def uncertain_action(self, call_id, reason):
        with self.transaction():
            row=self.db.execute('SELECT * FROM actions WHERE id=?',(call_id,)).fetchone()
            if not row:raise ValueError('Unknown action')
            value=json.loads(row['value']);value.update(status='uncertain',reason=reason)
            self.db.execute("UPDATE actions SET status='uncertain',value=? WHERE id=?",(packed(value),call_id))
            self.db.execute("UPDATE calls SET status='uncertain' WHERE id=?",(call_id,))
            self.update_agent(row['actor'],recovery_required=True)
            self.put('physics_state','recovery_required');self.put('lifecycle','recovery_required')
            self.event('EXECUTION_OUTCOME_UNCERTAIN','Meta',value)
            return value

    def finish_action(self, call_id, status, sim_time, reason=None):
        if status not in ('completed', 'failed'):
            raise ValueError('Invalid terminal action status')
        with self.transaction():
            row = self.db.execute('SELECT value FROM actions WHERE id=?', (call_id,)).fetchone()
            if not row:
                raise ValueError('Unknown action')
            value = json.loads(row[0])
            value.update(status=status, finished_sim_s=sim_time, finished_at=time.time(), reason=reason)
            self.db.execute('UPDATE actions SET status=?,value=? WHERE id=?', (status, packed(value), call_id))
            self.event('ACTION_FINISHED', value['arm'], value)
            return value

    def actions(self, actor=None):
        with self.lock:
            rows = self.db.execute('SELECT * FROM actions' + (' WHERE actor=?' if actor else ''), (actor,) if actor else ()).fetchall()
            return {r['id']: json.loads(r['value']) for r in rows}

    def latest_execution(self, actor):
        """Executor facts only; no goal, grasp or clearance inference."""
        with self.lock:
            self.agent(actor)
            row = self.db.execute("SELECT value FROM actions WHERE actor=? ORDER BY status IN ('pending','running','uncertain') DESC,rowid DESC LIMIT 1", (actor,)).fetchone()
            return json.loads(row['value']) if row else {'status':'none'}

    def recover_execution(self):
        """Only the execution owner calls this after acquiring the world lock."""
        with self.transaction():
            from .action_batch import cancel_collecting
            cancel_collecting(self,'execution_owner_lost_before_group_start',0.)
            rows = {r['id']: dict(r) for r in self.db.execute("SELECT * FROM actions WHERE status='running'")}
            # A group can lose its owner before reservation or during startup.
            # Preserve every proposed member as unknown; never silently replay.
            for batch in self.db.execute("SELECT * FROM calls WHERE status IN ('accepted','running') AND op='action_batch'").fetchall():
                from .action_batch import child_ids
                parameters=json.loads(batch['parameters']);members=parameters.get('actions',[])
                if isinstance(members,list):
                    for ident,member in zip(child_ids(batch['id'],len(members)),members):
                        if not isinstance(member,dict) or member.get('op') not in ('move','gripper'):continue
                        if not isinstance(member.get('parameters',{}),dict):continue
                        p={**member.get('parameters',{}),'robot':member.get('robot'),'wait':True,'cameras':[]}
                        self.begin_call(ident,batch['actor'],member['op'],p)
                self.db.execute("UPDATE calls SET status='uncertain' WHERE id=?",(batch['id'],))
            # An accepted physical call whose owner died is conservatively
            # uncertain even if no durable ACTION_STARTED was seen.
            for call in self.db.execute("SELECT * FROM calls WHERE status='accepted' AND op IN ('move','gripper')"):
                if call['id'] not in rows:
                    parameters=json.loads(call['parameters'])
                    value = {'action_id':call['id'],'arm':parameters.get('robot',call['actor']),'type':call['op'],
                             'parameters':parameters,'status':'uncertain'}
                    self.db.execute('INSERT OR IGNORE INTO actions VALUES (?,?,?,?)',
                                    (call['id'],call['actor'],'uncertain',packed(value)))
                    rows[call['id']] = {'id':call['id'],'actor':call['actor'],'value':packed(value)}
            for row in rows.values():
                value = json.loads(row['value'])
                value.update(status='uncertain', reason='Execution owner ended before a terminal result was committed')
                self.db.execute("UPDATE actions SET status='uncertain',value=? WHERE id=?", (packed(value), row['id']))
                self.db.execute("UPDATE calls SET status='uncertain' WHERE id=?", (row['id'],))
                self.update_agent(row['actor'], recovery_required=True)
            if rows:
                self.put('physics_state','recovery_required');self.put('lifecycle','recovery_required')
                self.event('EXECUTION_RECOVERY_REQUIRED','Meta',{'actions':list(rows)})
            return len(rows)

    def observe_session(self, actor, status, turn_id, observed_at, source, turn_status=None):
        if source not in ('codex.wait_threads', 'codex.app-server') or status not in ('idle', 'active', 'notLoaded', 'error', 'needsAttention'):
            raise ValueError('Use an actual Codex status observation')
        with self.transaction():
            self.update_agent(actor, session_status=status, session_turn=turn_id,
                              session_turn_status=turn_status, session_observed_at=observed_at)
            if self.get('config')['topology']=='adaptive':
                from .delegation import Delegations
                Delegations(self).observe_stopped(actor,status,turn_id,observed_at)
            # A changed turn proves activity resumed, not that a particular business request was accepted.
            for row in self.db.execute("SELECT * FROM wakes WHERE actor=? AND status IN ('sent','uncertain')", (actor,)).fetchall():
                v = json.loads(row['value'])
                if turn_id and turn_id != v['expected_turn']:
                    self.db.execute("UPDATE wakes SET status='observed' WHERE id=?", (row['id'],))
                    self.event('CONTINUATION_OBSERVED', 'Meta', {'wake_id': row['id'], 'actor': actor, 'turn_id': turn_id})

    def lease(self, owner, now, duration):
        with self.transaction():
            row = self.db.execute("SELECT * FROM leases WHERE name='scheduler'").fetchone()
            if row and row['owner'] != owner and row['expires'] > now:
                raise ValueError('Another Meta dispatcher owns the scheduler lease')
            self.db.execute('INSERT OR REPLACE INTO leases VALUES (?,?,?)', ('scheduler', owner, now + duration))

    def wake_rows(self):
        with self.lock:
            return [dict(r, value=json.loads(r['value'])) for r in self.db.execute('SELECT * FROM wakes ORDER BY rowid')]

    def record_dispatch(self, wake_id, outcome, result=None):
        if outcome not in ('sent', 'uncertain', 'cancelled'):
            raise ValueError('Invalid dispatch outcome')
        with self.transaction():
            row = self.db.execute('SELECT * FROM wakes WHERE id=?', (wake_id,)).fetchone()
            if not row or row['status'] != 'dispatching':
                raise ValueError('Wake was not claimed')
            value = json.loads(row['value'])
            value.update(dispatch_result=result, outcome=outcome)
            self.db.execute('UPDATE wakes SET status=?,value=? WHERE id=?', (outcome, packed(value), wake_id))
            self.event('WAKE_' + outcome.upper(), 'Meta', value)

    def events(self, limit=None):
        with self.lock:
            rows = self.db.execute('SELECT * FROM events ORDER BY seq' + (' DESC LIMIT ?' if limit else ''), (limit,) if limit else ()).fetchall()
            rows = list(reversed(rows)) if limit else rows
            return [{'seq': r['seq'], 'created_at': r['created'], 'kind': r['kind'], 'actor': r['actor'], 'details':json.loads(r['value'])} for r in rows]

    def export_events(self, path):
        path = Path(path)
        temp = path.with_suffix('.tmp')
        temp.write_text(''.join(packed(e) + '\n' for e in self.events()))
        temp.replace(path)
