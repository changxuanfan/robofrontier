import json
from pathlib import Path
import tempfile
import threading
import unittest
from robofrontier.runtime_store import RuntimeStore
from robofrontier.runtime_store import packed
from robofrontier.task_config import load_harness

class StoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)/'runtime.sqlite'
        self.s=RuntimeStore(self.path);self.s.initialize('fake-run',load_harness(),['A','B','C'])
    def tearDown(self):self.s.close();self.tmp.cleanup()
    def test_concurrent_send_and_retry_deduplicate_without_loss(self):
        errors=[]
        def send(i):
            try:self.s.send('Coordinator',['A'],str(i),'key-'+str(i))
            except Exception as e:errors.append(e)
        threads=[threading.Thread(target=send,args=(i,)) for i in range(40)]
        for t in threads:t.start()
        for t in threads:t.join()
        self.assertEqual(errors,[]);self.assertEqual(self.s.pending('A')['messages'],40)
        self.s.send('Coordinator',['A'],'0','key-0')
        self.assertEqual(self.s.pending('A')['messages'],40)
        with self.assertRaises(ValueError):self.s.send('Coordinator',['A'],'different','key-0')
    def test_bounded_batches_and_receipt_survive_reopen(self):
        for i in range(40):self.s.send('Coordinator',['A'],str(i),str(i))
        batch=self.s.delivery('A');self.assertEqual(len(batch['messages']),32)
        self.assertEqual(batch['remaining_messages'],8)
        self.s.close();self.s=RuntimeStore(self.path)
        self.assertEqual(self.s.delivery('A')['delivery_id'],batch['delivery_id'])
        self.s.acknowledge('A',batch['delivery_id'])
        self.assertEqual(len(self.s.delivery('A')['messages']),8)
        self.assertEqual(self.s.pending('B')['messages'],0)
    def test_transaction_failure_does_not_commit_message_or_batch(self):
        with self.assertRaises(RuntimeError):
            with self.s.transaction():
                self.s.send('Coordinator',['A'],'must roll back','x')
                self.s.delivery('A')
                raise RuntimeError('injected crash before commit')
        self.assertEqual(self.s.history('A'),[])
        self.assertEqual(self.s.pending('A')['messages'],0)
        self.assertIsNone(self.s.delivery('A')['delivery_id'])
    def test_peers_can_route_directly_without_a_hidden_coordinator(self):
        self.s.close();self.s=RuntimeStore(Path(self.tmp.name)/'peer.sqlite')
        config=load_harness();config['topology']='peer';self.s.initialize('peer',config,['A','B'])
        self.assertNotIn('Coordinator',self.s.agents())
        self.s.send('A',['B'],'peer work','peer-key')
        self.assertEqual(self.s.pending('B')['messages'],1)
    def test_accepted_physical_call_is_uncertain_after_owner_loss(self):
        self.s.begin_call('a','A','move',{'target_m':[.4,0,.6]})
        self.assertEqual(self.s.recover_execution(),1)
        self.assertEqual(self.s.call('a')['status'],'uncertain')
        self.assertEqual(self.s.actions()['a']['status'],'uncertain')
    def test_read_receipt_is_not_a_business_reply(self):
        self.s.send('Coordinator',['A'],'can you help?','x')
        batch=self.s.delivery('A');self.s.acknowledge('A',batch['delivery_id'])
        self.assertEqual(len(self.s.history('Coordinator')),1)
        self.assertFalse(self.s.events()[-1]['details']['business_acceptance'])
    def test_start_activates_every_policy_role_without_assigning_work(self):
        for mode in ('hierarchical','peer','centralized'):
            store=RuntimeStore(Path(self.tmp.name)/(mode+'-start.sqlite'))
            try:
                cfg=load_harness();cfg['topology']=mode;store.initialize(mode,cfg,['A','B','C'])
                policy={a:v for a,v in store.agents().items()if v['kind']!='manager'}
                for actor in policy:store.update_agent(actor,ready=True)
                store.start()
                for actor in policy:
                    self.assertEqual(store.agent(actor)['work_state'],'continue',(mode,actor))
                self.assertEqual(store.agent('Meta')['work_state'],'waiting')
                self.assertEqual(store.actions(),{})
                self.assertEqual(store.db.execute('SELECT count(*) FROM messages').fetchone()[0],0)
            finally:store.close()

    def test_messages_and_events_share_byte_budget_and_drain_without_loss(self):
        cfg=self.s.get('config');cfg['max_batch_bytes']=1600;self.s.put('config',cfg)
        for i in range(8):
            self.s.send('Coordinator',['A'],'m'+str(i)+'汉'*80,'m'+str(i))
            self.s.signal('A',{'event':'test_progress','index':i,'text':'文'*100})
        seen_messages=[];seen_events=[]
        while self.s.pending('A')['messages']+self.s.pending('A')['signals']:
            batch=self.s.delivery('A')
            self.assertLessEqual(len(packed(batch).encode()),1600)
            self.assertLessEqual(len(batch['messages'])+len(batch['system_events']),cfg['max_batch_messages'])
            self.assertEqual(self.s.delivery('A')['delivery_id'],batch['delivery_id'])
            seen_messages.extend(x['text'] for x in batch['messages'])
            seen_events.extend(x['index'] for x in batch['system_events'])
            self.assertIn('remaining_events',batch)
            self.s.acknowledge('A',batch['delivery_id'])
        self.assertEqual(seen_events,list(range(8)))
        self.assertEqual(seen_messages,['m'+str(i)+'汉'*80 for i in range(8)])

    def test_individual_oversized_event_is_rejected_before_enqueue(self):
        cfg=self.s.get('config');cfg['max_batch_bytes']=1000;self.s.put('config',cfg)
        with self.assertRaises(ValueError):self.s.signal('A',{'event':'test','text':'汉'*1000})
        self.assertEqual(self.s.pending('A')['signals'],0)

if __name__=='__main__':unittest.main()
