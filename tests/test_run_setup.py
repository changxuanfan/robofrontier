import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from robofrontier.run_setup import prepare_run,verify_inputs
from robofrontier.task_config import load_task,load_harness
from robofrontier.runtime_store import RuntimeStore

class SetupTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
    def tearDown(self):self.tmp.cleanup()
    def test_prepare_is_offline_and_copies_task_roles_skill_and_source(self):
        run=prepare_run(self.root/'run')
        self.assertEqual(verify_inputs(run),[])
        self.assertEqual(json.loads((run/'public/inputs/task.json').read_text())['goals'],load_task()['goals'])
        store=RuntimeStore(run/'private/state/runtime.sqlite')
        try:
            self.assertEqual(store.get('lifecycle'),'prepared')
            self.assertEqual(store.agent('Meta')['kind'],'manager')
            self.assertEqual(store.agent('Coordinator')['kind'],'coordinator')
            self.assertTrue(all(v['thread_id'] is None for v in store.agents().values()))
            self.assertEqual(store.actions(),{})
        finally:store.close()
        self.assertIsNone(json.loads((run/'private/state/connections/A.json').read_text())['url'])
        self.assertTrue((run/'public/inputs/skill/references/tools.md').exists())
        self.assertEqual((run/'public/inputs/role_tools.js').read_bytes(),(Path(__file__).resolve().parents[1]/'scripts/role_tools.js').read_bytes())
        self.assertTrue((run/'private/inputs/frozen-source/robofrontier/scheduling.py').exists())
        self.assertFalse((run/'private/outputs/global-realtime.mp4').exists())
        before=(run/'private/inputs/manifest.json').read_bytes()
        with self.assertRaises(ValueError):prepare_run(run)
        self.assertEqual((run/'private/inputs/manifest.json').read_bytes(),before)
    def test_snapshot_tamper_is_reported(self):
        run=prepare_run(self.root/'run')
        p=run/'public/inputs/task.json';p.write_text('{}')
        self.assertIn('input:public/inputs/task.json',verify_inputs(run))
    def test_private_scene_or_harness_tamper_is_also_detected(self):
        run=prepare_run(self.root/'run')
        p=run/'private/inputs/task.json';v=json.loads(p.read_text());v['cube_mass_kg']=.9;p.write_text(json.dumps(v))
        self.assertIn('private:task.json',verify_inputs(run))
    def test_public_positions_can_be_withheld_without_changing_private_scene_input(self):
        task=load_task();task['observation_contract']['initial_positions_public']=False
        p=self.root/'task.json';p.write_text(json.dumps(task))
        run=prepare_run(self.root/'run',p)
        public=json.loads((run/'public/inputs/task.json').read_text());private=json.loads((run/'private/inputs/task.json').read_text())
        self.assertNotIn('initial_position_m',public['objects'][0])
        self.assertIn('initial_position_m',private['objects'][0])
    def test_invalid_task_rejected_before_directory_creation(self):
        task=load_task();task['objects'][0]['initial_position_m']=[float('nan'),0,0]
        p=self.root/'bad.json';p.write_text(json.dumps(task))
        with self.assertRaises(ValueError):prepare_run(self.root/'run',p)
        self.assertFalse((self.root/'run').exists())

    def test_frozen_source_is_checked_and_observations_are_not_fixed_inputs(self):
        run=prepare_run(self.root/'run')
        (run/'public/observations/new.png').write_bytes(b'not real pixels')
        self.assertEqual(verify_inputs(run),[])
        (run/'private/inputs/frozen-source/robofrontier/scheduling.py').write_text('# changed')
        self.assertIn('frozen:robofrontier/scheduling.py',verify_inputs(run))

    def test_run_local_experience_is_mutable_and_not_imported_into_next_run(self):
        run=prepare_run(self.root/'first')
        experience=run/'public/results/A-experience.md'
        experience.write_text('Observed trial result; applicable only to the tested conditions.\n')
        store=RuntimeStore(run/'private/state/runtime.sqlite')
        try:store.checkpoint_work('A','working','Experience reference: '+str(experience))
        finally:store.close()
        experience.write_text('Updated after contrary observation; previous inference uncertain.\n')
        self.assertEqual(verify_inputs(run),[])
        fresh=prepare_run(self.root/'second')
        self.assertEqual(list((fresh/'public/results').iterdir()),[])
        store=RuntimeStore(fresh/'private/state/runtime.sqlite')
        try:self.assertEqual(store.agent('A')['note'],'')
        finally:store.close()
        self.assertEqual(verify_inputs(fresh),[])

    def test_legacy_layout_is_not_migrated_or_modified(self):
        from robofrontier.runtime_service import RuntimeService
        from fake_pipeline import FakeWorld,fake_capture
        run=self.root/'legacy';(run/'private').mkdir(parents=True)
        old=run/'private/manifest.json';old.write_text('{"schema_version":2}')
        before=old.read_bytes()
        with self.assertRaises(ValueError):RuntimeService(run,FakeWorld,fake_capture)
        self.assertEqual(old.read_bytes(),before)
        self.assertFalse((run/'private/state').exists())

    def test_six_workers_prepare_with_same_skill_in_both_topologies(self):
        task=load_task();sample=task['objects'][0]
        task['arms']={name:(i-2.5)*.65 for i,name in enumerate('ABCDEF')}
        task['objects']=[{**sample,'id':name+'_gray','region':name,
                          'initial_position_m':[.42,y,.42]} for name,y in task['arms'].items()]
        task['goals']={name:[{'slot':'one','position_m':[.57,y,.42],
                             'requires':{'color':'gray'}}] for name,y in task['arms'].items()}
        p=self.root/'six.json';p.write_text(json.dumps(task))
        skill=(Path(__file__).resolve().parents[1]/'.agents/skills/robot-collaboration/SKILL.md').read_bytes()
        for topology in ('hierarchical','peer'):
            cfg=load_harness();cfg['topology']=topology
            c=self.root/(topology+'.json');c.write_text(json.dumps(cfg))
            run=prepare_run(self.root/topology,p,c)
            roles=json.loads((run/'public/inputs/roles.json').read_text())['roles']
            self.assertEqual(sum(v['kind']=='worker' for v in roles.values()),6)
            self.assertEqual('Coordinator' in roles,topology=='hierarchical')
            self.assertEqual((run/'public/inputs/skill/SKILL.md').read_bytes(),skill)
            self.assertEqual(verify_inputs(run),[])

if __name__=='__main__':unittest.main()
