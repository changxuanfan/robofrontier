"""The shared runtime accepts evidence only from the engine a run was prepared for."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from robofrontier import runtime_cli
from robofrontier.claude.provenance import fork_provenance, transcript_rows
from robofrontier.run_setup import prepare_run, verify_inputs
from robofrontier.runtime_store import RuntimeStore
from robofrontier.task_config import ROOT, load_harness
from scripts.export_run_report import model_evidence

PARENT = '11111111-1111-4111-8111-111111111111'
CHILD = '22222222-2222-4222-8222-222222222222'
OTHER = '33333333-3333-4333-8333-333333333333'


def harness_file(root, engine):
    config = json.loads((ROOT / 'configs/harnesses/policy.json').read_text())
    if engine:
        config['engine'] = engine
    path = Path(root) / (str(engine) + '-harness.json')
    path.write_text(json.dumps(config))
    return path


def message(uuid, session, text, **extra):
    return {'type': 'user', 'uuid': uuid, 'parentUuid': None, 'sessionId': session,
            'promptId': 'p-' + session[:4], 'message': {'role': 'user', 'content': text}, **extra}


def write_transcript(config_dir, session, rows, project='-run'):
    folder = Path(config_dir) / 'projects' / project
    folder.mkdir(parents=True, exist_ok=True)
    (folder / (session + '.jsonl')).write_text(''.join(json.dumps(r) + '\n' for r in rows))


class EngineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_unknown_engine_is_rejected_and_default_stays_codex(self):
        with self.assertRaises(ValueError):
            load_harness(harness_file(self.root, 'gpt'))
        self.assertNotIn('engine', load_harness())
        self.assertEqual(load_harness(harness_file(self.root, 'claude'))['engine'], 'claude')

    def test_session_status_source_must_match_the_run_engine(self):
        for engine, accepted, refused in (
                (None, 'codex.wait_threads', 'claude.host'),
                ('codex', 'codex.app-server', 'claude.host'),
                ('claude', 'claude.host', 'codex.wait_threads')):
            with self.subTest(engine=engine):
                store = RuntimeStore(self.root / (str(engine) + '.sqlite'))
                try:
                    config = load_harness(harness_file(self.root, engine))
                    store.initialize('fake', config, ['A'])
                    store.update_agent('Controller', thread_id='t')
                    store.observe_session('Controller', 'idle', 'turn', time.time(), accepted)
                    with self.assertRaises(ValueError):
                        store.observe_session('Controller', 'idle', 'turn', time.time(), refused)
                    with self.assertRaises(ValueError):
                        store.observe_session('Controller', 'running', 'turn', time.time(), accepted)
                finally:
                    store.close()

    def test_frozen_sources_and_manifest_follow_the_engine(self):
        codex = prepare_run(self.root / 'codex-run', harness_path=harness_file(self.root, None))
        claude = prepare_run(self.root / 'claude-run', harness_path=harness_file(self.root, 'claude'))
        frozen = lambda run: {p.name for p in (run / 'private/inputs/frozen-source/scripts').glob('*')}
        self.assertIn('codex_policy_host.js', frozen(codex))
        self.assertNotIn('claude_benchmark.py', frozen(codex))
        self.assertIn('claude_benchmark.py', frozen(claude))
        self.assertFalse({'codex_policy_host.js', 'codex_rpc.js', 'benchmark.py'} & frozen(claude))
        self.assertIn('meta_dispatcher.js', frozen(claude))
        self.assertTrue((claude / 'private/inputs/frozen-source/robofrontier/claude/provenance.py').is_file())
        self.assertEqual(json.loads((codex / 'private/inputs/manifest.json').read_text())['engine'], 'codex')
        self.assertEqual(json.loads((claude / 'private/inputs/manifest.json').read_text())['engine'], 'claude')
        self.assertEqual(verify_inputs(claude), [])

    def test_claude_report_never_uses_matching_codex_session_as_model_proof(self):
        home=self.root/'home'; sessions=home/'.codex/sessions'; sessions.mkdir(parents=True)
        (sessions/('log-'+PARENT+'.jsonl')).write_text(json.dumps({
            'type':'turn_context','payload':{'model':'test-model','effort':'high','turn_id':'codex'}})+'\n')
        run=self.root/'run'; evidence=run/'private/evidence'; evidence.mkdir(parents=True)
        (evidence/'controller-summary.json').write_text(json.dumps({
            'actor':'Controller','session_id':PARENT,'turn_id':'claude','status':'idle',
            'record':{'summary':{'reported_model':'test-model'}}}))
        config={'model':'test-model','effort':'high'}
        with patch('pathlib.Path.home',return_value=home):
            codex=model_evidence(run,'Controller',PARENT,config)
            claude=model_evidence(run,'Controller',PARENT,{**config,'engine':'claude'})
        self.assertTrue(codex['verified'])
        self.assertFalse(claude['verified'])
        self.assertEqual([t['turn_id'] for t in claude['turns']], ['claude'])
        self.assertEqual(claude['turns'][0]['reported_model'], 'test-model')


class ClaudeProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.config = Path(self.tmp.name) / 'claude-config'
        self.history = [message('m1', PARENT, 'first'), message('m2', PARENT, 'second')]

    def tearDown(self):
        self.tmp.cleanup()

    def fork(self, rows=None, extra=()):
        rows = rows if rows is not None else [{**r, 'sessionId': CHILD, 'promptId': 'other'} for r in self.history]
        write_transcript(self.config, PARENT, [{'type': 'mode', 'sessionId': PARENT}] + self.history)
        write_transcript(self.config, CHILD, [{'type': 'mode', 'sessionId': CHILD}] + rows + list(extra))

    def test_exact_prefix_copy_is_proven_and_boundary_is_stable(self):
        extra = [message('m3', CHILD, 'child only')]
        self.fork(extra=extra)
        proof = fork_provenance(PARENT, CHILD, self.config)
        self.assertEqual(proof['source_thread_id'], PARENT)
        self.assertEqual(proof['source_ordinal'], 2)
        self.assertEqual(proof['source_last_uuid'], 'm2')
        self.assertEqual(proof, fork_provenance(PARENT, CHILD, self.config))

    def test_changed_dropped_reordered_or_foreign_history_is_rejected(self):
        child = lambda rows: [{**r, 'sessionId': CHILD} for r in rows]
        altered = [dict(self.history[0], message={'role': 'user', 'content': 'edited'}), self.history[1]]
        for name, rows in (('altered', child(altered)), ('dropped', child(self.history[:1])),
                           ('reordered', child(self.history[::-1])),
                           ('foreign', child(self.history[:1]) + [message('m2', PARENT, 'second')])):
            with self.subTest(name):
                self.fork(rows)
                with self.assertRaises(ValueError):
                    fork_provenance(PARENT, CHILD, self.config)

    def test_parent_that_continued_after_the_fork_no_longer_matches(self):
        self.fork()
        write_transcript(self.config, PARENT, [{'type': 'mode', 'sessionId': PARENT}] + self.history
                         + [message('m9', PARENT, 'parent kept working')])
        with self.assertRaises(ValueError):
            fork_provenance(PARENT, CHILD, self.config)

    def test_identifiers_and_transcript_files_are_validated(self):
        self.fork()
        for parent, child in ((PARENT, PARENT), ('not-a-uuid', CHILD), (PARENT, OTHER)):
            with self.assertRaises(ValueError):
                fork_provenance(parent, child, self.config)
        write_transcript(self.config, CHILD, [{'type': 'mode', 'sessionId': CHILD}], project='-second')
        with self.assertRaises(ValueError):
            transcript_rows(self.config, CHILD)

    def test_runtime_dispatch_reads_the_runs_claude_config_and_keeps_codex_path(self):
        run = Path(self.tmp.name) / 'run'
        self.fork()
        (run / 'private').mkdir(parents=True)
        (run / 'private/claude-config').symlink_to(self.config)
        store = RuntimeStore(run / 'private/state/runtime.sqlite')
        try:
            store.initialize('fake', load_harness(harness_file(self.tmp.name, 'claude')), ['A'])
            self.assertEqual(runtime_cli.native_fork_provenance(store, PARENT, CHILD)['engine'], 'claude')
            store.put('config', load_harness())
            with patch('robofrontier.runtime_cli.fork_provenance', return_value={'codex': True}) as codex:
                self.assertEqual(runtime_cli.native_fork_provenance(store, PARENT, CHILD), {'codex': True})
            codex.assert_called_once_with(PARENT, CHILD)
        finally:
            store.close()


if __name__ == '__main__':
    unittest.main()
