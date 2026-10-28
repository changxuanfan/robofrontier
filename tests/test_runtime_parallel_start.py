import sys
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from robofrontier.runtime_cli import execute


class FakeStore:
    def __init__(self, directory, run_id):
        self.path = directory / 'runtime.sqlite'
        self.run_id = run_id
    def get(self, key):
        return {'manager': 'Meta'} if key == 'config' else self.run_id


class ParallelStartTests(unittest.TestCase):
    def test_concurrent_start_keeps_process_stderr_and_separate_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            stores = []
            for name in ('one', 'two'):
                path = Path(directory)/name; path.mkdir()
                stores.append(FakeStore(path, name))
            barrier = threading.Barrier(2)
            seen = []
            original = sys.stderr
            def client(connection, op, data, call_id, *, log_call_id):
                barrier.wait(timeout=5)
                seen.append((sys.stderr is original, connection, call_id, log_call_id))
                return {'started': True}
            results = []
            def run(store): results.append(execute(store, 'request-start', {}, 'owner', 0))
            with patch('robofrontier.pipeline_client.client_call', client):
                threads = [threading.Thread(target=run, args=(store,)) for store in stores]
                for thread in threads: thread.start()
                for thread in threads: thread.join(10)
            self.assertTrue(all(not thread.is_alive() for thread in threads))
            self.assertEqual(results, [{'started':True}, {'started':True}])
            self.assertEqual(len(seen), 2)
            self.assertTrue(all(item[0] and not item[3] for item in seen))
            self.assertIs(sys.stderr, original)
            for store in stores:
                log = (store.path.parent/'admin-client.log').read_text().strip()
                self.assertTrue(log.startswith('call_id=start-'))
                self.assertEqual(len(log.splitlines()), 1)


if __name__ == '__main__': unittest.main()
