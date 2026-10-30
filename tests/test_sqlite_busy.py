import logging
import sqlite3
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from robofrontier import sqlite_busy
from robofrontier.runtime_store import RuntimeStore


class SQLiteBusyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / 'runtime.sqlite'
        self.store = RuntimeStore(self.path)
        self.other = sqlite3.connect(self.path, isolation_level=None, check_same_thread=False)

    def tearDown(self):
        if self.other.in_transaction:
            self.other.rollback()
        self.other.close()
        self.store.close()
        self.tmp.cleanup()

    def test_default_wait_is_30_seconds_and_nested_savepoints_work(self):
        self.assertEqual(self.store.db.execute('PRAGMA busy_timeout').fetchone()[0], 30000)
        with self.store.transaction():
            self.store.put('one', 1)
        self.assertEqual(self.store.get('one'), 1)

    def test_busy_retry_succeeds_after_release_without_replaying_body(self):
        self.store.db.execute('PRAGMA busy_timeout=20')
        self.other.execute('BEGIN IMMEDIATE')
        timer = threading.Timer(.11, self.other.rollback)
        timer.start()
        calls = []
        try:
            with patch.object(sqlite_busy, 'RETRY_DELAY_SECONDS', .01), \
                 patch.object(sqlite_busy, 'MAX_BEGIN_ATTEMPTS', 12), \
                 self.assertLogs('robofrontier.sqlite_busy', level='WARNING') as logs:
                with self.store.transaction():
                    calls.append('body')
                    self.store.db.execute('INSERT INTO meta VALUES (?,?)', ('test', '1'))
        finally:
            timer.join()
        self.assertEqual(calls, ['body'])
        self.assertEqual(self.store.get('test'), 1)
        self.assertTrue(any('writer busy' in line and 'elapsed_s=' in line for line in logs.output))
        self.assertTrue(any('writer acquired' in line and 'elapsed_s=' in line for line in logs.output))

    def test_busy_exhaustion_fails_before_transaction_body(self):
        self.store.db.execute('PRAGMA busy_timeout=10')
        self.other.execute('BEGIN IMMEDIATE')
        calls = []
        with patch.object(sqlite_busy, 'RETRY_DELAY_SECONDS', .001), \
             self.assertLogs('robofrontier.sqlite_busy', level='WARNING') as logs:
            with self.assertRaises(sqlite3.OperationalError) as caught:
                with self.store.transaction(): calls.append('body')
        self.assertEqual(caught.exception.sqlite_errorcode, sqlite3.SQLITE_BUSY)
        self.assertEqual(len(logs.output), 4)
        self.assertEqual(calls, [])
        self.assertFalse(self.store.db.in_transaction)

    def test_transaction_body_error_does_not_replay(self):
        calls = []
        with self.assertRaisesRegex(RuntimeError, 'body failed'):
            with self.store.transaction():
                calls.append('body')
                self.store.db.execute('INSERT INTO meta VALUES (?,?)', ('test', '1'))
                raise RuntimeError('body failed')
        self.assertEqual(calls, ['body'])
        self.assertIsNone(self.store.get('test'))

    def test_non_busy_or_extended_errors_are_not_retried(self):
        class Fake:
            in_transaction = False
            def __init__(self, code): self.code, self.calls = code, 0
            def execute(self, sql):
                self.calls += 1
                error = sqlite3.OperationalError('synthetic')
                error.sqlite_errorcode = self.code
                raise error
        for code in (sqlite3.SQLITE_LOCKED, sqlite3.SQLITE_IOERR, sqlite3.SQLITE_BUSY_SNAPSHOT):
            with self.subTest(code=code):
                db = Fake(code)
                with self.assertRaises(sqlite3.OperationalError): sqlite_busy.begin_immediate(db, 'test')
                self.assertEqual(db.calls, 1)


class SQLiteBusyServiceTests(unittest.TestCase):
    def test_client_call_completes_after_actual_writer_contention(self):
        import json
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        from fake_pipeline import FakeWorld, fake_capture
        from robofrontier.pipeline_client import client_call
        from robofrontier.run_setup import prepare_run, write_json
        from robofrontier.runtime_service import RuntimeService

        with tempfile.TemporaryDirectory() as tmp:
            run=prepare_run(Path(tmp)/'run')
            service=RuntimeService(run, FakeWorld, fake_capture)
            class Handler(BaseHTTPRequestHandler):
                def log_message(self,*args): pass
                def do_POST(self):
                    body=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                    job=service.accept(body['actor'],body['op'],body['data'],body['call_id'],body.get('ack_delivery'))
                    service.pump()
                    raw=json.dumps(job['result']).encode()
                    self.send_response(200);self.end_headers();self.wfile.write(raw)
            server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            conn=run/'private/state/connections/Meta.json';config=json.loads(conn.read_text())
            config['url']=f'http://127.0.0.1:{server.server_port}';write_json(conn,config)
            other=sqlite3.connect(run/'private/state/runtime.sqlite',isolation_level=None,check_same_thread=False)
            service.store.db.execute('PRAGMA busy_timeout=20');other.execute('BEGIN IMMEDIATE')
            timer=threading.Timer(.11,other.rollback);timer.start()
            try:
                with patch.object(sqlite_busy,'MAX_BEGIN_ATTEMPTS',12), \
                     patch.object(sqlite_busy,'RETRY_DELAY_SECONDS',.01), \
                     self.assertLogs('robofrontier.sqlite_busy',level='WARNING'):
                    result=client_call(conn,'observe',{'cameras':['global']},'one-call',log_call_id=False)
                self.assertEqual(result['call_id'],'one-call')
                self.assertIn('observation',result)
                self.assertEqual(service.store.call('one-call')['status'],'completed')
                self.assertEqual(len([e for e in service.store.events() if e['kind']=='TOOL_RETURN'
                                      and e['details'].get('call_id')=='one-call']),1)
            finally:
                timer.join();other.close();server.shutdown();server.server_close();service.close()


if __name__ == '__main__': unittest.main()
