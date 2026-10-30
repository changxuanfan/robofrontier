from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import tempfile
import sqlite3
import threading
import unittest
from unittest.mock import patch

from robofrontier.claude.budget import BudgetLedger


class SharedBudgetTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.path=Path(self.tmp.name)/'budget.sqlite'
        self.ledger=BudgetLedger.create(self.path, 1)
    def tearDown(self): self.ledger.close(); self.tmp.cleanup()

    def test_busy_budget_reservation_waits_and_is_recorded_once(self):
        from robofrontier import sqlite_busy
        self.assertEqual(self.ledger.db.execute('PRAGMA busy_timeout').fetchone()[0],30000)
        self.ledger.db.execute('PRAGMA busy_timeout=20')
        other=sqlite3.connect(self.path,isolation_level=None,check_same_thread=False)
        other.execute('BEGIN IMMEDIATE')
        timer=threading.Timer(.11,other.rollback);timer.start()
        try:
            with patch.object(sqlite_busy,'MAX_BEGIN_ATTEMPTS',12), \
                 patch.object(sqlite_busy,'RETRY_DELAY_SECONDS',.01), \
                 self.assertLogs('robofrontier.sqlite_busy',level='WARNING'):
                self.assertEqual(self.ledger.reserve('turn','Controller',.3),.3)
            rows=self.ledger.snapshot()['reservations']
            self.assertEqual(len(rows),1)
            self.assertEqual(rows[0]['id'],'turn')
        finally:
            timer.join();other.close()

    def test_concurrent_process_connections_cannot_overreserve(self):
        gate=threading.Barrier(10)
        def reserve(i):
            ledger=BudgetLedger(self.path)
            try:
                gate.wait()
                try: return ledger.reserve('turn-'+str(i),'actor',.3)
                except ValueError: return 0
            finally: ledger.close()
        with ThreadPoolExecutor(max_workers=10) as pool: amounts=list(pool.map(reserve,range(10)))
        self.assertAlmostEqual(sum(amounts),1.0)
        self.assertEqual(self.ledger.snapshot()['reserved_usd'],'1.0')
        self.assertEqual(self.ledger.snapshot()['status'],'exhausted')

    def test_settled_cost_releases_unused_allowance(self):
        self.assertEqual(self.ledger.reserve('parent','Controller',.6),.6)
        self.assertEqual(self.ledger.reserve('child','Delegate',.6),.4)
        result=self.ledger.settle('parent',.2,outcome_known=True)
        self.assertEqual(result['reported_spend_usd'],'0.2')
        self.assertEqual(result['available_usd'],'0.4')
        self.assertEqual(self.ledger.reserve('next','Controller',.6),.4)
        with self.assertRaises(ValueError): self.ledger.settle('parent',.2,outcome_known=True)

    def test_unknown_missing_or_above_reservation_blocks_future(self):
        for cost,known in ((None,True),(0,True),(.7,True),(.1,False)):
            with self.subTest(cost=cost,known=known), tempfile.TemporaryDirectory() as tmp:
                ledger=BudgetLedger.create(Path(tmp)/'budget.sqlite',1)
                try:
                    ledger.reserve('parent','Controller',.6)
                    self.assertEqual(ledger.settle('parent',cost,outcome_known=known)['status'],'uncertain')
                    with self.assertRaisesRegex(ValueError,'uncertain'): ledger.reserve('child','Delegate',.1)
                finally: ledger.close()

    def test_no_duplicate_or_implicit_release_after_reopen(self):
        self.ledger.reserve('turn','Controller',.2)
        other=BudgetLedger(self.path)
        try:
            self.assertEqual(other.snapshot()['reserved_usd'],'0.2')
            with self.assertRaises(ValueError): other.reserve('turn','Controller',.1)
            other.abandon_before_launch('turn')
            self.assertEqual(other.snapshot()['available_usd'],'1')
            with self.assertRaises(ValueError): other.reserve('turn','Controller',.1)
        finally: other.close()


if __name__=='__main__': unittest.main()
