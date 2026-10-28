"""Durable shared reservations against CLI-reported estimates, not provider bills."""
from contextlib import contextmanager
from decimal import Decimal, InvalidOperation
import os
from pathlib import Path
import sqlite3
import threading
import time


def _money(value, *, positive=True):
    if isinstance(value, bool): raise ValueError('Invalid monetary amount')
    try: result = Decimal(str(value))
    except (InvalidOperation, ValueError) as error: raise ValueError('Invalid monetary amount') from error
    if not result.is_finite() or (result <= 0 if positive else result < 0):
        raise ValueError('Invalid monetary amount')
    return result


class BudgetLedger:
    """Serialize task-wide CLI cost reservations across sessions and processes.

    A reservation with an unknown outcome blocks further spending and is never
    released automatically. Account statements and provider-enforced limits
    remain authoritative; CLI totals are estimates that can be absent or wrong.
    """
    def __init__(self, path):
        self.path = Path(path)
        if not self.path.is_file(): raise ValueError('Budget ledger does not exist')
        self.lock = threading.RLock()
        self.db = sqlite3.connect(self.path, isolation_level=None, check_same_thread=False, timeout=10)
        self.db.row_factory = sqlite3.Row
        self.db.execute('PRAGMA journal_mode=WAL'); self.db.execute('PRAGMA synchronous=FULL')
        if self.db.execute("SELECT value FROM meta WHERE key='limit'").fetchone() is None:
            self.db.close(); raise ValueError('Budget ledger is not initialized')

    @classmethod
    def create(cls, path, total_usd):
        total = _money(total_usd)
        path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600); os.close(fd)
        db = sqlite3.connect(path)
        try:
            db.executescript('''CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);
                CREATE TABLE reservations (id TEXT PRIMARY KEY, actor TEXT NOT NULL, allowance TEXT NOT NULL,
                  cost TEXT, status TEXT NOT NULL, created REAL NOT NULL, updated REAL NOT NULL);''')
            db.execute('INSERT INTO meta VALUES (?,?)', ('limit', str(total))); db.commit()
        finally: db.close()
        return cls(path)

    @contextmanager
    def transaction(self):
        with self.lock:
            self.db.execute('BEGIN IMMEDIATE')
            try:
                yield
                self.db.execute('COMMIT')
            except BaseException:
                self.db.execute('ROLLBACK')
                raise

    def _snapshot(self):
        limit = Decimal(self.db.execute("SELECT value FROM meta WHERE key='limit'").fetchone()[0])
        rows = [dict(r) for r in self.db.execute('SELECT * FROM reservations ORDER BY created, id')]
        spent = sum((Decimal(r['cost']) for r in rows if r['status'] == 'settled'), Decimal('0'))
        reserved = sum((Decimal(r['allowance']) for r in rows if r['status'] == 'reserved'), Decimal('0'))
        unknown = any(r['status'] == 'uncertain' for r in rows)
        return {'limit_usd': str(limit), 'reported_spend_usd': str(spent), 'reserved_usd': str(reserved),
                'available_usd': str(max(Decimal('0'), limit-spent-reserved)),
                'status': 'uncertain' if unknown else 'exhausted' if limit-spent-reserved <= 0 else 'tracking',
                'reservations': rows, 'source': 'cli_reported_estimates_not_provider_bill'}

    def snapshot(self):
        with self.transaction(): return self._snapshot()

    def reserve(self, reservation_id, actor, maximum_usd):
        maximum = _money(maximum_usd)
        if not isinstance(reservation_id, str) or not reservation_id or not isinstance(actor, str) or not actor:
            raise ValueError('Reservation identity required')
        with self.transaction():
            state = self._snapshot()
            if state['status'] != 'tracking': raise ValueError('Budget is exhausted or uncertain')
            allowance = min(maximum, Decimal(state['available_usd']))
            now = time.time()
            try:
                self.db.execute('INSERT INTO reservations VALUES (?,?,?,?,?,?,?)',
                                (reservation_id, actor, str(allowance), None, 'reserved', now, now))
            except sqlite3.IntegrityError as error:
                raise ValueError('Reservation identity already used; never replay') from error
            return float(allowance)

    def settle(self, reservation_id, reported_usd, *, outcome_known):
        with self.transaction():
            row = self.db.execute('SELECT * FROM reservations WHERE id=?', (reservation_id,)).fetchone()
            if not row or row['status'] != 'reserved': raise ValueError('Reservation is not pending')
            try:
                cost = _money(reported_usd)
                known = outcome_known is True and cost <= Decimal(row['allowance'])
            except ValueError:
                cost = None; known = False
            self.db.execute('UPDATE reservations SET status=?, cost=?, updated=? WHERE id=?',
                            ('settled' if known else 'uncertain', str(cost) if cost is not None else None,
                             time.time(), reservation_id))
            return self._snapshot()

    def abandon_before_launch(self, reservation_id):
        """Release only when the caller knows no process/thread was launched."""
        with self.transaction():
            updated = self.db.execute("UPDATE reservations SET status='not_launched', updated=? WHERE id=? AND status='reserved'",
                                      (time.time(), reservation_id)).rowcount
            if updated != 1: raise ValueError('Reservation is not pending')

    def close(self): self.db.close()
