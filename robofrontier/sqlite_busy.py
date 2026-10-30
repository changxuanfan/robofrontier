"""Bounded retries only while acquiring a SQLite write transaction."""
import logging
import sqlite3
import time

BUSY_TIMEOUT_SECONDS = 30
MAX_BEGIN_ATTEMPTS = 4
RETRY_DELAY_SECONDS = 0.25
MAX_BEGIN_WAIT_SECONDS = BUSY_TIMEOUT_SECONDS * MAX_BEGIN_ATTEMPTS + RETRY_DELAY_SECONDS * (MAX_BEGIN_ATTEMPTS - 1)
log = logging.getLogger(__name__)


def begin_immediate(db, label):
    """Acquire a writer without replaying any work inside the transaction.

    The connection must not already be in a transaction and its caller must
    serialize access. Extended BUSY codes are intentionally not retried.
    """
    started = time.monotonic()
    for attempt in range(1, MAX_BEGIN_ATTEMPTS + 1):
        try:
            db.execute('BEGIN IMMEDIATE')
        except sqlite3.OperationalError as error:
            elapsed = time.monotonic() - started
            if (getattr(error, 'sqlite_errorcode', None) != sqlite3.SQLITE_BUSY
                    or db.in_transaction):
                raise
            log.warning('SQLite writer busy database=%s attempt=%d/%d elapsed_s=%.3f',
                        label, attempt, MAX_BEGIN_ATTEMPTS, elapsed)
            if attempt == MAX_BEGIN_ATTEMPTS:
                raise
            time.sleep(RETRY_DELAY_SECONDS)
        else:
            elapsed = time.monotonic() - started
            if attempt > 1 or elapsed >= 0.1:
                log.warning('SQLite writer acquired database=%s attempts=%d elapsed_s=%.3f',
                            label, attempt, elapsed)
            return
