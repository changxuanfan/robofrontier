# SQLite writer contention

RuntimeStore uses SQLite WAL and a 30-second busy timeout. A non-nested `BEGIN IMMEDIATE` that returns the ordinary `SQLITE_BUSY` code is attempted at most four times, with 0.25 seconds between attempts. The total SQLite waiting budget is approximately 120.75 seconds. Warning logs go to standard logging rather than the contested database, including elapsed time, attempts and outcome. Nested savepoints, errors inside a transaction, commit, extended busy codes and other errors are not replayed. Existing transaction rollback and action call-ID semantics remain in effect.

The default local HTTP client timeout includes one maximum SQLite begin wait plus the normal 45-second response allowance. Callers that explicitly set a timeout retain their chosen value. Connection startup, multiple sequential acquisitions, other callers and external transport deadlines may still fail independently; a timeout does not establish whether an action executed. Reconcile or query its original call ID.

The Claude shared budget ledger uses the same writer-acquisition policy. Existing runs retain their frozen source and should be handled with their matching frozen code.
