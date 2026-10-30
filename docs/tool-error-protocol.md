# Robot tool error feedback

New runs distinguish a typed `CALL_ID_CONFLICT` from `TRANSPORT_UNCERTAIN`.
This changes error feedback, not robot authority, action execution, scoring,
or the existing call-ID guard. No failure automatically creates another ID,
replays a command, restarts a world, or extends the deadline.

`CALL_ID_CONFLICT` has `request_status: rejected` and
`original_call_status: unknown`. Only the newly conflicting request was
rejected; the request already associated with that ID may have executed or
may still be running. Inspect own `pending` records and, for own physical
actions, `job` with the original action ID. A new ID is appropriate only for
an intentionally new request after reconciling existing work. Other roles'
requests and parameters are not disclosed.

The client detects local conflicts before submitting a changed request. The
server returns HTTP 409 JSON for a typed conflict in its task-wide call
table. The client validates the schema and matching ID, saves that explicit
rejection separately, and does not list it as unresolved execution. Repeating
that rejected request locally raises the same conflict without resubmission.
The old request and its result remain unchanged.

Timeouts, lost replies, malformed output, unknown error codes and mismatched
IDs produce `TRANSPORT_UNCERTAIN` with `request_status: unknown`. This is a
conservative result-unconfirmed category, not proof of a network root cause.
It does not establish that an operation failed to execute. The durable request
remains available for reconciliation under its original ID.

Codex validates the failed client's JSON and reconstructs an allowlisted
response; raw stderr and command lines are not returned to the policy. The
Claude MCP bridge uses the same typed Python exception and safe payloads.
Successful tools continue to use their existing results. Optional image
handling remains independent in branches that already include that fix.

Use a fresh prepared run after updating. Existing runs retain their frozen
source and inputs; do not edit or reinterpret them as runs of this version.
This change does not qualify multi-role startup, long-term concurrency,
network-filesystem storage, or task-solving performance.

Offline checks (no paid model calls or physical simulation):

```sh
PYTHONPATH=tests:. .venv/bin/python -m unittest tests.test_tool_errors tests.test_pipeline_client tests.test_runtime_store
node --test tests/test_tool_errors.js
# Claude branch additionally:
PYTHONPATH=tests:. .venv/bin/python -m unittest tests.test_claude_tool_errors
```

## CLI request ID and action ID

For a new robot request, omit the outer `call_id` and retain the response's
`call_id`. To query an earlier physical action, call
`{"op":"job","data":{"call_id":"ACTION_ID"}}` with no outer `call_id`.
The new query receives its own request ID; `data.call_id` names the original
action. If the original request itself must be retried after an uncertain
result, use its original outer ID and exactly the same operation and data.
Do not assign a new ID to replay an uncertain physical action.
