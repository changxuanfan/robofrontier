"""Verify that a Claude Code fork inherited its parent's history, from native transcripts.

`claude --resume PARENT --fork-session` writes the child's transcript under the same
config directory. Observed with CLI 2.1.312: the child carries every parent message row
with the same uuid and content, differing only in `sessionId` and `promptId`. Anything
else that differs, or a parent that changed after the fork, is rejected rather than
tolerated. The model's own claims are never used as evidence.
"""
import hashlib
import json
import re
from pathlib import Path

UUID = re.compile(r'[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}')
# Per-session fields that legitimately differ between a parent row and its forked copy.
SESSION_FIELDS = ('sessionId', 'promptId')


def transcript_rows(config_dir, session_id):
    """Return the parsed rows of exactly one transcript for this native session ID."""
    if not UUID.fullmatch(session_id):
        raise ValueError('Expected a native session UUID')
    matches = sorted(Path(config_dir).glob('projects/*/' + session_id + '.jsonl'))
    if len(matches) != 1:
        raise ValueError('Native transcript missing or ambiguous for session ' + session_id)
    rows = []
    for number, line in enumerate(matches[0].read_text().splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except ValueError as error:
            raise ValueError('Native transcript line %d is not JSON' % number) from error
        if not isinstance(row, dict):
            raise ValueError('Native transcript line %d is not an object' % number)
        rows.append(row)
    return rows


def history(rows, session_id):
    """Message rows in file order with per-session fields removed; fail on foreign rows."""
    out = []
    for row in rows:
        if not isinstance(row.get('uuid'), str):
            continue  # bookkeeping rows are not conversation history
        if row.get('sessionId') != session_id:
            raise ValueError('Transcript row belongs to a different session')
        out.append({k: v for k, v in row.items() if k not in SESSION_FIELDS})
    if len({row['uuid'] for row in out}) != len(out):
        raise ValueError('Duplicate message UUID inside one transcript')
    return out


def fork_provenance(parent, child, config_dir):
    """Prove `child` was forked from the whole of `parent`'s current history."""
    if parent == child:
        raise ValueError('A fork cannot be its own source')
    source = history(transcript_rows(config_dir, parent), parent)
    copy = history(transcript_rows(config_dir, child), child)
    if not source:
        raise ValueError('Source session has no recorded history')
    if copy[:len(source)] != source:
        raise ValueError('Native fork history does not match the source session')
    digest = hashlib.sha256(json.dumps(source, sort_keys=True).encode()).hexdigest()
    return {'source_thread_id': parent, 'source_ordinal': len(source),
            'source_sha256': digest, 'source_last_uuid': source[-1]['uuid'], 'engine': 'claude'}
