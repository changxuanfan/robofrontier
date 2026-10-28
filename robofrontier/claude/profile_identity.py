"""Stable identity for non-secret model launch configuration."""
import hashlib
import json


def fingerprint(profile):
    return hashlib.sha256(json.dumps(profile, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
