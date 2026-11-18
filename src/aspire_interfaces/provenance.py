"""Canonical SHA-256 for paired run configurations."""
import hashlib
import json


def protocol_digest(protocol):
    data = json.dumps(protocol, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(data.encode()).hexdigest()
