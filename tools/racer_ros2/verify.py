#!/usr/bin/env python3
"""Fail before build if generated native sources differ from their recorded port."""
import argparse
import hashlib
import json
from pathlib import Path


def verify(workspace):
    return verified_migration(workspace)[0]


def verified_migration(workspace):
    data = (workspace/'migration.json').read_bytes()
    record = json.loads(data.decode())
    failures=[]
    for name,digest in record['generated_files'].items():
        path=workspace/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:failures.append(name)
    actual={str(p.relative_to(workspace)) for p in (workspace/'src').rglob('*') if p.is_file()}
    failures+=sorted(actual-set(record['generated_files']))
    if failures:raise ValueError('Unrecorded native source changes: '+', '.join(failures))
    return record, hashlib.sha256(data).hexdigest()

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('workspace',type=Path)
    record=verify(p.parse_args().workspace)
    print('Verified RACER '+record['upstream']['commit']+' native ROS2 generated sources')
