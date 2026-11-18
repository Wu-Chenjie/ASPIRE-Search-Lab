#!/usr/bin/env python3
"""Fetch the pinned OSRF static person under external/, verify Git blob digests."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[2]

def prepare(output):
    lock=json.loads(Path(__file__).with_name('assets.lock.json').read_text())
    for name,digest in lock['files'].items():
        path=Path(output)/name
        if not path.exists():
            path.parent.mkdir(parents=True,exist_ok=True)
            url=f"https://raw.githubusercontent.com/osrf/gazebo_models/{lock['commit']}/{name}"
            temporary=path.with_suffix(path.suffix+'.download')
            subprocess.run(['curl','--fail','--location','--silent','--show-error',url,'-o',str(temporary)],check=True)
            raw=temporary.read_bytes()
            if hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()!=digest:
                raise ValueError('Downloaded asset digest mismatch: '+name)
            temporary.rename(path)
        raw=path.read_bytes()
        if hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()!=digest:
            raise ValueError('Local asset digest mismatch: '+name)
    return Path(output).resolve()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=ROOT/'external/rgbd_scene_assets');a=p.parse_args();print(prepare(a.output))
