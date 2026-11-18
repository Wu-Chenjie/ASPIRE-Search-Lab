"""Validate and write paired configurations; never builds, launches or freezes a run."""
import argparse
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from aspire_planning.static_pair import prepare_pair


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime',required=True,type=Path)
    p.add_argument('--settings',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args()
    pair=prepare_pair(json.loads(a.runtime.read_text()),json.loads(a.settings.read_text()))
    a.output.mkdir(parents=True,exist_ok=False)
    for arm,runtime in pair.items():
        (a.output/(arm+'.json')).write_text(json.dumps(runtime,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(a.output.resolve()),common_runtime_sha256=pair['baseline']['static_pair']['common_runtime_sha256'],experiment_started=False,protocol_frozen=False)))


if __name__=='__main__':main()
