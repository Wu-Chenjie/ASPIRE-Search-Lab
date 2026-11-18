"""Offline reporting gate; does not access online policy or alter a protocol."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
from person_search.evaluation_gate import evaluate_claim

p=argparse.ArgumentParser()
p.add_argument('--protocol',type=Path,required=True)
p.add_argument('--runs',type=Path,required=True)
a=p.parse_args()
result=evaluate_claim(json.loads(a.protocol.read_text()),json.loads(a.runs.read_text()))
print(json.dumps(result,ensure_ascii=False,indent=2))
raise SystemExit(0 if result['claim_allowed'] else 2)
