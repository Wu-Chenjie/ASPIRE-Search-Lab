"""Score confirmed identities using offline annotations; never starts a run."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
from person_search.discovery_evaluation import evaluate_discoveries


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('events','annotations','truth-ids'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--time-budget-s',type=float,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    try:
        result=evaluate_discoveries(json.loads(a.events.read_text()),json.loads(a.annotations.read_text()),
            json.loads(a.truth_ids.read_text()),a.time_budget_s)
    except ValueError as error:
        p.exit(2, f'{error}\n')
    with a.output.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps({k:result[k] for k in ('unique_true_discoveries','false_positives','duplicates','identity_uncertain')}))


if __name__=='__main__':main()
