"""Audit a JSON list of target-domain annotation records without loading truth online."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
from person_search.training_data import audit_annotations

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest',type=Path)
    args=parser.parse_args()
    try:
        records=json.loads(args.manifest.read_text())
        if not isinstance(records, list):
            raise ValueError('Manifest must be a JSON array')
        result=audit_annotations(records)
    except ValueError as error:
        parser.exit(2, f'{error}\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
