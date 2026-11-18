"""Replay saved native receipt/decision inputs through the actual C++ policy.

No detector, motion, range inference, map reconstruction or truth is involved.
The reachable flags/costs are recorded native path results, not revalidated here.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile


def audit(run):
    events=[]
    for number,line in enumerate((run/'telemetry.jsonl').read_text().splitlines(),1):
        row=json.loads(line)
        if row['kind']=='bearing_decision' and row['data']['event'] in ('cue_receipt','decision'):
            events.append((number,row['data']))
    commands=[]
    def candidate(identifier,point,yaw,cost,reachable):
        return ' '.join(map(str,[identifier,*point,yaw,cost,int(reachable)]))
    for _,row in events:
        prefix=f"{row['processing_time_s']:.17g} {int(row['enabled'])}"
        if row['event']=='cue_receipt':
            if len(row['packet'])!=17 or any(x is None or not math.isfinite(x) for x in row['packet']):
                raise ValueError('Cannot replay malformed receipt with non-finite/missing values')
            commands.append('R '+prefix+' 0 '+' '.join(format(x,'.17g') for x in row['packet']))
        else:
            scores=row['candidates'];base=next((c for c in scores if c['candidate_id']==-1),None)
            fields=candidate(-1,row['baseline_goal'],row['baseline_yaw'],base['native_cost'] if base else 0,
                             base['reachable'] if base else row['reason']!='baseline_not_reachable_no_override')
            alternatives=[c for c in scores if c['candidate_id']!=-1]
            commands.append('D '+prefix+' '+fields+' '+str(len(alternatives))+' '+
                ' '.join(candidate(c['candidate_id'],c['goal'],c['yaw'],c['native_cost'],c['reachable']) for c in alternatives))
    here=Path(__file__).parent;compiler=shutil.which('g++') or shutil.which('clang++')
    if not compiler:raise RuntimeError('C++17 compiler required')
    with tempfile.TemporaryDirectory(prefix='bearing-audit-') as temporary:
        binary=Path(temporary)/'probe'
        subprocess.run([compiler,'-std=c++17','-O2','-I',str(here),str(here/'bearing_replay_probe.cpp'),'-o',str(binary)],check=True,capture_output=True,text=True)
        output=subprocess.run([str(binary)],input='\n'.join(commands)+'\n',check=True,capture_output=True,text=True).stdout.splitlines()
    failures=[];decisions=[];accepted=0
    if len(output)!=len(events):raise ValueError('Replay event count differs')
    for (number,row),line in zip(events,output):
        parts=line.split();errors=[]
        if row['event']=='cue_receipt':
            actual=bool(int(parts[1]));accepted+=actual
            if actual!=row['accepted']:errors.append('receipt acceptance')
        else:
            selected,changed,cue=int(parts[1]),bool(int(parts[2])),int(parts[3])
            for name,actual,expected in [('selected',selected,row['chosen_candidate_id']),('changed',changed,row['changed']),
                ('cue',cue,row['cue_id']),('reason',parts[4],row['reason']),('mission_selections',int(parts[5]),row['mission_bias_selections'])]:
                if actual!=expected:errors.append(name)
            if int(parts[6])!=len(row['candidates']):errors.append('score count')
            else:
                for index,score in enumerate(row['candidates']):
                    values=parts[7+index*7:14+index*7]
                    if int(values[0])!=score['candidate_id'] or int(values[1])!=score['cue_id'] or bool(int(values[5]))!=score['reachable'] or values[6]!=score['reason']:
                        errors.append(f'score {index} metadata')
                    for token,key in zip(values[2:5],('native_cost','adjusted_cost','utility')):
                        if not math.isclose(float(token),score[key],rel_tol=1e-10,abs_tol=1e-10):errors.append(f'score {index} {key}')
            decisions.append(dict(line=number,selection_id=row['selection_id'],changed=changed,reason=parts[4],replay_matches=not errors))
        if errors:failures.append(dict(line=number,event=row['event'],errors=errors))
    return dict(scope='offline exact C++ policy replay, not path revalidation or new flight',run=str(run),
        status='passed' if events and not failures else 'failed' if failures else 'not_exercised',events=len(events),
        receipts=sum(row['event']=='cue_receipt' for _,row in events),accepted_receipts=accepted,
        decisions=len(decisions),changed=sum(row['changed'] for row in decisions),
        decision_reasons=dict(Counter(row['reason'] for row in decisions)),failures=failures,decision_details=decisions)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=audit(args.run);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='decision_details'},indent=2))
