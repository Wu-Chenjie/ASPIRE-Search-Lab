"""Truth-isolated metrics; run only AFTER decisions have finished."""
import argparse,json
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();rows=[]
    for run in a.runs:
        r=json.loads((run/'result.json').read_text());truth=np.array(json.loads((run/'viewpoints.json').read_text())['person_pose'][:2])
        confirmed=[t for t in r.get('tracks',[]) if t['state']=='confirmed']
        match=[t for t in confirmed if not r['negative_fixture'] and np.linalg.norm(np.array(t['xyz'])[:2]-truth)<.65]
        rows.append({'run':run.name,'strategy':r['strategy'],'fixture':r['start'],'height':r['height'],'negative':r['negative_fixture'],'frontier_recovery':r.get('frontier_recovery',False),'budget_s':r['budget_wall_s'],'status':r['status'],'true_discovery':bool(match),'false_confirmations':len(confirmed)-len(match),'latency_s':r.get('latency_wall_s') if match else None,'path_m':r['path_m'],'aligned_pairs':r['aligned_pairs'],'accepted_curves':r['accepted_curves'],'remaining_processes':r['cleanup']['remaining_pids']})
    report={'scope':'Development ideal camera rig, one run per condition unless named repeats. Not flight safety, statistical discovery rate, or algorithm superiority. Truth used here only.','results':rows}
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
