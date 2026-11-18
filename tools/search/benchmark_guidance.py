"""Host microbenchmark only: task dedup, not entity association or ROS latency."""
import argparse
import json
from pathlib import Path
import platform
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
from person_search.guidance_scheduler import GuidanceScheduler,Reason


def summarize(samples):
    values=sorted(samples)
    return {name:values[int((len(values)-1)*q)]/1000
            for name,q in [('p50_us',.5),('p95_us',.95),('p99_us',.99),('max_us',1.)]}


def run(count):
    result=[]
    for entities in (1,8,64):
        for label in ('duplicate','material_coalescing'):
            scheduler=GuidanceScheduler()
            for i in range(entities):
                scheduler.request(str(i),1,Reason.ELIGIBLE,1.)
            samples=[]
            started=time.perf_counter_ns();cpu=time.process_time_ns()
            for i in range(count):
                entity=str(i%entities)
                revision=1 if label=='duplicate' else 2+i//entities
                before=time.perf_counter_ns()
                scheduler.request(entity,revision,Reason.VISIBILITY,1.)
                samples.append(time.perf_counter_ns()-before)
            wall=time.perf_counter_ns()-started;cpu=time.process_time_ns()-cpu
            state=scheduler.snapshot()
            result.append(dict(workload=label,entities=entities,operations=count,
                wall_ms=wall/1e6,cpu_ms=cpu/1e6,**summarize(samples),
                pending=len(state['pending']),active=len(state['active']),
                audit_dropped=state['audit_dropped']))
    return dict(platform=platform.platform(),python=sys.version,
                scope='single process task dedup only; no association, ROS, solver or flight',
                hard_realtime_guarantee=False,rows=result)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--operations',type=int,default=10000)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 100<=args.operations<=1000000:
        parser.error('operations must be between 100 and 1000000')
    args.output.write_text(json.dumps(run(args.operations),indent=2)+'\n')
