"""Independent post-run PX4 ULog and bridge trace audit, no control outputs."""
import argparse,collections,json
from pathlib import Path
import numpy as np
from tools.flight.search_evidence import rows,stats


def audit(run):
    from pyulog import ULog
    run=Path(run);ulog=next((run/'px4_1/log').rglob('*.ulg'));u=ULog(str(ulog))
    vs=u.get_dataset('vehicle_status').data
    armed=vs['arming_state']==2;armed_any=armed.any();start=float(vs['timestamp'][armed][0])*1e-6 if armed_any else None
    end=float(vs['timestamp'][armed][-1])*1e-6 if armed_any else None
    def active(t):return start is not None and start<=t<=end
    f=u.get_dataset('failsafe_flags').data;ft=f['timestamp']*1e-6
    transitions={}
    for key in ('offboard_control_signal_lost','attitude_invalid','local_position_invalid','battery_warning'):
        v=f[key];transitions[key]=[dict(sim_s=float(ft[i]),value=int(v[i])) for i in range(1,len(v)) if v[i]!=v[i-1]]
    trace=rows(run/'timing/drone-0-bridge.jsonl');hb=[r for r in trace if r['event']=='heartbeat_publish' and active(r['stamp_ns']*1e-9)]
    flags=[r for r in trace if r['event']=='input_receive' and r['input']=='flags' and active(r['received_sim_s'])]
    position=[r for r in trace if r['event']=='input_receive' and r['input']=='raw_position' and active(r['received_sim_s'])]
    gates=[r for r in trace if r['event']=='heartbeat_gate' and active(r['sim_s'])]
    fault=[r for r in gates if r['failed_gates']]
    d=u.get_dataset('estimator_status_flags').data;t=d['timestamp']*1e-6;t=t[(t>=start)&(t<=end)] if start is not None else np.array([])
    result=dict(scope='Actual PX4 ULog + bridge diagnostics. ULog can subsample messages; heartbeat maxima below are every recorded bridge publication, not network receive latency.',run=str(run),ulog=str(ulog),armed_interval_sim_s=[start,end],
        offboard_lost_while_armed=bool(np.any(f['offboard_control_signal_lost'][(ft>=start)&(ft<=end)])) if start is not None else None,
        px4_failsafe_while_armed=bool(np.any(vs['failsafe'][armed])),flags_transitions=transitions,
        heartbeat_interval_wall_s=stats(np.diff([r['wall_ns'] for r in hb])*1e-9),heartbeat_interval_sim_s=stats(np.diff([r['stamp_ns'] for r in hb])*1e-9),
        flags_ulog_sample_interval_sim_s=stats(np.diff(t)),flags_receive_interval_sim_s=stats(r['interval_sim_s'] for r in flags if r['interval_sim_s'] is not None),flags_receive_interval_wall_s=stats(r['interval_wall_s'] for r in flags if r['interval_wall_s'] is not None),
        raw_position_receive_interval_wall_s=stats(r['interval_wall_s'] for r in position if r['interval_wall_s'] is not None),
        armed_gate_failure_samples=len(fault),armed_gate_failure_counts=dict(collections.Counter(k for r in fault for k in r['failed_gates'])),
        gate_failure_scope='Logged on gate change and at least every 0.2 wall s; counts are snapshots, not number of distinct failures.',
        armed_gate_failures=fault,clock_discontinuities=[r for r in trace if r['event']=='clock_discontinuity'],
        flags_age_wall_s=stats(r['ages']['flags_wall_s'] for r in gates if r['ages']['flags_wall_s'] is not None),
        pose_age_sim_s=stats(r['ages']['position_sim_s'] for r in gates if r['ages']['position_sim_s'] is not None))
    (run/'heartbeat-audit.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='armed_gate_failures'},indent=2));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();audit(a.run)
