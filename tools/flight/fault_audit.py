"""Post-run audit of actual fault injection, ROS receipts and PX4 ULog."""
import argparse,collections,json
from pathlib import Path
import numpy as np
from tools.flight.search_evidence import rows,stats


def audit(run):
    from pyulog import ULog
    from px4_msgs.msg import VehicleStatus,VehicleCommand
    run=Path(run);result=json.loads((run/'result.json').read_text());mode=result['fault_injection']
    schedule=rows(run/'fault-schedule.jsonl');observer=rows(run/'fault-messages.jsonl');trace=rows(run/'timing/drone-0-bridge.jsonl');telemetry=rows(run/'telemetry.jsonl')
    if mode=='dds':
        begin=next(r for r in schedule if r['event']=='agent_stop_observed')
        end=next(r for r in schedule if r['event']=='agent_resume_observed')
    else:
        begin=next(r for r in observer if r['event']=='relay_gate_changed' and r['drop'])
        end=next(r for r in observer if r['event']=='relay_gate_changed' and not r['drop'])
    start,stop=begin['wall_ns'],end['wall_ns'];sim_start,sim_stop=begin['sim_s'],end['sim_s']
    def phase(ns):return 'before' if ns<start else 'interrupted' if ns<stop else 'restored'
    messages=[r for r in observer if r['event']=='message'];count=collections.Counter((phase(r['wall_ns']),r['input'],r['action']) for r in messages)
    message_counts={p:{k:{a:n for (pp,kk,a),n in count.items() if pp==p and kk==k} for k in sorted({r['input'] for r in messages})} for p in ('before','interrupted','restored')}
    received=[r for r in trace if r['event']=='input_receive'];receipt_counts=collections.Counter((phase(r['wall_ns']),r['input']) for r in received)
    gates=[r for r in trace if r['event']=='heartbeat_gate' and r['wall_ns']>=start and r['failed_gates']]
    hb=[r for r in trace if r['event']=='heartbeat_publish'];before_restore=[r for r in hb if r['wall_ns']<stop];after_restore=[r for r in hb if r['wall_ns']>=stop]
    restored=[r['data'] for r in telemetry if r['kind']=='flight' and r['data']['time']>=sim_stop]
    commands=[r for r in messages if r['input']=='command' and r['wall_ns']>=stop]
    rearm=[r for r in commands if r['command']==VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM and r['param1']>0]
    u=ULog(str(next((run/'px4_1/log').rglob('*.ulg'))));v=u.get_dataset('vehicle_status').data;f=u.get_dataset('failsafe_flags').data
    def first(data,mask):
        indices=np.flatnonzero(mask)
        return float(data['timestamp'][indices[0]])*1e-6 if len(indices) else None
    vt=v['timestamp']*1e-6;ft=f['timestamp']*1e-6
    lost=first(f,(ft>=sim_start)&(f['offboard_control_signal_lost']!=0));failsafe=first(v,(vt>=sim_start)&(v['failsafe']!=0))
    disarmed=first(v,(vt>=sim_start)&(v['arming_state']!=VehicleStatus.ARMING_STATE_ARMED))
    nav=[dict(sim_s=float(vt[i]),state=int(v['nav_state'][i]),armed=int(v['arming_state'][i]),failsafe=bool(v['failsafe'][i])) for i in range(1,len(vt)) if vt[i]>=sim_start and any(v[k][i]!=v[k][i-1] for k in ('nav_state','arming_state','failsafe'))]
    offboard_after=bool(np.any((vt>=sim_stop)&(v['nav_state']==VehicleStatus.NAVIGATION_STATE_OFFBOARD)&(v['arming_state']==VehicleStatus.ARMING_STATE_ARMED)))
    selected={'flags':'flags','position':'raw_position'}.get(mode)
    drop_count=sum(r['input']==selected and r['action']=='dropped' and phase(r['wall_ns'])=='interrupted' for r in messages)
    selected_forwarded=sum(r['input']==selected and r['action']=='forwarded' and phase(r['wall_ns'])=='interrupted' for r in messages)
    resumed_input=any(r['input']==(selected or 'raw_position') and r['wall_ns']>=stop and -.1<=r['received_sim_s']-r['stamp_s']<=.5 for r in received)
    terminal=[r for r in messages if r['input']=='status' and r['wall_ns']>=stop and (r['armed']!=VehicleStatus.ARMING_STATE_ARMED or r['failsafe'])]
    latched=[r for r in gates if r['wall_ns']>=stop and 'failsafe_not_latched' in r['failed_gates']]
    latched_after_receipt=bool(terminal and latched and restored) and restored[-1]['failsafe_latched'] and all('failsafe_not_latched' in r['failed_gates'] for r in gates if r['wall_ns']>=latched[0]['wall_ns'])
    checks=dict(injection_completed=result.get('fault_fixture_state')=='complete',harness_no_error=result['error'] is None,
        actual_input_interrupted=(drop_count>0 and selected_forwarded==0 if selected else not any(r['input'] in ('flags','raw_position','attitude','status') and start+1_000_000_000<=r['wall_ns']<stop for r in messages)),
        bridge_stopped_heartbeat=bool(gates) and bool(before_restore) and stop-before_restore[-1]['wall_ns']>500_000_000,
        px4_detected_offboard_loss=lost is not None,px4_entered_failsafe=failsafe is not None,
        fresh_input_received_after_restore=resumed_input,no_heartbeat_after_restore=not after_restore,
        bridge_latched_after_terminal_status_and_stayed_latched=bool(latched_after_receipt),
        no_rearm_command_after_restore=not rearm,no_armed_px4_offboard_after_restore=not offboard_after,
        cleaned=not result['cleanup']['forced_kill_pids'] and not result['cleanup']['remaining_pids'])
    summary=dict(scope='Actual Gazebo/PX4 SITL, real ROS2 messages; selective application drops or owned MicroXRCEAgent SIGSTOP/SIGCONT, never OS network changes',mode=mode,
        injection_actual=begin,restoration_actual=end,interruption_wall_s=(stop-start)*1e-9,
        bridge_tick_count_during_fault=sum(r['event']=='tick' and start<=r['wall_ns']<stop for r in trace),
        message_counts=message_counts,bridge_receipt_counts={p:{k:n for (pp,k),n in receipt_counts.items() if pp==p} for p in ('before','interrupted','restored')},
        bridge_receive_intervals_wall_s={k:stats(r['interval_wall_s'] for r in received if r['input']==k and r['interval_wall_s'] is not None) for k in ('flags','raw_position','attitude','status')},
        first_failed_gate=gates[0] if gates else None,first_gate_delay_wall_s=(gates[0]['wall_ns']-start)*1e-9 if gates else None,
        last_heartbeat=before_restore[-1] if before_restore else None,heartbeat_after_restore_count=len(after_restore),
        heartbeat_interval_wall_s_before_injection=stats(np.diff([r['wall_ns'] for r in hb if r['wall_ns']<start])*1e-9),
        px4_offboard_lost_sim_s=lost,px4_failsafe_sim_s=failsafe,px4_disarmed_sim_s=disarmed,px4_state_transitions=nav,
        terminal_status_after_restore=terminal[0] if terminal else None,
        terminal_status_delay_after_process_restore_wall_s=(terminal[0]['wall_ns']-stop)*1e-9 if terminal else None,
        first_latched_gate_after_restore=latched[0] if latched else None,
        latch_delay_from_observer_terminal_receipt_wall_s=(latched[0]['wall_ns']-terminal[0]['wall_ns'])*1e-9 if terminal and latched else None,
        strict_latch_within_half_sim_second_of_process_restore=bool(restored) and all(r['failsafe_latched'] for r in restored if r['time']>=sim_stop+.5),
        commands_after_restore=commands,last_bridge_status=restored[-1] if restored else None,
        expectation_checks=checks,expected_protection_observed=all(checks.values()),
        caveat='Whole-link scope is the owned PX4 XRCE/DDS agent, not Gazebo/ROS clock or every ROS participant. Counts are observed deliveries, not all PX4 internal publications. Intentional failsafe and ground contact during landing are not nominal flight failures.')
    (run/'fault-audit.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:summary[k] for k in ('mode','interruption_wall_s','first_gate_delay_wall_s','px4_offboard_lost_sim_s','px4_failsafe_sim_s','px4_disarmed_sim_s','expectation_checks','expected_protection_observed')},indent=2));return summary


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);audit(p.parse_args().run)
