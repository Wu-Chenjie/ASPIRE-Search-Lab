#!/usr/bin/env python3
"""Offline evidence metrics. No ROS imports, scene truth, flight commands or writes to runs.

Usage: python3 tools/flight/stage23_metrics.py RUN --output /tmp/metrics.json
Optional pyulog must already be installed (or supplied with PYTHONPATH).
Unknown measurements remain null; normalized thrust commands are not physical force.
"""
import argparse
import collections
import json
from pathlib import Path
import numpy as np


def stats(values):
    a = np.asarray(values, dtype=float).reshape(-1)
    a = a[np.isfinite(a)]
    if not len(a):
        return None
    return dict(samples=int(len(a)), minimum=float(a.min()), mean=float(a.mean()),
                median=float(np.median(a)), p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def rows(path, issues):
    if not path.is_file():
        issues.append(f'Missing {path.name}')
        return None
    result = []
    with path.open() as stream:
        for number, line in enumerate(stream, 1):
            try:
                result.append(json.loads(line))
            except (ValueError, TypeError):
                issues.append(f'{path.name}:{number}: invalid JSON row omitted')
    return result


def document(path, issues):
    if not path.is_file():
        issues.append(f'Missing {path.name}')
        return {}
    try:
        return json.loads(path.read_text())
    except ValueError:
        issues.append(f'Invalid {path.name}')
        return {}


def in_intervals(t, intervals):
    mask = np.zeros(len(t), dtype=bool)
    for lo, hi in intervals:
        mask |= (t >= lo) & (t <= hi)
    return mask


def vector(data, prefix=None, names=None):
    names = names or [f'{prefix}[{i}]' for i in range(3)]
    if data is None or any(k not in data for k in names):
        return None
    return np.column_stack([data[k] for k in names]).astype(float)


def sampled(data, intervals):
    if data is None or 'timestamp' not in data:
        return None, None
    t = np.asarray(data['timestamp'], float) * 1e-6
    return t, in_intervals(t, intervals)


def vector_stats(data, intervals, prefix=None, names=None):
    a = vector(data, prefix, names)
    t, mask = sampled(data, intervals)
    if a is None or t is None:
        return None
    valid = mask & np.isfinite(a).all(axis=1)
    if not valid.any():
        return None
    return dict(norm=stats(np.linalg.norm(a[valid], axis=1)),
                axes=[stats(a[valid, i]) for i in range(a.shape[1])],
                sample_interval_s=stats(np.diff(t)[valid[:-1] & valid[1:]]),
                first_sim_s=float(t[valid][0]), last_sim_s=float(t[valid][-1]))


def attitude_stats(data, intervals):
    q = vector(data, names=[f'q[{i}]' for i in range(4)])
    t, mask = sampled(data, intervals)
    if q is None or t is None:
        return None
    norm = np.linalg.norm(q, axis=1)
    valid = mask & np.isfinite(q).all(axis=1) & (norm > 1e-6)
    if not valid.any():
        return None
    w, x, y, z = (q[valid]/norm[valid, None]).T
    roll = np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = np.arcsin(np.clip(2*(w*y-z*x), -1, 1))
    tilt = np.arccos(np.clip(1-2*(x*x+y*y), -1, 1))
    return dict(roll_deg=stats(np.degrees(roll)), pitch_deg=stats(np.degrees(pitch)),
                abs_roll_deg=stats(np.abs(np.degrees(roll))),
                abs_pitch_deg=stats(np.abs(np.degrees(pitch))), tilt_deg=stats(np.degrees(tilt)),
                sample_interval_s=stats(np.diff(t)[valid[:-1] & valid[1:]]))


def rotation_rate_stats(data, intervals, maximum_gap=.1):
    """Shortest quaternion rotation / dt; invariant to q versus -q and yaw wrap.

    This is an interval-average magnitude, not instantaneous body-axis rates.
    The local PX4 GZBridge groundtruth rate channel instead differences Euler
    angles without unwrapping; preserve that raw channel separately.
    """
    q = vector(data, names=[f'q[{i}]' for i in range(4)])
    t, mask = sampled(data, intervals)
    if q is None or t is None or len(t)<2:return None
    norm=np.linalg.norm(q,axis=1)
    valid=mask & np.isfinite(q).all(axis=1) & (norm>1e-6)
    unit=np.zeros_like(q);unit[valid]=q[valid]/norm[valid,None]
    dt=np.diff(t)
    pair=valid[:-1] & valid[1:] & (dt>0) & (dt<=maximum_gap)
    if not pair.any():return None
    dot=np.abs(np.sum(unit[:-1]*unit[1:],axis=1))
    angle=2*np.arccos(np.clip(dot[pair],0,1))
    return dict(norm=stats(angle/dt[pair]),sample_interval_s=stats(dt[pair]),
                excluded_pairs=int((~pair).sum()),
                meaning='Shortest quaternion rotation magnitude per recorded interval; not instantaneous body rates')


def flag_stats(t, values, mask, maximum_gap=.5):
    values = np.asarray(values, bool)
    selected = values[mask]
    if not len(selected):
        return None
    dt = np.diff(t)
    eligible = mask[:-1] & mask[1:] & (dt > 0) & (dt <= maximum_gap)
    return dict(samples=int(mask.sum()), true_samples=int(selected.sum()),
                sample_fraction=float(selected.mean()),
                observed_true_s=float(np.sum(dt[eligible & values[:-1]])),
                observed_interval_s=float(np.sum(dt[eligible])),
                duration_method=f'Left-held samples, adjacent selected samples only, gaps >{maximum_gap}s excluded')


def output_stats(data, intervals, prefix, motor_channels=None):
    t, mask = sampled(data, intervals)
    if t is None or not mask.any():
        return None
    channels = {}
    for i in range(16):
        name = f'{prefix}[{i}]'
        if name not in data or (motor_channels is not None and i not in motor_channels):
            continue
        a = np.asarray(data[name], float)
        valid = mask & np.isfinite(a)
        if not valid.any():
            continue
        channels[str(i)] = dict(value=stats(a[valid]))
        if prefix == 'control':
            channels[str(i)]['at_or_above_0_95'] = flag_stats(t, a >= .95, valid)
    return dict(channels=channels) if channels else None


def allocator_stats(data, intervals, motor_channels):
    t, mask = sampled(data, intervals)
    if t is None or not mask.any():
        return None
    result = {};channels = sorted(motor_channels)
    saturation = [np.asarray(data[f'actuator_saturation[{i}]']) for i in channels
                  if f'actuator_saturation[{i}]' in data]
    result['motor_channel_indices'] = channels
    if saturation:
        a = np.column_stack(saturation)
        result['any_active_motor_nonzero_saturation_code'] = flag_stats(t, np.any(a != 0, axis=1), mask)
        result['active_motor_saturation_code_histogram'] = dict(collections.Counter(map(str, a[mask].reshape(-1).tolist())))
    else:
        result['any_active_motor_nonzero_saturation_code'] = None
    for key in ('torque_setpoint_achieved', 'thrust_setpoint_achieved'):
        result[key+'_false'] = flag_stats(t, ~np.asarray(data[key], bool), mask) if key in data else None
    for key in ('unallocated_torque', 'unallocated_thrust'):
        result[key+'_normalized'] = vector_stats(data, intervals, prefix=key)
    result['meaning'] = 'Allocator-reported codes for finite actuator_motors channels only; unused channels excluded. No inferred motor damage or physical force.'
    return result


def path_stats(poses, intervals, maximum_gap=.25):
    # Deduplicate timestamps and refuse connecting missing-time gaps.
    records = {float(r['stamp']): r['xyz'] for r in poses if 'stamp' in r and 'xyz' in r}
    if len(records) < 2:
        return None
    t = np.array(sorted(records)); p = np.array([records[k] for k in t], float)
    valid = in_intervals(t, intervals) & np.isfinite(p).all(axis=1)
    dt = np.diff(t); pair = valid[:-1] & valid[1:] & (dt > 0) & (dt <= maximum_gap)
    if not pair.any():
        return None
    return dict(distance_m=float(np.linalg.norm(np.diff(p, axis=0)[pair], axis=1).sum()),
                accepted_segments=int(pair.sum()), skipped_long_gap_segments=int(np.sum(valid[:-1] & valid[1:] & (dt > maximum_gap))),
                source='Estimated odometry positions; includes estimator jitter/drift; no interpolation over gaps',
                maximum_pair_gap_s=maximum_gap)


def growth(events, field, start):
    points = sorted((float(r['time']), r[field]) for r in events
                    if r.get('type') == 'execution_grid_published' and field in r and 'time' in r)
    if not points or start is None:
        return None
    before = [p for p in points if p[0] <= start]
    after = [p for p in points if p[0] >= start]
    if not after:
        return None
    first = before[-1] if before else after[0]
    last = after[-1]
    return dict(start_sim_s=first[0], end_sim_s=last[0], start_count=int(first[1]),
                end_count=int(last[1]), net_change=int(last[1]-first[1]),
                baseline_offset_from_ready_s=first[0]-start,
                meaning='Net cell-count change, not cumulative first-time discoveries')


def audit(run, ulog_path=None):
    run = Path(run); issues = []
    tel = rows(run/'telemetry.jsonl', issues)
    events = rows(run/'flight/0/events.jsonl', issues)
    result = document(run/'result.json', issues)
    config = document(run/'runtime.json', issues)
    executions = [r['data'] for r in tel or [] if r.get('kind') == 'execution']
    poses = [r['data'] for r in tel or [] if r.get('kind') == 'estimated_odometry']
    first_ready = min((r['time'] for r in executions if r.get('ready')), default=None)
    end = max((r['time'] for r in executions if 'time' in r), default=None)
    tokens = {r['token'] for r in events or [] if r.get('type') == 'native_trajectory_activated'}
    tracking = [r for r in executions if r.get('token') in tokens and r.get('reason') in
                ('tracking_view', 'tracking_error_hold', 'observation_dwell')]
    native_intervals = []
    for token in sorted(tokens):
        stamps = [r['time'] for r in tracking if r['token'] == token]
        if stamps:
            native_intervals.append([min(stamps), max(stamps)])
    ulog = None; datasets = {}; ulog_error = None; path = None
    candidates = [Path(ulog_path)] if ulog_path else sorted(run.glob('px4_*/log/**/*.ulg'))
    if len(candidates) != 1:
        ulog_error = f'Expected one ULog, found {len(candidates)}; use --ulog to select explicitly'
    else:
        path = candidates[0]
        try:
            from pyulog import ULog
            ulog = ULog(str(path))
            datasets = {d.name: d.data for d in ulog.data_list if d.multi_id == 0}
        except (ImportError, OSError, ValueError, KeyError) as error:
            ulog_error = f'{type(error).__name__}: {error}'
    if ulog_error:
        issues.append(ulog_error)
    armed_intervals = []
    status = datasets.get('vehicle_status')
    if status is not None and 'arming_state' in status:
        t = status['timestamp'] * 1e-6; armed = status['arming_state'] == 2
        edges = np.diff(np.r_[False, armed, False].astype(int))
        for lo, hi in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
            armed_intervals.append([float(t[lo]), float(t[min(hi, len(t)-1)])])
    scopes = dict(post_ready=[[first_ready, end]] if first_ready is not None and end is not None else [],
                  armed=armed_intervals, native_execution=native_intervals)
    metrics = {}
    for name, intervals in scopes.items():
        selected = [r for r in tracking if in_intervals(np.array([r['time']]), intervals)[0]]
        motors = output_stats(datasets.get('actuator_motors'), intervals, 'control')
        channels = set(map(int, motors['channels'])) if motors else set()
        metrics[name] = dict(intervals_sim_s=intervals,
            simulator_speed_m_s=vector_stats(datasets.get('vehicle_local_position_groundtruth'), intervals, names=['vx','vy','vz']),
            simulator_acceleration_m_s2=vector_stats(datasets.get('vehicle_local_position_groundtruth'), intervals, names=['ax','ay','az']),
            estimated_speed_m_s=vector_stats(datasets.get('vehicle_local_position'), intervals, names=['vx','vy','vz']),
            estimated_acceleration_m_s2=vector_stats(datasets.get('vehicle_local_position'), intervals, names=['ax','ay','az']),
            tracking_error_m=stats([r['tracking_error'] for r in selected if 'tracking_error' in r]),
            tracking_scope='Estimated position minus commanded reference, activated native tracking/hold/dwell samples only',
            tracking_hold_samples=sum(r.get('reason') == 'tracking_error_hold' for r in selected) if selected else None,
            estimated_attitude_deg=attitude_stats(datasets.get('vehicle_attitude'), intervals),
            simulator_attitude_deg=attitude_stats(datasets.get('vehicle_attitude_groundtruth'), intervals),
            estimated_body_rate_rad_s=vector_stats(datasets.get('vehicle_angular_velocity'), intervals, prefix='xyz'),
            simulator_reported_euler_rate_rad_s=vector_stats(datasets.get('vehicle_angular_velocity_groundtruth'), intervals, prefix='xyz'),
            simulator_rotation_rate_rad_s=rotation_rate_stats(datasets.get('vehicle_attitude_groundtruth'), intervals),
            normalized_thrust_command=vector_stats(datasets.get('vehicle_thrust_setpoint'), intervals, prefix='xyz'),
            physical_thrust_newtons=None,
            normalized_motor_commands=motors,
            raw_actuator_output=output_stats(datasets.get('actuator_outputs'), intervals, 'output', channels),
            allocator=allocator_stats(datasets.get('control_allocator_status'), intervals, channels),
            estimated_path=path_stats(poses, intervals))
    attempts = {r['native_id'] for r in events or [] if r.get('type') == 'native_planning_attempt'}
    rejected = {r['traj_id'] for r in events or [] if r.get('type') == 'native_proposal_rejected'}
    rejected_attempts = rejected & attempts
    activated = {r['traj_id'] for r in events or [] if r.get('type') == 'native_trajectory_activated'}
    known_free = growth(events or [], 'known_free_cells', first_ready)
    if known_free is None:
        issues.append('known_free_cells not logged: observed_cells and safe_cells cannot substitute for known-free occupancy count')
    proposed = [r for r in events or [] if r.get('type') == 'path_proposed' and r.get('token') in tokens]
    return dict(schema='racer.stage23.metrics/2', run=str(run),
        scope='Independent offline evaluation of existing logs. No scene truth read and no control outputs.',
        units=dict(thrust='vehicle_thrust_setpoint normalized body command, NOT Newtons or measured force',
                   motors='actuator_motors normalized control; >=.95 is a descriptive threshold, not a hardware saturation diagnosis',
                   outputs='Raw actuator_outputs units are backend-specific; same indices as finite motor command channels, not measured RPM',
                   acceleration='ULog local-position kinematic acceleration, not certified IMU shock peaks',
                   quaternion='PX4 wxyz; tilt is body-z versus world-z; norms are frame-rotation invariant',
                   reported_simulator_rate='Local PX4 GZBridge.cpp directly differences Euler angles without wrapping. Raw channel can spike at +/-pi and is not a body-rate vector.',
                   simulator_rotation_rate='Quaternion interval-average rotation magnitude, sign/wrap invariant; not an instantaneous measurement'),
        ulog=dict(path=str(path) if path else None, error=ulog_error,
                  datasets={name:len(d.get('timestamp',[])) for name,d in datasets.items()},
                  parameters={k:ulog.initial_parameters.get(k) for k in ('MPC_THR_MIN','MPC_THR_MAX','MPC_THR_HOVER')} if ulog else None),
        clock='ULog microsecond boot timestamps compared to recorded ROS simulation seconds; no inferred wall-time alignment',
        flight_metrics=metrics,
        authorized_command_peaks={key:max((r.get('trajectory_limits',{}).get(key,0) for r in proposed),default=None) for key in ('speed','acceleration','jerk')},
        planning=dict(attempts=len(attempts) if events is not None else None,
                      rejected_attempts=len(rejected_attempts) if events is not None else None,
                      rejection_rate=len(rejected_attempts)/len(attempts) if attempts else None,
                      rejection_denominator='Distinct native_planning_attempt native_id, not number of callback rows',
                      rejected_ids_without_attempt=sorted(rejected-attempts),
                      rejection_reasons=dict(collections.Counter(r['reason'] for r in events or [] if r.get('type')=='native_proposal_rejected')) if events is not None else None,
                      activated=len(activated) if events is not None else None,
                      completed_observations=len({r['token'] for r in events or [] if r.get('type')=='terminal_observation_completed'}) if events is not None else None),
        exploration=dict(known_free_cells=known_free, observed_cells=growth(events or [],'observed_cells',first_ready),
                         clearance_safe_cells=growth(events or [],'safe_cells',first_ready),
                         note='Counts have different semantics. No known-free value is inferred from clearance-safe or observed count.'),
        budget=dict(clock=result.get('budget_clock'),requested_seconds=result.get('budget_seconds'),
                    end_sim_s=result.get('budget_end_sim'),recorded_wall_s=result.get('wall_s'),
                    first_ready_sim_s=first_ready, last_execution_sim_s=end,
                    post_ready_sim_s=end-first_ready if first_ready is not None and end is not None else None,
                    configured_motion_limits=config.get('motion_limits'),configured_native_limits=config.get('racer_planning_limits')),
        issues=issues,
        limits=['Missing channels remain null; an empty selection is not evidence of zero motion or saturation.',
                'ULog samples may be decimated; reported maxima are sampled maxima, not continuous-time physical certificates.',
                'No full mesh clearance metric here; no collision-free conclusion from these metrics.',
                'Native interval endpoints are first/last observed activated tracking/hold/dwell samples, with telemetry sampling uncertainty.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ulog', type=Path)
    args = parser.parse_args()
    if args.output.resolve().is_relative_to(args.run.resolve()):
        parser.error('--output must be outside the original run directory')
    if args.output.exists():
        parser.error('--output already exists; choose a new file to preserve prior evidence')
    result = audit(args.run, args.ulog)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output), issues=result['issues'], planning=result['planning']), indent=2))


if __name__ == '__main__':
    main()
