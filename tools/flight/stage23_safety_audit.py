"""Offline audit of actual online safety certificates; never flight control.

No truth or reconstructed sensor maps are read. Continuous corridor validation
uses saved online states. Prefix identity is checked algebraically on the union
of polynomial spans, including derivatives, rather than by point sampling.
"""
import argparse
from contextlib import contextmanager
import hashlib
from io import BytesIO
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from zipfile import BadZipFile
from zlib import error as ZlibError

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / 'src/annual_planner/src') not in sys.path:
    sys.path.insert(0, str(ROOT / 'src/annual_planner/src'))
from core.exploration.voxel_mapping import VoxelMap
from core.exploration.voxel_config import voxel_environment
from core.planning.motion_limits import motion_environment
from core.planning.continuous_trajectory import ContinuousTrajectory, derivative, maximum_norm
from core.planning import execution_safety
from core.planning.execution_safety import NativeSafetyMap, ExecutionCorridor, stopping_reserve, certified_stopped_reference

CHECK_CLOCK = 'monotonic_wall_upper_bound_at_rtf_le_1'
TOLERANCE = 1e-6
STRICT_REFERENCE_JUMP_M = .05
SITL_REFERENCE_JUMP_M = .075
C2_CANDIDATES = ('c2_admission', 'c2_transition')
HOLD_WITHOUT_OBSERVATION = ('cancelling', 'collision_stop')
ACTIVE_CERTIFICATES = ('brake', 'terminal_hold', 'observation_abort', 'fault')
CORRIDOR_CHECKS = frozenset({
    'full_curve_continuous_corridor', 'initial_stopping_reserve', 'remaining_continuous_corridor',
    'stopped_fixed_endpoint_and_tube', 'fixed_stopped_reference',
})
BUDGETS = dict(min_reaction_horizon_s=1.2, max_map_age_s=1.,
               max_tracking_error_m=.10, max_check_delay_bound_s=.20)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@contextmanager
def run_environment(config):
    updates = {**motion_environment(config), **voxel_environment(config)}
    old = {key: os.environ.get(key) for key in updates}
    os.environ.update(updates)
    try:
        yield
    finally:
        for key, value in old.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def affine_coefficients(c, lo, hi):
    """Power coefficients on a normalized subinterval, preserving the polynomial."""
    q = np.zeros_like(c, dtype=float)
    for k in range(len(c)):
        for j in range(k, len(c)):
            q[k] += c[j] * math.comb(j, k) * lo ** (j-k) * (hi-lo) ** k
    return q


def spans(curve, yaw=False):
    if not yaw:
        return curve.knots, curve.durations, curve.coefficients
    if curve.yaw_durations is None:
        return np.array([0., curve.duration]), np.array([curve.duration]), curve.yaw_coefficients[None, :, None]
    return curve.yaw_knots, curve.yaw_durations, curve.yaw_segments[:, :, None]


def prefix_errors(original, replacement, progress):
    """Exact-polynomial maxima of position/yaw derivative differences on prefix."""
    errors = dict(position=0., velocity=0., acceleration=0., yaw=0., yaw_rate=0., yaw_acceleration=0.)
    if progress == 0:
        return errors
    for yaw, names in [(False, ('position', 'velocity', 'acceleration')),
                       (True, ('yaw', 'yaw_rate', 'yaw_acceleration'))]:
        ka, ta, ca = spans(original, yaw)
        kb, tb, cb = spans(replacement, yaw)
        breaks = sorted(set([0., progress] + [float(t) for t in np.r_[ka, kb] if 0 < t < progress]))
        for start, end in zip(breaks[:-1], breaks[1:]):
            if end-start < 1e-12:
                continue
            mid = (start+end)/2
            ia = min(len(ta)-1, np.searchsorted(ka, mid, side='right')-1)
            ib = min(len(tb)-1, np.searchsorted(kb, mid, side='right')-1)
            a = affine_coefficients(ca[ia], (start-ka[ia])/ta[ia], (end-ka[ia])/ta[ia])
            b = affine_coefficients(cb[ib], (start-kb[ib])/tb[ib], (end-kb[ib])/tb[ib])
            for order, name in enumerate(names):
                errors[name] = max(errors[name], maximum_norm(derivative(a-b, order))/(end-start)**order)
    return errors


def boundary_errors(original, replacement, progress):
    a, b = original.sample(progress), replacement.sample(progress)
    return dict(position=float(np.linalg.norm(a[0]-b[0])), velocity=float(np.linalg.norm(a[1]-b[1])),
                acceleration=float(np.linalg.norm(a[2]-b[2])), yaw=abs(a[3]-b[3]),
                yaw_rate=abs(a[4]-b[4]), yaw_acceleration=abs(original.yaw_acceleration(progress)-replacement.yaw_acceleration(progress)))


def internal_c2_errors(curve):
    errors = dict(position=0., velocity=0., acceleration=0., yaw=0., yaw_rate=0., yaw_acceleration=0.)
    for yaw, names in [(False, ('position', 'velocity', 'acceleration')),
                       (True, ('yaw', 'yaw_rate', 'yaw_acceleration'))]:
        _, durations, coefficients = spans(curve, yaw)
        for i in range(len(durations)-1):
            for order, name in enumerate(names):
                left = np.polynomial.polynomial.polyval(1., derivative(coefficients[i], order))/durations[i]**order
                right = np.polynomial.polynomial.polyval(0., derivative(coefficients[i+1], order))/durations[i+1]**order
                errors[name] = max(errors[name], float(np.linalg.norm(left-right)))
    return errors


def terminal_errors(curve):
    end = curve.sample(curve.duration)
    return dict(velocity=float(np.linalg.norm(end[1])), acceleration=float(np.linalg.norm(end[2])),
                yaw_rate=abs(end[4]), yaw_acceleration=abs(curve.yaw_acceleration(curve.duration)))


def corridor_profile_available(profile):
    return profile == 'strict' or (profile is not None and profile in (
        getattr(execution_safety, 'SITL_RELAXED_PROFILE', None),
        getattr(execution_safety, 'SITL_FRONTIER_PROFILE', None),
    ))


def replay_corridor(runtime, native, margin, profile, *, at_terminal):
    if profile == 'strict':
        return ExecutionCorridor(runtime, native, margin=margin)
    frontier = profile == execution_safety.SITL_FRONTIER_PROFILE
    return ExecutionCorridor(runtime, native, margin=margin,
                             relaxed_sitl=not at_terminal, frontier_sitl=frontier)


def stopped_reference_valid(curve, progress, reference, actual, corridor):
    try:
        return certified_stopped_reference(curve, progress, reference, actual, corridor)['reference_stopped']
    except ValueError as error:
        if str(error) != 'terminal_root_no_longer_certified':
            raise
        # Continue checking deadline, token, and freshness even if the
        # policy-dependent terminal root failed the map query.
        return False


def c2_candidate_checks(result, curve, progress, actual, corridor):
    # The saved original is the candidate. Its root does not yet equal the
    # predecessor reference, and the snapshot cannot prove the switch applied.
    checks = result['checks']
    checks['piecewise_yaw'] = curve.yaw_durations is not None
    checks['candidate_terminal_stopped'] = sum(terminal_errors(curve).values()) <= 1e-5
    checks['tracking_within_budget'] = result['tracking_within_budget']
    if result['kind'] == 'c2_admission':
        checks['candidate_initial_progress'] = progress == 0.
    else:
        checks['candidate_transition_progress'] = 0. <= progress < curve.duration and progress <= .04+1e-12
        checks['next_reference_within_tracking_tube'] = (
            np.linalg.norm(curve.sample(progress)[0]-actual) <= execution_safety.TRACKING_MARGIN+TOLERANCE)
    now = result['decision_time']
    if now is not None:
        checks['candidate_map_age'] = all(
            0. <= now-result[key] <= execution_safety.MAX_MAP_AGE
            for key in ('native_stamp', 'mapper_stamp'))
    checks['candidate_continuous_corridor'] = corridor.safe_curve(curve, progress)
    try:
        stopping_reserve(curve, progress, corridor)
        checks['candidate_stopping_reserve'] = True
    except ValueError as error:
        checks['candidate_stopping_reserve'] = False
        result['reserve_error'] = str(error)
    result['handoff_binding_verified'] = False


def audit_certificate(path, *, profile='strict', verify_reserve_dynamics=False, static_handoff_default=None):
    if not corridor_profile_available(profile):
        raise ValueError('Unsupported certificate replay profile')
    result = dict(path=str(path), sha256=None, kind=path.stem.split('-', 1)[-1],
                  replay_profile=profile, checks={})
    try:
        raw = path.read_bytes()
        result['sha256'] = hashlib.sha256(raw).hexdigest()
        with np.load(BytesIO(raw), allow_pickle=False) as data:
            state = np.asarray(data['mapper_state'])
            native_state = np.asarray(data['native_state'])
            if state.ndim != 3 or native_state.ndim != 3 or not np.isin(state, [-1, 0, 1]).all():
                raise ValueError('Invalid saved map states')
            origin = np.asarray(data['mapper_origin'], float)
            resolution = float(data['mapper_resolution'])
            clearance = float(data['clearance'])
            limits = np.asarray(data['flight_limits'], float)
            margin = float(data['tracking_margin'])
            if (origin.shape != (3,) or limits.shape != (2,) or
                    not np.isfinite([*origin, resolution, clearance, *limits, margin]).all() or
                    resolution <= 0 or clearance <= 0 or margin < .10-TOLERANCE):
                raise ValueError('Invalid or weakened saved map geometry')
            runtime = VoxelMap([origin, origin+np.asarray(state.shape)*resolution],
                               resolution=resolution, clearance=clearance, flight_limits=limits)
            # Saved integer shape is authoritative; avoid floating ceil roundoff.
            runtime.shape = state.shape
            runtime.state = state.astype(np.int8).copy()
            runtime.rebuild()
            native = NativeSafetyMap()
            native.ingest([1., float(data['native_stamp']), float(data['native_sequence']),
                           float(data['native_resolution']), *np.asarray(data['native_origin']),
                           *native_state.shape, *native_state.ravel(order='C')])
            original = ContinuousTrajectory.from_dict(json.loads(str(data['original'].item())))
            replacement_text = str(data['replacement'].item())
            replacement = ContinuousTrajectory.from_dict(json.loads(replacement_text)) if replacement_text else None
            progress = float(data['progress'])
            actual, reference = np.asarray(data['actual'], float), np.asarray(data['reference'], float)
            if (not np.isfinite(progress) or not 0 <= progress <= original.duration or
                    actual.shape != (3,) or reference.shape != (3,) or not np.isfinite([*actual, *reference]).all()):
                raise ValueError('Invalid saved execution boundary')
            kind = result['kind']
            at_terminal = kind != 'activation' and progress >= original.duration
            held_without_observation = False
            if kind in ACTIVE_CERTIFICATES:
                static_handoff = data['static_handoff'].item() if 'static_handoff' in data else static_handoff_default
                if static_handoff is not None and type(static_handoff) is not bool:
                    raise ValueError('Invalid saved static handoff flag')
                result['static_handoff'] = static_handoff
            # C2 candidate validation and subsequent moving-handoff execution
            # use the strict corridor, even in a relaxed SITL run.
            if (kind in C2_CANDIDATES or result.get('static_handoff') is False or
                    (at_terminal and profile == getattr(execution_safety, 'SITL_RELAXED_PROFILE', None))):
                profile = 'strict'
            result['replay_profile'] = profile
            corridor = replay_corridor(runtime, native, margin, profile, at_terminal=at_terminal)
            result.update(progress=progress, original_duration=original.duration,
                          mapper_shape=list(state.shape), native_shape=list(native_state.shape),
                          collision_profile=runtime.collision_profile, native_stamp=native.stamp,
                          native_sequence=native.sequence, mapper_stamp=float(data['mapper_stamp']),
                          tracking_error_m=float(np.linalg.norm(actual-reference)), tracking_margin_m=margin)
            result['token'] = str(data['token'].item()) if 'token' in data else None
            result['decision_time'] = float(data['decision_time']) if 'decision_time' in data else None
            result['decision_binding_available'] = bool(result['token'] and
                isinstance(result['decision_time'], (int, float)) and math.isfinite(result['decision_time']))
            result['tracking_within_budget'] = result['tracking_error_m'] <= .10+TOLERANCE
            if result['kind'] == 'activation':
                root = original.sample(0.)[0]
                result['root_actual_error_m'] = float(np.linalg.norm(root-actual))
                result['root_reference_error_m'] = float(np.linalg.norm(root-reference))
                result['checks']['progress_is_zero'] = abs(progress) <= TOLERANCE
                result['checks']['root_actual_within_tracking_tube'] = result['root_actual_error_m'] <= .10+TOLERANCE
                reference_limit = STRICT_REFERENCE_JUMP_M if profile == 'strict' else SITL_REFERENCE_JUMP_M
                result['checks']['root_reference_discontinuity_bounded'] = result['root_reference_error_m'] <= reference_limit+TOLERANCE
                result['checks']['full_curve_continuous_corridor'] = corridor.safe_curve(original, 0.)
                result['checks']['tracking_within_budget'] = result['tracking_within_budget']
                try:
                    reserve, boundary = stopping_reserve(original, 0., corridor, horizon=1.2)
                    result['checks']['initial_stopping_reserve'] = True
                    result['reserve_boundary'] = boundary
                    result['reserve_duration'] = reserve.duration
                except ValueError as error:
                    result['checks']['initial_stopping_reserve'] = False
                    result['reserve_error'] = str(error)
                    if verify_reserve_dynamics:
                        # An unavailable policy can mask a map disagreement,
                        # but not a stop that is impossible without obstacles.
                        unchecked_geometry = SimpleNamespace(safe_curve=lambda *_: True)
                        try:
                            stopping_reserve(original, 0., unchecked_geometry, horizon=1.2)
                            result['checks']['initial_reserve_dynamics_feasible'] = True
                        except ValueError:
                            result['checks']['initial_reserve_dynamics_feasible'] = False
            elif kind in C2_CANDIDATES:
                c2_candidate_checks(result, original, progress, actual, corridor)
            elif result['kind'] == 'brake':
                if replacement is None or progress >= min(original.duration, replacement.duration):
                    raise ValueError('Brake certificate has no complete replacement')
                result['replacement_duration'] = replacement.duration
                result['checks']['remaining_continuous_corridor'] = corridor.safe_curve(replacement, progress)
                result['checks']['tracking_within_budget'] = result['tracking_within_budget']
                result['prefix_errors'] = prefix_errors(original, replacement, progress)
                result['boundary_errors'] = boundary_errors(original, replacement, progress)
                result['internal_c2_errors'] = internal_c2_errors(replacement)
                result['terminal_errors'] = terminal_errors(replacement)
                for key in ('prefix', 'boundary', 'internal_c2', 'terminal'):
                    result['checks'][key] = max(result[key+'_errors'].values()) <= TOLERANCE
            elif result['kind']=='terminal_hold':
                hold=json.loads(str(data['terminal_hold'].item()))
                if not isinstance(hold,dict):raise ValueError('missing terminal hold binding')
                now = result['decision_time']
                result['terminal_hold']=hold
                result['checks']['stopped_fixed_endpoint_and_tube']=stopped_reference_valid(original,progress,reference,actual,corridor)
                result['checks']['receipt_not_fabricated']=hold['observation_complete'] is False
                held_without_observation = hold.get('task_state') in HOLD_WITHOUT_OBSERVATION
                if held_without_observation:
                    stated_endpoint = np.asarray(hold['fixed_endpoint'], float)
                    result['checks']['recorded_hold_matches_endpoint'] = bool(
                        hold['reference_stopped'] is True and stated_endpoint.shape == (3,) and
                        np.linalg.norm(stated_endpoint-reference) <= TOLERANCE)
                else:
                    since, wait_s = hold['started_at'], hold['wait_budget_s']
                    deadline = since + wait_s
                    result['checks']['finite_original_deadline']=bool(
                        np.isfinite([now, since, wait_s, deadline]).all() and wait_s > 0 and
                        since <= now <= deadline and abs(hold['absolute_deadline']-deadline)<=TOLERANCE)
                    result['checks']['original_observation_budget']=wait_s==1.5
                delay=hold['check_wall_s']
                native_age=result['decision_time']-native.stamp+delay
                mapper_age=result['decision_time']-result['mapper_stamp']+delay
                result['checks']['delay_and_freshness']=bool(0<=delay<=.2 and
                    -.05<=native_age<=1. and -.05<=mapper_age<=1. and
                    abs(hold['native_input_age_s']-native_age)<=TOLERANCE and
                    abs(hold['mapper_input_age_s']-mapper_age)<=TOLERANCE)
            elif result['kind']=='observation_abort':
                hold=json.loads(str(data['terminal_hold'].item()))
                failure=json.loads(str(data['observation_failure'].item()))
                delay=hold['check_wall_s'];now=result['decision_time']
                result['observation_failure']=failure
                result['checks']['fixed_stopped_reference']=stopped_reference_valid(original,progress,reference,actual,corridor)
                result['checks']['bound_to_failed_token']=failure['token']==result['token'] and failure['outcome']=='unobserved'
                result['checks']['wait_closed_without_extension']=bool(failure['closed_at']==now and now>failure['observation_deadline'] and abs(failure['observation_deadline']-failure['started_at']-1.5)<TOLERANCE)
                count = failure['mission_failure_count']
                result['checks']['valid_failure_count'] = type(count) is int and count >= 1
                if 'replan_deadline' in failure:
                    result['checks']['legacy_native_admission'] = (
                        abs(failure['replan_deadline']-now-6.) < TOLERANCE and count <= 3)
                result['checks']['no_observation_credit']=hold['observation_complete'] is False and hold['task_state']=='failed'
                result['checks']['delay_and_freshness']=bool(0<=delay<=.2 and -.05<=now-native.stamp+delay<=1. and -.05<=now-result['mapper_stamp']+delay<=1.)
            elif result['kind'] == 'fault':
                result['remaining_original_corridor_safe'] = corridor.safe_curve(original, progress)
                result['status'] = 'recorded_safety_fault_not_success'
                return result
            else:
                raise ValueError('Unrecognized certificate kind')
            result['status'] = ('failed' if not all(result['checks'].values()) else
                                'candidate_partial_evidence' if kind in C2_CANDIDATES else
                                'hold_partial_evidence' if held_without_observation else 'passed')
    except (ValueError, KeyError, TypeError, OSError, OverflowError, AttributeError, IndexError,
            BadZipFile, EOFError, ZlibError) as error:
        result['status'] = 'failed'
        result['error'] = str(error)
    return result


def audit_unknown_corridor(path):
    certificate = audit_certificate(path, verify_reserve_dynamics=True)
    if (certificate['status'] == 'recorded_safety_fault_not_success' or
            certificate['kind'] in C2_CANDIDATES or certificate.get('static_handoff') is False):
        return certificate
    affected = set(CORRIDOR_CHECKS)
    if certificate.get('root_reference_error_m', math.inf) <= SITL_REFERENCE_JUMP_M + TOLERANCE:
        affected.add('root_reference_discontinuity_bounded')
    checks = certificate['checks']
    certificate['strict_reconstruction_status'] = certificate['status']
    failed = {key for key, ok in checks.items() if not ok}
    certificate['unverified_corridor_checks'] = sorted(failed & affected)
    if failed and failed <= affected and not certificate.get('error'):
        certificate['status'] = 'unverified_corridor'
    return certificate


def audit_profiled_certificate(path, profile, *, moving_c2_disabled):
    certificate = audit_certificate(path, profile=profile,
        static_handoff_default=True if moving_c2_disabled else None)
    if (certificate['replay_profile'] == 'strict' or 'error' in certificate or
            certificate['kind'] not in ACTIVE_CERTIFICATES
            or certificate['kind'] == 'fault' or certificate['static_handoff'] is not None):
        return certificate

    # Legacy files do not say whether the active token began stationary. Hold
    # a finding only if the recorded geometry leads to the same conclusion
    # under the configured context and the strict moving context.
    strict = audit_certificate(path)
    differences = sorted(key for key in CORRIDOR_CHECKS & strict['checks'].keys()
                         if strict['checks'][key] != certificate['checks'][key])
    if not differences or 'error' in strict:
        return strict
    strict['alternate_sitl_reconstruction'] = {
        'profile': profile, 'checks': {key: certificate['checks'][key] for key in differences}}
    strict['strict_reconstruction_status'] = strict['status']
    strict['unverified_corridor_checks'] = differences
    if all(ok for key, ok in strict['checks'].items() if key not in differences):
        strict['status'] = 'unverified_corridor'
    return strict


def jsonl(path):
    rows, errors = [], []
    if not path.is_file():
        return rows, ['missing: '+str(path)]
    with path.open() as stream:
        for number, line in enumerate(stream, 1):
            try:
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise ValueError('Expected object')
                rows.append(value)
            except (ValueError, TypeError) as error:
                errors.append(dict(file=str(path), line=number, error=str(error)))
    return rows, errors


def timing_contract(config):
    contract = config.get('execution_safety_timing_contract')
    issues = []
    if not isinstance(contract, dict):
        issues.append('missing execution_safety_timing_contract')
    else:
        for key in ('physics_real_time_factor', 'max_sim_to_wall_rate'):
            value = contract.get(key)
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not 0 < value <= 1.):
                issues.append(key+': must be finite and in (0,1]')
        if contract.get('check_clock') != CHECK_CLOCK:
            issues.append('check_clock: unsupported or missing')
        rtf, rate = contract.get('physics_real_time_factor'), contract.get('max_sim_to_wall_rate')
        if isinstance(rtf, (int, float)) and isinstance(rate, (int, float)) and rtf > rate:
            issues.append('physics_real_time_factor exceeds declared max_sim_to_wall_rate')
    return dict(declared=contract, valid=not issues, issues=issues,
                evidence='Configured run timing contract; wall elapsed bounds simulation elapsed under this contract, not a direct simulation-duration measurement.')


def audit_reserve_telemetry(rows, contract=None):
    result = dict(execution_rows=0, observed_unique_checks=0, violations=[],
                  missing_reserve_rows=[], continuing_rows=0, continuing_rows_with_recent_reserve=0,
                  safety_faults=[], execution_report_gaps=[], brake_certificate_references=[],
                  delay_evidence_modes={}, delay_bound_condition='wall time upper-bounds simulated elapsed only when run RTF <= 1',
                  measured_simulated_check_duration_available=False)
    seen, previous = set(), {}
    requirements = {'reaction_horizon_s': (1.2, float('inf')), 'native_input_age_s': (-.05, 1.),
                    'mapper_input_age_s': (-.05, 1.), 'tracking_error_m': (0., .10)}
    checked_values = {key: [] for key in (*requirements, 'check_delay_bound_s', 'check_wall_s', 'legacy_check_sim_s')}

    def check(field, minimum, maximum, range_key=None):
        value = reserve.get(field)
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            issues.append(field+': missing/nonfinite')
            return None
        checked_values[range_key or field].append(value)
        if not minimum-TOLERANCE <= value <= maximum+TOLERANCE:
            issues.append(field+': outside budget')
        return value

    for record in rows:
        row = record.get('data')
        if record.get('kind') != 'execution' or not isinstance(row, dict):
            continue
        result['execution_rows'] += 1
        time = row.get('time')
        token = row.get('token')
        drone = row.get('drone', 0)
        reserve = row.get('stopping_reserve')
        revision = row.get('safety_revision')
        reported_error = row.get('tracking_error')
        if token and reported_error is not None and (not isinstance(reported_error, (int, float)) or
                not 0. <= reported_error <= .10+TOLERANCE):
            result['violations'].append(dict(time=time, token=token,
                issues=['reported tracking_error: outside tracking tube'], tracking_error=reported_error))
        if row.get('safety_fault'):
            item = dict(time=time, token=token, fault=row['safety_fault'])
            if not result['safety_faults'] or result['safety_faults'][-1]['fault'] != item['fault']:
                result['safety_faults'].append(item)
        if isinstance(revision, dict) and revision.get('certificate'):
            name = Path(revision['certificate']).name
            if name not in result['brake_certificate_references']:
                result['brake_certificate_references'].append(name)
        if isinstance(reserve, dict):
            key = (drone, token, reserve.get('checked_at'))
            if key not in seen:
                seen.add(key)
                result['observed_unique_checks'] += 1
                issues = []
                for field, limits in requirements.items():
                    check(field, *limits)
                if 'check_delay_bound_s' in reserve or 'check_wall_s' in reserve:
                    mode = 'monotonic_wall_bound_conditional_on_rtf'
                    if reserve.get('check_clock') != CHECK_CLOCK:
                        issues.append('check_clock: unsupported or missing wall-bound clock')
                    bound = check('check_delay_bound_s', 0., .20)
                    wall = check('check_wall_s', 0., .20)
                    if bound is not None and wall is not None and bound < wall-TOLERANCE:
                        issues.append('check_delay_bound_s: smaller than measured wall elapsed')
                    if reserve.get('check_sim_s') is not None:
                        issues.append('check_sim_s: wall-bound mode must not claim measured simulation elapsed')
                else:
                    mode = 'legacy_same_callback_sim_clock_not_elapsed_proof'
                    check('check_sim_s', 0., .20, 'legacy_check_sim_s')
                result['delay_evidence_modes'][mode] = result['delay_evidence_modes'].get(mode, 0)+1
                if issues:
                    result['violations'].append(dict(time=time, token=token, checked_at=reserve.get('checked_at'), issues=issues, reserve=reserve))
        moving = bool(token and not row.get('arrived') and not row.get('safety_fault') and not revision and
                      row.get('trajectory_time', 0.) < row.get('trajectory_duration', 0.)-TOLERANCE)
        if moving:
            result['continuing_rows'] += 1
            if (isinstance(reserve, dict) and isinstance(time, (int, float)) and
                    isinstance(reserve.get('checked_at'), (int, float)) and
                    -.05 <= time-reserve['checked_at'] <= .30+TOLERANCE):
                result['continuing_rows_with_recent_reserve'] += 1
            else:
                result['missing_reserve_rows'].append(dict(time=time, token=token,
                    reason='missing_or_more_than_0.30s_old_sampled_reserve'))
            old = previous.get((drone, token))
            if old is not None and isinstance(time, (int, float)) and time-old > .35:
                result['execution_report_gaps'].append(dict(token=token, start=old, end=time, seconds=time-old))
            previous[drone, token] = time
    result['observed_ranges'] = {key: dict(min=min(values), max=max(values)) if values else None for key, values in checked_values.items()}
    result['all_observed_numeric_budgets_pass'] = bool(seen) and not result['violations']
    valid_contract = bool(contract and contract.get('valid'))
    result['delay_bound_verified'] = bool(seen) and valid_contract and not result['violations'] and set(result['delay_evidence_modes']) == {'monotonic_wall_bound_conditional_on_rtf'}
    result['all_observed_reserve_budgets_pass'] = False if result['violations'] else (True if result['delay_bound_verified'] else None)
    result['delay_bound_verification_basis'] = 'configured RTF contract plus recorded monotonic wall durations' if result['delay_bound_verified'] else 'unverified'
    result['coverage_note'] = 'Execution telemetry samples the latest successful check; it is not a journal of every control tick or every reserve computation. Recent means <=0.30s only for identifying sampled coverage gaps, not a relaxed execution/check budget.'
    return result


def audit_run(run):
    run = Path(run)
    config_path = run/'runtime.json'
    config = json.loads(config_path.read_text()) if config_path.is_file() else {}
    completion_path=run/'result.json'
    completion=json.loads(completion_path.read_text()) if completion_path.is_file() else {}
    certificates = sorted((run/'execution-safety').glob('*.npz'))
    telemetry, read_errors = jsonl(run/'telemetry.jsonl')
    result = dict(schema='racer.stage23.execution-safety-audit/4', run=str(run),
                  scope='Saved online map decisions and sampled execution telemetry only; no truth input and no full-flight safety certification.',
                  budgets=BUDGETS, tolerance=TOLERANCE, full_flight_certified=False,
                  run_error=completion.get('error'),
                  run_result_sha256=sha256(completion_path) if completion_path.is_file() else None,
                  runtime_sha256=sha256(config_path) if config_path.is_file() else None,
                  telemetry_sha256=sha256(run/'telemetry.jsonl') if (run/'telemetry.jsonl').is_file() else None,
                  read_errors=read_errors, coverage_gaps=[], certificates=[])
    if not config_path.is_file():
        result['coverage_gaps'].append('runtime.json missing; legacy environment defaults used and run configuration equivalence unproven')
    profile = config.get('execution_safety_profile', 'strict') if config_path.is_file() else None
    result['configured_corridor_profile'] = profile
    matched_profile = corridor_profile_available(profile)
    # The two SITL source trees can use different frontier padding. A
    # different recorded implementation cannot be inferred from the profile name.
    safety_source = 'src/annual_planner/src/core/planning/execution_safety.py'
    recorded_hashes = completion.get('code_sha256')
    recorded_source = recorded_hashes.get(safety_source) if isinstance(recorded_hashes, dict) else None
    if profile == 'sitl_frontier' and recorded_source != sha256(ROOT/safety_source):
        matched_profile = False
    if certificates and not matched_profile:
        result['coverage_gaps'].append(
            'online corridor unavailable for replay; policy-dependent checks show a strict reconstruction only')
    if read_errors:
        result['coverage_gaps'].append('telemetry missing or malformed rows; see read_errors')
    with run_environment(config):
        for path in certificates:
            certificate = (audit_profiled_certificate(path, profile,
                                moving_c2_disabled=config.get('static_moving_c2') is False)
                           if matched_profile else audit_unknown_corridor(path))
            result['certificates'].append(certificate)
    result['timing_contract'] = timing_contract(config)
    result['telemetry'] = audit_reserve_telemetry(telemetry, result['timing_contract'])
    failed = [c['path'] for c in result['certificates'] if c['status'] == 'failed']
    faults = [c['path'] for c in result['certificates'] if c['kind'] == 'fault']
    result['certificate_counts'] = {kind: sum(c['kind'] == kind for c in result['certificates'])
                                    for kind in ('activation', *ACTIVE_CERTIFICATES, *C2_CANDIDATES)}
    result['failed_certificates'] = failed
    result['observed_safety_fault'] = bool(faults or result['telemetry']['safety_faults'])
    if not certificates:
        result['coverage_gaps'].append('no certificates: no online-map continuous decision can be reconstructed')
    if any(result['certificate_counts'][kind] for kind in C2_CANDIDATES):
        result['coverage_gaps'].append(
            'C2 certificates retain candidate geometry; predecessor binding, final budget and applied transition require separate evidence')
    if any(c.get('terminal_hold', {}).get('task_state') in HOLD_WITHOUT_OBSERVATION
           for c in result['certificates']):
        result['coverage_gaps'].append(
            'cancellation and collision holds retain the stopped reference; request and terminal receipt require separate evidence')
    if any('alternate_sitl_reconstruction' in c for c in result['certificates']):
        result['coverage_gaps'].append(
            'legacy active certificates with differing strict and SITL checks lack a saved static handoff context')
    if not result['certificate_counts']['activation']:
        result['coverage_gaps'].append('no activation map certificate')
    if not result['certificate_counts']['brake']:
        result['coverage_gaps'].append('no brake map certificate: no observed brake C2/corridor validation')
    available = {p.name for p in certificates}
    for reference in result['telemetry']['brake_certificate_references']:
        if reference not in available:
            result['coverage_gaps'].append('telemetry references missing brake certificate: '+reference)
    if result['telemetry']['missing_reserve_rows']:
        result['coverage_gaps'].append('continuing execution telemetry includes absent or stale sampled reserve statuses')
    if not result['telemetry']['observed_unique_checks']:
        result['coverage_gaps'].append('no stopping_reserve telemetry')
    if result['telemetry']['delay_evidence_modes'].get('legacy_same_callback_sim_clock_not_elapsed_proof'):
        result['coverage_gaps'].append('legacy check_sim_s is a same-callback ROS-clock subtraction, not valid elapsed-time evidence under the default single-threaded executor')
    if (result['telemetry']['delay_evidence_modes'].get('monotonic_wall_bound_conditional_on_rtf') and
            not result['timing_contract']['valid']):
        result['coverage_gaps'].append('wall-time bound is conditional on RTF <= 1; missing or invalid runtime timing contract prevents accepting that bound')
    if result['telemetry']['execution_report_gaps']:
        result['coverage_gaps'].append('gaps between sampled execution reports')
    if any(not c.get('decision_binding_available') for c in result['certificates']):
        result['coverage_gaps'].append('some certificate formats lack token or decision timestamp; those decisions cannot be bound to execution tokens or used alone for input-age validation')
    result['coverage_gaps'].extend([
        'saved token/decision bindings identify individual decisions; they do not establish that every activation or map update was recorded',
        'stopping_reserve maps between saved decisions cannot be reconstructed',
        'telemetry is sampled, so unrecorded checks and between-report tracking/map-age excursions are not certified',
        'single-aircraft certificate contains no peer reservations; this audit does not certify multi-aircraft execution'])
    result['status'] = ('execution_safety_fault' if result['observed_safety_fault'] else
                        'run_failed' if result['run_error'] else
                        'violations' if failed or result['telemetry']['violations'] else
                        'no_certificates' if not certificates else 'partial_evidence')
    result['auditor_sha256'] = sha256(__file__)
    result['source_sha256'] = {name: sha256(ROOT/'src/annual_planner/src'/name) for name in
        ['core/planning/execution_safety.py', 'core/planning/continuous_trajectory.py', 'core/exploration/voxel_mapping.py']}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output == ROOT or ROOT in output.parents:
        raise ValueError('Audit output must be outside the source tree')
    report = audit_run(args.run)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output), status=report['status'], certificate_counts=report['certificate_counts'],
                          reserve_checks=report['telemetry']['observed_unique_checks'], full_flight_certified=False)))


if __name__ == '__main__':
    main()
