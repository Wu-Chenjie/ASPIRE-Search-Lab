"""Independent saved-evidence audit and counterfactual old-frame replay.

No ROS, simulator, truth or control inputs. Replay reuses recorded detections;
it neither reruns the detector nor predicts a new flight or a new review path.
"""
import argparse
import bisect
import collections
from dataclasses import asdict
from contextlib import contextmanager
from io import BytesIO
import hashlib
import json
import math
from pathlib import Path
import sys
from zipfile import BadZipFile
from zlib import error as ZlibError

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / 'src') not in sys.path:
    sys.path.insert(0, str(ROOT / 'src'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_text(run, name, hashes, *, required=False):
    path = run/name
    if required or path.is_file():
        raw = path.read_bytes()
        hashes[name] = hashlib.sha256(raw).hexdigest()
        return raw.decode()
    return None


def close(a, b, tolerance):
    try:
        a, b = np.asarray(a, float), np.asarray(b, float)
        return bool(a.shape == b.shape == (3,) and np.isfinite([a, b]).all()
                    and np.linalg.norm(a-b) <= tolerance)
    except (TypeError, ValueError):
        return False


def rows(text, *, allow_blank=False):
    return [json.loads(line) for line in text.splitlines() if not allow_blank or line.strip()]


def log_packets(text, marker, *, allow_malformed=False):
    for line in text.splitlines():
        if marker in line:
            try:
                yield json.loads(line.split(marker, 1)[1])
            except ValueError:
                if not allow_malformed:
                    raise


def fresh_timings(observations, ns):
    return [r for r in observations if r.get('kind') == 'timing'
            and r.get('stamp_ns') == ns and r.get('status') == 'accepted'
            and 0 <= (r.get('now_ns', -1)-ns)*1e-9 <= .75
            and 0 <= r.get('received_age_wall_s', math.inf) <= .75]


@contextmanager
def capture_array(path, sources):
    archive = path
    if sources is not None:
        raw = path.read_bytes()
        archive = BytesIO(raw)
    with np.load(archive, allow_pickle=False) as data:
        yield data
    if sources is not None:
        sources[path.name] = hashlib.sha256(raw).hexdigest()


def require_capture_stamp(data, ns, kind):
    if 'stamp_ns' not in data:
        raise ValueError(f'{kind}_capture_stamp_missing')
    stamp = data['stamp_ns']
    if stamp.size == 1:
        value = stamp.item()
        if stamp.dtype.kind in 'iu' and value == ns:
            return
        if stamp.dtype.kind == 'f':
            # Both adjacent nanoseconds must still be distinguishable.
            exact_limit = 2**(int(np.finfo(stamp.dtype).nmant)+1)
            if np.isfinite(value) and abs(value) < exact_limit and value == ns:
                return
    raise ValueError(f'{kind}_capture_stamp_mismatch')


def reconstruct(run, ns, *, hash_sources=True):
    """Reproduce a saved pair or retain semantic evidence for quality replay."""
    root = Path(run)/'perception'
    sources = {} if hash_sources else None
    origin, detections, semantic, localized, errors = None, [], [], [], []
    try:
        from person_search.perception import validate_transform
        from person_search.segmentation import PersonMask, localize_masks
        with capture_array(root/f'{ns}-rgb-masks.npz', sources) as data:
            require_capture_stamp(data, ns, 'mask')
            boxes, scores, masks = data['boxes'], data['scores'], data['masks']
            if not len(boxes) == len(scores) == len(masks):
                raise ValueError('mask_array_lengths_disagree')
            detections = [PersonMask(tuple(box), float(score), mask)
                          for box, score, mask in zip(boxes, scores, masks)]
        with capture_array(root/f'{ns}-geometry.npz', sources) as data:
            require_capture_stamp(data, ns, 'geometry')
            transform = validate_transform(data['camera_to_map'])
            origin = transform[:3, 3].tolist()
            semantic, localized = localize_masks(detections, data['depth'], data['k'], transform)
    except (OSError, ValueError, KeyError, TypeError, ImportError, EOFError, BadZipFile, ZlibError) as error:
        if sources is not None and detections and not semantic:
            try:
                semantic, _ = localize_masks(detections, None, None, None)
            except (ValueError, TypeError) as mask_error:
                error = mask_error
        errors.append(type(error).__name__+':'+str(error))
    return dict(stamp_ns=ns, errors=errors, sources=sources or {}, origin=origin, semantic=semantic,
                positions=[c.xyz.tolist() for c in localized]), localized


def distribution(values):
    values = np.asarray(values, float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(count=len(values), minimum=float(values.min()),
                median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                maximum=float(values.max()))


def provenance(run, hashes):
    names = ('result.json', 'runtime.json', 'perception/observations.jsonl',
             'telemetry.jsonl', 'flight/0/events.jsonl',
             'native-racer/exploration_node_1.log', 'person_search.log')
    return {name: hashes[name] if name in hashes else digest(run/name)
            for name in names if name in hashes or (run/name).is_file()}


def quality_policy_provenance(run, hashes):
    """Show the current replay/audit proxy separately from the recorded source."""
    from person_search.confirmation_quality import ConfirmationQuality, QUALITY_PROXY_VERSION

    paths = ('src/person_search/active_confirmation.py', 'src/person_search/segmentation.py',
             'src/person_search/confirmation_quality.py')
    current = {rel:digest(ROOT/rel) for rel in paths}
    result_text = source_text(run, 'result.json', hashes)
    result = json.loads(result_text) if result_text is not None else {}
    recorded = result.get('code_sha256', {})
    return dict(current_proxy_version=QUALITY_PROXY_VERSION,
                current_threshold_fields=sorted(asdict(ConfirmationQuality())),
                current_implementation_sha256=current,
                recorded_implementation_sha256={rel:recorded.get(rel) for rel in paths},
                source_matches={rel:current[rel] == recorded[rel] if rel in recorded else None for rel in paths},
                interpretation='Current proxy re-evaluation; old as-run reports remain unchanged. Missing or different logged proxy/schema cannot pass current-standard confirmation audit.')


def logged_value_matches(expected, actual):
    """Quality diagnostics may contain version strings or unavailable values."""
    if isinstance(expected, bool) or expected is None:
        return actual is expected
    if isinstance(expected, str):
        return isinstance(actual, str) and actual == expected
    if isinstance(expected, (int, float)):
        return (isinstance(actual, (int, float)) and not isinstance(actual, bool)
                and math.isfinite(expected) and math.isfinite(actual) and abs(expected-actual) <= 1e-6)
    return False


def initial_confirmations(events):
    """Count each algorithm track once; updates are not new confirmations."""
    initial_reasons = {'reliable_direct_confirmed', 'direct_visible_confirmed', 'active_multiview_confirmed'}
    first, duplicates, unknown = {}, [], []
    for event in events:
        if event.get('state') != 'confirmed':
            continue
        if event.get('reason') not in initial_reasons:
            if event.get('reason') != 'confirmed_position_frozen':
                unknown.append(event)
            continue
        track = event.get('track')
        if track in first:
            duplicates.append(dict(track=track, first_time=first[track].get('time'), duplicate_event=event))
        else:
            first[track] = event
    return list(first.values()), duplicates, unknown


def replay_old(run, quality=None):
    """Direct-confirmation counterfactual, never replay historical review success.

    Processing time comes from accepted timing records; execution reports are
    selected by recorded simulation timestamp. This is not an exact ROS/DDS
    scheduling replay. Missing/rejected intervening captures break continuity.
    """
    from person_search.active_confirmation import ActiveConfirmationTracker
    from person_search.confirmation_quality import ConfirmationQuality

    run = Path(run)
    quality = quality or ConfirmationQuality()
    hashes = {}
    observations = rows(source_text(run, 'perception/observations.jsonl', hashes, required=True), allow_blank=True)
    telemetry = rows(source_text(run, 'telemetry.jsonl', hashes, required=True), allow_blank=True)
    execution = sorted((r['data'] for r in telemetry if r.get('kind') == 'execution'),
                       key=lambda r:r['time'])
    execution_times = [r['time'] for r in execution]
    stamps = {int(p.name.split('-', 1)[0]) for p in (run/'perception').glob('*-rgb-masks.npz')}
    timings = collections.defaultdict(list)
    for row in observations:
        if row.get('kind') in ('semantic', 'timing') and row.get('stamp_ns', -1) >= 0:
            stamps.add(row['stamp_ns'])
            if row['kind'] == 'timing':
                timings[row['stamp_ns']].append(row)
    tracker = ActiveConfirmationTracker(mode='hybrid', quality=quality)
    frames, events = [], []
    counts, gate_counts, qualities = collections.Counter(), collections.Counter(), collections.defaultdict(list)
    for ns in sorted(stamps):
        captured = ns*1e-9
        accepted = fresh_timings(timings.get(ns, ()), ns)
        now = min((r['now_ns']*1e-9 for r in accepted), default=captured)
        index = bisect.bisect_right(execution_times, now)-1
        tracker.advance(now, execution[index] if index >= 0 else {})
        frame, localized = reconstruct(run, ns)
        frame['accepted_fresh_timing'] = bool(accepted)
        frame['capture_time_s'] = captured
        frame['processing_time_s'] = now
        counts['capture_timestamps'] += 1
        counts['semantic_candidate_frames'] += bool(frame['semantic'])
        counts['localized_frames'] += bool(localized)
        counts['missing_or_invalid_saved_pair'] += bool(frame['errors'])
        frame['candidate_gates'] = []
        for candidate in localized:
            reasons, evidence = quality.evaluate(candidate)
            gate_counts.update(reasons)
            frame['candidate_gates'].append(dict(xyz=candidate.xyz.tolist(), rejections=reasons,
                                                quality=evidence, quality_pass=not reasons))
            for key, value in evidence.items():
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    qualities[key].append(value)
        tracker.semantic_memory.update(frame['semantic'], captured)
        if frame['errors'] or not accepted:
            tracker.reject_capture(captured, 'unavailable_saved_rgbd' if frame['errors'] else 'no_accepted_fresh_timing')
            counts['rejected_capture_frames'] += 1
        else:
            tracker.update(localized, frame['origin'], captured)
            counts['accepted_capture_frames'] += 1
        events.extend(tracker.events)
        tracker.events.clear()
        # Empty frames still count and break continuity; avoid retaining their
        # irrelevant array hashes in a large human-review report.
        if frame['semantic'] or frame['errors']:
            frames.append(frame)
    confirmations, duplicates, unknown = initial_confirmations(events)
    policy = quality_policy_provenance(run, hashes)
    return dict(mode='old_recorded_frame_replay', run=str(run),
                scope='Offline direct-confirmation counterfactual on recorded masks/RGB-D; NOT new flight, detector inference, active-review execution or search-effectiveness evidence.',
                thresholds=asdict(quality), counts=dict(counts),
                quality_distributions={k:distribution(v) for k, v in qualities.items()},
                quality_gate_rejections=dict(gate_counts),
                event_reason_counts=dict(collections.Counter(e['reason'] for e in events)),
                rejection_reason_counts=dict(collections.Counter(r for e in events for r in e.get('rejections', []))),
                confirmations=confirmations, confirmation_count=len(confirmations),
                confirmation_count_scope='Unique algorithm track IDs with an explicit initial confirmation event; not distinct real people.',
                duplicate_initial_confirmation_events=duplicates,
                unrecognized_confirmed_events=unknown,
                event_integrity_passed=not duplicates and not unknown,
                final_tracks=[dict(id=t.id, state=t.state, reason=t.reason, xyz=t.xyz.tolist()) for t in tracker.tracks],
                frames=frames, events=events, source_sha256=provenance(run, hashes),
                quality_policy=policy, implementation_sha256=policy['current_implementation_sha256'],
                limitations=['Only stored detections are reused; the detector is not rerun.',
                             'Historical paths remain fixed; no new request or counterfactual movement is issued.',
                             'Receipt ordering is approximated by recorded simulation times, not reproduced DDS scheduling.',
                             'Confirmation is an algorithm decision; no truth is read and this report does not count real people.',
                             'The exterior mask ring is a depth proxy, not semantic proof of a complete unoccluded body or a true person.'])


def capture_checks(run, confirmed, quality, observations, telemetry):
    times = confirmed.get('capture_times', confirmed.get('post_capture_times', []))
    evidence = confirmed.get('evidence', [])
    reports, errors, points = [], [], []
    try:
        times = [float(t) for t in times]
        ordered = bool(len(times) >= quality.min_frames and np.isfinite(times).all()
                       and all(a < b for a, b in zip(times, times[1:]))
                       and times[-1]-times[0] >= quality.min_span_s-1e-9
                       and max(np.diff(times), default=math.inf) <= quality.max_gap_s+1e-9
                       and abs(times[-1]-confirmed['time']) <= 1e-6)
    except (ValueError, TypeError, KeyError):
        ordered, times = False, []
    if not ordered:
        errors.append('insufficient_or_unordered_capture_times')
    if ordered:
        used_ns = {round(t*1e9) for t in times}
        intermediate = {r['stamp_ns'] for r in observations if r.get('kind') in ('semantic', 'timing')
                        and times[0]*1e9 <= r.get('stamp_ns', -1) <= times[-1]*1e9}
        if intermediate-used_ns:
            errors.append('omitted_intervening_capture_breaks_continuity')
    for stamp in times:
        ns = round(stamp*1e9)
        report, localized = reconstruct(run, ns)
        frame_errors = report['errors']
        if not fresh_timings(observations, ns):
            frame_errors.append('missing_accepted_fresh_capture_timing')
        logged = [r for r in evidence if abs(r.get('time', -1e9)-stamp) <= 1e-6]
        record = logged[0] if len(logged) == 1 else {}
        matched = [c for c in localized if close(c.xyz, record.get('xyz'), 1e-6)]
        candidate = matched[0] if len(matched) == 1 else None
        if candidate is None:
            frame_errors.append('missing_unique_reproduced_logged_position')
        else:
            rejections, computed = quality.evaluate(candidate)
            frame_errors.extend(rejections)
            report['recomputed_quality'] = computed
            logged_quality = record.get('quality', {})
            for key, value in computed.items():
                if key not in logged_quality or not logged_value_matches(value, logged_quality[key]):
                    frame_errors.append('quality_evidence_mismatch:'+key)
            points.append(candidate.xyz)
        if not close(record.get('origin'), report['origin'], 1e-6):
            frame_errors.append('capture_origin_mismatch')
        packets = [r['data'] for r in telemetry if r.get('kind') == 'tracks' and r['data'].get('stamp_ns') == ns]
        same_track = [t for p in packets for t in p.get('tracks', [])
                      if t.get('id') == confirmed.get('track') and candidate is not None
                      and close(t.get('xyz'), candidate.xyz, .65)]
        if not same_track:
            frame_errors.append('missing_same_capture_track_association')
        execution = [r['data'] for r in telemetry if r.get('kind') == 'execution'
                     and r['data'].get('time', math.inf) <= stamp]
        report['capture_before_ready'] = not any(e.get('ready') for e in execution)
        if execution and max(execution, key=lambda e:e['time']).get('flight_session_fault'):
            frame_errors.append('capture_after_reported_flight_fault')
        report['passed'] = not frame_errors
        reports.append(report)
    deviation = None
    if len(points) == len(times) and points:
        deviation = float(np.max(np.linalg.norm(np.array(points)-np.median(points, axis=0), axis=1)))
        if deviation > quality.max_position_deviation_m+1e-9:
            errors.append('position_stability_gate')
    else:
        errors.append('missing_complete_position_series')
    return dict(captures=reports, position_deviation_m=deviation, errors=errors,
                passed=bool(reports) and not errors and all(r['passed'] for r in reports))


def review_chain(confirmed, reviews, statuses, requests, flight, telemetry):
    """Join every segment to native IDs, actual executor tokens and proofs."""
    rid = confirmed.get('request_id')
    errors, segments = [], []
    request_events = [r for r in reviews if r.get('reason') == 'visual_candidate_requested_safe_view'
                      and r.get('request_id') == rid and r.get('track') == confirmed.get('track')
                      and r.get('attempt') == confirmed.get('attempt') and r['time'] < confirmed['time']]
    request = request_events[0] if len(request_events) == 1 else {}
    wires = [r for r in requests if str(r.get('request_id', r.get('stamp_ns'))) == rid
             and r.get('track') == confirmed.get('track') and r.get('attempt') == confirmed.get('attempt')]
    wire = wires[0] if len(wires) == 1 else {}
    base = wire.get('goal')
    if not isinstance(rid, str) or not rid.isdecimal() or not request or not wire:
        errors.append('missing_unique_request_identity')
    if not close(base, request.get('request_base'), 1e-6):
        errors.append('request_base_wire_mismatch')
    if not close(request.get('goal'), confirmed.get('goal'), 1e-6):
        errors.append('request_lens_goal_mismatch')
    if abs(wire.get('stamp_ns', -1)*1e-9-request.get('time', math.inf)) > 1e-6:
        errors.append('wire_request_timestamp_mismatch')
    relevant = [r for r in statuses if r.get('request_id') == rid
                and request.get('time', math.inf) <= r.get('time', -1) <= confirmed['time']]
    if any(not close(r.get('goal'), base, 1e-4) for r in relevant):
        errors.append('native_request_goal_changed')
    if any(r.get('state') in ('cancelled', 'cancel_pending', 'timeout', 'unreachable',
                              'invalid', 'budget_exhausted', 'rejected') for r in relevant):
        errors.append('native_request_terminated_before_confirmation')
    plans = [r for r in relevant if r.get('state') == 'planned']
    plan_ids = [r.get('native_plan_id') for r in plans]
    if (not plans or len(set(plan_ids)) != len(plans)
            or [r.get('segment_index') for r in plans] != list(range(1, len(plans)+1))):
        errors.append('missing_or_ambiguous_native_segments')
    for i, plan in enumerate(plans):
        pid = plan.get('native_plan_id')
        end = plans[i+1]['time'] if i+1 < len(plans) else confirmed['time']
        authority = [r for r in relevant if r.get('state') == 'executing'
                     and r.get('native_plan_id') == pid and plan['time'] <= r['time'] <= end]
        acts = [r for r in flight if r.get('type') == 'native_trajectory_activated'
                and r.get('traj_id') == pid and plan['time'] <= r.get('time', -1) <= end]
        act = acts[0] if len(acts) == 1 else {}
        token = act.get('token')
        proposals = [r for r in flight if r.get('type') == 'path_proposed' and token is not None
                     and r.get('token') == token and r.get('native_id') == pid
                     and plan['time'] <= r.get('time', -1) <= act.get('time', -1)]
        terminal = [r for r in flight if token is not None and r.get('token') == token
                    and r.get('type') == 'terminal_observation_completed'
                    and act.get('time', math.inf) <= r.get('time', -1) <= end
                    and r.get('proof', {}).get('token') == token
                    and r['proof'].get('native_plan_id') == pid
                    and r['proof'].get('completion_reason') == 'post_arrival_integrated_sensor_frame'
                    and act.get('time', math.inf) <= r['proof'].get('arrived_at', -1)
                    <= r['proof'].get('time', -1) <= r['proof'].get('completed_at', -1) <= r['time']]
        completed = [r for r in relevant if r.get('state') == 'segment_completed'
                     and r.get('native_plan_id') == pid and plan['time'] <= r['time'] <= end]
        checks = dict(unique_authorization=len(authority) == 1, unique_activation=len(acts) == 1,
                      unique_proposal=len(proposals) == 1, terminal_observation=bool(terminal),
                      native_completion=bool(completed))
        segments.append(dict(segment_index=plan.get('segment_index'), native_plan_id=pid,
                             token=token, plan=plan, activation=act, terminal=terminal,
                             checks=checks, passed=all(checks.values())))
    if not segments or not all(s['passed'] for s in segments):
        errors.append('incomplete_segment_execution_chain')
    final = segments[-1] if segments else {}
    arrivals = [r for r in relevant if r.get('state') == 'arrived'
                and r.get('native_plan_id') == final.get('native_plan_id')]
    completions = [r for r in reviews if r.get('request_id') == rid
                   and r.get('reason') == 'execution_completed_waiting_for_fresh_visual_evidence'
                   and r.get('token') == final.get('token')
                   and r.get('native_plan_id') == final.get('native_plan_id')
                   and request.get('time', math.inf) < r['time'] < confirmed['time']]
    completion = completions[0] if len(completions) == 1 else {}
    reports = [r['data'] for r in telemetry if r.get('kind') == 'execution'
               and r['data'].get('token') == final.get('token') and final.get('token') is not None
               and r['data'].get('native_plan_id') == final.get('native_plan_id')
               and r['data'].get('ready') and r['data'].get('arrived')
               and not r['data'].get('flight_session_fault')
               and 0 <= completion.get('time', -1)-r['data'].get('time', math.inf) < .5
               and 0 <= completion.get('time', -1)-r['data'].get('position_stamp', math.inf) < .5
               and close(r['data'].get('actual_position'), base, .25)
               and close(r['data'].get('reference'), base, .25)
               and any(r['data'].get('observation_proof') == t['proof'] for t in final.get('terminal', []))]
    if not arrivals or not completion or not reports or confirmed.get('token') != final.get('token'):
        errors.append('missing_fresh_actual_arrival_and_matching_observation_proof')
    if not any(r['time'] <= completion.get('time', -1) and r.get('distance_m', math.inf) <= .25 for r in arrivals):
        errors.append('missing_native_actual_arrival')
    cutoff=completion.get('arrival_event_time',completion.get('time',math.inf))
    if 'arrival_event_time' in completion and not any(abs(r['time']-cutoff)<=1e-6 for r in arrivals):
        errors.append('unmatched_arrival_event_time')
    for hit in confirmed.get('evidence', []):
        if not cutoff < hit.get('time', -1) <= confirmed['time']:
            errors.append('capture_not_after_review_completion')
        if not close(hit.get('origin'), request.get('goal'), .4):
            errors.append('capture_not_at_requested_lens')
        try:
            if np.linalg.norm(np.asarray(hit['origin'])[:2]-np.asarray(request['request_origin'])[:2]) < .5:
                errors.append('insufficient_measured_view_change')
        except (KeyError, ValueError, TypeError):
            errors.append('missing_request_origin')
    budgets = [r for r in relevant if all(k in r for k in ('max_segments', 'max_duration_s', 'max_path_m', 'path_used_m'))]
    if not budgets:
        errors.append('missing_native_request_budgets')
    else:
        limits = {(r['max_segments'], r['max_duration_s'], r['max_path_m']) for r in budgets}
        if len(limits) != 1:
            errors.append('native_request_budget_changed')
        max_segments, max_duration, max_path = next(iter(limits))
        if (not np.isfinite([max_segments, max_duration, max_path]).all() or min(max_segments, max_duration, max_path) <= 0
                or len(plans) > max_segments
                or completion.get('time', math.inf)-request.get('time', 0) > max_duration
                or any(not math.isfinite(r['path_used_m']) or r['path_used_m'] > max_path for r in budgets)):
            errors.append('native_request_budget_exceeded')
    return dict(request_id=rid, request=request, wire=wire, segments=segments,
                native_arrivals=arrivals, completion=completion, errors=sorted(set(errors)), passed=not errors)


def audit(run):
    from person_search.confirmation_quality import ConfirmationQuality

    run = Path(run)
    hashes = {}
    observations = rows(source_text(run, 'perception/observations.jsonl', hashes, required=True), allow_blank=True)
    telemetry = rows(source_text(run, 'telemetry.jsonl', hashes, required=True), allow_blank=True)
    flight_path = 'flight/0/events.jsonl'
    flight = rows(source_text(run, flight_path, hashes, required=True), allow_blank=True) if (run/flight_path).exists() else []
    reviews = [r for r in observations if r.get('kind') == 'active_confirmation']
    statuses = list(log_packets(source_text(run, 'native-racer/exploration_node_1.log', hashes) or '',
                               'TASK_REVIEW_STATUS '))
    requests = list(log_packets(source_text(run, 'person_search.log', hashes) or '', 'Review request '))

    chains = []
    current_policy = quality_policy_provenance(run, hashes)
    initial, duplicates, unknown = initial_confirmations(reviews)
    required = set(current_policy['current_threshold_fields'])
    for event in initial:
        errors = []
        logged_thresholds = event.get('quality_thresholds')
        provided = set(logged_thresholds) if isinstance(logged_thresholds, dict) else set()
        missing, unexpected = sorted(required-provided), sorted(provided-required)
        if missing:
            errors.append('missing_logged_quality_thresholds:'+','.join(missing))
        if unexpected:
            errors.append('unexpected_logged_quality_thresholds:'+','.join(unexpected))
        if event.get('quality_proxy_version') != current_policy['current_proxy_version']:
            errors.append('quality_proxy_version_missing_or_mismatch')
        try:
            quality = ConfirmationQuality(**logged_thresholds)
        except (KeyError, ValueError, TypeError):
            quality = ConfirmationQuality()
            errors.append('missing_or_invalid_logged_quality_thresholds')
        captures = capture_checks(run, event, quality, observations, telemetry)
        if not captures['passed']:
            errors.append('capture_quality_or_freshness_chain_failed')
        full_review = event.get('reason') == 'active_multiview_confirmed' and event.get('confirmation_path') == 'active_review'
        direct = event.get('reason') in ('reliable_direct_confirmed', 'direct_visible_confirmed') and event.get('confirmation_path') == 'direct'
        classification = ('completed_active_review' if full_review else
                          'sufficient_direct_evidence_during_review' if direct and event.get('during_review') else
                          'direct_quality_proxy_confirmation' if direct else 'unrecognized_confirmation')
        if not direct and not full_review:
            errors.append('unrecognized_confirmation_path')
        if direct and event.get('during_review') and not any(
                r.get('reason') == 'visual_candidate_requested_safe_view'
                and r.get('request_id') == event.get('request_id')
                and r.get('track') == event.get('track')
                and r.get('attempt') == event.get('attempt')
                and r['time'] < event['time'] for r in reviews):
            errors.append('missing_request_context_for_during_review_claim')
        chain = review_chain(event, reviews, statuses, requests, flight, telemetry) if full_review else None
        if chain is not None and not chain['passed']:
            errors.append('active_review_chain_failed')
        chains.append(dict(track=event.get('track'), time=event.get('time'), classification=classification,
                           event=event, capture_evidence=captures, review_chain=chain,
                           policy_schema=dict(logged_proxy_version=event.get('quality_proxy_version'),
                                              current_proxy_version=current_policy['current_proxy_version'],
                                              missing_threshold_fields=missing, unexpected_threshold_fields=unexpected),
                           errors=errors, passed=not errors))
    confirmed_ids = {t['id'] for r in telemetry if r.get('kind') == 'tracks'
                     for t in r['data'].get('tracks', []) if t.get('state') == 'confirmed'}
    without_event = sorted(confirmed_ids-{c['track'] for c in chains})
    all_verified = bool(chains) and not without_event and not duplicates and not unknown and all(c['passed'] for c in chains)
    return dict(mode='saved_confirmation_evidence_audit', run=str(run),
                scope='Recorded request/native-ID/token/actual-arrival/RGB-D evidence only. No truth, no control outputs and no inferred real-person count.',
                confirmation_count=len(chains), confirmations=chains,
                confirmation_count_scope='Unique algorithm track IDs with an explicit initial confirmation event; not distinct real people.',
                duplicate_initial_confirmation_events=duplicates,
                unrecognized_confirmed_events=unknown,
                classifications=dict(collections.Counter(c['classification'] for c in chains)),
                confirmed_track_ids_without_event=without_event,
                all_reported_confirmations_verified=all_verified,
                outcome='verified_recorded_confirmation_evidence' if all_verified else 'no_verified_confirmation_claim',
                native_request_statuses=statuses, source_sha256=provenance(run, hashes),
                quality_policy=current_policy,
                limitations=['Absence of a verified chain is not evidence that the scene contains no person.',
                             'Quality is an observed proxy and a visible-surface position, not body-centre accuracy or semantic ground truth.',
                             'Direct confirmation during a review is separate from arrived-plus-new-evidence review success.',
                             'Startup captures that pass the quality proxy may confirm, but are not autonomous exploration discoveries.',
                             'The exterior mask ring is a depth proxy; it cannot semantically prove an unoccluded full body or count true people.',
                             'A preserved old as-run audit is not retroactively changed; missing new proxy/version fields cannot pass this current-standard audit.'])


def write_report(output, report, run):
    output, run = Path(output).resolve(), Path(run).resolve()
    if output == run or run in output.parents:
        raise ValueError('Output must be outside the source run directory')
    with output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--replay-old', action='store_true')
    args = parser.parse_args()
    report = replay_old(args.run) if args.replay_old else audit(args.run)
    write_report(args.output, report, args.run)
    print(json.dumps(dict(output=str(args.output), mode=report['mode'],
                          confirmation_count=report.get('confirmation_count', 0)), indent=2))


if __name__ == '__main__':
    main()
