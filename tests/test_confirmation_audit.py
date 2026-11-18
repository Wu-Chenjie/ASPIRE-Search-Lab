"""Recorded-evidence counterexamples; no ROS, simulator or detector inference."""
import json
from collections import Counter
from pathlib import Path

import numpy as np
import pytest

from tools.flight.confirmation_audit import audit, reconstruct, replay_old, review_chain, write_report


def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r)+'\n' for r in rows))


def saved_run(tmp_path, confidence=.9, times=(1., 1.5, 2.), boundary='clear'):
    run = tmp_path/'run'
    folder = run/'perception'
    folder.mkdir(parents=True)
    mask = np.zeros((80, 100), bool)
    mask[10:70, 35:65] = True
    depth = np.full(mask.shape, {'clear':4., 'near':2., 'unknown':np.nan}[boundary], np.float32)
    depth[mask] = 3.
    transform = np.eye(4)
    transform[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    transform[:3, 3] = [0, 0, 2]
    k = [100., 0., 50., 0., 100., 40., 0., 0., 1.]
    observations, telemetry = [], []
    for stamp in times:
        ns = round(stamp*1e9)
        np.savez_compressed(folder/f'{ns}-rgb-masks.npz', stamp_ns=ns,
                            rgb=np.zeros((80, 100, 3), np.uint8),
                            boxes=[[35, 10, 30, 60]], scores=[confidence], masks=[mask])
        np.savez_compressed(folder/f'{ns}-geometry.npz', stamp_ns=ns,
                            depth=depth, k=k, camera_to_map=transform)
        observations.append(dict(kind='timing', stamp_ns=ns, now_ns=ns+20_000_000,
                                 status='accepted', received_age_wall_s=.02))
        telemetry.append(dict(kind='execution', data=dict(time=stamp-.05, ready=False, arrived=False)))
        _, candidates = reconstruct(run, ns)
        assert len(candidates) == 1
        telemetry.append(dict(kind='tracks', data=dict(stamp_ns=ns,
                         tracks=[dict(id=1, xyz=candidates[0].xyz.tolist(), state='tentative')])))
    write_rows(folder/'observations.jsonl', observations)
    write_rows(run/'telemetry.jsonl', telemetry)
    return run


def install_confirmation(run):
    report = replay_old(run)
    assert report['confirmation_count'] == 1
    event = report['confirmations'][0]
    observations = [json.loads(s) for s in (run/'perception/observations.jsonl').read_text().splitlines()]
    observations.append(dict(kind='active_confirmation', **event))
    write_rows(run/'perception/observations.jsonl', observations)
    return event


def test_old_frames_replay_confirms_clear_before_ready_without_claiming_flight(tmp_path):
    run = saved_run(tmp_path)
    result = replay_old(run)
    assert result['confirmation_count'] == 1
    assert result['confirmations'][0]['is_before_ready'] is True
    assert result['confirmations'][0]['reason'] == 'reliable_direct_confirmed'
    assert 'NOT new flight' in result['scope']
    assert result['quality_distributions']['confidence']['minimum'] == .9
    assert not (run/'evaluation-truth.json').exists()


@pytest.mark.parametrize('failure', ['missing_pair', 'stale_timing', 'low_confidence'])
def test_old_frames_cannot_confirm_from_missing_stale_or_weak_evidence(tmp_path, failure):
    run = saved_run(tmp_path, confidence=.5 if failure == 'low_confidence' else .9)
    if failure == 'missing_pair':
        (run/'perception/1500000000-geometry.npz').unlink()
    if failure == 'stale_timing':
        rows = [json.loads(s) for s in (run/'perception/observations.jsonl').read_text().splitlines()]
        rows[1]['received_age_wall_s'] = 1.
        write_rows(run/'perception/observations.jsonl', rows)
    report = replay_old(run)
    assert report['confirmation_count'] == 0
    assert report['rejection_reason_counts']
    if failure == 'missing_pair':
        frame = next(f for f in report['frames'] if f['stamp_ns'] == 1_500_000_000)
        assert frame['semantic'][0]['state'] == 'unlocalized' and not frame['positions']
        assert set(frame['sources']) == {'1500000000-rgb-masks.npz'} and frame['errors']


def test_replay_hashes_each_source_from_the_same_read_once(tmp_path, monkeypatch):
    run = saved_run(tmp_path)
    (run/'result.json').write_text('{}')
    original = Path.open
    counts = Counter()

    def counted(path, mode='r', *args, **kwargs):
        if mode in ('r','rb') and (run in path.parents or path.parent.name == 'person_search'):
            counts[path] += 1
        return original(path, mode, *args, **kwargs)

    monkeypatch.setattr(Path, 'open', counted)
    report = replay_old(run)
    assert report['confirmation_count'] == 1
    assert report['implementation_sha256'] == report['quality_policy']['current_implementation_sha256']
    assert len(counts) == 12 and set(counts.values()) == {1}


@pytest.mark.parametrize('corruption', [b'', b'PK\x03\x04truncated'])
def test_corrupt_pair_is_unavailable_in_replay_and_confirmation_audit(tmp_path, corruption):
    run = saved_run(tmp_path)
    install_confirmation(run)
    (run/'perception/1500000000-geometry.npz').write_bytes(corruption)
    replay = replay_old(run)
    assert replay['confirmation_count'] == 0
    assert replay['counts']['missing_or_invalid_saved_pair'] == 1
    assert replay['counts']['accepted_capture_frames'] == 2
    frame = replay['frames'][1]
    assert frame['errors'] and not frame['positions']
    assert frame['semantic'][0]['state'] == 'unlocalized'
    assert not audit(run)['all_reported_confirmations_verified']


@pytest.mark.parametrize('recorded, matches', [
    (np.float64(1_011_000_000), True),
    (np.float64(1_011_000_000.25), False),
    (np.float64(1_011_000_001), False),
    (np.float32(1_011_000_000), False),
    (np.bool_(True), False),
])
def test_saved_capture_stamp_requires_exact_nanoseconds(tmp_path, recorded, matches):
    run = saved_run(tmp_path, times=(1.011,))
    expected = 1_011_000_000
    for name in ('rgb-masks', 'geometry'):
        path = run/f'perception/{expected}-{name}.npz'
        with np.load(path) as saved:
            arrays = dict(saved)
        np.savez_compressed(path, **dict(arrays, stamp_ns=recorded))
    frame, localized = reconstruct(run, expected)
    assert bool(localized) is matches
    assert frame['errors'] == ([] if matches else ['ValueError:mask_capture_stamp_mismatch'])


@pytest.mark.parametrize('issue, reason, exit_code', [
    ('none', None, 0),
    ('missing_truth', 'missing_truth_bracket', 0),
    ('zero_frames', None, 0),
    ('mask_stamp', 'mask_capture_stamp_mismatch', 1),
    ('geometry_stamp', 'geometry_capture_stamp_mismatch', 1),
    ('missing_stamp', 'geometry_capture_stamp_missing', 1),
    ('missing_geometry', '1500000000-geometry.npz', 0),
    ('wrong_position', 'localized_candidate_position_mismatch', 1),
    ('invalid_xyz', 'invalid_truth_pose', 1),
    ('invalid_quat', 'invalid_truth_pose', 1),
])
def test_geometry_audit_binds_saved_pairs_and_preserves_other_frames(tmp_path, monkeypatch, issue, reason, exit_code):
    import sys
    from types import SimpleNamespace as NS
    from tools.hm3d.search_geometry_audit import main

    run = saved_run(tmp_path, times=(1.011, 1.5, 2.))
    stamps = (1_011_000_000, 1_500_000_000, 2_000_000_000)
    observations = []
    for ns in stamps:
        positions = reconstruct(run, ns)[0]['positions']
        if issue == 'wrong_position' and ns == stamps[1]:
            positions[0][0] += .01
        observations.append(dict(kind='semantic', stamp_ns=ns,
                                 candidates=[dict(state='localized', xyz=p) for p in positions]))
    write_rows(run/'perception/observations.jsonl', observations if issue != 'zero_frames' else [])
    truth = [dict(stamp=ns//10**9 + ns%10**9*1e-9, xyz=[0, 0, 2], quat=[0, 0, 0, 1]) for ns in stamps]
    assert truth[0]['stamp'] != stamps[0]*1e-9
    if issue == 'missing_truth':
        del truth[1]
    elif issue == 'invalid_xyz':
        truth[1]['xyz'] = [.1]
    elif issue == 'invalid_quat':
        truth[1]['quat'] = [0, 0, 0, 0]
    write_rows(run/'truth-poses.jsonl', truth)
    (run/'evaluation-truth.json').write_text(json.dumps(dict(person=[0]*6)))
    if issue == 'missing_geometry':
        (run/f'perception/{stamps[1]}-geometry.npz').unlink()
    if issue in ('mask_stamp', 'geometry_stamp', 'missing_stamp'):
        archive = 'rgb-masks' if issue == 'mask_stamp' else 'geometry'
        path = run/f'perception/{stamps[1]}-{archive}.npz'
        with np.load(path) as saved:
            arrays = dict(saved)
        if issue == 'missing_stamp':
            del arrays['stamp_ns']
        else:
            arrays['stamp_ns'] = 9_000_000_000
        np.savez_compressed(path, **arrays)
    mesh = NS(apply_transform=lambda _: None,
              ray=NS(intersects_location=lambda *a, **k: ([], [], [])))
    monkeypatch.setitem(sys.modules, 'trimesh', NS(load_scene=lambda *a, **k: NS(to_mesh=lambda: mesh),
                        proximity=NS(closest_point=lambda *a: (None, [.1], None))))
    monkeypatch.setattr(sys, 'argv', ['geometry-audit', str(run), '--person', 'synthetic-person.glb'])
    if exit_code:
        with pytest.raises(SystemExit) as stopped:
            main()
        assert stopped.value.code == exit_code
    else:
        main()
    report = json.loads((run/'search-geometry-audit.json').read_text())
    rows = report['observations']
    assert report['status'] == ('complete' if issue == 'none' else 'incomplete')
    if issue == 'zero_frames':
        assert not rows and report['selected_frames'] == 0
    else:
        assert rows[0]['position_reproduced'] and rows[2]['position_reproduced']
        if issue == 'missing_geometry':
            assert reason in rows[1]['unscored']
        else:
            assert rows[1].get('unscored') == reason
        assert report['unscored_frames'] == int(reason is not None)
        assert report['summary']['visible_position_distance_to_person_surface_m']['count'] == (2 if reason else 3)


def test_truth_lookup_distinguishes_adjacent_stamps_and_bounds_interpolation():
    from tools.flight.flight_localization import interpolate_ns

    first = 1_011_000_000
    adjacent = first+1
    last = adjacent+60_000_000
    rows = [dict(stamp=ns//10**9 + ns%10**9*1e-9, xyz=[position, 0, 0], quat=[0, 0, 0, 1])
            for ns, position in [(first, 1.), (adjacent, 2.), (last, 3.)]]
    assert interpolate_ns(rows, first)[0][0] == 1.
    assert interpolate_ns(rows, adjacent)[0][0] == 2.
    assert interpolate_ns(rows, adjacent+30_000_000)[0][0] == 2.5
    assert interpolate_ns([rows[0], rows[2]], first+30_000_000) is None


def test_direct_audit_recomputes_saved_quality_and_distinguishes_startup(tmp_path):
    run = saved_run(tmp_path)
    install_confirmation(run)
    report = audit(run)
    assert report['all_reported_confirmations_verified']
    chain = report['confirmations'][0]
    assert chain['classification'] == 'direct_quality_proxy_confirmation'
    assert chain['review_chain'] is None
    assert all(c['capture_before_ready'] for c in chain['capture_evidence']['captures'])


@pytest.mark.parametrize('boundary', ['near', 'unknown'])
def test_exterior_boundary_veto_rejects_confirmation_without_moving_surface_point(tmp_path, boundary):
    clear = saved_run(tmp_path/'clear')
    blocked = saved_run(tmp_path/'blocked', boundary=boundary)
    _, clear_candidates = reconstruct(clear, 1_000_000_000)
    _, blocked_candidates = reconstruct(blocked, 1_000_000_000)
    assert np.allclose(clear_candidates[0].xyz, blocked_candidates[0].xyz)
    assert blocked_candidates[0].quality['foreground_occlusion_fraction'] == 0.
    report = replay_old(blocked)
    assert report['confirmation_count'] == 0
    assert report['quality_policy']['current_proxy_version'] == 'rgbd_mask_boundary_v1'
    if boundary == 'near':
        assert report['quality_gate_rejections']['boundary_near_fraction_gate'] > 0
    else:
        assert report['quality_gate_rejections']['boundary_valid_fraction_gate'] > 0
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('missing', ['thresholds', 'metrics', 'version'])
def test_old_proxy_or_missing_boundary_fields_fail_explicitly_not_by_exception(tmp_path, missing):
    run = saved_run(tmp_path)
    install_confirmation(run)
    rows = [json.loads(s) for s in (run/'perception/observations.jsonl').read_text().splitlines()]
    event = rows[-1]
    if missing == 'thresholds':
        del event['quality_thresholds']['max_boundary_near_fraction']
        del event['quality_thresholds']['min_boundary_valid_fraction']
    elif missing == 'metrics':
        for item in event['evidence']:
            del item['quality']['boundary_near_fraction']
            del item['quality']['boundary_valid_fraction']
    else:
        event['quality_proxy_version'] = 'old_box_only_proxy'
    write_rows(run/'perception/observations.jsonl', rows)
    historical = tmp_path/'preserved-old-as-run-audit.json'
    historical.write_text('{"passed": true, "scope": "old as-run result"}\n')
    before = historical.read_bytes()
    report = audit(run)
    assert not report['all_reported_confirmations_verified']
    chain = report['confirmations'][0]
    if missing == 'thresholds':
        assert chain['policy_schema']['missing_threshold_fields'] == ['max_boundary_near_fraction', 'min_boundary_valid_fraction']
        assert any(e.startswith('missing_logged_quality_thresholds:') for e in chain['errors'])
    elif missing == 'metrics':
        errors = chain['capture_evidence']['captures'][0]['errors']
        assert 'quality_evidence_mismatch:boundary_near_fraction' in errors
    else:
        assert 'quality_proxy_version_missing_or_mismatch' in chain['errors']
    assert historical.read_bytes() == before
    json.dumps(report, allow_nan=False)


def test_twenty_frozen_position_updates_are_not_twenty_new_confirmations(tmp_path):
    run = saved_run(tmp_path, times=(1., 1.5, 2., *[2.5+i*.5 for i in range(20)]))
    replay = replay_old(run)
    assert replay['event_reason_counts']['confirmed_position_frozen'] == 20
    assert replay['confirmation_count'] == 1
    assert replay['event_integrity_passed']
    rows = [json.loads(s) for s in (run/'perception/observations.jsonl').read_text().splitlines()]
    rows.extend(dict(kind='active_confirmation', **event) for event in replay['events'])
    write_rows(run/'perception/observations.jsonl', rows)
    report = audit(run)
    assert report['confirmation_count'] == 1
    assert report['all_reported_confirmations_verified']
    assert report['duplicate_initial_confirmation_events'] == []


def test_repeated_initial_confirmation_is_an_alert_not_an_additional_track(tmp_path):
    run = saved_run(tmp_path)
    event = install_confirmation(run)
    rows = [json.loads(s) for s in (run/'perception/observations.jsonl').read_text().splitlines()]
    rows.append(dict(event, kind='active_confirmation', time=3.))
    write_rows(run/'perception/observations.jsonl', rows)
    report = audit(run)
    assert report['confirmation_count'] == 1
    assert len(report['duplicate_initial_confirmation_events']) == 1
    assert report['duplicate_initial_confirmation_events'][0]['track'] == event['track']
    assert not report['all_reported_confirmations_verified']


def test_sufficient_direct_evidence_during_review_is_not_review_arrival(tmp_path):
    run = saved_run(tmp_path)
    install_confirmation(run)
    rows = [json.loads(s) for s in (run/'perception/observations.jsonl').read_text().splitlines()]
    rows[-1].update(during_review=True, request_id='500000000', attempt=1)
    rows.insert(0, dict(kind='active_confirmation', reason='visual_candidate_requested_safe_view',
                        request_id='500000000', track=1, attempt=1, time=.5, state='reviewing'))
    write_rows(run/'perception/observations.jsonl', rows)
    report = audit(run)
    assert report['all_reported_confirmations_verified']
    assert report['confirmations'][0]['classification'] == 'sufficient_direct_evidence_during_review'
    assert report['confirmations'][0]['review_chain'] is None


@pytest.mark.parametrize('failure', ['quality_forgery', 'missing_array', 'skipped_capture', 'unlogged_thresholds'])
def test_direct_claim_fails_closed_when_evidence_is_incomplete_or_inconsistent(tmp_path, failure):
    run = saved_run(tmp_path)
    install_confirmation(run)
    rows = [json.loads(s) for s in (run/'perception/observations.jsonl').read_text().splitlines()]
    if failure == 'quality_forgery':
        rows[-1]['evidence'][0]['quality']['confidence'] = .99
    elif failure == 'missing_array':
        (run/'perception/1500000000-geometry.npz').unlink()
    elif failure == 'skipped_capture':
        rows.insert(1, dict(kind='semantic', stamp_ns=1250000000, candidates=[]))
    else:
        del rows[-1]['quality_thresholds']
    write_rows(run/'perception/observations.jsonl', rows)
    report = audit(run)
    assert not report['all_reported_confirmations_verified']
    assert report['outcome'] == 'no_verified_confirmation_claim'


def review_fixture():
    rid, base, lens = '100000000', [1., 0., 1.], [1.13233, 0., 1.02078]
    request = dict(reason='visual_candidate_requested_safe_view', request_id=rid,
                   track=1, attempt=1, time=.1, goal=lens, request_base=base, request_origin=[0., 0., 1.])
    wire = dict(request_id=rid, stamp_ns=100000000, track=1, attempt=1, goal=base)
    statuses, flight = [], []
    for segment, pid, start, end in [(1, 3, .2, 1.), (2, 7, 1.2, 2.)]:
        common = dict(request_id=rid, native_plan_id=pid, segment_index=segment, goal=base,
                      max_segments=4, max_duration_s=20., max_path_m=8., path_used_m=float(segment))
        statuses.extend(dict(common, state=state, time=t) for state, t in
                        [('planned', start), ('executing', start+.1), ('segment_completed', end+.05)])
        proof = dict(token=f't{segment}', native_plan_id=pid, arrived_at=end-.1,
                     time=end-.05, completed_at=end-.02,
                     completion_reason='post_arrival_integrated_sensor_frame')
        flight.extend([dict(type='path_proposed', native_id=pid, token=f't{segment}', time=start+.05),
                       dict(type='native_trajectory_activated', traj_id=pid, token=f't{segment}', time=start+.2),
                       dict(type='terminal_observation_completed', token=f't{segment}', time=end, proof=proof)])
    statuses.append(dict(common, state='arrived', time=2.1, distance_m=0.))
    completion = dict(reason='execution_completed_waiting_for_fresh_visual_evidence',
                      request_id=rid, native_plan_id=7, token='t2', time=2.2)
    confirmed = dict(request_id=rid, track=1, attempt=1, time=4., token='t2', goal=lens,
                     evidence=[dict(time=t, origin=lens) for t in [3., 3.5, 4.]])
    telemetry = [dict(kind='execution', data=dict(time=2.15, position_stamp=2.15, native_plan_id=7,
                 token='t2', ready=True, arrived=True, actual_position=base, reference=base,
                 observation_proof=proof))]
    return [confirmed, [request, completion], statuses, [wire], flight, telemetry]


def test_multiple_native_segments_join_to_the_same_request_and_actual_arrival():
    report = review_chain(*review_fixture())
    assert report['passed'], report['errors']
    assert [s['native_plan_id'] for s in report['segments']] == [3, 7]


@pytest.mark.parametrize('failure', ['actual_far', 'stale_pose', 'unrelated_request', 'missing_terminal',
                                      'wrong_proof_id', 'late_capture', 'wrong_final_token', 'unbounded', 'cancelled'])
def test_review_chain_rejects_false_arrival_or_unrelated_evidence(failure):
    args = review_fixture()
    if failure == 'actual_far':
        args[5][0]['data']['actual_position'] = [1.3, 0., 1.]
    elif failure == 'stale_pose':
        args[5][0]['data']['position_stamp'] = 1.
    elif failure == 'unrelated_request':
        args[2][-1]['request_id'] = '999'
    elif failure == 'missing_terminal':
        args[4].pop()
    elif failure == 'wrong_proof_id':
        args[4][-1]['proof']['native_plan_id'] = 9
    elif failure == 'late_capture':
        args[0]['evidence'][0]['time'] = 2.1
    elif failure == 'wrong_final_token':
        args[0]['token'] = 'ordinary_exploration'
    elif failure == 'cancelled':
        args[2].append(dict(args[2][-1], state='cancelled', time=2.15))
    else:
        for status in args[2]:
            status.pop('max_duration_s')
    assert not review_chain(*args)['passed']


def test_output_never_overwrites_old_run_or_existing_report(tmp_path):
    run = saved_run(tmp_path)
    with pytest.raises(ValueError, match='outside'):
        write_report(run/'audit.json', {}, run)
    output = tmp_path/'audit.json'
    write_report(output, {}, run)
    with pytest.raises(FileExistsError):
        write_report(output, {}, run)
