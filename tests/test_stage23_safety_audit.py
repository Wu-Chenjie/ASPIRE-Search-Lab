"""Audit failures must remain visible; synthetic fixtures are not flight evidence."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from core.exploration.voxel_mapping import VoxelMap
from core.planning.continuous_trajectory import ContinuousTrajectory
from core.planning.execution_safety import NativeSafetyMap, ExecutionCorridor, certified_brake
from tools.flight.stage23_safety_audit import audit_run, audit_certificate, prefix_errors


def fixture_maps_curve():
    runtime=VoxelMap([[0,0,0],[9.,9.,3.9]],resolution=.3,clearance=.6,flight_limits=(.7,3.))
    runtime.state[:]=0;runtime.rebuild()
    native=NativeSafetyMap();native.ingest([1,100,1,.3,0,0,0,*runtime.shape,*np.ones(runtime.state.size)])
    c=np.zeros((1,6,3));c[0,0]=[3.,3.,1.5];c[0,3:,0]=[10.,-15.,6.]
    curve=ContinuousTrajectory([10.],c,0.,0.,yaw_durations=[10.],yaw_segments=np.zeros((1,6)))
    return runtime,native,curve


def write_certificate(path, kind='activation', blocked=False):
    runtime,native,curve=fixture_maps_curve();progress=0. if kind=='activation' else 3.
    replacement=certified_brake(curve,progress,ExecutionCorridor(runtime,native)) if kind=='brake' else None
    actual=curve.sample(progress)[0]
    if blocked:native.state[tuple(runtime.indices(actual))]=2
    np.savez_compressed(path,mapper_state=runtime.state,native_state=native.state,
        mapper_origin=runtime.origin,mapper_resolution=.3,flight_limits=[.7,3.],clearance=.6,
        native_origin=native.origin,native_resolution=.3,native_stamp=100.,native_sequence=1,
        mapper_stamp=100.,mapper_version=1,tracking_margin=.25,actual=actual,reference=actual,
        progress=progress,original=json.dumps(curve.to_dict()),
        replacement=json.dumps(replacement.to_dict()) if replacement else '')


def test_online_certificate_reconstruction_and_brake_analytic_prefix(tmp_path):
    activation=tmp_path/'0001-activation.npz';brake=tmp_path/'0002-brake.npz'
    write_certificate(activation);write_certificate(brake,'brake')
    a,b=audit_certificate(activation),audit_certificate(brake)
    assert a['status']=='passed' and a['checks']['full_curve_continuous_corridor']
    assert b['status']=='passed' and b['checks']['remaining_continuous_corridor']
    assert all(b['checks'].values()) and max(b['prefix_errors'].values())<1e-9
    assert max(b['terminal_errors'].values())<1e-9


def test_unreadable_certificate_leaves_other_reconstruction_available(tmp_path, monkeypatch):
    certificates = tmp_path/'execution-safety'
    certificates.mkdir()
    unreadable = certificates/'0001-activation.npz'
    unreadable.touch()
    write_certificate(certificates/'0002-activation.npz')
    original = Path.read_bytes

    def read(path):
        if path == unreadable:
            raise PermissionError('unreadable certificate')
        return original(path)

    monkeypatch.setattr(Path, 'read_bytes', read)
    report = audit_run(tmp_path)
    assert report['status'] == 'violations'
    failed, passed = report['certificates']
    assert failed['status'] == 'failed' and failed['sha256'] is None
    assert failed['error'] == 'unreadable certificate'
    assert passed['status'] == 'passed' and len(passed['sha256']) == 64


def test_actual_saved_native_inflation_cannot_be_ignored(tmp_path):
    path=tmp_path/'0001-activation.npz';write_certificate(path,blocked=True)
    report=audit_certificate(path)
    assert report['status']=='failed' and not report['checks']['full_curve_continuous_corridor']


@pytest.mark.parametrize('offset', [.13, .26])
def test_stationary_tracking_does_not_hide_new_root_jump(tmp_path, offset):
    path=tmp_path/'0001-activation.npz';write_certificate(path)
    with np.load(path, allow_pickle=False) as saved:
        data={k:saved[k].copy() for k in saved.files}
    data['actual'][0]+=offset;data['reference'][0]+=offset
    np.savez_compressed(path,**data)
    report=audit_certificate(path)
    assert report['tracking_error_m']==0.
    assert report['status']=='failed'
    assert not report['checks']['root_reference_discontinuity_bounded']
    if offset>.25: assert not report['checks']['root_actual_within_tracking_tube']


def test_prefix_audit_detects_between_endpoint_polynomial_change():
    _,_,curve=fixture_maps_curve()
    coeff=curve.coefficients.copy()
    # Zero at both endpoints but a nonzero interior excursion.
    coeff[0,1,1]+=.02;coeff[0,2,1]-=.02
    changed=ContinuousTrajectory(curve.durations,coeff,0.,0.)
    assert np.allclose(curve.sample(0)[0],changed.sample(0)[0])
    assert np.allclose(curve.sample(10)[0],changed.sample(10)[0])
    errors=prefix_errors(curve,changed,10.)
    assert errors['position']>=.005-1e-12 and errors['velocity']>0 and errors['acceleration']>0


def test_pre_step_reserve_cannot_hide_post_step_tracking_violation(tmp_path):
    (tmp_path/'runtime.json').write_text('{}')
    reserve=dict(checked_at=100.,reaction_horizon_s=2.2,native_input_age_s=.5,
        mapper_input_age_s=.5,check_sim_s=.10,tracking_error_m=.08)
    row=dict(kind='execution',data=dict(time=100.04,token='one',arrived=False,
        trajectory_time=1.,trajectory_duration=10.,tracking_error=.2512,stopping_reserve=reserve))
    (tmp_path/'telemetry.jsonl').write_text(json.dumps(row)+'\n')
    report=audit_run(tmp_path)
    assert report['status']=='violations'
    assert 'reported tracking_error' in report['telemetry']['violations'][0]['issues'][0]


def test_no_certificates_and_missing_budget_fields_are_not_full_coverage(tmp_path):
    (tmp_path/'runtime.json').write_text('{}')
    report=audit_run(tmp_path)
    assert report['status']=='no_certificates' and not report['full_flight_certified']
    assert any('no certificates' in x for x in report['coverage_gaps'])
    (tmp_path/'telemetry.jsonl').write_text(json.dumps(dict(kind='execution',data=dict(
        time=100.,token='one',arrived=False,trajectory_time=1.,trajectory_duration=10.,
        stopping_reserve={'checked_at':100.})))+'\n')
    report=audit_run(tmp_path)
    assert report['status']=='violations'
    assert len(report['telemetry']['violations'][0]['issues'])==5


@pytest.mark.parametrize('evidence', ['telemetry', 'certificate'])
def test_hard_guard_fault_is_failure_even_before_numeric_tube_violation(tmp_path,evidence):
    # map-delay-09 refused the NEXT reference; the recorded current error
    # remained 9.67cm. Valid maps/numeric samples cannot hide that failed run.
    (tmp_path/'runtime.json').write_text('{}')
    folder=tmp_path/'execution-safety';folder.mkdir()
    write_certificate(folder/'0001-activation.npz')
    if evidence=='certificate':
        write_certificate(folder/'0002-fault.npz','fault')
    else:
        (tmp_path/'telemetry.jsonl').write_text(json.dumps(dict(kind='execution',data=dict(
            time=124.56,token='one',arrived=True,tracking_error=.0967255,
            safety_fault='next_reference_tracking_tube_exceeded')))+'\n')
    report=audit_run(tmp_path)
    assert report['observed_safety_fault']
    assert report['status']=='execution_safety_fault'
    assert not report['full_flight_certified']


def test_interrupted_run_is_not_hidden_by_valid_saved_certificates(tmp_path):
    (tmp_path/'runtime.json').write_text('{}')
    (tmp_path/'result.json').write_text(json.dumps(dict(error='KeyboardInterrupt',budget_end_sim=106.04)))
    folder=tmp_path/'execution-safety';folder.mkdir()
    write_certificate(folder/'0001-activation.npz')
    report=audit_run(tmp_path)
    assert report['status']=='run_failed'
    assert report['run_error']=='KeyboardInterrupt'
    assert report['certificates'][0]['status']=='passed'


def test_sampled_reserve_budgets_and_gaps_reported_explicitly(tmp_path):
    (tmp_path/'runtime.json').write_text('{}')
    reserve=dict(checked_at=100.,reaction_horizon_s=2.2,native_input_age_s=.5,
        mapper_input_age_s=.5,check_sim_s=.10,tracking_error_m=.05)
    rows=[dict(kind='execution',data=dict(time=100.,token='one',arrived=False,
          trajectory_time=1.,trajectory_duration=10.,stopping_reserve=reserve)),
          dict(kind='execution',data=dict(time=100.8,token='one',arrived=False,
          trajectory_time=1.8,trajectory_duration=10.,stopping_reserve=reserve))]
    (tmp_path/'telemetry.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    report=audit_run(tmp_path)
    assert report['telemetry']['all_observed_numeric_budgets_pass']
    assert report['telemetry']['all_observed_reserve_budgets_pass'] is None
    assert report['telemetry']['observed_unique_checks']==1
    assert report['telemetry']['continuing_rows_with_recent_reserve']==1
    assert report['telemetry']['missing_reserve_rows'] and report['telemetry']['execution_report_gaps']
    assert not report['full_flight_certified']
    reserve.update(reaction_horizon_s=2.19,native_input_age_s=2.01,
                   mapper_input_age_s=2.01,check_sim_s=.201,tracking_error_m=.251)
    (tmp_path/'telemetry.jsonl').write_text(json.dumps(rows[0])+'\n')
    bad=audit_run(tmp_path)
    assert len(bad['telemetry']['violations'][0]['issues'])==5


def test_monotonic_wall_bound_is_not_reported_as_measured_sim_time(tmp_path):
    (tmp_path/'runtime.json').write_text('{}')
    reserve=dict(checked_at=100.,reaction_horizon_s=2.2,native_input_age_s=.5,
        mapper_input_age_s=.5,check_sim_s=None,check_wall_s=.12,check_delay_bound_s=.12,
        tracking_error_m=.05,check_clock='monotonic_wall_upper_bound_at_rtf_le_1')
    row=dict(kind='execution',data=dict(time=100.,token='one',arrived=False,
        trajectory_time=1.,trajectory_duration=10.,stopping_reserve=reserve))
    def audit():
        (tmp_path/'telemetry.jsonl').write_text(json.dumps(row)+'\n')
        return audit_run(tmp_path)
    report=audit()
    assert not report['telemetry']['violations']
    assert report['telemetry']['delay_evidence_modes']=={'monotonic_wall_bound_conditional_on_rtf':1}
    assert report['telemetry']['measured_simulated_check_duration_available'] is False
    assert report['telemetry']['all_observed_reserve_budgets_pass'] is None
    assert any('conditional on RTF' in gap for gap in report['coverage_gaps'])
    reserve['check_delay_bound_s']=.10
    assert any('smaller than' in issue for issue in audit()['telemetry']['violations'][0]['issues'])
    reserve['check_delay_bound_s']=.12;reserve['check_sim_s']=.12
    assert any('must not claim measured' in issue for issue in audit()['telemetry']['violations'][0]['issues'])


@pytest.mark.parametrize('rtf,rate,clock,valid', [
    (.5, 1., 'monotonic_wall_upper_bound_at_rtf_le_1', True),
    (1., 1., 'monotonic_wall_upper_bound_at_rtf_le_1', True),
    (0., 1., 'monotonic_wall_upper_bound_at_rtf_le_1', False),
    (1.01, 1., 'monotonic_wall_upper_bound_at_rtf_le_1', False),
    (.5, 1.1, 'monotonic_wall_upper_bound_at_rtf_le_1', False),
    (.5, .4, 'monotonic_wall_upper_bound_at_rtf_le_1', False),
    (.5, 1., 'ros_clock', False)])
def test_wall_bound_requires_matching_valid_runtime_contract(tmp_path, rtf, rate, clock, valid):
    contract=dict(physics_real_time_factor=rtf,max_sim_to_wall_rate=rate,check_clock=clock)
    (tmp_path/'runtime.json').write_text(json.dumps(dict(execution_safety_timing_contract=contract)))
    reserve=dict(checked_at=100.,reaction_horizon_s=2.2,native_input_age_s=.5,
        mapper_input_age_s=.5,check_sim_s=None,check_wall_s=.12,check_delay_bound_s=.12,
        tracking_error_m=.05,check_clock='monotonic_wall_upper_bound_at_rtf_le_1')
    row=dict(kind='execution',data=dict(time=100.,token='one',arrived=False,
        trajectory_time=1.,trajectory_duration=10.,stopping_reserve=reserve))
    (tmp_path/'telemetry.jsonl').write_text(json.dumps(row)+'\n')
    report=audit_run(tmp_path)
    assert report['timing_contract']['valid'] is valid
    assert report['telemetry']['delay_bound_verified'] is valid
    assert report['telemetry']['all_observed_reserve_budgets_pass'] is (True if valid else None)
    assert report['telemetry']['measured_simulated_check_duration_available'] is False
    assert not report['full_flight_certified']
    if not valid:
        assert any('invalid runtime timing contract' in gap for gap in report['coverage_gaps'])


@pytest.mark.parametrize('corruption',[None,'deadline','forged_freshness'])
def test_terminal_certificate_reconstructs_stopped_root_without_trusting_metadata(tmp_path,corruption):
    runtime,native,curve=fixture_maps_curve()
    native.stamp=99.98
    from core.planning.execution_safety import certified_terminal_hold
    ref=curve.sample(curve.duration)[0]
    hold=certified_terminal_hold(curve,curve.duration,ref,ref,ExecutionCorridor(runtime,native),100.,99.5,3.0)
    hold.update(check_wall_s=.02,native_input_age_s=.02,mapper_input_age_s=.02)
    if corruption=='deadline':hold['absolute_deadline']+=1.
    if corruption=='forged_freshness':native.stamp=97.
    path=tmp_path/'0003-terminal_hold.npz'
    np.savez_compressed(path,mapper_state=runtime.state,native_state=native.state,
        mapper_origin=runtime.origin,mapper_resolution=.3,flight_limits=[.7,3.],clearance=.6,
        native_origin=native.origin,native_resolution=.3,native_stamp=native.stamp,native_sequence=1,
        mapper_stamp=99.98,mapper_version=1,tracking_margin=.25,actual=ref,reference=ref,
        progress=curve.duration,original=json.dumps(curve.to_dict()),replacement='',
        token='test-terminal',decision_time=100.,terminal_hold=json.dumps(hold))
    report=audit_certificate(path)
    assert report['status']==('passed' if corruption is None else 'failed')


@pytest.mark.parametrize('corruption',[None,'token','early_close','unbounded_replan','budget','map','new_report'])
def test_unobserved_abort_certificate_cannot_hide_unsafe_or_fabricated_exit(tmp_path,corruption):
    runtime,native,curve=fixture_maps_curve();native.stamp=103.4
    from core.planning.execution_safety import certified_stopped_reference
    ref=curve.sample(curve.duration)[0]
    hold=certified_stopped_reference(curve,curve.duration,ref,ref,ExecutionCorridor(runtime,native))
    hold.update(started_at=100.,absolute_deadline=103.0,wait_budget_s=3.0,task_state='failed',check_wall_s=.02)
    failure=dict(token='abort',epoch=1,outcome='unobserved',closed_at=103.5,started_at=100.,
        observation_deadline=103.0,replan_deadline=109.5,mission_failure_count=1)
    if corruption=='token':failure['token']='unrelated'
    if corruption=='early_close':failure['observation_deadline']=103.6
    if corruption=='unbounded_replan':failure['replan_deadline']=110.5
    if corruption=='budget':failure['mission_failure_count']=4
    if corruption=='new_report':
        failure.pop('replan_deadline')
        failure['mission_failure_count']=4
    if corruption=='map':native.state[tuple(runtime.indices(ref))]=2
    path=tmp_path/'0004-observation_abort.npz'
    np.savez_compressed(path,mapper_state=runtime.state,native_state=native.state,
        mapper_origin=runtime.origin,mapper_resolution=.3,flight_limits=[.7,3.],clearance=.6,
        native_origin=native.origin,native_resolution=.3,native_stamp=native.stamp,native_sequence=1,
        mapper_stamp=103.4,mapper_version=1,tracking_margin=.25,actual=ref,reference=ref,
        progress=curve.duration,original=json.dumps(curve.to_dict()),replacement='',
        token='abort',decision_time=103.5,terminal_hold=json.dumps(hold),observation_failure=json.dumps(failure))
    report=audit_certificate(path)
    assert report['status']==('passed' if corruption in (None, 'new_report') else 'failed')


def test_profile_replay_keeps_strict_only_failure_separate_from_real_violations(tmp_path):
    from core.planning import execution_safety

    folder = tmp_path/'execution-safety'
    folder.mkdir()
    path = folder/'0001-activation.npz'
    write_certificate(path)
    with np.load(path, allow_pickle=False) as saved:
        data = {key: saved[key].copy() for key in saved.files}
    data['actual'][0] -= .135
    data['reference'][0] -= .135
    np.savez_compressed(path, **data)
    config = tmp_path/'runtime.json'
    config.write_text('{}')
    strict = audit_run(tmp_path)
    assert strict['status'] == 'violations'
    assert not strict['certificates'][0]['checks']['root_reference_discontinuity_bounded']

    config.write_text(json.dumps(dict(execution_safety_profile='sitl_relaxed')))
    relaxed = audit_run(tmp_path)
    assert relaxed['status'] == 'partial_evidence'
    assert not relaxed['failed_certificates']
    certificate = relaxed['certificates'][0]
    if hasattr(execution_safety, 'SITL_RELAXED_PROFILE'):
        assert certificate['status'] == 'passed'
        assert certificate['checks']['root_reference_discontinuity_bounded']
        assert certificate['checks']['full_curve_continuous_corridor']
    else:
        assert certificate['status'] == 'unverified_corridor'
        assert certificate['strict_reconstruction_status'] == 'failed'
        assert not certificate['checks']['root_reference_discontinuity_bounded']
        assert any('online corridor unavailable' in gap for gap in relaxed['coverage_gaps'])

    if hasattr(execution_safety, 'SITL_RELAXED_PROFILE'):
        data['native_state'][11, 10, 5] = 2
        np.savez_compressed(path, **data)
        blocked = audit_run(tmp_path)
        assert blocked['status'] == 'violations'
        assert not blocked['certificates'][0]['checks']['full_curve_continuous_corridor']
        data['native_state'][11, 10, 5] = 1

    # A reference jump outside both real policies is still a failed decision.
    data['reference'][0] -= .03
    np.savez_compressed(path, **data)
    outside = audit_run(tmp_path)
    assert outside['status'] == 'violations'
    assert not outside['certificates'][0]['checks']['root_reference_discontinuity_bounded']

    # A simultaneous tracking violation must remain visible even when the
    # strict-only map and root checks are unverified.
    data['reference'][0] += .03
    data['actual'][1] += .26
    np.savez_compressed(path, **data)
    tracking = audit_run(tmp_path)
    assert tracking['status'] == 'violations'
    assert not tracking['certificates'][0]['checks']['tracking_within_budget']


def test_unavailable_corridor_keeps_independent_certificate_errors(tmp_path):
    folder = tmp_path/'execution-safety'
    folder.mkdir()
    path = folder/'0001-activation.npz'
    write_certificate(path, blocked=True)
    (tmp_path/'runtime.json').write_text(json.dumps(dict(execution_safety_profile='unknown_profile')))
    unknown = audit_run(tmp_path)
    assert unknown['status'] == 'partial_evidence'
    assert unknown['certificates'][0]['strict_reconstruction_status'] == 'failed'
    assert not unknown['certificates'][0]['checks']['full_curve_continuous_corridor']

    # Simulate a damaged ZIP-based NPZ rather than a mere corridor difference.
    path.write_bytes(b'PK\x03\x04broken')
    damaged = audit_run(tmp_path)
    assert damaged['status'] == 'violations'
    assert damaged['certificates'][0]['status'] == 'failed'
    assert damaged['failed_certificates'] == [str(path)]


def test_unavailable_corridor_does_not_hide_brake_prefix_discontinuity(tmp_path):
    folder = tmp_path/'execution-safety'
    folder.mkdir()
    path = folder/'0001-brake.npz'
    write_certificate(path, 'brake', blocked=True)
    with np.load(path, allow_pickle=False) as saved:
        data = {key: saved[key].copy() for key in saved.files}
    replacement = json.loads(data['replacement'].item())
    for span in replacement['coefficients']:
        span[0][1] += .02
    data['replacement'] = json.dumps(replacement)
    np.savez_compressed(path, **data)
    (tmp_path/'runtime.json').write_text(json.dumps(dict(execution_safety_profile='unknown_profile')))
    report = audit_run(tmp_path)
    certificate = report['certificates'][0]
    assert report['status'] == 'violations'
    assert not certificate['checks']['prefix']
    assert not certificate['checks']['boundary']
    assert 'remaining_continuous_corridor' in certificate['unverified_corridor_checks']


def test_frontier_certificate_uses_provenance_to_choose_padding(tmp_path):
    from core.planning import execution_safety
    from tools.flight.stage23_safety_audit import ROOT, sha256

    folder = tmp_path/'execution-safety'
    folder.mkdir()
    path = folder/'0001-activation.npz'
    write_certificate(path)
    with np.load(path, allow_pickle=False) as saved:
        data = {key: saved[key].copy() for key in saved.files}
    data['actual'][0] -= .135
    data['reference'][0] -= .135
    np.savez_compressed(path, **data)
    (tmp_path/'runtime.json').write_text(json.dumps(dict(execution_safety_profile='sitl_frontier')))

    unknown = audit_run(tmp_path)
    assert unknown['certificates'][0]['status'] == 'unverified_corridor'
    assert unknown['certificates'][0]['strict_reconstruction_status'] == 'failed'
    assert not unknown['failed_certificates']
    if hasattr(execution_safety, 'SITL_FRONTIER_PROFILE'):
        source = 'src/annual_planner/src/core/planning/execution_safety.py'
        (tmp_path/'result.json').write_text(json.dumps(dict(code_sha256={source:sha256(ROOT/source)})))
        known = audit_run(tmp_path)
        assert known['certificates'][0]['status'] == 'passed'
        assert known['certificates'][0]['replay_profile'] == 'sitl_frontier'

def read_certificate_arrays(path):
    with np.load(path, allow_pickle=False) as saved:
        return {key: saved[key].copy() for key in saved.files}


@pytest.mark.parametrize('kind', ['c2_admission', 'c2_transition'])
def test_c2_candidate_checks_saved_geometry_without_crediting_a_switch(tmp_path, kind):
    folder = tmp_path/'execution-safety'
    folder.mkdir()
    path = folder/f'0001-{kind}.npz'
    write_certificate(path)
    data = read_certificate_arrays(path)
    data.update(token='candidate', decision_time=100.0)
    if kind == 'c2_admission':
        # The predecessor can be far from the future boundary at admission.
        data['actual'][0] -= .30
        data['reference'][0] -= .30
    else:
        data['progress'] = .02
    np.savez_compressed(path, **data)
    (tmp_path/'runtime.json').write_text(json.dumps(dict(execution_safety_profile='sitl_relaxed')))
    report = audit_run(tmp_path)
    assert report['status'] == 'partial_evidence'
    assert report['certificate_counts'][kind] == 1
    certificate = report['certificates'][0]
    assert certificate['status'] == 'candidate_partial_evidence'
    assert certificate['replay_profile'] == 'strict'
    assert not certificate['handoff_binding_verified']
    assert all(certificate['checks'].values())
    assert any('C2 certificates retain candidate geometry' in gap for gap in report['coverage_gaps'])

    if kind == 'c2_admission':
        data['native_state'][10, 10, 5] = 2
        failed_check = 'candidate_continuous_corridor'
    else:
        data['actual'][0] -= .28
        data['reference'][0] -= .28
        failed_check = 'next_reference_within_tracking_tube'
    np.savez_compressed(path, **data)
    report = audit_run(tmp_path)
    assert report['status'] == 'violations'
    assert not report['certificates'][0]['checks'][failed_check]


def test_moving_handoff_brake_uses_strict_corridor_during_relaxed_run(tmp_path):
    from core.planning import execution_safety
    from tools.flight.stage23_safety_audit import ROOT, sha256

    folder = tmp_path/'execution-safety'
    folder.mkdir()
    path = folder/'0001-brake.npz'
    write_certificate(path, 'brake')
    data = read_certificate_arrays(path)
    stopped = ContinuousTrajectory.from_dict(json.loads(data['replacement'].item()))
    start = stopped.sample(float(data['progress']))[0][0]
    end = stopped.sample(stopped.duration)[0][0]
    assert end > start
    # Place an occupied native cell just outside frontier padding (.03) but inside strict margin (.25).
    data['native_origin'][1] = -.14
    data['native_state'][10:15, 11, 5] = 2
    config = dict(execution_safety_profile='sitl_frontier', static_moving_c2=True)
    source = 'src/annual_planner/src/core/planning/execution_safety.py'
    (tmp_path/'result.json').write_text(json.dumps(dict(code_sha256={source: sha256(ROOT/source)})))
    (tmp_path/'runtime.json').write_text(json.dumps(config))

    data['static_handoff'] = False
    np.savez_compressed(path, **data)
    moving = audit_run(tmp_path)
    assert moving['status'] == 'violations'
    assert moving['certificates'][0]['replay_profile'] == 'strict'
    assert not moving['certificates'][0]['checks']['remaining_continuous_corridor']

    data['static_handoff'] = True
    np.savez_compressed(path, **data)
    stationary = audit_run(tmp_path)
    assert stationary['certificates'][0]['status'] == 'passed'
    assert not stationary['failed_certificates']

    del data['static_handoff']
    np.savez_compressed(path, **data)
    legacy = audit_run(tmp_path)
    assert legacy['certificates'][0]['status'] == 'unverified_corridor'
    assert legacy['certificates'][0]['strict_reconstruction_status'] == 'failed'
    assert not legacy['failed_certificates']
    assert legacy['certificates'][0]['alternate_sitl_reconstruction']['checks']['remaining_continuous_corridor']
    data['native_state'][10:15, 11, 5] = 1
    np.savez_compressed(path, **data)
    assert audit_run(tmp_path)['certificates'][0]['status'] == 'passed'


@pytest.mark.parametrize('state', ['cancelling', 'collision_stop'])
def test_non_observation_terminal_hold_retains_physical_and_timing_checks(tmp_path, state):
    folder = tmp_path/'execution-safety'
    folder.mkdir()
    path = folder/'0001-terminal_hold.npz'
    write_certificate(path)
    data = read_certificate_arrays(path)
    curve = ContinuousTrajectory.from_dict(json.loads(data['original'].item()))
    endpoint = curve.sample(curve.duration)[0]
    hold = dict(task_state=state, fixed_endpoint=endpoint.tolist(), reference_stopped=True,
                observation_complete=False, check_wall_s=.02, native_input_age_s=0., mapper_input_age_s=0.)
    data.update(progress=curve.duration, actual=endpoint, reference=endpoint,
                terminal_hold=json.dumps(hold), token='held', decision_time=100.0)
    np.savez_compressed(path, **data)
    (tmp_path/'runtime.json').write_text('{}')
    report = audit_run(tmp_path)
    assert report['status'] == 'partial_evidence'
    assert report['certificates'][0]['status'] == 'hold_partial_evidence'
    assert all(report['certificates'][0]['checks'].values())
    assert any('terminal receipt require separate evidence' in gap for gap in report['coverage_gaps'])

    hold['fixed_endpoint'][0] += .20
    data['terminal_hold'] = json.dumps(hold)
    np.savez_compressed(path, **data)
    damaged = audit_run(tmp_path)
    assert damaged['status'] == 'violations'
    assert not damaged['certificates'][0]['checks']['recorded_hold_matches_endpoint']
