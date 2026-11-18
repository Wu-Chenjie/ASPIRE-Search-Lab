"""Audit tests use explicitly synthetic logs, never flight acceptance evidence."""
import json
from tools.flight.return_audit import audit


def fixture(tmp_path, *, returned=False, completed=True, home_pose=False, late=False):
    (tmp_path/'native-racer').mkdir();(tmp_path/'flight/0').mkdir(parents=True)
    (tmp_path/'result.json').write_text(json.dumps(dict(budget_end_sim=30.,cleanup={},error=None,native_binary_sha256='synthetic')))
    state='returned' if returned else 'waiting_return_budget'
    reason='home_pose_and_execution_complete' if returned else 'segment_complete_resume_original_home'
    actual='0,0,1' if home_pose else '3,0,1'
    text=f'TASK_RETURN sim=10 state={state} reason={reason} attempts=8 home=0,0,1 goal=3,0,1 actual={actual} active=7 done=1 segment=7\n'
    if returned:text+='[verified return]: Drone 1 from EXEC_TRAJ to FINISH\n'
    (tmp_path/'native-racer/exploration_node_1.log').write_text(text)
    events=[dict(type='native_trajectory_activated',time=5.,traj_id=7,token='test:7')]
    if completed:events.append(dict(type='terminal_observation_completed',time=10.,token='test:7',proof=dict(completed_at=9.9)))
    if late:events.append(dict(type='native_trajectory_activated',time=11.,traj_id=8,token='test:8'))
    (tmp_path/'flight/0/events.jsonl').write_text('\n'.join(map(json.dumps,events)))
    (tmp_path/'telemetry.jsonl').write_text(json.dumps(dict(kind='estimated_odometry',data=dict(stamp=9.95,xyz=[float(x) for x in actual.split(',')]))))
    return tmp_path


def test_return_audit_distinguishes_wait_from_success(tmp_path):
    result=audit(fixture(tmp_path))
    assert result['outcome']=='waiting_return_budget' and not result['violations']
    assert result['terminal_distance_home_m']==3.


def test_return_audit_requires_both_physical_and_execution_proof(tmp_path):
    result=audit(fixture(tmp_path,returned=True,home_pose=True,completed=False))
    assert result['violations'][0]['reason']=='missing_execution_or_physical_completion'


def test_return_audit_rejects_premature_finish_and_late_activation(tmp_path):
    result=audit(fixture(tmp_path,returned=True,home_pose=False,late=True))
    assert {r['reason'] for r in result['violations']}=={'missing_execution_or_physical_completion','executor_activation_after_terminal'}


def test_return_audit_accepts_measured_home_and_completed_execution(tmp_path):
    result=audit(fixture(tmp_path,returned=True,home_pose=True))
    assert not result['violations'] and result['outcome']=='returned'


def test_return_audit_allows_retry_and_activation_after_candidate_wait(tmp_path):
    run=fixture(tmp_path,late=True)
    log=run/'native-racer/exploration_node_1.log'
    fields='home=0,0,1 goal=0,0,1 actual=3,0,1 active=-1 done=0 segment=-1\n'
    log.write_text('TASK_RETURN sim=10 state=waiting_return_candidates reason=no_goal_hold attempts=8 '+fields)
    assert audit(run)['terminal_sim_s'] is None
    assert audit(run)['outcome']=='waiting_return_candidates'
    with log.open('a') as out:
        out.write('TASK_RETURN sim=16 state=planning_home reason=planning_attempt attempts=9 '+fields)
    result=audit(run)
    assert result['outcome']=='in_progress' and result['attempts']==9 and not result['violations']
