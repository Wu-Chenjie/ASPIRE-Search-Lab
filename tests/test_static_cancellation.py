"""Real ledger/adapter and extracted safety callback; no ROS transport claim."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from core.planning.static_cancellation import StaticCancellation, PlannerCancellation
from annual_runtime.static_cancellation import StaticCancellationExecutor

ROOT = Path(__file__).resolve().parents[1]


def task(**changes):
    return dict(session='isolated-session', task=7, proposal=12, map_revision=9, **changes)


def test_cancel_before_delivery_is_idempotent_and_rejects_late_activation():
    key = task(); executor = StaticCancellation(key['session'])
    planner = PlannerCancellation(key)
    request = planner.request()
    ack = executor.request(request)
    assert ack['state'] == 'revoked_before_start'
    assert executor.request(request) == ack
    assert planner.acknowledge(ack) and planner.resolved
    assert not executor.admit(key, 'late-token')
    assert not planner.may_accept_result(key)
    with pytest.raises(ValueError):
        executor.activated(key, 'late-token')


def test_active_cancel_requires_exact_ack_and_original_stop_certificate():
    key = task(); executor = StaticCancellation(key['session'])
    assert executor.admit(key, 'execution-1')
    executor.activated(key, 'execution-1')
    planner = PlannerCancellation(key)
    ack = executor.request(planner.request())
    assert ack['state'] == 'waiting_safe_rest' and not planner.acknowledge(ack)
    assert not planner.resolved
    for change in ({'session': 'old-session'}, {'task': 8}, {'map_revision': 10}, {'proposal': 13}):
        wrong = copy.deepcopy(ack); wrong['task'].update(change)
        wrong.update(terminal=True, state='revoked_before_start')
        assert not planner.acknowledge(wrong)
    with pytest.raises(ValueError):
        executor.retire(key, 'execution-1', {'reference_stopped': True})
    proof = dict(reference_stopped=True, observation_complete=False,
                 original_safety_checks_passed=True, checked_at=20.)
    with pytest.raises(ValueError):
        executor.retire(key, 'wrong-token', proof)
    final = executor.retire(key, 'execution-1', proof)
    assert planner.acknowledge(final) and planner.acknowledge(final)
    assert executor.request(planner.request()) == final
    assert not executor.admit(key, 'execution-2')


def test_conflicting_version_session_capacity_and_invalid_wire_values_fail_closed():
    key = task(); executor = StaticCancellation(key['session'], capacity=1)
    assert executor.admit(key, 'one')
    for field, value in (('map_revision', 10), ('task', 8), ('session', 'wrong'),
                         ('task', True), ('proposal', 2**53)):
        changed = dict(key); changed[field] = value
        with pytest.raises(ValueError):
            executor.admit(changed, 'one')
    changed = dict(key, proposal=13, task=8)
    with pytest.raises(ValueError, match='full'):
        executor.admit(changed, 'two')
    assert len(executor.records) == 1
    for revision in (0, 2, True, 1.0):
        with pytest.raises(ValueError):
            executor.request(dict(version=1, task=key, revision=revision))


def safety_callback():
    path = ROOT/'src/annual_planner/src/annual_runtime/view_executor_node.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ViewExecutor')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and
               n.name in {'check_execution_safety', '_odometry_fresh', '_defer_execution_check'}]
    for method in methods:
        method.decorator_list = []
    from core.planning import execution_safety
    env = dict(vars(execution_safety))
    env.update(np=np, time=SimpleNamespace(monotonic=lambda: 0.),
               evidence_corridor=lambda node, corridor: corridor,
               record=lambda *args: None, executor_state=lambda node: {})
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), env)
    return env['check_execution_safety'], env


class Harness(StaticCancellationExecutor):
    def report(self):
        pass

    def fail_execution_safety(self, reason):
        self.safety_fault = reason

    def save_safety_certificate(self, *args):
        pass


def harness():
    from test_execution_safety import maps, recorded_curve
    node = Harness(); node.safety_enabled = True; node._static_cancel_setup('isolated-session')
    node.current_static_handoff = True
    node.collision_stop_request = None
    node.collision_stop_receipt = None
    node.token = 'execution-1'; node.static_task = task(); node.pending = None
    node.static_cancellations.activated(node.static_task, node.token)
    node.curve = recorded_curve(); node.curve_time = node.curve.duration
    node.ref = node.curve.sample(node.curve.duration)[0].copy()
    runtime, native = maps()
    node.id = 0; node.positions = {0: node.ref.copy()}; node.stamps = {0: 89.5}
    node.native_safety = native; node.safety_stamp = 89.2
    node.safety_replica = SimpleNamespace(map=runtime)
    node.safety_fault = None; node.arrived = False; node.safety_stopping = False
    node.safety_stop_deadline = None; node.terminal_hold_since = 89.4
    node.observation_failure = None; node.observation_completion = SimpleNamespace(max_age_s=1.5)
    node.terminal_hold_status = None; node.safety_dirty = True; node.last_safety_check = -1.
    node.observation_proof = {'must_not_be_awarded': True}; node.hold_since = 89.4
    node._static_cancel_command({'static_cancel': PlannerCancellation(task()).request()})
    return node


@pytest.mark.parametrize('failure', [None, 'unknown_geometry', 'stale_map', 'stale_odometry', 'over_budget', 'evidence_over_budget'])
def test_actual_terminal_cancel_waits_for_current_safety(failure):
    callback, env = safety_callback(); node = harness()
    node._odometry_fresh = env['_odometry_fresh'].__get__(node)
    node._defer_execution_check = env['_defer_execution_check'].__get__(node)
    reference = node.ref.copy(); curve = node.curve; token = node.token
    if failure == 'unknown_geometry':
        cell = node.safety_replica.map.indices(node.ref)
        node.safety_replica.map.state[tuple(cell)] = -1
    elif failure == 'stale_map':
        node.safety_stamp = 80.
    elif failure == 'stale_odometry':
        node.stamps[node.id] = 88.
    elif failure == 'over_budget':
        values = iter([0., .201])
        env['time'] = SimpleNamespace(monotonic=lambda: next(values, .201))
    elif failure == 'evidence_over_budget':
        elapsed = [0.]
        env['time'] = SimpleNamespace(monotonic=lambda: elapsed[0])
        node.save_safety_certificate = lambda *args: elapsed.__setitem__(0, .201)
    callback(node, 89.5)
    assert node.curve is curve and node.token == token
    np.testing.assert_array_equal(node.ref, reference)
    assert node.observation_proof is None
    if failure == 'unknown_geometry':
        assert node.safety_fault and not node.static_cancel_ack['terminal']
        assert node._static_cancel_waiting()
        return
    if failure in ('stale_map', 'stale_odometry', 'over_budget'):
        assert node.safety_fault is None and not node.safety_hold_valid
        assert node._static_cancel_waiting() and not node.static_cancel_ack['terminal']
        node.safety_stamp = 89.5
        node.stamps[node.id] = 89.5
        env['time'] = SimpleNamespace(monotonic=lambda: 0.)
        callback(node, 89.5)
    assert node.safety_fault is None and node.arrived
    assert node.static_cancel_ack['state'] == 'retired_at_safe_rest'
    assert node.static_cancel_ack['proof']['observation_complete'] is False


def test_executor_default_disabled_and_request_does_not_mutate_motion():
    node = Harness(); node.safety_enabled = True; node._static_cancel_setup('')
    node.token = 'old'; node.ref = np.ones(3); node.curve = object()
    curve = node.curve
    assert node._static_cancel_command({'static_cancel': PlannerCancellation(task()).request()})
    assert node.static_cancel_ack['state'] == 'rejected'
    assert node.curve is curve and node.token == 'old'
    np.testing.assert_array_equal(node.ref, np.ones(3))
    assert not node._static_task_admit(dict(static_task=task(), token='new'))


def gateway():
    from annual_runtime.static_cancel_gateway import StaticCancelGateway
    class Gateway(StaticCancelGateway):
        def now(self):return 20.
        def publish(self, pub, packet):self.commands.append(packet)
        def _static_send_ack(self, key, status):self.receipts.append((key, status))
    node = Gateway(); node.safety_enabled = True; node.config = {'stop_and_plan': True}
    node._static_gateway_setup('123')
    node.authorization_session = '123'; node.failed_observations = {}
    node.pending = None; node.intent = None; node.command_pub = object()
    node.commands = []; node.receipts = []; node.last_rejection = -1
    return node


def test_default_cancellation_ledgers_keep_accepting_ids_and_retaining_tombstones():
    from annual_runtime.static_cancel_gateway import task_wire
    node = gateway()
    ledger = StaticCancellation('123')
    first = dict(task(), session='123')
    for proposal in range(first['proposal'], first['proposal']+260):
        key = dict(first, proposal=proposal)
        node._static_cancel_request(task_wire(key))
        assert ledger.request(PlannerCancellation(key).request())['state'] == 'revoked_before_start'
    fresh = dict(first, proposal=first['proposal']+260)
    assert node._static_before_propose(dict(traj_id=fresh['proposal'], static_task=fresh))
    assert ledger.admit(fresh, 'fresh-token')
    assert not node._static_before_propose(dict(traj_id=first['proposal'], static_task=first))
    assert not ledger.admit(first, 'replayed-token')


def test_gateway_late_proposal_tombstone_and_terminal_retries():
    from annual_runtime.static_cancel_gateway import task_wire
    node = gateway(); key = dict(task(), session='123')
    wire = task_wire(key)
    node._static_cancel_request(wire)
    assert node.receipts[-1] == (key, 2)
    assert not node._static_before_propose({'traj_id': 12, 'static_task': key})
    assert not node._static_before_propose({'traj_id': 12})
    node._static_cancel_request(wire)
    assert node.receipts == [(key, 2), (key, 2)] and not node.commands
    wrong = list(wire); wrong[4] += 1
    with pytest.raises(ValueError):
        node._static_cancel_request(wrong)
    assert node.receipts == [(key, 2), (key, 2)]


def test_gateway_sent_task_waits_for_exact_fresh_executor_receipt():
    from annual_runtime.static_cancel_gateway import task_wire
    node = gateway(); key = dict(task(), session='123')
    row = dict(traj_id=12, static_task=key)
    assert node._static_before_propose(row)
    node.pending = dict(native_id=12, token='sent-token', committed=True)
    node._static_attach_pending(row)
    node._static_command_sent(dict(node.pending, trajectory={'real_curve': 'not evaluated in protocol test'}))
    node._static_cancel_request(task_wire(key))
    assert node.pending and node.commands and not node.receipts
    executor = StaticCancellation('123'); executor.activated(key, 'sent-token')
    ack = executor.request(node.commands[-1]['static_cancel'])
    node._static_execution_ack(dict(time=20., ready=True, static_cancel_ack=ack))
    assert node.pending and node.receipts[-1][1] == 1
    proof = dict(reference_stopped=True, observation_complete=False,
                 original_safety_checks_passed=True, checked_at=19.9)
    final = executor.retire(key, 'sent-token', proof)
    wrong = dict(final, executor_token='another-token')
    for packet in (dict(time=19., ready=True, static_cancel_ack=final),
                   dict(time=20., ready=False, static_cancel_ack=final),
                   dict(time=20., ready=True, static_cancel_ack=wrong)):
        node._static_execution_ack(packet)
        assert node.pending and node.receipts[-1][1] == 1
    node._static_execution_ack(dict(time=20., ready=True, static_cancel_ack=final))
    assert node.pending is None and node.receipts[-1] == (key, 3)
    node._static_cancel_request(task_wire(key))
    assert node.receipts[-1] == (key, 3)  # No downgrade to never-started.
    assert 'sent-token' in node.static_retired_tokens


@pytest.mark.parametrize('slot', ['pending', 'intent'])
def test_late_static_metadata_cannot_revoke_committed_or_active_unlabeled_proposal(slot):
    from annual_runtime.static_cancel_gateway import task_wire
    node = gateway()
    key = dict(task(), session='123')
    record = dict(native_id=key['proposal'], token='sent-without-static-task', committed=True)
    setattr(node, slot, record)
    node._static_cancel_request(task_wire(key))
    assert getattr(node, slot) is record
    assert node.commands[-1]['static_cancel']['task'] == key
    assert not node.receipts


def test_gateway_completed_local_intent_is_not_evidence_of_never_executed():
    from annual_runtime.static_cancel_gateway import task_wire
    node = gateway(); key = dict(task(), session='123')
    node._static_command_sent(dict(static_task=key, token='completed', trajectory={}))
    node._static_cancel_request(task_wire(key))
    assert node.commands and not node.receipts


def test_native_cancellation_gate_keeps_slot_until_inactive_feedback(tmp_path):
    from test_native_static_events import compile_run
    compile_run(tmp_path, r'''#include "static_cancellation.hpp"
#include <cassert>
using namespace racer_task;
int main(){
 StaticCancellation c;assert(!c.bind(7,12,9));c.session=123;
 assert(c.bind(7,12,9));assert(!c.released(12,false,false));
 assert(c.due(20,.5));assert(!c.due(20.1,.5));assert(c.due(20.5,.5));
 assert(!c.released(-1,false,true)); // Rejection before terminal receipt is not enough.
 auto receipt=c.identity;receipt.push_back(3);
 auto wrong=receipt;wrong[4]++;assert(!c.acknowledge(wrong));
 assert(c.acknowledge(receipt));assert(!c.released(12,false,false));
 assert(c.released(-1,false,true));assert(!c.due(21,.5));
 assert(c.acknowledge(receipt));receipt[6]=1;assert(c.acknowledge(receipt)&&c.terminal);
 assert(c.bind(8,13,10));assert(!c.acknowledge(wrong));
 assert(c.released(13,true,false));assert(!c.released(13,false,false));
 assert(!c.bind(9,2147483648.,10)); // Native IDs are signed ints.
}''')


def test_generated_cancellation_source_and_metadata_are_connected(tmp_path):
    import subprocess
    output = tmp_path/'native'
    subprocess.run(['python3', str(ROOT/'tools/search/build_native.py'), str(output),
                    '--execution-safety', '--consistent-inflation'],
                   check=True, capture_output=True, text=True)
    fsm = (output/'src/racer_core/src/exploration_manager/fast_exploration_fsm.cpp').read_text()
    assert '/baseline/static_cancel_ack_' in fsm and '/baseline/static_cancel_' in fsm
    assert 'task_static_metadata_.data.push_back(task_static_cancel_.session)' in fsm
    assert 'task_static_cancel_.released(active,d[2]>0.' in fsm
    assert 'task_static_cancel_.acknowledge(msg->data)' in fsm
    assert 'task_static_cancel_.due(now,task_static_gate_.min_interval_s)' in fsm
    assert 'task_static_cancel_.enabled()&&(state_==PLAN_TRAJ||state_==PUB_TRAJ||state_==IDLE)' in fsm
    assert 'if(task_stop_and_plan_ && live_execution_ && executed_id_>=0 && !executed_done_ && !task_c2_requested) return;' in fsm


def test_actual_gateway_feedback_does_not_resurrect_a_retired_completed_token():
    path = ROOT/'tools/research/frontier_baselines/live_ros2.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'NativeLeaseAgent')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('tick','send_execution_feedback')]
    env = dict(np=np, completed_execution_proof=lambda report: True)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), env)
    node = gateway(); sent = []
    node.pose = np.zeros(3); node.proposals = SimpleNamespace(empty=lambda: True)
    node.last_sensor = 20.; node.last_grid = 20.; node.sequence = 0
    node.ledger = SimpleNamespace(acknowledge=lambda state, time: [])
    node.state = lambda: {}; node.state_pub = object()
    node.execution = dict(token='finished', ready=True, trajectory_time=3.)
    node.collision_stop_released = None
    node.fast_stop_feedback = False
    node.send_execution_feedback = lambda stamp:env['send_execution_feedback'](node,stamp)
    node.completed = {'finished'}; node.last_completed_id = 12
    node.transport = SimpleNamespace(send=sent.append); node.id = 0
    node.flight_fault = SimpleNamespace(reason=None)
    env['tick'](node)
    assert sent[-1]['data'][0] == 12 and sent[-1]['data'][2] == 1
    node.static_retired_tokens.add('finished')
    env['tick'](node)
    assert sent[-1]['data'][0] == -1 and sent[-1]['data'][2] == 0
    assert node.completed == {'finished'}  # Preserve historical observation evidence.


def test_actual_executor_duplicate_epoch_is_ignored_before_rejecting_active_task():
    import json
    path = ROOT/'src/annual_planner/src/annual_runtime/view_executor_node.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ViewExecutor')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'command')
    env = dict(json=json, record=lambda *args: None, time=SimpleNamespace(monotonic=lambda: 0.))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    node = Harness(); node.safety_enabled = True; node._static_cancel_setup('isolated-session')
    node.current_static_handoff = True
    node.flight_fault = SimpleNamespace(reason=None); node.safety_fault = None; node.epoch = 1
    def unexpected(packet):
        raise AssertionError('Duplicate delivery must not reject an already active task')
    node._static_task_admit = unexpected
    packet = dict(epoch=1, token='already-active', static_task=task(),
                  authorization_session='isolated-session')
    env['command'](node, SimpleNamespace(data=json.dumps(packet)))


@pytest.mark.parametrize('session', [None, 'previous-run'])
def test_actual_executor_old_session_cannot_cancel_current_motion(session):
    import json
    path = ROOT/'src/annual_planner/src/annual_runtime/view_executor_node.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ViewExecutor')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'command')
    env = dict(json=json, record=lambda *args: None, time=SimpleNamespace(monotonic=lambda: 0.))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    node = Harness(); node.safety_enabled = True; node._static_cancel_setup('isolated-session')
    node.current_static_handoff = True
    node.flight_fault = SimpleNamespace(reason=None); node.safety_fault = None
    node.epoch = 0; node.ready = True; node.ref = np.ones(3)
    def forbidden(reason):
        raise AssertionError('Old-session cancellation changed current motion')
    node.stop = forbidden
    packet = dict(epoch=99, cancel=True, authorization_session=session)
    env['command'](node, SimpleNamespace(data=json.dumps(packet)))
    assert node.epoch == 0
    np.testing.assert_array_equal(node.ref, np.ones(3))


def test_actual_authorizer_stamps_commands_without_mutating_source_packet():
    import json
    path = ROOT/'tools/research/frontier_baselines/live_ros2.py'
    cls = next(n for n in ast.parse(path.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'NativeLeaseAgent')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'publish')
    env = dict(json=json, String=lambda **kw: SimpleNamespace(**kw))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    node = gateway(); sent = []; node.command_pub = SimpleNamespace(publish=sent.append)
    packet = dict(epoch=99, cancel=True)
    env['publish'](node, node.command_pub, packet)
    assert json.loads(sent[-1].data)['authorization_session'] == '123'
    assert packet == dict(epoch=99, cancel=True)
    other = SimpleNamespace(publish=sent.append)
    env['publish'](node, other, packet)
    assert 'authorization_session' not in json.loads(sent[-1].data)


@pytest.mark.parametrize('change', ['session','map','token','successor','proof','stale','not_ready','none'])
def test_superseded_late_cancel_requires_exact_sent_successor_relation(change):
    from annual_runtime.static_cancel_gateway import task_wire
    node=gateway();key=dict(task(),session='123');successor=dict(key,task=8,proposal=13)
    node._static_command_sent(dict(static_task=key,token='old',trajectory={}))
    node._static_command_sent(dict(static_task=successor,token='new',trajectory={},handoff={'from_token':'old'}))
    ledger=StaticCancellation('123');ledger.activated(key,'old')
    ledger.supersede(key,'old','new',dict(authorized_transition_passed=True,
                     observation_complete=False,checked_at=19.8))
    node._static_cancel_request(task_wire(key))
    ack=ledger.request(node.commands[-1]['static_cancel'])
    # Another successor may already be running when this retry returns.
    report=dict(time=20.,ready=True,token='later',handoff_from_token='new',static_cancel_ack=ack)
    if change=='session':ack['task']['session']='previous'
    elif change=='map':ack['task']['map_revision']+=1
    elif change=='token':ack['executor_token']='unrelated'
    elif change=='successor':ack['proof']['successor_token']='never-sent'
    elif change=='proof':ack['proof']['authorized_transition_passed']=False
    elif change=='stale':report['time']=19.
    elif change=='not_ready':report['ready']=False
    node._static_execution_ack(report)
    if change!='none':assert not node.static_receipts and not node.static_retired_tokens
    else:
        assert node.static_receipts=={12:4} and node.static_retired_tokens=={'old'}
        node._static_cancel_request(task_wire(key));assert node.receipts[-1]==(key,4)
        node._static_execution_ack(report);assert node.static_receipts=={12:4}
