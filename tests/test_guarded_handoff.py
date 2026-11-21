"""Host geometric/transaction contracts, not native ROS or physical C2 proof."""
import math
import numpy as np
import pytest
from core.planning.continuous_trajectory import ContinuousTrajectory
from core.planning.execution_safety import ExecutionCorridor
from core.planning.handoff import curve_fingerprint, validate_safe_handoff, HandoffBudgetExceeded
from test_execution_safety import maps
from types import SimpleNamespace
from annual_runtime.guarded_c2 import GuardedC2Executor
from annual_runtime.static_cancellation import StaticCancellationExecutor


def pair():
    coefficients = np.zeros((6, 3))
    coefficients[0] = [5., 5., 1.5]
    coefficients[3:, 0] = [.4*10, -.4*15, .4*6]
    old = ContinuousTrajectory([8.], [coefficients], 0., 0.,
                               yaw_durations=[8.], yaw_segments=np.zeros((1, 6)))
    # Exact polynomial suffix is a fixture, not a new online planning algorithm.
    alpha = 2./8.; beta = 6./8.
    shifted = np.zeros_like(coefficients)
    for j in range(6):
        shifted[j] = sum(coefficients[k]*math.comb(k, j)*alpha**(k-j)*beta**j
                         for k in range(j, 6))
    new = ContinuousTrajectory([6.], [shifted], 0., 0.,
                               yaw_durations=[6.], yaw_segments=np.zeros((1, 6)))
    runtime, native = maps()
    return old, new, runtime, native


def inputs(old, runtime, native):
    return dict(expected_old=curve_fingerprint(old), now=89.5, native_stamp=89.2,
                mapper_stamp=89.2, corridor=ExecutionCorridor(runtime, native),
                started=0., clock=lambda: 0.)


@pytest.mark.parametrize('phase', ['admission', 'transition'])
def test_actual_c2_boundary_and_stopping_reserve_at_both_validation_points(phase):
    old, new, runtime, native = pair()
    kwargs = inputs(old, runtime, native)
    progress = 1.7
    if phase == 'transition':
        progress = 1.98
        kwargs.update(new_progress=.02, reference_step=.04,
                      expected_new=curve_fingerprint(new))
    result = validate_safe_handoff(old, new, 2., progress, phase=phase, **kwargs)
    assert max(result['continuity'].values()) < 1e-12
    assert result['phase'] == phase
    assert result['new_curve_sha256'] == curve_fingerprint(new)


@pytest.mark.parametrize('failure', ['old_curve', 'new_curve', 'safety_revision', 'late',
                                    'velocity', 'unknown', 'stale', 'dynamics', 'rest'])
def test_changes_since_planning_or_invalid_current_geometry_are_rejected(failure):
    old, new, runtime, native = pair()
    kwargs = inputs(old, runtime, native); progress = 1.7
    if failure == 'old_curve':old.coefficients[0, 1, 0] += .001
    elif failure == 'new_curve':kwargs['expected_new'] = 'wrong'
    elif failure == 'safety_revision':kwargs['old_safety_revision'] = 1
    elif failure == 'late':progress = 1.96
    elif failure == 'velocity':new.coefficients[0, 1, 0] += .001
    elif failure == 'unknown':
        runtime.state[tuple(runtime.indices(new.sample(0)[0]))] = -1
        runtime.rebuild()
    elif failure == 'stale':kwargs['mapper_stamp'] = 87.4
    elif failure == 'dynamics':
        # Change the interior while preserving the initial p/v/a jet.
        new.coefficients[0, 3, 0] += 100.
    elif failure == 'rest':new.coefficients[0, 5, 0] += .001
    with pytest.raises(ValueError):
        validate_safe_handoff(old, new, 2., progress, **kwargs)


def test_transition_uses_actual_step_and_rejects_consumed_or_skipped_boundaries():
    old, new, runtime, native = pair()
    kwargs = inputs(old, runtime, native)
    for progress, new_progress, step in [(2.01, .02, .01), (1.98, .2, .04), (1.98, .02, .03)]:
        with pytest.raises(ValueError):
            validate_safe_handoff(old, new, 2., progress, phase='transition',
                                  new_progress=new_progress, reference_step=step, **kwargs)


def test_shared_safety_cycle_budget_is_not_restarted_for_new_curve_check():
    old, new, runtime, native = pair()
    kwargs = inputs(old, runtime, native); elapsed = [.199]
    kwargs['clock'] = lambda: elapsed[0]
    corridor = kwargs['corridor']; original = corridor.safe_curve
    def expensive(*args):
        result = original(*args); elapsed[0] = .201
        return result
    corridor.safe_curve = expensive
    with pytest.raises(HandoffBudgetExceeded):
        validate_safe_handoff(old, new, 2., 1.7, **kwargs)
    elapsed[0] = .201
    with pytest.raises(HandoffBudgetExceeded):
        validate_safe_handoff(old, new, 2., 1.7, **kwargs)


class Executor(GuardedC2Executor, StaticCancellationExecutor):
    def report(self):pass
    def reject_activation(self, packet, reason):self.rejected=reason
    def _defer_execution_check(self):self.safety_hold_valid=False
    def save_safety_certificate(self, *args, **kwargs):pass


def executor(monkeypatch):
    import annual_runtime.guarded_c2 as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 0.)
    old, new, runtime, native = pair()
    node = Executor(); node.safety_enabled = True; node._static_cancel_setup('123');node._c2_setup(True)
    node.curve = old; node.curve_time = 1.7; node.token = 'old'; node.epoch = 3
    node.id = 0; node.ref = old.sample(1.7)[0].copy(); node.positions = {0: node.ref.copy()}
    node.stamps = {0: 89.5}; node.native_safety = native; node.safety_stamp = 89.2
    node.safety_replica = SimpleNamespace(map=runtime)
    node.arrived = node.paused = node.done = False
    node.safety_fault = node.safety_revision = node.pending = None
    node.safety_hold_valid = True
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=89_500_000_000))
    packet = dict(token='new', epoch=4, authorization_session='123',
                  static_task=dict(session='123', task=1, proposal=12, map_revision=9),
                  handoff=dict(from_token='old', from_epoch=3, trajectory_time=2.,
                               old_curve_sha256=curve_fingerprint(old)))
    return node, packet, new


def test_executor_rechecks_at_transition_and_retains_old_motion_on_revision(monkeypatch):
    node, packet, new = executor(monkeypatch)
    assert node._c2_admit(packet, new, 0.)
    old = node.curve; pending = node.pending
    node.curve_time = 1.98; node.ref = old.sample(1.98)[0].copy();node.positions[0] = node.ref.copy()
    assert node._c2_transition(pending, 2.02, 89.5, 0.)
    assert node.curve is old and node.token == 'old'  # Validation is not activation.
    node.safety_revision = {'number': 1}
    assert not node._c2_transition(pending, 2.02, 89.5, 0.)
    assert node.pending is None and node.curve is old and node.token == 'old'
    assert 'revision' in node.rejected


def test_executor_pending_cancel_blocks_late_switch(monkeypatch):
    node, packet, new = executor(monkeypatch)
    assert node._c2_admit(packet, new, 0.)
    node._static_cancel_command(dict(static_cancel=dict(version=1, revision=1, task=packet['static_task'])))
    assert node.pending is None and node.static_cancel_ack['state'] == 'revoked_before_start'
    assert not node._static_task_admit(packet) and node.token == 'old'


def test_transition_evidence_overrun_rejects_candidate_and_defers_predecessor(monkeypatch):
    import annual_runtime.guarded_c2 as module
    node, packet, new = executor(monkeypatch)
    assert node._c2_admit(packet, new, 0.)
    node.curve_time = 1.98; node.ref = node.curve.sample(1.98)[0].copy();node.positions[0] = node.ref.copy()
    old = node.curve
    elapsed = [.1];monkeypatch.setattr(module.time, 'monotonic', lambda: elapsed[0])
    node.save_safety_certificate = lambda *a, **kw: elapsed.__setitem__(0, .201)
    assert not node._c2_transition(node.pending, 2.02, 89.5, 0.)
    assert node.safety_fault is None
    assert node.pending is None and not node.safety_hold_valid
    assert not node._static_task_admit(packet)
    assert node.rejected == 'handoff_safety_check_delay_budget_exceeded'
    assert node.curve is old and node.token == 'old'


def test_native_window_rejects_planning_delay_stale_versions_and_insufficient_lead(tmp_path):
    from test_native_static_events import compile_run
    compile_run(tmp_path, r'''#include "static_c2_gate.hpp"
#include <cassert>
using namespace racer_task;
int main(){
 StaticC2Gate gate;assert(!gate.prepare(4,1.,5.,10.,10.,3));
 gate.enabled=true;gate.lead=1.;assert(gate.prepare(4,1.,5.,10.,10.,3));
 assert(gate.boundary==2.);gate.proposal=5;assert(gate.owns(5)&&!gate.owns(4));
 assert(gate.current(4,1.2,10.2,10.2,3));
 assert(!gate.current(6,1.2,10.2,10.2,3));assert(!gate.current(4,1.2,10.2,10.2,4));
 gate.planning_wall=.96;assert(!gate.current(4,1.,10.1,10.1,3));
 gate.planning_wall=0.;assert(!gate.current(4,1.96,10.5,10.5,3));
 assert(!gate.current(4,1.,10.6,10.,3));assert(!gate.prepare(4,4.2,5.,10.,10.,3));
 assert(gate.boundary<0.&&!gate.owns(5));
}''')


def test_actual_activation_supersedes_old_task_and_late_cancel_is_terminal(monkeypatch):
    from core.planning.static_cancellation import PlannerCancellation
    node, packet, new = executor(monkeypatch)
    old_task = dict(packet['static_task'], task=9, proposal=11)
    node._static_task_activated(dict(static_task=old_task, token='old'))
    assert node._c2_admit(packet, new, 0.)
    pending = node.pending
    node.curve_time=1.98;node.ref=node.curve.sample(1.98)[0].copy();node.positions[0]=node.ref.copy()
    assert node._c2_transition(pending, 2.02, 89.5, 0.)
    node._c2_applied(pending, 89.5)
    assert node.static_task == packet['static_task']
    assert node.static_cancel_ack['state'] == 'superseded_by_c2'
    assert node.static_cancel_ack['revision'] == 0
    planner=PlannerCancellation(old_task)
    receipt=node.static_cancellations.request(planner.request())
    assert receipt['revision']==1 and receipt['proof']['successor_token']=='new'
    assert planner.acknowledge(receipt) and planner.acknowledge(receipt)
    assert node.static_cancellations.request(planner.request())==receipt
    assert not node.static_cancellations.admit(old_task, 'old')


def test_admission_evidence_overrun_rejects_only_candidate(monkeypatch):
    import annual_runtime.guarded_c2 as module
    node, packet, new = executor(monkeypatch)
    elapsed=[0.];monkeypatch.setattr(module.time,'monotonic',lambda:elapsed[0])
    node.save_safety_certificate=lambda *a,**kw:elapsed.__setitem__(0,.201)
    old=node.curve
    assert not node._c2_admit(packet,new,0.)
    assert node.pending is None and node.curve is old and node.token == 'old'
    assert node.safety_fault is None and node.safety_dirty
    assert node.rejected == 'handoff_safety_check_delay_budget_exceeded'
    assert not node._static_task_admit(packet)
    node._static_cancel_command(dict(static_cancel=dict(version=1, revision=1, task=packet['static_task'])))
    assert node.static_cancel_ack['state'] == 'revoked_before_start'
    assert node.static_cancel_ack['revision'] == 1


def test_native_actual_accepted_feedback_releases_only_compute_slot(tmp_path):
    from test_native_static_events import compile_run
    compile_run(tmp_path, r'''#include "static_execution_monitor.hpp"
#include "static_c2_gate.hpp"
#include "static_replan_gate.hpp"
#include <memory>
#include <cassert>
using namespace racer_task;
struct Manager {StaticGuidanceEvents task_static_events_;std::optional<StaticGuidanceEvents::Work> task_static_evaluation_;};
struct FastExplorationFSM {
 StaticC2Gate task_static_c2_;StaticExecutionMonitor task_static_monitor_;
 StaticCancellation task_static_cancel_;StaticReplanGate task_static_gate_;
 std::optional<StaticGuidanceEvents::Work> task_static_work_;
 std::unique_ptr<int> executed_data_=std::make_unique<int>(1);int executed_id_=7;
 bool task_static_prior_goal_valid_=true;
 std::shared_ptr<Manager> expl_manager_=std::make_shared<Manager>();
 void taskStaticExecutionFeedback(int,bool,int,bool,double=0.,double=0.);
};
#include "static_c2_methods.cpp"
TargetObservation cue(int id){TargetObservation o;o.key={"uav0","s",static_cast<std::uint64_t>(id)};o.sequence=1;
 o.first=1.;o.capture=1.4;o.frame_version="map";o.has_bearing=true;
 o.direction={1,0,0};o.support=3;o.support_first=1.;o.confidence=.6;return o;}
void bind(FastExplorationFSM& f,int proposal){
 f.task_static_work_=f.expl_manager_->task_static_events_.start();assert(f.task_static_work_);
 f.expl_manager_->task_static_evaluation_=f.task_static_work_;
 assert(f.task_static_gate_.planned(proposal,f.executed_id_,2.,9));f.task_static_gate_.published=true;
 assert(f.task_static_cancel_.bind(f.task_static_work_->token,proposal,9));
}
void checkSameEntityReservation(){
 TargetRegistry registry;
 auto original=cue(1);
 assert(registry.ingest(original,1.4).accepted);
 StaticGuidanceEvents inbox;
 inbox.sync(registry,1.4);
 auto running=inbox.start();
 assert(running);
 StaticCancellation cancel;cancel.session=123;
 assert(cancel.bind(running->token,12,9));
 StaticExecutionMonitor monitor;
 assert(monitor.accept(*running,cancel,12,12,false));
 monitor.feedback(12,false);
 assert(inbox.finish(running->token));

 auto reacquired=original;
 reacquired.sequence=2;reacquired.capture=4.0;reacquired.support_first=3.5;
 assert(registry.ingest(reacquired,4.0).accepted);
 inbox.sync(registry,4.0);
 assert(inbox.stats().pending==1&&!inbox.current(*running));
 assert(!inbox.start(4.0,monitor.reservedSources()));

 auto other=cue(2);
 other.first=3.6;other.capture=4.1;other.support_first=3.6;
 assert(registry.ingest(other,4.1).accepted);
 inbox.sync(registry,4.1);
 auto admitted=inbox.start(4.1,monitor.reservedSources());
 assert(admitted&&admitted->entity==other.key);
 assert(inbox.finish(admitted->token));

 int requests=0;
 monitor.tick(inbox,4.1,.5,[&](const auto& identity){
  assert(identity==cancel.identity);++requests;
 });
 assert(requests==1);
 auto receipt=cancel.identity;
 receipt.push_back(3);
 assert(monitor.acknowledge(receipt));
 assert(!inbox.start(4.1,monitor.reservedSources()));
 monitor.feedback(-1,false);
 auto followup=inbox.start(4.1,monitor.reservedSources());
 assert(followup&&followup->entity==reacquired.key&&followup->token!=running->token);
}
void checkExecutingTaskPastCaptureTTL(){
 FastExplorationFSM f;f.task_static_c2_.enabled=true;f.task_static_cancel_.session=123;
 TargetRegistry registry;auto original=cue(1);
 assert(registry.ingest(original,1.4).accepted);
 auto& inbox=f.expl_manager_->task_static_events_;
 inbox.sync(registry,1.4);bind(f,12);
 const auto running=*f.task_static_work_;
 f.executed_id_=12;f.taskStaticExecutionFeedback(12,false,12,false);
 assert(inbox.stats().active==0&&f.task_static_monitor_.size()==1);
 registry.retire(4.0);inbox.sync(registry,4.0);
 assert(inbox.current(running));
 int requests=0;
 auto send=[&](const auto&){++requests;};
 f.task_static_monitor_.tick(inbox,4.0,.5,send);assert(requests==0);
 auto confirmed=original;confirmed.sequence=2;confirmed.capture=4.1;
 confirmed.visual="visually_confirmed";
 assert(registry.ingest(confirmed,4.1).accepted);
 inbox.sync(registry,4.1);
 assert(!inbox.current(running));
 f.task_static_monitor_.tick(inbox,4.1,.5,send);assert(requests==1);
}
void checkConfirmationAfterIntegratedArrival(bool confirmed_after_arrival){
 TargetRegistry registry;auto observation=cue(1);
 assert(registry.ingest(observation,1.4).accepted);
 StaticGuidanceEvents inbox;inbox.sync(registry,1.4);
 auto task=inbox.start();assert(task);
 StaticCancellation cancel;cancel.session=123;assert(cancel.bind(task->token,12,9));
 StaticExecutionMonitor monitor;assert(monitor.accept(*task,cancel,12,12,false));
 assert(inbox.finish(task->token));
 inbox.setExecutingSources(monitor.reservedSources());
 monitor.feedback(12,true,5.,1.);
 std::vector<std::vector<double>> requests;
 auto send=[&](const auto& data){requests.push_back(data);};
 assert(monitor.tick(inbox,5.1,.5,send)&&requests.empty());
 monitor.feedback(12,true,5.5,1.); // repeated completion must not extend the hold
 if(!confirmed_after_arrival){
  assert(!monitor.tick(inbox,6.,.5,send)&&requests.empty()&&monitor.size()==0);
  return;
 }
 observation.sequence=2;observation.capture=5.7;observation.visual="visually_confirmed";
 assert(registry.ingest(observation,5.8).accepted);inbox.sync(registry,5.8);
 assert(monitor.tick(inbox,5.8,.5,send)&&requests.size()==1);
 assert(requests.front()==cancel.identity);
 auto receipt=cancel.identity;receipt.push_back(3);
 assert(monitor.acknowledge(receipt)&&monitor.size()==1);
 monitor.feedback(-1,false,6.1,1.);
 assert(!monitor.tick(inbox,6.1,.5,send)&&monitor.size()==0);
}
int main(){checkSameEntityReservation();checkExecutingTaskPastCaptureTTL();
 checkConfirmationAfterIntegratedArrival(false);checkConfirmationAfterIntegratedArrival(true);
 FastExplorationFSM f;f.task_static_c2_.enabled=true;f.task_static_cancel_.session=123;
 TargetRegistry r;r.ingest(cue(1),1.4);r.ingest(cue(2),1.4);
 auto& inbox=f.expl_manager_->task_static_events_;inbox.sync(r,1.4);bind(f,12);
 auto old=*f.task_static_work_;
 f.taskStaticExecutionFeedback(7,false,12,false); // Published/pending is not accepted.
 assert(f.task_static_work_&&inbox.stats().active==1&&!inbox.start());
 f.taskStaticExecutionFeedback(12,false,12,false); // Actual native applied ID still old.
 assert(f.task_static_work_&&inbox.stats().active==1);
 f.executed_id_=12;f.taskStaticExecutionFeedback(12,false,12,false);
 assert(!f.task_static_work_&&!f.expl_manager_->task_static_evaluation_);
 assert(inbox.stats().active==0&&f.task_static_monitor_.size()==1&&inbox.current(old));
 assert(!f.task_static_prior_goal_valid_&&!f.task_static_gate_.published);
 bind(f,13);assert(inbox.stats().active==1&&!inbox.start()); // Old trajectory still moving.
 r.records.at(old.evidence.key).confirmed=true;inbox.sync(r,1.5);assert(!inbox.current(old));
 std::vector<std::vector<double>> requests;
 auto send=[&](const auto& d){requests.push_back(d);};
 f.task_static_monitor_.tick(inbox,2.,.5,send);assert(requests.size()==1);
 f.task_static_monitor_.tick(inbox,2.1,.5,send);assert(requests.size()==1);
 auto receipt=requests[0];receipt.push_back(4);auto wrong=receipt;wrong[1]++;
 assert(!f.task_static_monitor_.acknowledge(wrong));wrong=receipt;wrong[4]++;
 assert(!f.task_static_monitor_.acknowledge(wrong));
 assert(f.task_static_monitor_.acknowledge(receipt)); // Receipt first: old still active.
 assert(f.task_static_monitor_.size()==1&&inbox.stats().active==1);
 f.executed_id_=13;f.taskStaticExecutionFeedback(13,false,13,false);
 assert(!f.task_static_work_&&inbox.stats().active==0&&f.task_static_monitor_.size()==1);
 f.task_static_monitor_.feedback(-1,false); // Feedback first, retain until exact receipt.
 f.task_static_monitor_.tick(inbox,3.,.5,send);assert(requests.size()==2);
 auto second=requests[1];second.push_back(3);
 assert(!f.task_static_monitor_.acknowledge(receipt));
 assert(f.task_static_monitor_.size()==1);
 assert(f.task_static_monitor_.acknowledge(second));assert(f.task_static_monitor_.size()==0);
 // Cancellation requested before acceptance follows its monitor, not a new compute task.
 TargetRegistry r2;r2.ingest(cue(3),1.4);inbox.sync(r2,1.4);bind(f,14);
 assert(f.task_static_cancel_.due(4.,.5));auto third=f.task_static_cancel_.identity;third.push_back(3);
 f.executed_id_=14;f.taskStaticExecutionFeedback(14,false,14,false);
 assert(!f.task_static_work_&&f.task_static_monitor_.size()==1);
 f.task_static_monitor_.tick(inbox,4.5,.5,send);assert(requests.size()==3);
 assert(f.task_static_monitor_.acknowledge(third));assert(f.task_static_monitor_.size()==1);
 f.task_static_monitor_.feedback(-1,false);assert(f.task_static_monitor_.size()==0);
}
''')


def test_missing_curve_is_rejected_without_mutating_active_execution(monkeypatch):
    node,packet,new=executor(monkeypatch);old=node.curve
    assert not node._c2_admit(packet,None,0.)
    assert node.pending is None and node.curve is old and node.token=='old'
    assert 'curve and boundary' in node.rejected


def test_generated_moving_exception_keeps_default_stop_and_requires_live_transaction(tmp_path):
    import subprocess
    from test_native_static_events import ROOT, compile_run
    generated=tmp_path/'native'
    subprocess.run(['python3','-B',str(ROOT/'tools/search/build_native.py'),str(generated),
                    '--execution-safety','--consistent-inflation'],check=True,capture_output=True,text=True)
    fsm=(generated/'src/racer_core/src/exploration_manager/fast_exploration_fsm.cpp').read_text()
    block=fsm.split('      const bool task_c2_requested=')[1].split('      if(task_return_.started()')[0]
    block='      const bool task_c2_requested='+block
    compile_run(tmp_path,r'''#include "static_c2_gate.hpp"
#include "static_replan_gate.hpp"
#include <memory>
#include <optional>
#include <cassert>
namespace racer_ros {struct Time {static Time now(){return {};};double toSec()const{return 10.;}};}
struct Map {std::uint64_t taskStaticMapRevision(){return 3;}};
struct Manager {std::shared_ptr<Map> sdf_map_=std::make_shared<Map>();};
struct Data {double duration_=5.;};
struct FSM {
 racer_task::StaticC2Gate task_static_c2_;racer_task::StaticReplanGate task_static_gate_;
 std::optional<int> task_static_work_;
 bool live_execution_=true,executed_done_=false,task_stop_and_plan_=true,task_fresh_review_=false;
 bool planned=false,released=false;int executed_id_=4;
 double executed_progress_=1.,feedback_stamp_=10.;
 std::unique_ptr<Data> executed_data_=std::make_unique<Data>();
 std::shared_ptr<Manager> expl_manager_=std::make_shared<Manager>();
 void taskStaticRelease(bool,const char*){released=true;}
 void plan(){'''+block+r'''
 planned=true;
 }
};
int main(){
 FSM off;off.task_static_work_=1;off.plan();assert(!off.planned&&!off.released);
 FSM no_work;no_work.task_static_c2_.enabled=true;no_work.plan();assert(!no_work.planned);
 FSM valid;valid.task_static_c2_.enabled=true;valid.task_static_c2_.lead=1.;valid.task_static_work_=1;
 valid.plan();assert(valid.planned&&!valid.released&&valid.task_static_c2_.boundary==2.);
 FSM fresh;fresh.task_fresh_review_=true;fresh.task_static_c2_.enabled=true;
 fresh.task_static_c2_.lead=1.;fresh.task_static_work_=1;
 fresh.plan();assert(!fresh.planned&&fresh.task_static_c2_.boundary<0.);
 fresh.executed_done_=true;fresh.plan();assert(fresh.planned);
 FSM submitted;submitted.task_static_c2_.enabled=true;submitted.task_static_work_=1;
 submitted.task_static_gate_.published=true;submitted.plan();assert(!submitted.planned);
 FSM late;late.task_static_c2_.enabled=true;late.task_static_c2_.lead=5.;late.task_static_work_=1;
 late.plan();assert(!late.planned&&!late.released&&late.task_static_work_==1);
 late.plan();assert(!late.planned&&!late.released&&late.task_static_work_==1);
 late.executed_done_=true;
 late.plan();assert(late.planned&&!late.released&&late.task_static_work_==1);
 FSM stopped;stopped.executed_done_=true;stopped.plan();assert(stopped.planned);
}
''')
