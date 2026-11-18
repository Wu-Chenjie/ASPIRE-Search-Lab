"""Executable host scheduling contracts, not ROS or flight certification."""
import subprocess
from test_native_static_events import compile_run, ROOT


def test_event_retry_and_publication_transaction(tmp_path):
    compile_run(tmp_path,'''#include "static_guidance_events.hpp"
#include "static_replan_gate.hpp"
#include "static_mission_budget.hpp"
#include <cassert>
#include <limits>
using namespace racer_task;
int main(){
 BearingBias scoped;BearingCue a;a.id=1;BearingCue b;b.id=2;b.uses=1;
 scoped.cues[{0,1}]=a;scoped.cues[{0,2}]=b;
 try{ScopedBearingEntity entity(scoped,0,1,true);assert(scoped.cues.size()==1);
  ++scoped.cues.at({0,1}).uses;throw 42;}catch(int){}
 assert(scoped.cues.size()==2&&scoped.cues.at({0,1}).uses==1&&scoped.cues.at({0,2}).uses==1);
 StaticReplanGate gate;assert(!gate.configured());gate.min_interval_s=.5;
 assert(!gate.ready(2.,1.4,false));assert(!gate.ready(2.,2.1,false));
 assert(!gate.ready(2.,2.,true));assert(gate.ready(2.,2.,false));
 gate.started(2.);assert(!gate.ready(2.1,2.1,false));assert(gate.ready(2.5,2.5,false));
 TargetRegistry r;TargetObservation o;o.key={"uav0","s",1};o.sequence=1;
 o.first=1.;o.capture=1.4;o.frame_version="map";o.has_bearing=true;
 o.direction={1,0,0};o.support=3;o.support_first=1.;o.confidence=.6;
 assert(r.ingest(o,1.4).accepted);StaticGuidanceEvents inbox;inbox.sync(r,1.4);
 auto w=inbox.start();assert(w&&inbox.valid(w->token));
 // Actual source survives retry; resource cap covers retry and outstanding work.
 inbox.retry(w->token);assert(!inbox.valid(w->token));assert(inbox.stats().pending==1);
 auto next=inbox.start();assert(next&&next->token!=w->token);
 assert(next->budget_version==w->budget_version&&inbox.review_budget.reservations.size()==1);
 assert(inbox.reserveRoute(next->token,2.,.4));
 // Native proposal was planned from certified stopped state on map revision 3.
 assert(gate.planned(12,11,-1.,3));
 auto allow=[&](bool current,std::uint64_t map,int active,double progress,bool stopped){
  return gate.mayPublish(current,map,active,progress,stopped,true,2.5,2.5);};
 assert(allow(inbox.valid(next->token),3,11,5.,true));
 assert(!allow(true,4,11,5.,true)); // changed map: requeue and replan
 assert(!allow(true,3,10,5.,true)); // changed executor predecessor
 assert(!allow(true,3,11,5.,false)); // stop-and-plan remains mandatory
 assert(!allow(false,3,11,5.,true));
 assert(!gate.mayPublish(true,3,11,5.,true,true,3.1,2.5));
 assert(!gate.mayPublish(true,3,11,5.,true,true,2.4,2.5));
 gate.published=true;assert(!allow(true,3,11,5.,true));
 assert(!gate.resolved(11,9,false));assert(gate.resolved(12,12,false));
 assert(gate.resolved(11,12,true));assert(!gate.resolved(11,13,true));
 gate.resetProposal();assert(!allow(true,3,11,5.,true));
 // Moving mode still cannot publish a past boundary, and is not a C2 proof.
 assert(gate.planned(13,11,5.,3));
 assert(gate.mayPublish(true,3,11,4.,false,false,2.5,2.5));
 assert(!gate.mayPublish(true,3,11,5.,false,false,2.5,2.5));
 assert(!gate.mayPublish(true,3,11,4.,false,true,2.5,2.5));
 o.sequence++;o.capture=2.;o.visual="visually_confirmed";r.ingest(o,2.);inbox.sync(r,2.);
 assert(!inbox.valid(next->token));inbox.retry(next->token);
 assert(!inbox.start()&&inbox.stats().active==0);
 assert(!gate.planned(14,11,-1.,0));assert(gate.proposal==-1);
 assert(!gate.planned(14,11,-.5,3));assert(!gate.planned(14,-1,2.,3));
 assert(!gate.planned(14,11,std::numeric_limits<double>::quiet_NaN(),3));
}
''')


def test_retry_does_not_transfer_to_new_evidence(tmp_path):
    compile_run(tmp_path,'''#include "static_guidance_events.hpp"
#include <cassert>
using namespace racer_task;
int main(){
 for(bool change_visibility:{false,true})for(std::size_t limit:{1,2}){
  TargetRegistry registry;
  StaticGuidanceEvents inbox;
  inbox.allow_weak=true;
  inbox.review_budget.mission_limit=limit;
  TargetObservation observation;
  observation.key={"uav0","retry",1};observation.sequence=1;
  observation.first=observation.support_first=1.;observation.capture=1.4;
  observation.has_bearing=true;observation.direction={1,0,0};
  observation.frame_version="map";observation.confidence=.6;observation.support=1;
  observation.has_position=true;observation.localization="reliable";
  observation.position={0,0,3};observation.radius=.1;
  assert(registry.ingest(observation,1.4).accepted);
  inbox.sync(registry,1.4);
  auto original=inbox.start();assert(original);
  inbox.retry(original->token);
  if(change_visibility){
   TargetVisibility visibility;
   visibility.key=observation.key;visibility.capture=1.5;visibility.anchor_capture=1.4;
   visibility.frame_version="map";visibility.center={0,0,3};visibility.extent={.1,.1,.1};
   visibility.state="occluded";visibility.scope="visible_surface_region";visibility.model_id="test";
   assert(registry.observeVisibility(visibility,1.5));
  }else{
   observation.sequence=2;observation.capture=1.5;observation.support=3;
   assert(registry.ingest(observation,1.5).accepted);
  }
  inbox.sync(registry,1.5);
  assert(!inbox.start(1.5)); // A fresh version needs its own reservation and cooldown.
  auto fresh=inbox.start(1.7);
  if(limit==1){
   assert(!fresh&&inbox.review_budget.reservations.size()==1);
   continue;
  }
  assert(fresh&&fresh->budget_version!=original->budget_version);
  assert(inbox.review_budget.reservations.size()==2);
  assert(inbox.reserveRoute(fresh->token,2.,.4));
  inbox.retry(fresh->token);
  auto retry=inbox.start(1.7);
  assert(retry&&retry->budget_version==fresh->budget_version);
  assert(inbox.reserveRoute(retry->token,2.,.4));
  assert(inbox.review_budget.reservations.size()==2);
  inbox.retry(retry->token);
  assert(!inbox.start(1.7));
 }
}
''')


def test_generated_event_path_preserves_original_execution_gates(tmp_path):
    output=tmp_path/'native'
    subprocess.run(['python3',str(ROOT/'tools/search/build_native.py'),str(output),
                    '--execution-safety','--consistent-inflation'],check=True,capture_output=True,text=True)
    core=output/'src/racer_core'
    fsm=(core/'src/exploration_manager/fast_exploration_fsm.cpp').read_text()
    manager=(core/'src/exploration_manager/fast_exploration_manager.cpp').read_text()
    # Inspect generated placement, not merely the generator's input strings.
    callback=fsm.split('void FastExplorationFSM::FSMCallback(')[1].split('int FastExplorationFSM::callExplorationPlanner(')[0]
    assert callback.index('feedback_stamp_ > 0.5')<callback.index('if(taskStaticEventTick())return;')
    assert callback.index('if(taskReturnTick())return;')<callback.index('if(taskStaticEventTick())return;')
    planning=callback.split('case PLAN_TRAJ:')[1].split('case PUB_TRAJ:')[0]
    assert planning.index('task_depth_review_pending_') < planning.index('int res = callExplorationPlanner();')
    assert planning.index('if(task_stop_and_plan_ && live_execution_ && executed_id_>=0 && !executed_done_ && !task_c2_requested) return;')<planning.index('int res = callExplorationPlanner();')
    assert 'start(now,task_static_monitor_.reservedSources(),task_review_scheduling_priority_?' in fsm
    publishing=callback.split('case PUB_TRAJ:')[1].split('case EXEC_TRAJ:')[0]
    assert publishing.index('!live_sent_&&task_depth_exploration_proposal_') < publishing.index('bspline_pub_.publish(')
    assert publishing.count('curve.getTimeSum()') == 1
    assert publishing.count('taskStaticMapRevision()') == 1
    assert 'current(executed_id_,executed_progress_,publish_now,feedback_stamp_,publish_map)' in publishing
    assert 'task_stop_and_plan_&&!moving_publication,publish_now,feedback_stamp_' in publishing
    assert publishing.index('task_static_budget_.rejection(')<publishing.index('mayPublish(')
    assert 'taskStaticRelease(false,rejection)' in publishing
    assert publishing.index('weak_evidence_trajectory_budget')<publishing.index('mayPublish(')
    assert publishing.index('mayPublish(')<publishing.index('boundary_pub_.publish(task_static_metadata_)')<publishing.index('bspline_pub_.publish(')
    assert 'if(task_static_work_)task_static_metadata_=metadata;else boundary_pub_.publish(metadata);' in planning
    assert 'taskStaticRelease(true,"publication_snapshot_invalid")' in publishing
    assert 'task_static_gate_.published=true' in publishing
    feedback=fsm.split('void FastExplorationFSM::executionFeedback(')[1].split('void FastExplorationFSM::swarmTrajTimerCallback')[0]
    assert feedback.index('executed_id_ = active;') < feedback.index('taskStaticExecutionFeedback(active,d[2]>0.,candidate,d[4]<0.,')
    assert 'task_static_monitor_.tick(manager.task_static_events_' in fsm
    assert 'task_static_monitor_.acknowledge(msg->data)' in fsm

    tick=fsm.split('bool FastExplorationFSM::taskStaticEventTick(){')[1].split('void FastExplorationFSM::taskStaticRelease')[0]
    assert 'taskBearingRerank(' in tick and 'transitState(PLAN_TRAJ' in tick
    assert 'bspline_pub_.publish' not in tick
    assert 'manager.task_bearing_policy_=policy' in tick
    assert 'task_static_evaluation_->evidence' in manager
    assert 'if(task_static_event_consumer_&&!task_static_evaluation_){task_bearing_policy_.original_next=false;return false;}' in manager
    assert 'ScopedBearingEntity event_scope' in manager
    assert 'evaluation_mode' in manager
    assert 'task_static_map_revision_=execution_safety_map_seq_' in (core/'src/plan_env/map_ros.cpp').read_text()


def test_actual_event_methods_with_host_native_interfaces(tmp_path):
    # Compile the shipped methods, using host doubles ONLY for ROS/native
    # interfaces. This exercises orchestration, not real paths or trajectories.
    compile_run(tmp_path,r'''#include "static_guidance_events.hpp"
#include "static_replan_gate.hpp"
#include "static_mission_budget.hpp"
#include <cassert>
#include <memory>
#define ROS_INFO(...) ((void)0)
namespace Eigen {struct Vector3d {double x=0,y=0,z=0;
 Vector3d()=default;Vector3d(double a,double b,double c):x(a),y(b),z(c){}
 bool allFinite()const{return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z);}};}
namespace racer_ros {struct Time {static Time now(){return {};};double toSec()const{return 1.5;}};}
struct Flag {bool enabled=false;bool active()const{return enabled;}};
struct Data {Eigen::Vector3d odom_pos_,odom_vel_;double odom_yaw_=0;
 bool go_back_=false,avoid_collision_=false,static_state_=false;int newest_traj_=0;};
struct Goals {Eigen::Vector3d next_pos_;double next_yaw_=0;};
struct Planner {int local_data_=123;};
struct Manager {racer_task::StaticGuidanceEvents task_static_events_;
 bool task_depth_review_pending_=false;
 std::optional<racer_task::StaticGuidanceEvents::Work> task_static_evaluation_;
 std::shared_ptr<Goals> ed_=std::make_shared<Goals>();
 racer_task::BearingBias task_bearing_policy_;bool task_bearing_preview_=false;
 int task_bearing_sequence_=0;bool task_bearing_pending_=false,task_bearing_changed_=false;
 Eigen::Vector3d task_bearing_target_;int task_bearing_selected_cue_=-1;
 int previews=0;bool useful=true,throws=false;
 bool taskBearingRerank(const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,Eigen::Vector3d&,double&){
  assert(task_static_evaluation_&&task_bearing_preview_);++previews;++task_bearing_sequence_;
  task_bearing_policy_.selections=99;task_bearing_pending_=true;
  if(throws)throw 42;return useful;
 }};
struct FastExplorationFSM {
 enum State{EXEC_TRAJ,IDLE,PLAN_TRAJ,PUB_TRAJ};State state_=EXEC_TRAJ;
 bool task_static_replan_=true,task_review_pending_=false,task_stop_and_plan_=true;
 bool executed_done_=false;int executed_id_=7,executed_spline_=77;
 double feedback_stamp_=1.5,task_return_odom_stamp_=1.5;
 racer_task::StaticReplanGate task_static_gate_;
 racer_task::StaticMissionBudget task_static_budget_;
 bool task_static_prior_goal_valid_=false;
 Eigen::Vector3d task_static_prior_goal_;double task_static_prior_yaw_=0.;
 std::optional<racer_task::StaticGuidanceEvents::Work> task_static_work_;
 std::shared_ptr<Manager> expl_manager_=std::make_shared<Manager>();
 std::shared_ptr<Planner> planner_manager_=std::make_shared<Planner>();
 std::shared_ptr<Data> fd_=std::make_shared<Data>();
 std::unique_ptr<int> executed_data_=std::make_unique<int>(777);
 Flag task_review_session_,task_np_policy_;
 void transitState(State s,const char*){state_=s;}
 bool taskStaticEventTick();void taskStaticRelease(bool,const char*);
};
#include "static_replan_methods.cpp"
using namespace racer_task;
void seed(FastExplorationFSM& f,TargetRegistry&r){TargetObservation o;o.key={"uav0","s",1};
 o.sequence=1;o.first=1.;o.capture=1.4;o.frame_version="map";o.has_bearing=true;
 o.direction={1,0,0};o.support=3;o.support_first=1.;o.confidence=.6;r.ingest(o,1.4);
 f.expl_manager_->task_static_events_.sync(r,1.4);f.task_static_gate_.min_interval_s=.5;
 f.task_static_budget_={100.,10.,1.,10000.,100.,10.,"host-fixture",0.};}
int main(){FastExplorationFSM f;TargetRegistry r;seed(f,r);
 f.expl_manager_->task_depth_review_pending_=true;
 assert(!f.taskStaticEventTick()&&f.expl_manager_->previews==0);
 f.expl_manager_->task_depth_review_pending_=false;
 f.expl_manager_->ed_->next_pos_=Eigen::Vector3d(1,2,3);f.expl_manager_->ed_->next_yaw_=.4;
 assert(f.taskStaticEventTick());assert(f.state_==f.PLAN_TRAJ&&!f.executed_done_);
 assert(f.expl_manager_->previews==1&&f.task_static_work_);
 assert(f.expl_manager_->task_bearing_policy_.selections==0); // preview not an executed detour
 assert(f.expl_manager_->task_bearing_sequence_==1); // audit identity not reused
 assert(!f.expl_manager_->task_bearing_pending_&&!f.expl_manager_->task_bearing_preview_);
 for(int i=0;i<100;++i)assert(!f.taskStaticEventTick());assert(f.expl_manager_->previews==1);
 f.expl_manager_->ed_->next_pos_=Eigen::Vector3d(8,9,10);f.expl_manager_->ed_->next_yaw_=1.2;
 r.records.begin()->second.confirmed=true;f.expl_manager_->task_static_events_.sync(r,1.5);
 assert(f.taskStaticEventTick());assert(f.state_==f.EXEC_TRAJ&&!f.task_static_work_);
 assert(f.planner_manager_->local_data_==777&&f.fd_->newest_traj_==77);
 assert(f.expl_manager_->ed_->next_pos_.x==1&&f.expl_manager_->ed_->next_yaw_==.4);
 FastExplorationFSM deferred;TargetRegistry dr;seed(deferred,dr);assert(deferred.taskStaticEventTick());
 deferred.task_review_pending_=true;assert(deferred.taskStaticEventTick());
 assert(!deferred.task_static_work_&&deferred.expl_manager_->task_static_events_.stats().pending==1);
 FastExplorationFSM depth;TargetRegistry other;seed(depth,other);assert(depth.taskStaticEventTick());
 depth.expl_manager_->task_depth_review_pending_=true;
 assert(depth.taskStaticEventTick()&&!depth.task_static_work_);
 assert(depth.expl_manager_->task_static_events_.stats().pending==1);
 FastExplorationFSM collision;TargetRegistry cr;seed(collision,cr);assert(collision.taskStaticEventTick());
 collision.fd_->avoid_collision_=true;collision.expl_manager_->ed_->next_pos_.x=9;
 assert(collision.taskStaticEventTick());
 assert(!collision.task_static_work_&&collision.expl_manager_->ed_->next_pos_.x==9);
 assert(collision.expl_manager_->task_static_events_.stats().pending==1);
 FastExplorationFSM recovery;TargetRegistry rr;seed(recovery,rr);assert(recovery.taskStaticEventTick());
 recovery.task_np_policy_.enabled=true;assert(recovery.taskStaticEventTick()&&!recovery.task_static_work_);
 FastExplorationFSM submitted;TargetRegistry sr;seed(submitted,sr);assert(submitted.taskStaticEventTick());
 submitted.task_static_gate_.published=true;sr.records.begin()->second.confirmed=true;
 submitted.expl_manager_->task_static_events_.sync(sr,1.5);
 assert(!submitted.taskStaticEventTick());submitted.taskStaticRelease(false,"must_wait_for_executor");
 assert(submitted.task_static_work_&&submitted.expl_manager_->task_static_events_.stats().active==1);
 // Native frontier refresh can enter PLAN_TRAJ while the accepted curve is
 // still moving. A qualified static event must not wait until confirmation.
 FastExplorationFSM waiting;TargetRegistry wr;seed(waiting,wr);waiting.state_=waiting.PLAN_TRAJ;
 assert(waiting.taskStaticEventTick());assert(waiting.state_==waiting.PLAN_TRAJ);
 assert(waiting.expl_manager_->previews==1&&waiting.task_static_work_&&!waiting.executed_done_);
 for(int i=0;i<100;++i)assert(!waiting.taskStaticEventTick());
 assert(waiting.expl_manager_->previews==1); // one global outstanding transaction
 for(int gate=0;gate<6;++gate){
  FastExplorationFSM blocked;TargetRegistry br;seed(blocked,br);blocked.state_=blocked.PLAN_TRAJ;
  if(gate==0)blocked.executed_done_=true;
  if(gate==1)blocked.executed_data_.reset();
  if(gate==2)blocked.executed_id_=-1;
  if(gate==3)blocked.task_stop_and_plan_=false;
  if(gate==4)blocked.fd_->avoid_collision_=true;
  if(gate==5)blocked.feedback_stamp_=.1;
  assert(!blocked.taskStaticEventTick());assert(!blocked.task_static_work_);
  assert(blocked.expl_manager_->task_static_events_.stats().pending==1);
 }
 FastExplorationFSM publishing;TargetRegistry pr;seed(publishing,pr);publishing.state_=publishing.PUB_TRAJ;
 assert(!publishing.taskStaticEventTick());assert(!publishing.task_static_work_);
 FastExplorationFSM idle;TargetRegistry idle_registry;seed(idle,idle_registry);
 idle.state_=idle.IDLE;idle.executed_id_=-1;idle.executed_data_.reset();
 assert(idle.taskStaticEventTick());assert(idle.state_==idle.PLAN_TRAJ&&idle.expl_manager_->previews==0);
 FastExplorationFSM no;TargetRegistry r2;seed(no,r2);no.expl_manager_->useful=false;
 assert(!no.taskStaticEventTick());assert(no.state_==no.EXEC_TRAJ&&!no.task_static_work_);
 FastExplorationFSM fail;TargetRegistry r3;seed(fail,r3);fail.expl_manager_->throws=true;
 try{fail.taskStaticEventTick();assert(false);}catch(int){}
 assert(!fail.task_static_work_&&fail.expl_manager_->task_static_events_.stats().active==0);
 assert(fail.expl_manager_->task_bearing_policy_.selections==0&&!fail.expl_manager_->task_bearing_preview_);
}
''')


def test_runtime_configuration_preserves_safety_and_defaults(tmp_path):
    import pytest
    from tools.racer_ros2.configuration import configuration, static_event_parameters, static_feature_options
    hello=dict(starts=[[0,0,2.2]],bounds=[[-3,-4,0],[7,4,4]],
               intrinsics=[216,0,160,0,216,120,0,0,1],flight_limits=[.7,3.])
    template=ROOT/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml'
    disabled=next(c['parameters'] for c in configuration(hello,tmp_path,template) if c['executable']=='exploration_node')
    assert disabled['search.static_event_replan'] is False
    enabled=dict(hello,static_event_replan=True,static_event_audit=True,
                 static_event_min_interval_s=.5,confirmation_mode='registry',
                 stop_and_plan=True,bearing_bias=True,execution_safety=True,
                 static_guidance_budget=dict(duration_s=100.,reserve_time_s=10.,observation_s=1.,initial_energy_j=10000.,reserve_energy_j=100.,power_upper_w=10.,model_id='contract_fixture'))
    params=next(c['parameters'] for c in configuration(enabled,tmp_path,template) if c['executable']=='exploration_node')
    assert params['search.static_event_replan'] and params['search.static_event_audit']
    assert params['fsm.authorized_execution'] and params['search.stop_and_plan']
    for key in ('manager.max_vel','manager.max_acc','sdf_map.obstacles_inflation','bspline.limit_ratio'):
        assert params[key]==disabled[key]
    for field,value in [('static_event_audit',False),('stop_and_plan',False),
                        ('bearing_bias',False),('execution_safety',False),
                        ('confirmation_mode','visual'),('static_event_min_interval_s',0.),
                        ('static_event_min_interval_s',float('nan')),('static_event_replan','false'),
                        ('static_guidance_budget',{}),('static_guidance_budget',[1]),
                        ('static_weak_enabled',True)]:
        with pytest.raises(ValueError):static_event_parameters(dict(enabled,**{field:value}))
    weak=dict(enabled,static_weak_enabled=True,static_weak_trajectory_s=2.,
              static_weak_extra_cost=.1,static_weak_extra_path_m=.1)
    assert static_event_parameters(weak)['search.static_weak_enabled']
    for inactive_limit in (None, 'unused', float('nan'), float('inf'), True, 10**400):
        disabled_weak=dict(weak,static_weak_enabled=False,static_weak_trajectory_s=inactive_limit)
        params=next(c['parameters'] for c in configuration(disabled_weak,tmp_path,template)
                    if c['executable']=='exploration_node')
        assert params['search.static_event_replan'] and not params['search.static_weak_enabled']
        assert params['search.static_weak_trajectory_s']==-1.
        assert params['search.static_weak_extra_cost']==.1 and params['search.static_weak_extra_path_m']==.1
        with pytest.raises(ValueError, match='explicit low-cost limits'):
            static_event_parameters(dict(disabled_weak,static_weak_enabled=True))

    fallback = static_feature_options(dict(enabled, stop_and_plan=False, static_cancel_enabled=True,
                                           static_cancel_session='invalid', static_moving_c2=True))
    assert fallback['stop_and_plan'] and fallback['static_event_replan']
    assert not fallback['static_cancel_enabled'] and not fallback['static_moving_c2']
    no_event = dict(enabled, execution_safety=False, static_event_replan=True)
    params = next(c['parameters'] for c in configuration(no_event, tmp_path, template)
                  if c['executable']=='exploration_node')
    assert params['fsm.authorized_execution'] and not params['search.static_event_replan']
