// TEST ONLY: real generated FSM callbacks and unobserved native map. Lifecycle
// setup below is a fixture, not evidence of autonomous flight or perception.
#include <racer_ros2/runtime.hpp>
#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <exploration_manager/expl_data.h>
#include <plan_manage/planner_manager.h>
#include <iostream>
#include <stdexcept>
namespace fast_planner {
struct TaskReviewContinuationProbe {
  static void require(bool b,const char* why) {if(!b)throw std::runtime_error(why);}
  static void feedback(FastExplorationFSM& f,int active,bool done,double age=0.) {
    auto msg=std::make_shared<std_msgs::Float64MultiArray>();
    msg->data={double(active),.564,double(done),-1.,0.,racer_ros::Time::now().toSec()-age,5.69};
    f.executionFeedback(msg);
  }
  static void prepare(FastExplorationFSM& f,std::int64_t request,int plan,double x) {
    const double now=racer_ros::Time::now().toSec();
    f.task_review_session_=racer_task::ReviewContinuation();
    require(f.task_review_session_.begin(request,{x,0.,1.8},now),"fixture begin failed");
    f.task_review_goal_=Eigen::Vector3d(x,0,1.8);f.task_review_exit_reported_=false;
    f.fd_->have_odom_=true;f.fd_->odom_pos_=Eigen::Vector3d(0,0,1.8);
    f.fd_->start_pt_=f.fd_->odom_pos_;f.fd_->start_vel_.setZero();f.fd_->start_acc_.setZero();
    f.fd_->start_yaw_.setZero();f.fd_->go_back_=false;
    f.task_return_odom_stamp_=now;f.feedback_stamp_=now;
    f.executed_id_=-1;f.executed_done_=false;
    require(f.task_review_session_.attempt(now),"fixture attempt failed");
    require(f.task_review_session_.proposed(plan,1.),"fixture proposal failed");
    f.fd_->newest_traj_.traj_id=plan;f.planner_manager_->local_data_.duration_=5.69;
    f.planner_manager_->local_data_.traj_id_=plan;f.state_=PUB_TRAJ;
    feedback(f,plan,false);
    require(f.task_review_session_.phase==racer_task::ReviewContinuation::Phase::executing,"real activation was not correlated");
  }
  static void run(racer_ros::NodeHandle& nh) {
    FastExplorationFSM f;f.init(nh);
    require(f.task_review_continuation_ && f.task_stop_and_plan_ && f.live_execution_,"opt-in parameters missing");
    prepare(f,100000000001LL,41,.249);
    feedback(f,42,true);f.taskReviewContinuationTick();
    require(f.task_review_session_.active(),"wrong-plan completion admitted arrival");
    feedback(f,41,true,1.);f.taskReviewContinuationTick();
    require(f.task_review_session_.active(),"stale completion admitted arrival");
    f.task_return_odom_stamp_=racer_ros::Time::now().toSec()-1.;
    feedback(f,41,true);f.taskReviewContinuationTick();
    require(f.task_review_session_.active(),"stale odometry admitted arrival");
    f.task_return_odom_stamp_=racer_ros::Time::now().toSec();f.taskReviewContinuationTick();
    require(f.task_review_session_.phase==racer_task::ReviewContinuation::Phase::holding,
            "fresh matching arrival did not enter bounded observation hold");
    require(f.taskReviewContinuationTick() && f.state_==EXEC_TRAJ && f.executed_id_==41 && f.executed_done_,
            "observation hold emitted or selected another motion");
    f.task_review_session_.hold_started-=5.1;f.taskReviewContinuationTick();
    require(!f.task_review_session_.active() && f.task_review_session_.exit_state=="released",
            "observation window did not release automatically");

    prepare(f,100000000002LL,42,.251);
    feedback(f,42,true);f.taskReviewContinuationTick();
    require(f.task_review_session_.exit_state=="invalid" && f.task_review_session_.exit_reason=="endpoint_not_currently_free",
            "unobserved endpoint admitted continuation or relaxed .25 m arrival");
    require(f.task_review_session_.segment_index==1,"unknown map created another segment");

    prepare(f,100000000003LL,43,1.);
    auto cancel=std::make_shared<std_msgs::msg::String>();
    cancel->data="{\"request_id\":\"999\",\"reason\":\"target_invalid\"}";f.taskReviewCancel(cancel);
    require(f.task_review_session_.exit_state.empty(),"wrong-request cancel admitted");
    cancel->data="{\"request_id\":\"100000000003\",\"reason\":\"direct_confirmed\"}";f.taskReviewCancel(cancel);
    require(f.task_review_session_.active() && f.executed_id_==43 && f.state_==EXEC_TRAJ,"cancel revoked authorized execution");
    feedback(f,43,true);f.taskReviewContinuationTick();
    require(!f.task_review_session_.active() && f.task_review_session_.exit_state=="cancelled","cancel did not retire after matching completion");

    prepare(f,100000000004LL,44,1.);
    f.task_review_session_.received-=61.;f.taskReviewContinuationTick();
    require(f.task_review_session_.active() && f.task_review_session_.exit_state=="timeout","timeout must await current safe segment");
    feedback(f,44,true);f.taskReviewContinuationTick();
    require(!f.task_review_session_.active(),"timeout did not retire after completion");
    // A prior published segment leaves live_sent_ true. A newly generated
    // proposal rejected before PUB_TRAJ must retire rather than wait forever
    // for a plan ID which was never assigned/published.
    prepare(f,100000000005LL,45,1.);
    f.task_review_session_.phase=racer_task::ReviewContinuation::Phase::proposed;
    f.state_=PLAN_TRAJ;f.live_sent_=true;f.fd_->newest_traj_.traj_id=44;
    f.taskReviewExit("timeout","proposed_duration_exceeds_remaining_budget");
    require(!f.task_review_session_.active() && f.state_==PLAN_TRAJ,
            "old live_sent flag trapped unpublished review in cancel_pending");
    // Conversely, an actually published proposal cannot be retracted by
    // clearing a flag: accept its authorization and finish the safe segment.
    prepare(f,100000000006LL,46,1.);
    f.task_review_session_.phase=racer_task::ReviewContinuation::Phase::proposed;
    f.state_=PUB_TRAJ;f.live_sent_=true;f.executed_id_=45;f.executed_done_=true;
    f.taskReviewExit("cancelled","target_invalid");
    require(f.task_review_session_.active(),"published proposal was prematurely retired");
    feedback(f,46,false);feedback(f,46,true);f.taskReviewContinuationTick();
    require(!f.task_review_session_.active() && f.task_review_session_.exit_state=="cancelled",
            "published proposal cancellation did not finish after authorization/completion");
    prepare(f,100000000007LL,47,.249);
    feedback(f,47,true);f.taskReviewContinuationTick();
    cancel->data="{\"request_id\":\"100000000007\",\"reason\":\"active_multiview_confirmed\"}";
    f.taskReviewCancel(cancel);
    require(!f.task_review_session_.active() && f.task_review_session_.exit_state=="released",
            "matching evidence completion did not release hold");
    prepare(f,100000000008LL,48,.249);
    feedback(f,48,true);f.taskReviewContinuationTick();
    f.task_review_session_.received-=61.;f.taskReviewContinuationTick();
    require(!f.task_review_session_.active() && f.task_review_session_.exit_state=="timeout",
            "observation hold extended original request time budget");
    prepare(f,100000000009LL,49,.249);
    feedback(f,49,true);f.taskReviewContinuationTick();
    f.fd_->go_back_=true;f.taskReviewContinuationTick();
    require(!f.task_review_session_.active() && f.task_review_session_.exit_reason=="return_in_progress",
            "observation hold blocked return");
    std::cout<<"PASS_NATIVE_REVIEW_HOLD same_budget=1 bounded_5s=1 matching_evidence_release=1 return_release=1\n";
    std::cout<<"PASS_NATIVE_REVIEW_PUBLICATION_BOUNDARY stale_live_sent_ignored=1 published_cancel_deferred=1\n";
    std::cout<<"PASS_NATIVE_REVIEW_CALLBACK correlation=1 fresh_actual_arrival=1 unchanged_arrival_0.25=1 unknown_map_rejected=1 deferred_cancel=1 timeout=1 no_flight=1\n";
  }
};
}
int main(int argc,char** argv) {
  racer_ros::init(argc,argv,"task_review_continuation_probe");
  try {racer_ros::NodeHandle nh("~");fast_planner::TaskReviewContinuationProbe::run(nh);rclcpp::shutdown();return 0;}
  catch(const std::exception& e) {std::cerr<<"FAIL_NATIVE_REVIEW_CALLBACK "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
