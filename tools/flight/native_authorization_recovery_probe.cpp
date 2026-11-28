// TEST ONLY: real ROS message + generated native FSM callback; no spin, flight,
// truth, trajectory publisher, replacement planner or injected occupancy.
#include <racer_ros2/runtime.hpp>
#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <exploration_manager/expl_data.h>
#include <plan_manage/planner_manager.h>
#include <future>
#include <iostream>
#include <stdexcept>
namespace fast_planner {
struct TaskAuthorizationRecoveryProbe {
  static void require(bool value,const char* reason) {
    if(!value)throw std::runtime_error(reason);
  }
  static void proposal(FastExplorationFSM& f,int id) {
    f.fd_->newest_traj_.traj_id=id;
    f.task_authorization_proposal_=id;
    f.task_authorization_goal_=Eigen::Vector3d(5.+id,0,1.8);
    f.expl_manager_->ed_->next_pos_=Eigen::Vector3d(99.,0,1.8); // Deliberately stale metadata.
  }
  static void feedback(FastExplorationFSM& f,int candidate,double decision=-1.,int active=-1,double age=0.) {
    auto msg=std::make_shared<std_msgs::Float64MultiArray>();
    msg->data={double(active),0.,1.,double(candidate),decision,racer_ros::Time::now().toSec()-age,2.};
    f.executionFeedback(msg);
  }
  static void run(racer_ros::NodeHandle& nh) {
    FastExplorationFSM f;f.init(nh);
    f.live_execution_=true;f.task_stop_and_plan_=true;
    f.fd_->start_pt_=Eigen::Vector3d(0,0,1.8);f.fd_->odom_pos_=f.fd_->start_pt_;
    f.fd_->start_vel_.setZero();f.fd_->start_acc_.setZero();f.fd_->start_yaw_.setZero();
    f.state_=PLAN_TRAJ;
    f.task_no_path_recovery_=false;proposal(f,1);feedback(f,1);
    require(f.task_np_policy_.failures()==0,"Disabled runtime switch admitted recovery");
    f.task_no_path_recovery_=true;proposal(f,2);f.task_review_proposal_=2;feedback(f,2);
    require(f.task_np_policy_.failures()==0,"Review rejection polluted ordinary recovery");
    f.task_review_proposal_=-1;proposal(f,3);feedback(f,3,-1,-1,1.);
    feedback(f,4);feedback(f,3,0.);
    require(f.task_np_policy_.failures()==0,"Stale/wrong/pending feedback counted as rejection");
    for(int id=3;id<=5;++id) {
      proposal(f,id);feedback(f,id);
      for(int i=0;i<20;++i)feedback(f,id);
      require(f.task_np_policy_.failures()==id-2,"Distinct proposal failure count incorrect");
      require(f.task_np_policy_.active()==(id==5),"Inactive admission threshold incorrect");
    }
    const racer_task::Point root={0,0,1.8};
    const double now=racer_ros::Time::now().toSec();
    require(!f.task_np_policy_.eligible({10,0,1.8},root,now),"Captured requested goal not cooled down");
    require(f.task_np_policy_.eligible({99,0,1.8},root,now),"Stale ed goal incorrectly cooled down");
    require(f.task_np_policy_.eligible({1,0,1.8},root,now),"Spline prefix endpoint incorrectly cooled down");
    int attempts=0;
    for(int i=0;i<9;++i) {
      const double t=now+(i<8?1.+i:14.);
      require(f.task_np_policy_.ready(t),"Recovery missed a due retry");
      proposal(f,20+i);f.task_np_policy_.attempted({25.+i,0,1.8},t);++attempts;
      feedback(f,20+i);require(f.task_np_policy_.active(),"Proposal or rejection reset recovery");
      require(!f.task_np_policy_.ready(t+.49),"Recovery attempts were too frequent");
      if(i>=7)require(!f.task_np_policy_.ready(t+1.99),"Recovery missed the slower retry interval");
    }
    require(attempts==9 && f.task_np_policy_.attempts()==9,"Recovery never resumed after eight failures");
    proposal(f,123);feedback(f,123,0.);
    require(f.task_np_policy_.attempts()==9,"Pending proposal reset recovery");
    f.planner_manager_->local_data_.duration_=2.;
    f.planner_manager_->local_data_.traj_id_=123;
    f.planner_manager_->local_data_.start_time_=racer_ros::Time::now();
    feedback(f,123,1.,123);
    require(f.executed_id_==123 && f.executed_data_ && f.state_==EXEC_TRAJ,"Real authorization callback did not activate");
    require(!f.task_np_policy_.active() && f.task_np_policy_.attempts()==0 && f.task_np_policy_.failures()==0,
            "Actual activation failed to reset episode");
    // Recorded regression: a 5.69 s proposal is safely stopped at 0.564 s.
    // All original time thresholds remain false. Exercise the actual FSM with
    // an unobserved map: completion must reach its ordinary no-frontier IDLE
    // branch, never invent free cells or force another planning destination.
    f.executed_data_->duration_=5.69;
    f.fp_->replan_thresh1_=1.;f.fp_->replan_thresh2_=1.;f.fp_->replan_thresh3_=2.;
    f.fd_->go_back_=false;
    auto completion=[&](int active,bool done,double age=0.) {
      auto msg=std::make_shared<std_msgs::Float64MultiArray>();
      msg->data={double(active),.564,double(done),-1.,-1.,racer_ros::Time::now().toSec()-age,0.};
      f.executionFeedback(msg);
    };
    completion(123,false);f.FSMCallback({});
    require(f.state_==EXEC_TRAJ && !f.executed_done_,"Unfinished stop released EXEC_TRAJ");
    completion(124,true);f.FSMCallback({});
    require(f.state_==EXEC_TRAJ && !f.executed_done_,"Wrong execution completion released EXEC_TRAJ");
    completion(123,true,1.);f.FSMCallback({});
    require(f.state_==EXEC_TRAJ && !f.executed_done_,"Stale completion released EXEC_TRAJ");
    f.task_stop_and_plan_=false;completion(123,true);f.FSMCallback({});
    require(f.state_==EXEC_TRAJ,"Disabled stop-and-plan changed native thresholds");
    f.task_stop_and_plan_=true;completion(123,true);f.FSMCallback({});
    require(f.state_==IDLE && f.fd_->static_state_,"Completed short brake remained stuck in EXEC_TRAJ");
    std::cout<<"PASS_NATIVE_SHORT_STOP progress=0.564 duration=5.69 stale_and_wrong_id_rejected=1 native_empty_map_idle=1\n";
    // Saved P01-B: cluster-covered replan already entered PLAN_TRAJ before
    // terminal observation failed. Negative feedback must release its old ID.
    f.state_=PLAN_TRAJ;f.executed_id_=22;f.executed_done_=false;
    f.fd_->static_state_=false;f.fd_->newest_traj_.traj_id=22;
    auto failed_view=[&](double age=0.) {
      auto msg=std::make_shared<std_msgs::Float64MultiArray>();
      // Actual recorded fields except stamp rebased to this no-flight fixture.
      msg->data={-1.,4.872000000000014,0.,21.,-1.,racer_ros::Time::now().toSec()-age,0.};
      f.executionFeedback(msg);
    };
    failed_view(1.);
    require(f.executed_id_==22 && !f.executed_done_ && f.state_==PLAN_TRAJ,
            "stale unobserved stop released owned execution");
    failed_view();
    require(f.executed_id_==-1 && f.executed_done_ && f.fd_->static_state_ && f.state_==PLAN_TRAJ,
            "fresh unobserved stop left PLAN_TRAJ blocked by old execution ID");
    f.FSMCallback({});
    require(f.state_==IDLE,"released PLAN_TRAJ did not reach native empty-map IDLE");
    failed_view();require(f.state_==IDLE,"duplicate stopped feedback restarted IDLE");
    f.state_=WAIT_TRIGGER;failed_view();
    require(f.state_==WAIT_TRIGGER,"unowned stopped feedback restarted WAIT_TRIGGER");
    std::cout<<"PASS_NATIVE_STOPPED_FEEDBACK original_saved_fields=1 plan_state_released=1 no_observation_credit=1 stale_rejected=1 native_empty_map_idle=1 no_flight=1\n";
    f.task_no_path_recovery_=false;
    const auto trigger=std::make_shared<geometry_msgs::PoseStamped>();
    f.triggerCallback(trigger);
    require(f.state_==IDLE,"Authorized live entry permanently finished before a measured frontier arrived");
    f.task_no_path_recovery_=true;
    for(int i=0;i<3;++i)f.task_np_policy_.failed(root,now);
    f.task_return_.begin(root);f.fd_->go_back_=true;f.state_=PLAN_TRAJ;
    f.FSMCallback({});
    require(f.state_==PLAN_TRAJ && f.task_return_.state()==racer_task::ReturnPolicy::State::waiting_no_goal &&
            !f.task_return_.terminal() && !f.task_return_.mayUseTemporary(),
            "Empty return recovery ended or left PLAN_TRAJ");
    f.FSMCallback({});
    require(f.task_return_.attempts()==1,"Immediate callback bypassed the no-goal cooldown");

    auto launch_return=[&](int id) {
      f.task_return_=racer_task::ReturnPolicy{};
      f.task_return_.begin(root);
      require(f.task_return_.attempt(racer_ros::Time::now().toSec(),false),"Return attempt was not due");
      proposal(f,id);f.task_return_.proposed(id);
      f.planner_manager_->local_data_.duration_=2.;
      f.planner_manager_->local_data_.traj_id_=id;
      completion(id,false);
      require(f.task_return_.executing() && f.executed_id_==id,"Return fixture was not accepted");
    };
    auto stopped=[&](int id,double age=0.) {
      auto msg=std::make_shared<std_msgs::Float64MultiArray>();
      msg->data={-1.,.564,0.,-1.,-1.,racer_ros::Time::now().toSec()-age,0.,double(id)};
      f.executionFeedback(msg);
    };
    launch_return(124);
    stopped(124,1.);stopped(123);
    require(f.task_return_.executing() && f.executed_id_==124,"Stale or wrong stop de-owned the current return");
    stopped(124);
    require(f.state_==PLAN_TRAJ && f.executed_id_<0 && !f.task_return_.terminal() &&
            !f.task_return_.executing(),"Verified stop failed to reopen return without completion");
    completion(124,true);
    require(f.state_==PLAN_TRAJ && f.executed_id_<0,"Released return trajectory was re-used");
    f.task_collision_stop_enabled_=true;
    launch_return(42);
    const double same_stamp=f.feedback_stamp_, request_stamp=same_stamp-.1;
    auto collision_feedback=[&](double stamp,int id,double request,double done=0.,int candidate=-1) {
      auto msg=std::make_shared<std_msgs::Float64MultiArray>();
      msg->data={-1.,.564,done,double(candidate),-1.,stamp,0.,double(id),request};
      f.executionFeedback(msg);
    };
    collision_feedback(same_stamp,41,request_stamp-.1);
    require(f.executed_id_==42 && f.task_return_.executing() && f.state_==EXEC_TRAJ,
            "Previous same-stamp collision receipt retired the new execution");
    f.task_collision_stop_id_=42;
    f.task_collision_stop_stamp_=request_stamp;
    collision_feedback(same_stamp+.02,41,request_stamp);
    collision_feedback(same_stamp+.02,42,request_stamp-.01);
    collision_feedback(same_stamp+.02,42,request_stamp,1.);
    require(f.executed_id_==42 && f.task_return_.executing() && f.state_==EXEC_TRAJ &&
            f.feedback_stamp_==same_stamp && f.task_collision_stop_id_==42,
            "Invalid collision receipt changed execution or feedback freshness");
    collision_feedback(same_stamp,42,request_stamp);
    require(f.executed_id_<0 && f.state_==PLAN_TRAJ && !f.task_return_.terminal() &&
            !f.task_return_.executing() && f.task_collision_stopped_ && f.task_collision_stop_id_<0,
            "Matching collision receipt did not release the stopped execution");
    const int rejections_after_stop=f.task_rejections_;
    const double hold_stamp=racer_ros::Time::now().toSec();
    collision_feedback(hold_stamp,42,request_stamp,0.,42);
    require(f.feedback_stamp_==hold_stamp && f.state_==PLAN_TRAJ &&
            f.task_rejections_==rejections_after_stop,
            "Duplicate stopped receipt replayed rejection or failed to preserve the hold heartbeat");
    f.task_return_=racer_task::ReturnPolicy{};
    f.task_return_.begin(root);
    require(f.task_return_.attempt(hold_stamp-1.,false),"Retry fixture had no due return attempt");
    proposal(f,43);f.task_return_.proposed(43);f.state_=PUB_TRAJ;f.live_sent_=true;
    collision_feedback(hold_stamp,42,request_stamp,0.,43);
    require(f.state_==PLAN_TRAJ && f.task_return_.state()==racer_task::ReturnPolicy::State::waiting_retry,
            "Parked collision feedback swallowed denial of a new proposal");
    launch_return(125);
    collision_feedback(f.feedback_stamp_,42,request_stamp);
    require(f.executed_id_==125 && f.task_return_.executing() && f.state_==EXEC_TRAJ,
            "A consumed collision receipt was replayed onto the next execution");
    completion(-1,false);
    require(f.task_return_.state()==racer_task::ReturnPolicy::State::waiting_retry &&
            !f.task_return_.terminal() && !f.task_return_.executing() &&
            f.executed_id_<0 && f.state_==PLAN_TRAJ,
            "In-flight loss did not schedule a return retry");
    f.task_return_=racer_task::ReturnPolicy{};
    f.task_np_policy_=racer_task::RecoveryPolicy{};
    f.fd_->go_back_=false;
    f.fd_->start_pos_=Eigen::Vector3d(3.,0.,1.8);
    const double failed_at=racer_ros::Time::now().toSec()-101.;
    for(int i=0;i<3;++i)f.task_np_policy_.failed(root,failed_at+i);
    f.state_=PUB_TRAJ;
    f.taskReturnTick();
    require(!f.task_return_.started(),"A pending search proposal lost ownership");
    f.state_=PLAN_TRAJ;
    f.taskReturnTick();
    require(f.fd_->go_back_ && f.task_return_.home()==racer_task::Point{3.,0.,1.8} &&
        !f.task_return_.mayUseTemporary(),"Stalled search did not plan to the original home first");
    require(f.task_return_.attempt(failed_at,false),"No direct return attempt");
    f.task_return_.noGoal(failed_at);
    require(f.taskReturnTick() && f.state_==FINISH &&
        f.task_return_.state()==racer_task::ReturnPolicy::State::blocked,
        "An unavailable return route did not end the failed mission");
    auto observer=std::make_shared<rclcpp::Node>("blocked_return_observer");
    std::promise<void> blocked;
    auto received=blocked.get_future();
    auto subscription=observer->create_subscription<std_msgs::Empty>(
        "/baseline/return_blocked_0",rclcpp::QoS(1).transient_local(),
        [&](std_msgs::EmptyConstPtr){blocked.set_value();});
    require(rclcpp::spin_until_future_complete(observer,received,std::chrono::seconds(5))==
        rclcpp::FutureReturnCode::SUCCESS,"The runner could not receive the terminal status over DDS");
    std::cout<<"PASS_NATIVE_STALLED_SEARCH fixed_home_first=1 blocked_return=1\n";
    std::cout<<"PASS_NATIVE_COLLISION_FEEDBACK same_stamp_isolated=1 exact_release=1 duplicate_hold_only=1 current_denial=1 no_flight=1\n";
    std::cout<<"PASS_NATIVE_RETURN_RETRY empty_trigger_waits=1 no_goal_retries=1 verified_stop_resumes=1 stale_and_wrong_id_rejected=1 loss_retried=1 no_flight=1\n";
    std::cout<<"PASS_NATIVE_AUTHORIZATION_RECOVERY trigger=3 recovery_attempts=9 real_ros_callback=1 no_flight=1\n";
  }
};
}
int main(int argc,char** argv) {
  racer_ros::init(argc,argv,"task_authorization_recovery_probe");
  try {
    racer_ros::NodeHandle nh("~");fast_planner::TaskAuthorizationRecoveryProbe::run(nh);
    rclcpp::shutdown();return 0;
  } catch(const std::exception& e) {
    std::cerr<<"FAIL_NATIVE_AUTHORIZATION_RECOVERY "<<e.what()<<"\n";
    rclcpp::shutdown();return 1;
  }
}
