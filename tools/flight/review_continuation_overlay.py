"""Opt-in bounded review scheduling in the generated FSM; RACER stays locked."""
from pathlib import Path
import shutil
from tools.flight.optimizer_overlay import replace_once


def apply(core):
    here = Path(__file__).parent
    for source in ('review_continuation.hpp', 'review_protocol.hpp'):
        shutil.copyfile(here/source, core/'include/exploration_manager'/('task_'+source))
    p = core/'include/exploration_manager/fast_exploration_fsm.h'
    s = p.read_text()
    s = replace_once(s, '#include <Eigen/Eigen>', '''#include <Eigen/Eigen>
#include <std_msgs/msg/string.hpp>
#include <exploration_manager/task_review_continuation.hpp>
#include <exploration_manager/task_review_protocol.hpp>''')
    s = replace_once(s, '  bool task_stop_and_plan_', '''  bool task_review_continuation_ = false, task_review_exit_reported_ = false;
  racer_task::ReviewContinuation task_review_session_;
  racer_ros::Publisher task_review_status_pub_;
  racer_ros::Subscriber task_review_cancel_sub_;
  bool taskReviewContinuationTick();
  bool taskReviewContinuationReady();
  void taskReviewContinuationRequest(const geometry_msgs::PoseStampedConstPtr& msg);
  void taskReviewCancel(const std_msgs::msg::String::ConstSharedPtr& msg);
  void taskReviewReport(const std::string& state,const std::string& reason,
      std::int64_t id=-1,const Eigen::Vector3d* goal=nullptr);
  void taskReviewExit(const std::string& state,const std::string& reason);
  friend struct TaskReviewContinuationProbe;
  bool task_stop_and_plan_''')
    p.write_text(s)
    p = core/'src/exploration_manager/fast_exploration_fsm.cpp'
    s = '#include <iomanip>\n#include <sstream>\n'+p.read_text()
    s = replace_once(s, '  fp_.reset(new FSMParam);', '''  nh.param("search/review_continuation", task_review_continuation_, false);
  nh.param("search/review_max_segments", task_review_session_.max_segments, 3);
  nh.param("search/review_max_duration_s", task_review_session_.max_duration_s, 60.);
  nh.param("search/review_max_path_m", task_review_session_.max_path_m, 6.);
  nh.param("search/review_observation_hold_s", task_review_session_.observation_hold_s, 5.);
  if(task_review_continuation_) {
    if(!task_review_enabled_ || !task_stop_and_plan_ || !task_review_session_.validConfig())
      throw std::runtime_error("Review continuation requires active_review, stop_and_plan and bounded valid budgets");
    task_review_status_pub_=nh.advertise<std_msgs::msg::String>("/search/review_status",20);
    task_review_cancel_sub_=nh.subscribe("/search/review_cancel",10,&FastExplorationFSM::taskReviewCancel,this);
  }
  fp_.reset(new FSMParam);''')
    # Timeouts/cancellation are processed even when the ordinary feedback
    # freshness gate is holding planning; this never bypasses that gate.
    s = replace_once(s, '  if (live_execution_ && state_ != INIT', '''  if(taskReviewContinuationTick())return;
  if (live_execution_ && state_ != INIT''')
    s = replace_once(s, '  task_review_pending_=false;\n  if(!review', '  if(!task_review_continuation_)task_review_pending_=false;\n  if(!review')
    s = replace_once(s, '  } else if(review) {\n    expl_manager_', '''  } else if(review) {
    if(task_review_continuation_) {
      if(!task_review_session_.attempt(racer_ros::Time::now().toSec())) {
        taskReviewExit(task_review_session_.exit_state,task_review_session_.exit_reason);return FAIL;
      }
      taskReviewReport("planning","native_planTrajToView");
    }
    expl_manager_''')
    s = replace_once(s, '  if(review && res==SUCCEED)task_review_proposal_', '''  if(review && task_review_continuation_) {
    if(res!=SUCCEED) {taskReviewExit("unreachable","native_planning_failed");return res;}
    auto& curve=planner_manager_->local_data_.position_traj_;
    const double duration=curve.getTimeSum();double length=0.;
    Eigen::Vector3d previous=fd_->start_pt_;
    if(!std::isfinite(duration) || duration<=0. || duration>60.)length=INFINITY;
    else for(double t=0.;t<duration+.05;t+=.05) {
      const Eigen::Vector3d point=curve.evaluateDeBoorT(std::min(t,duration));
      if(!point.allFinite()){length=INFINITY;break;}
      length+=(point-previous).norm();previous=point;
    }
    if(!task_review_session_.proposed(live_counter_+1,length)) {
      taskReviewExit(task_review_session_.exit_state,task_review_session_.exit_reason);return FAIL;
    }
    task_review_pending_=false;
  }
  if(review && res==SUCCEED)task_review_proposal_''')
    s = replace_once(s, '    info->start_time_ = (racer_ros::Time::now() - time_r)', '''    if(review && task_review_continuation_ &&
        racer_ros::Time::now().toSec()+info->position_traj_.getTimeSum()-task_review_session_.received>
            task_review_session_.max_duration_s) {
      // Account for any original feasibility time-scaling before publication.
      taskReviewExit("timeout","proposed_duration_exceeds_remaining_budget");return FAIL;
    }
    info->start_time_ = (racer_ros::Time::now() - time_r)''')
    s = replace_once(s, '    fd_->newest_traj_ = bspline;', '''    fd_->newest_traj_ = bspline;
    if(review && task_review_continuation_)
      taskReviewReport("planned","native_proposal_requires_external_authorization");''')
    s = replace_once(s, 'bool FastExplorationFSM::taskReviewReady() {', '''bool FastExplorationFSM::taskReviewReady() {
  if(task_review_continuation_)return taskReviewContinuationReady();''')
    s = replace_once(s, 'void FastExplorationFSM::taskReview(const geometry_msgs::PoseStampedConstPtr& msg) {', '''void FastExplorationFSM::taskReview(const geometry_msgs::PoseStampedConstPtr& msg) {
  if(task_review_continuation_){taskReviewContinuationRequest(msg);return;}''')
    s = replace_once(s, '    executed_id_ = active;', '''    if(task_review_continuation_ && task_review_session_.activated(active,
        {fd_->odom_pos_.x(),fd_->odom_pos_.y(),fd_->odom_pos_.z()},task_return_odom_stamp_))
      taskReviewReport("executing","authorized_native_plan");
    executed_id_ = active;''')
    s = replace_once(s, '  if (candidate == fd_->newest_traj_.traj_id && d[4] < 0.) {', '''  if (candidate == fd_->newest_traj_.traj_id && d[4] < 0.) {
    if(task_review_continuation_ && task_review_session_.active() && candidate==task_review_session_.plan_id) {
      task_review_session_.requestExit("unreachable","execution_authorization_rejected");
      task_review_session_.finishExit();task_review_pending_=false;
      taskReviewReport(task_review_session_.exit_state,task_review_session_.exit_reason);
    }''')
    s = replace_once(s, '  if (active < 0 && state_ == EXEC_TRAJ) {', '''  if (active < 0 && executed_id_ >= 0) {
    if(task_review_continuation_ && task_review_session_.phase==racer_task::ReviewContinuation::Phase::executing) {
      task_review_session_.requestExit("invalid","authorized_execution_lost");
      task_review_session_.finishExit();task_review_pending_=false;
      taskReviewReport(task_review_session_.exit_state,task_review_session_.exit_reason);
    }''')
    # A cluster-covered replan may already be in PLAN_TRAJ when a stopped
    # unobserved execution is released. Clear owned execution in every state;
    # only EXEC_TRAJ needs a transition. No observation success is invented.
    s = replace_once(s, '    transitState(PLAN_TRAJ, "execution stopped");',
                     '    if (state_ == EXEC_TRAJ) transitState(PLAN_TRAJ, "execution stopped");')
    # The odometry callback itself tracks actual movement rather than timer
    # positions or the planned spline; out-of-order/stale poses add no evidence.
    s = replace_once(s, '  task_return_odom_stamp_=rclcpp::Time(msg->header.stamp).seconds();', '''  task_return_odom_stamp_=rclcpp::Time(msg->header.stamp).seconds();
  if(task_review_continuation_) {
    const auto& p=msg->pose.pose.position;
    const double age=racer_ros::Time::now().toSec()-task_return_odom_stamp_;
    task_review_session_.observe({p.x,p.y,p.z},task_return_odom_stamp_,age>=0. && age<=.5);
  }''')
    s = replace_once(s, 'void FastExplorationFSM::taskReview(', (here/'review_continuation_methods.cpp').read_text()+'\nvoid FastExplorationFSM::taskReview(')
    p.write_text(s)
