"""Route opt-in native collision alerts to the guarded single-aircraft executor."""
from .optimizer_overlay import replace_once


def apply(core):
    path = core/'include/exploration_manager/fast_exploration_fsm.h'
    text = replace_once(path.read_text(), '  bool task_static_replan_=false;', '''  bool task_collision_stop_enabled_=false, task_collision_stopped_=false;
  int task_collision_stop_id_=-1, task_collision_stop_released_id_=-1;
  double task_collision_stop_stamp_=-1., task_collision_stop_last_sent_=-1.;
  racer_ros::Publisher task_collision_stop_pub_;
  void taskCollisionStopTick();
  bool task_static_replan_=false;''')
    path.write_text(text)

    path = core/'src/exploration_manager/fast_exploration_fsm.cpp'
    text = '#include <cstdlib>\n'+path.read_text()
    anchor = '  expl_manager_->task_bearing_policy_.event_driven_static=task_static_replan_;'
    text = replace_once(text, anchor, anchor+'''
  const char* collision_stop=std::getenv("ASPIRE_NATIVE_COLLISION_STOP");
  task_collision_stop_enabled_=collision_stop&&std::string(collision_stop)=="1"&&
      task_static_replan_&&task_static_cancel_.enabled();
  if(task_collision_stop_enabled_)
    task_collision_stop_pub_=nh.advertise<std_msgs::Float64MultiArray>(
        "/baseline/collision_stop_"+std::to_string(getId()-1),10);''')
    text = replace_once(text, '  if(taskStaticEventTick())return;', '''  taskCollisionStopTick();
  if(taskStaticEventTick())return;''')
    anchor = '      transitState(PLAN_TRAJ, "safetyCallback");'
    text = replace_once(text, anchor, '''      if(task_collision_stop_enabled_&&executed_id_>=0&&!executed_done_&&
          !fd_->go_back_&&!task_review_session_.active()&&!task_np_policy_.active()){
        task_collision_stop_id_=executed_id_;
        task_collision_stop_stamp_=racer_ros::Time::now().toSec();
        task_collision_stop_last_sent_=-1.;
        taskCollisionStopTick();
        ROS_WARN("COLLISION_STOP requested id=%d sim=%.6f",executed_id_,task_collision_stop_stamp_);
      }
'''+anchor)
    text = replace_once(text, 'void FastExplorationFSM::safetyCallback(', '''void FastExplorationFSM::taskCollisionStopTick() {
  if(task_collision_stop_id_<0||task_collision_stop_id_!=executed_id_||executed_done_)return;
  const double now=racer_ros::Time::now().toSec();
  if(now-task_collision_stop_last_sent_<.2)return;
  std_msgs::Float64MultiArray request;
  request.data={double(task_collision_stop_id_),task_collision_stop_stamp_,now};
  task_collision_stop_pub_.publish(request);
  task_collision_stop_last_sent_=now;
}

void FastExplorationFSM::safetyCallback(''')
    text = replace_once(text, '  if (!live_execution_ || (msg->data.size() != 7 && msg->data.size() != 8)) return;', '''  if (!live_execution_ || (msg->data.size() != 7 && msg->data.size() != 8 &&
      !(task_collision_stop_enabled_ && msg->data.size() == 9))) return;''')
    text = replace_once(text, '  if (d[5] < feedback_stamp_ || fabs(racer_ros::Time::now().toSec()-d[5]) > 0.5) return;', '''  const double received_at=racer_ros::Time::now().toSec();
  if (d[5] < feedback_stamp_ || fabs(received_at-d[5]) > 0.5) {
    if(d.size()==9&&d[7]==task_collision_stop_id_&&d[8]==task_collision_stop_stamp_)
      ROS_WARN("STOP_FEEDBACK rejected id=%.0f sent=%.6f received=%.6f request=%.6f",d[7],d[5],received_at,d[8]);
    return;
  }''')
    text = replace_once(text, '  feedback_stamp_ = d[5];', '''  const bool collision_feedback=d.size()==9;
  const bool collision_stop=collision_feedback && task_collision_stop_id_>=0 &&
      task_collision_stop_id_==executed_id_ && !executed_done_ &&
      d[7]==task_collision_stop_id_ && d[8]==task_collision_stop_stamp_;
  const bool collision_hold_feedback=collision_feedback && executed_id_<0 &&
      task_collision_stopped_ && task_collision_stop_released_id_>=0 &&
      d[7]==task_collision_stop_released_id_ && d[8]==task_collision_stop_stamp_;
  // A delayed or mismatched stop receipt is no evidence about a newer execution.
  if(collision_feedback && (d[0]!=-1. || d[2]!=0. ||
      (!collision_stop && !collision_hold_feedback) ||
      !(d[3]>=-1.) || d[3]>std::numeric_limits<int>::max() || std::floor(d[3])!=d[3]))return;
  feedback_stamp_ = d[5];
  // While the executor is still holding, the gateway repeats its last receipt.
  // Reuse its heartbeat and a denial of a distinct currently published proposal.
  if(collision_hold_feedback && !(state_==PUB_TRAJ && live_sent_ &&
      d[3]>=0. && d[3]!=task_collision_stop_released_id_ &&
      d[3]==fd_->newest_traj_.traj_id && d[4]<0.))return;''')
    text = replace_once(text, '  bool verified_stop=observation_stop;', '''  bool verified_stop=observation_stop;
  if(collision_stop){
    verified_stop=true;
    fd_->avoid_collision_=false;
    task_collision_stopped_=true;
    task_collision_stop_released_id_=task_collision_stop_id_;
    task_collision_stop_id_=-1;
    ROS_WARN("COLLISION_STOP released id=%.0f sim=%.6f request=%.6f received=%.6f",d[7],d[5],d[8],received_at);
  }''')
    text = replace_once(text, '    executed_id_ = active;', '''    task_collision_stopped_=false;
    task_collision_stop_id_=-1;
    task_collision_stop_released_id_=-1;
    executed_id_ = active;''')
    text = replace_once(text, ' if(state_!=EXEC_TRAJ&&state_!=IDLE&&!waiting_execution)return false;', ''' const bool stopped_after_collision=state_==PLAN_TRAJ&&task_collision_stopped_&&executed_id_<0;
 if(state_!=EXEC_TRAJ&&state_!=IDLE&&!waiting_execution&&!stopped_after_collision)return false;''')
    text = replace_once(text, ' if(state_==IDLE){\n  // No current exploration goal', ''' if(stopped_after_collision){
  ROS_INFO("COLLISION_STOP evaluating fresh event token=%llu source=%llu sim=%.6f capture=%.6f",
    static_cast<unsigned long long>(task_static_work_->token),
    static_cast<unsigned long long>(task_static_work_->evidence.key.counter),now,
    task_static_work_->evidence.capture);
  return false; // Obtain the native exploration baseline from the new stopped pose.
 }
 if(state_==IDLE){
  // No current exploration goal''')
    path.write_text(text)
