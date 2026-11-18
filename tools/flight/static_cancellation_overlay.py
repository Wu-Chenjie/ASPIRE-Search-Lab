"""Optional cancellation transaction around the existing static execution gate."""
from pathlib import Path
import shutil
from .optimizer_overlay import replace_once


def apply(core):
    here=Path(__file__).parent
    inc=core/'include/exploration_manager'
    shutil.copyfile(here/'static_cancellation.hpp',inc/'static_cancellation.hpp')
    p=inc/'fast_exploration_fsm.h';s=p.read_text()
    s=replace_once(s,'#include <exploration_manager/static_replan_gate.hpp>',
        '#include <exploration_manager/static_replan_gate.hpp>\n#include <exploration_manager/static_cancellation.hpp>')
    s=replace_once(s,'  racer_task::StaticReplanGate task_static_gate_;','''  racer_task::StaticReplanGate task_static_gate_;
  racer_task::StaticCancellation task_static_cancel_;
  racer_ros::Publisher task_static_cancel_pub_;
  racer_ros::Subscriber task_static_cancel_sub_;
  void taskStaticCancelAck(const std_msgs::Float64MultiArrayConstPtr& msg);''')
    p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_fsm.cpp';s=p.read_text()
    s=replace_once(s,'  expl_manager_->task_static_event_consumer_=task_static_replan_;','''  std::string cancel_session;
  nh.param("search.static_cancel_session",cancel_session,std::string(""));
  if(task_static_replan_&&!cancel_session.empty()){
    if(cancel_session.find_first_not_of("0123456789")!=std::string::npos)throw std::runtime_error("Invalid static cancel session");
    task_static_cancel_.session=std::stod(cancel_session);
    if(!task_static_cancel_.enabled())throw std::runtime_error("Invalid exact static cancel session");
    const auto suffix=std::to_string(getId()-1);
    task_static_cancel_pub_=nh.advertise<std_msgs::Float64MultiArray>("/baseline/static_cancel_"+suffix,10);
    task_static_cancel_sub_=nh.subscribe("/baseline/static_cancel_ack_"+suffix,10,&FastExplorationFSM::taskStaticCancelAck,this);
  }
  expl_manager_->task_static_event_consumer_=task_static_replan_;''')
    s=replace_once(s,'  if(task_static_gate_.published)return false; // executor owns an already submitted proposal','''  if(task_static_gate_.published){
   if(task_static_cancel_.enabled()&&!manager.task_static_events_.valid(task_static_work_->token)&&
      task_static_cancel_.due(now,task_static_gate_.min_interval_s)){
    std_msgs::Float64MultiArray request;request.data=task_static_cancel_.identity;
    task_static_cancel_pub_.publish(request);
   }
   // Feedback may move the FSM to PLAN before the separate receipt arrives.
   // Do not compute/overwrite another proposal while this transaction owns it.
   return task_static_cancel_.enabled()&&(state_==PLAN_TRAJ||state_==PUB_TRAJ||state_==IDLE);
  }''')
    s=replace_once(s,'        if(task_static_work_)boundary_pub_.publish(task_static_metadata_);','''        if(task_static_work_&&task_static_cancel_.enabled()){
          if(!task_static_cancel_.bind(double(task_static_work_->token),task_static_gate_.proposal,
                                      double(task_static_gate_.map_revision))){
            taskStaticRelease(false,"invalid_static_cancel_identity");break;
          }
          task_static_metadata_.data.push_back(task_static_cancel_.session);
          task_static_metadata_.data.push_back(double(task_static_work_->token));
          task_static_metadata_.data.push_back(double(task_static_gate_.map_revision));
        }
        if(task_static_work_)boundary_pub_.publish(task_static_metadata_);''')
    old='  if(task_static_work_&&task_static_gate_.resolved(active,candidate,d[4]<0.)){'
    s=replace_once(s,old,'''  if(task_static_work_&&task_static_gate_.published&&(
      (!task_static_cancel_.enabled()&&task_static_gate_.resolved(active,candidate,d[4]<0.))||
      (task_static_cancel_.enabled()&&task_static_cancel_.released(active,d[2]>0.,
          candidate==task_static_gate_.proposal&&d[4]<0.)))){''')
    s=replace_once(s,'    task_static_prior_goal_valid_=false;\n    task_static_gate_.resetProposal();',
        '    task_static_prior_goal_valid_=false;\n    task_static_cancel_.reset();\n    task_static_gate_.resetProposal();')
    s=replace_once(s,'void FastExplorationFSM::executionFeedback(','''void FastExplorationFSM::taskStaticCancelAck(const std_msgs::Float64MultiArrayConstPtr& msg){
 if(task_static_work_&&task_static_gate_.published&&task_static_cancel_.enabled())
   task_static_cancel_.acknowledge(msg->data);
 // Never mutate executor-owned trajectory or release solely on this topic.
}
void FastExplorationFSM::executionFeedback(''')
    p.write_text(s)
