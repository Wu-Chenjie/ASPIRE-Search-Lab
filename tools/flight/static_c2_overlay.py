"""Opt-in moving proposal scheduling; all external safety checks stay required."""
from pathlib import Path
import shutil
from .optimizer_overlay import replace_once


def apply(core):
    here=Path(__file__).parent;inc=core/'include/exploration_manager'
    shutil.copyfile(here/'static_c2_gate.hpp',inc/'static_c2_gate.hpp')
    shutil.copyfile(here/'static_execution_monitor.hpp',inc/'static_execution_monitor.hpp')
    p=inc/'fast_exploration_fsm.h';s=p.read_text()
    s=replace_once(s,'#include <exploration_manager/static_cancellation.hpp>',
        '#include <exploration_manager/static_cancellation.hpp>\n#include <exploration_manager/static_c2_gate.hpp>\n#include <exploration_manager/static_execution_monitor.hpp>')
    s=replace_once(s,'  racer_task::StaticCancellation task_static_cancel_;',
        '  racer_task::StaticCancellation task_static_cancel_;\n  racer_task::StaticC2Gate task_static_c2_;\n  racer_task::StaticExecutionMonitor task_static_monitor_;\n  void taskStaticExecutionFeedback(int active,bool done,int candidate,bool rejected,double now,double observation_s);')
    p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_fsm.cpp';s=p.read_text()
    s=replace_once(s,'  expl_manager_->task_static_event_consumer_=task_static_replan_;','''  nh.param("search.static_moving_c2",task_static_c2_.enabled,false);
  nh.param("search.static_c2_lead_s",task_static_c2_.lead,-1.);
  if(task_static_c2_.enabled&&task_static_replan_&&(!task_static_cancel_.enabled()||
      !std::isfinite(task_static_c2_.lead)||task_static_c2_.lead<=0.))
    throw std::runtime_error("Moving C2 requires cancellation and explicit positive lead");
  task_static_c2_.enabled=task_static_c2_.enabled&&task_static_replan_;
  expl_manager_->task_static_event_consumer_=task_static_replan_;''')
    old='      if(task_stop_and_plan_ && live_execution_ && executed_id_>=0 && !executed_done_) return;'
    s=replace_once(s,old,'''      const bool task_c2_requested=task_static_c2_.enabled&&task_static_work_&&
        !task_static_gate_.published&&live_execution_&&executed_id_>=0&&!executed_done_&&executed_data_;
      if(task_c2_requested){
        if(!task_static_c2_.prepare(executed_id_,executed_progress_,executed_data_->duration_,
            racer_ros::Time::now().toSec(),feedback_stamp_,expl_manager_->sdf_map_->taskStaticMapRevision())){
          // Keep the admitted event while the predecessor completes. On a
          // fresh stopped state the original stop-and-plan path can use it.
          // Event invalidation, competing missions and reserve checks still run.
          return;
        }
      }else task_static_c2_.reset();
      // The legacy stop gate remains the fallback. Only the explicit static
      // transaction may submit a moving proposal to independent safety checks.
      if(task_stop_and_plan_ && live_execution_ && executed_id_>=0 && !executed_done_ && !task_c2_requested) return;''')
    s=replace_once(s,'double t_r = live_execution_ ? executed_progress_ + fp_->replan_time_ :',
        'double t_r = live_execution_ ? (task_c2_requested?task_static_c2_.boundary:executed_progress_ + fp_->replan_time_) :')
    s=replace_once(s,'      int res = callExplorationPlanner();','''      int res = callExplorationPlanner();
      if(task_c2_requested){
        task_static_c2_.planning_wall=(racer_ros::WallTime::now()-wall_started).toSec();
        if(!task_static_c2_.current(executed_id_,executed_progress_,racer_ros::Time::now().toSec(),
             feedback_stamp_,expl_manager_->sdf_map_->taskStaticMapRevision())){
          taskStaticRelease(true,"moving_C2_result_expired");break;
        }
        task_static_c2_.proposal=live_counter_+1;
      }''')
    s=replace_once(s,'        if(!task_static_gate_.mayPublish(','''        const bool moving_publication=task_static_c2_.owns(task_static_gate_.proposal);
        if(moving_publication&&
           !task_static_c2_.current(executed_id_,executed_progress_,publish_now,feedback_stamp_,publish_map)){
          taskStaticRelease(true,"moving_C2_publication_expired");break;
        }
        if(!task_static_gate_.mayPublish(''')
    s=replace_once(s,'executed_id_<0||executed_done_,task_stop_and_plan_,',
        'executed_id_<0||executed_done_,task_stop_and_plan_&&!moving_publication,')
    s=replace_once(s,' if(task_static_work_){\n  if(task_static_gate_.published){',''' if(task_static_c2_.enabled){
  const bool holding=task_static_monitor_.tick(manager.task_static_events_,now,task_static_gate_.min_interval_s,
    [&](const std::vector<double>& identity){
      std_msgs::Float64MultiArray request;request.data=identity;task_static_cancel_pub_.publish(request);
    });
  manager.task_static_events_.setExecutingSources(task_static_monitor_.reservedSources());
  if(holding&&(!task_static_work_||!task_static_gate_.published))return true;
  if(!task_static_work_&&!task_static_monitor_.room())return false;
 }
 if(task_static_work_){
  if(task_static_gate_.published){''')
    s=replace_once(s,'task_static_events_.start(now);',
        'task_static_events_.start(now,task_static_monitor_.reservedSources());')
    s=replace_once(s,' // Never mutate executor-owned trajectory or release solely on this topic.',''' if(task_static_c2_.enabled&&task_static_monitor_.acknowledge(msg->data))
   expl_manager_->task_static_events_.setExecutingSources(task_static_monitor_.reservedSources());
 // Receipt collection requires separately observed inactive feedback; no motion mutation.''')
    s=replace_once(s,'  if(task_static_work_&&task_static_gate_.published&&(\n','''  taskStaticExecutionFeedback(active,d[2]>0.,candidate,d[4]<0.,
      racer_ros::Time::now().toSec(),task_static_budget_.observation_s);
  if(task_static_work_&&task_static_gate_.published&&(
''')
    s=replace_once(s,'void FastExplorationFSM::executionFeedback(',
        (here/'static_c2_methods.cpp').read_text()+'\nvoid FastExplorationFSM::executionFeedback(')
    p.write_text(s)
