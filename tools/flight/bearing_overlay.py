"""Opt-in native next-view cost bias on a generated build; migration untouched."""
from pathlib import Path
import shutil
from .optimizer_overlay import replace_once

def apply(core):
    here=Path(__file__).parent
    shutil.copyfile(here/'bearing_bias.hpp',core/'include/exploration_manager/task_bearing_bias.hpp')
    p=core/'include/exploration_manager/fast_exploration_manager.h';s=p.read_text()
    s=replace_once(s,'#include <racer_ros2/runtime.hpp>','#include <racer_ros2/runtime.hpp>\n#include <exploration_manager/task_bearing_bias.hpp>\n#include <racer_ros2/messages.hpp>\n#include <std_msgs/msg/string.hpp>')
    s=replace_once(s,'  shared_ptr<ExplorationData> ed_;','''  racer_task::BearingBias task_bearing_policy_;
  racer_ros::Subscriber task_bearing_sub_;
  racer_ros::Publisher task_bearing_log_;
  int task_bearing_sequence_=0;
  bool task_bearing_preview_=false;
  bool task_bearing_pending_=false;
  bool task_bearing_changed_=false;
  Eigen::Vector3d task_bearing_target_=Eigen::Vector3d::Zero();
  void taskBearingReceive(const std_msgs::Float64MultiArrayConstPtr& msg);
  bool taskBearingRerank(const Vector3d& pos,const Vector3d& vel,const Vector3d& yaw,Vector3d& goal,double& goal_yaw);
  void taskBearingPlan(int native_id,int result);
  shared_ptr<ExplorationData> ed_;''');p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_manager.cpp';s=p.read_text()
    s='#include <sstream>\n#include <iomanip>\n#include <algorithm>\n'+s
    s=replace_once(s,'  ed_->plan_num_ = 0;','''  ed_->plan_num_ = 0;
  nh.param("search/bearing_bias",task_bearing_policy_.enabled,false);
  if(task_bearing_policy_.enabled && ep_->drone_num_!=1)
    throw std::runtime_error("Stage26 bearing bias is validated for one UAV only");
  task_bearing_log_=nh.advertise<std_msgs::msg::String>("/search/bearing_decision",100);
  task_bearing_sub_=nh.subscribe("/search/bearing_evidence",30,&FastExplorationManager::taskBearingReceive,this);''')
    s=replace_once(s,'  std::cout << "Next view: "','  taskBearingRerank(pos,vel,yaw,next_pos,next_yaw);\n\n  std::cout << "Next view: "')
    marker='int FastExplorationManager::planExploreMotion('
    s=replace_once(s,marker,(here/'bearing_methods.cpp').read_text()+'\n\n'+marker);p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_fsm.cpp';s=p.read_text()
    start=s.index('int FastExplorationFSM::callExplorationPlanner()')
    index=s.index('  if (res == SUCCEED) {',start)
    s=s[:index]+'  if(!review)expl_manager_->taskBearingPlan(live_counter_+1,res);\n'+s[index:]
    p.write_text(s)
