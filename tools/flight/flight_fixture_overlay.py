"""TEST BUILD ONLY: resource fault / constant-yaw inputs in the real native FSM.

No fabricated return codes, optimizer outputs, occupancy, odometry or controls.
Only a --flight-fixture build contains these hooks; environment opts in per run.
"""
from tools.flight.optimizer_overlay import replace_once


def apply(core):
    p = core/'include/exploration_manager/fast_exploration_fsm.h'
    s = p.read_text()
    s = replace_once(s, '  bool task_no_path_recovery_', '''  std::string task_fixture_mode_;
  bool task_fixture_armed_ = false, task_fixture_done_ = false;
  int task_fixture_proposal_ = -1;
  bool taskFixtureYaw(int& result);
  bool task_no_path_recovery_''')
    p.write_text(s)
    p = core/'src/plan_manage/planner_manager.cpp'
    s = '#include <cstdlib>\n#include <string>\n'+p.read_text()
    s = replace_once(s, '  // Call B-spline optimization solver', '''  // TEST ONLY: request a genuinely constant optimizer INITIAL guess at the
  // measured heading. Native nonzero-yaw initialization otherwise has zero
  // interior controls, even when start/end headings match, so that would not
  // exercise the zero-spacing bug. The real objective/NLopt/output stay intact.
  const char* fixture=std::getenv("RACER_FLIGHT_FIXTURE");
  if(fixture && std::string(fixture)=="constant-yaw" &&
     std::abs(start_yaw3d[0]-end_yaw3d[0])<1e-10 &&
     std::abs(start_yaw3d[1])+std::abs(start_yaw3d[2])<1e-10) {
    yaw.setConstant(start_yaw3d[0]);
    ROS_WARN("TASK_FLIGHT_FIXTURE constant_initializer heading=%.9f points=%d",
      start_yaw3d[0],int(yaw.rows()));
  }
  // Call B-spline optimization solver''')
    p.write_text(s)
    p = core/'src/exploration_manager/fast_exploration_fsm.cpp'
    s = '#include <cstdlib>\n'+p.read_text()
    s = replace_once(s, '  fp_.reset(new FSMParam);', '''  const char* mode=std::getenv("RACER_FLIGHT_FIXTURE");
  task_fixture_mode_=mode?mode:"none";
  if(task_fixture_mode_!="none" && task_fixture_mode_!="timeout-primary" &&
     task_fixture_mode_!="timeout-all" && task_fixture_mode_!="constant-yaw" &&
     task_fixture_mode_!="return-home" && task_fixture_mode_!="return-primary" &&
     task_fixture_mode_!="return-all")
    throw std::runtime_error("Unknown flight fixture");
  ROS_WARN("TASK_FLIGHT_FIXTURE configured mode=%s",task_fixture_mode_.c_str());
  fp_.reset(new FSMParam);''')
    s = replace_once(s, '  if(task_return_.started()) {\n    recovery=recovery', '''  // TEST ONLY: request return after actual completed motion away from home.
  // Neither the measured home nor the return/recovery planner is fabricated.
  if(task_fixture_mode_.rfind("return-",0)==0 && !task_return_.started() &&
     executed_id_>=0 && executed_done_ && (fd_->odom_pos_-fd_->start_pos_).norm()>1.5 &&
     (task_fixture_mode_=="return-home" || task_np_breadcrumbs_.size()>=3)) {
    task_return_.begin({fd_->start_pos_.x(),fd_->start_pos_.y(),fd_->start_pos_.z()});
    fd_->go_back_=true;task_fixture_armed_=true;
    taskReturnLog("controlled_return_trigger");
  }
  if(task_return_.started()) {
    recovery=recovery''')
    s = replace_once(s, '  int res;\n  bool review', '''  if(task_fixture_mode_!="none" && !task_fixture_done_ &&
     !task_fixture_armed_ && executed_id_>=0 && executed_done_) {
    task_fixture_armed_=true;
    ROS_WARN("TASK_FLIGHT_FIXTURE armed sim=%.6f mode=%s previous_id=%d root=%.6f,%.6f,%.6f",
      racer_ros::Time::now().toSec(),task_fixture_mode_.c_str(),executed_id_,
      fd_->start_pt_.x(),fd_->start_pt_.y(),fd_->start_pt_.z());
  }
  auto& astar=planner_manager_->path_finder_;
  struct RestoreBudget { double& budget; double saved;
    ~RestoreBudget(){budget=saved;}
  } restore{astar->max_search_time_,astar->max_search_time_};
  const bool inject=(task_return_.started() &&
    (task_fixture_mode_=="return-all" ||
     (task_fixture_mode_=="return-primary" && !task_np_policy_.active()))) ||
    (task_fixture_armed_ && !task_fixture_done_ &&
    (task_fixture_mode_=="timeout-all" ||
     (task_fixture_mode_=="timeout-primary" && !task_np_policy_.active())));
  // Run the real A* search with zero allotted wall time. Its original timeout
  // branch returns NO_PATH; recovery gets the original budget in primary mode.
  if(inject) {
    astar->max_search_time_=0.;
    ROS_WARN("TASK_FLIGHT_FIXTURE budget sim=%.6f mode=%s recovery=%d original=%.6f injected=0",
      racer_ros::Time::now().toSec(),task_fixture_mode_.c_str(),int(task_np_policy_.active()),restore.saved);
  }
  int res;
  bool review''')
    s = replace_once(s, '  } else if(review) {', '''  } else if(task_fixture_armed_ && !task_fixture_done_ &&
            task_fixture_mode_=="constant-yaw" && taskFixtureYaw(res)) {
    // Same native planner, with a measured-map goal and unchanged start yaw.
  } else if(review) {''')
    s = replace_once(s, '  if (res == SUCCEED) {\n    auto info', '''  if(task_fixture_armed_ && !task_fixture_done_ && res==SUCCEED) {
    task_fixture_proposal_=live_counter_+1;
    ROS_WARN("TASK_FLIGHT_FIXTURE proposal sim=%.6f mode=%s id=%d",
      racer_ros::Time::now().toSec(),task_fixture_mode_.c_str(),task_fixture_proposal_);
  }
  if (res == SUCCEED) {\n    auto info''')
    s = replace_once(s, '    task_rejections_=0;', '''    if(task_fixture_armed_ && !task_fixture_done_ && active==task_fixture_proposal_) {
      ROS_WARN("TASK_FLIGHT_FIXTURE activated sim=%.6f mode=%s id=%d",
        racer_ros::Time::now().toSec(),task_fixture_mode_.c_str(),active);
      if(task_fixture_mode_!="timeout-all")task_fixture_done_=true;
    }
    task_rejections_=0;''')
    s = replace_once(s, 'void FastExplorationFSM::taskReview(', '''bool FastExplorationFSM::taskFixtureYaw(int& result) {
  auto map=expl_manager_->sdf_map_;auto ed=expl_manager_->ed_;
  // Test input chosen exclusively from live observed native frontier goals.
  int selected=-1;double best=INFINITY;
  for(size_t i=0;i<ed->points_.size();++i) {
    const auto p=ed->points_[i];const double d=(p-fd_->start_pt_).norm();
    if(d>.6 && d<best && map->getOccupancy(p)==SDFMap::FREE && map->getInflateOccupancy(p)==0)
      {selected=int(i);best=d;}
  }
  if(selected<0)return false;
  ed->next_pos_=ed->points_[selected];ed->next_yaw_=fd_->start_yaw_[0];
  ROS_WARN("TASK_FLIGHT_FIXTURE constant_yaw sim=%.6f yaw=%.9f goal=%.6f,%.6f,%.6f",
    racer_ros::Time::now().toSec(),ed->next_yaw_,ed->next_pos_.x(),ed->next_pos_.y(),ed->next_pos_.z());
  result=expl_manager_->planTrajToView(fd_->start_pt_,fd_->start_vel_,fd_->start_acc_,
    fd_->start_yaw_,ed->next_pos_,ed->next_yaw_);
  return true;
}

void FastExplorationFSM::taskReview(''')
    p.write_text(s)
