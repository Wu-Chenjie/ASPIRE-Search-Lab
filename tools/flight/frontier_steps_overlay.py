"""Stage difficult frontier observations from current measured space only."""
from pathlib import Path
import shutil
from .optimizer_overlay import replace_once

def apply(core):
    shutil.copyfile(Path(__file__).with_name('frontier_steps.hpp'),
                    core/'include/exploration_manager/aspire_frontier_steps.hpp')
    inc='#include <exploration_manager/aspire_frontier_steps.hpp>\n'
    f=core/'src/exploration_manager/fast_exploration_manager.cpp'
    at='  std::cout << "Next view: "'
    f.write_text(inc+replace_once(f.read_text(),at,'''  if (aspire_steps::enabled() && !ed_->points_.empty() && (next_pos-pos).norm()>1.6) {
    size_t k=0;
    for(size_t i=1;i<ed_->points_.size();++i)
      if((ed_->points_[i]-next_pos).squaredNorm()<(ed_->points_[k]-next_pos).squaredNorm()) k=i;
    Vector3d staged;double look=next_yaw;
    if(k<ed_->frontiers_.size() && k<ed_->averages_.size() &&
       aspire_steps::select(*sdf_map_,pos,next_pos,ed_->frontiers_[k],ed_->averages_[k],
                            [](const Vector3d&){return true;},staged,look)){
      ROS_INFO("FRONTIER_STEP native observation range=%.3f yaw=%.3f",(staged-pos).norm(),look);
      next_pos=staged;next_yaw=look;
    }
  }
'''+at))
    f=core/'src/exploration_manager/fast_exploration_fsm.cpp'
    at='  // Last observed, previously authorized roots'
    f.write_text(inc+replace_once(f.read_text(),at,'''  if(aspire_steps::enabled()) {
    // Give a frontier a reachable intermediate observation before falling back
    // to a previously visited root. The recovery budget/blacklist still applies.
    double best=1e9; Eigen::Vector3d chosen;double heading=0;
    for(size_t i=0;i<ed->points_.size() && i<ed->frontiers_.size() &&
                       i<ed->averages_.size() && i<ed->yaws_.size();++i){
      Eigen::Vector3d p; double h=ed->yaws_[i];
      auto eligible=[&](const Eigen::Vector3d& q){
        return task_np_policy_.eligible({q.x(),q.y(),q.z()},root,now);
      };
      if(!aspire_steps::select(*map,fd_->start_pt_,ed->points_[i],ed->frontiers_[i],
                               ed->averages_[i],eligible,p,h))continue;
      const double cost=(ed->points_[i]-fd_->start_pt_).norm();
      if(cost<best){best=cost;chosen=p;heading=h;}
    }
    if(best<1e9){
      task_np_goal_=chosen;ed->next_pos_=chosen;ed->next_yaw_=heading;
      task_np_policy_.attempted({chosen.x(),chosen.y(),chosen.z()},now);
      ROS_INFO("FRONTIER_STEP recovery range=%.3f yaw=%.3f",(chosen-fd_->start_pt_).norm(),heading);
      return expl_manager_->planTrajToView(fd_->start_pt_,fd_->start_vel_,fd_->start_acc_,
                                           fd_->start_yaw_,chosen,heading);
    }
  }
'''+at))
    # A stopped native curve may end a few ULPs past a full turn. Give the
    # next plan the equivalent principal heading, preserving both derivatives.
    at = '  bool task_planned_alternative = false;'
    f.write_text(replace_once(f.read_text(), at, """  if(aspire_steps::enabled() && fd_->static_state_ &&
     std::isfinite(fd_->start_yaw_[0]) && std::abs(fd_->start_yaw_[0])<=3*M_PI+1e-6){
    const double raw=fd_->start_yaw_[0];
    fd_->start_yaw_[0]=std::remainder(raw,2*M_PI);
    if(std::abs(fd_->start_yaw_[0]-raw)>1.)ROS_INFO("FRONTIER_STEP equivalent heading raw=%.9f principal=%.9f",raw,fd_->start_yaw_[0]);
  }
""" + at))
