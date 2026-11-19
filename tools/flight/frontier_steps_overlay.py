"""Stage difficult frontier observations from current measured space only."""
from pathlib import Path
import shutil
from .optimizer_overlay import replace_once

def apply(core):
    shutil.copyfile(Path(__file__).with_name('frontier_steps.hpp'),
                    core/'include/exploration_manager/aspire_frontier_steps.hpp')
    inc='#include <exploration_manager/aspire_frontier_steps.hpp>\n'
    header=core/'include/exploration_manager/fast_exploration_manager.h'
    text=replace_once(header.read_text(),
        'const Vector3d& acc, const Vector3d& yaw);',
        'const Vector3d& acc, const Vector3d& yaw, bool settled = false);')
    header.write_text(inc+replace_once(text, 'private:\n', '''private:
  double frontier_step_radius_;
  aspire_steps::ReplacementHistory frontier_step_history_;
'''))
    f=core/'src/exploration_manager/fast_exploration_manager.cpp'
    anchor='  planner_manager_->initPlanModules(nh);'
    s=replace_once(f.read_text(), anchor, anchor+'''
  nh.param("sdf_map/obstacles_inflation", frontier_step_radius_, -1.0);
  frontier_step_radius_ = std::max(.50, frontier_step_radius_);''')
    start=s.index('int FastExplorationManager::planExploreMotion(')
    end=s.index('\nint FastExplorationManager::planTrajToView(', start)
    plan=s[start:end]
    plan=replace_once(plan, 'const Vector3d& acc, const Vector3d& yaw) {',
                     'const Vector3d& acc, const Vector3d& yaw, bool settled) {')
    anchor='  findGridAndFrontierPath(pos, vel, yaw, grid_ids, frontier_ids);'
    plan=replace_once(plan, anchor, anchor+'''
  int next_frontier = frontier_ids.empty() ? -1 : frontier_ids.front();
  vector<vector<double>> n_yaws;''')
    plan=replace_once(plan, '    // // Simply go to the center of the unknown grid',
                     '    next_frontier = min_cost_id;\n\n    // // Simply go to the center of the unknown grid')
    plan=replace_once(plan, '      vector<vector<double>> n_yaws;\n', '')
    plan=replace_once(plan, '    vector<vector<double>> n_yaws;\n', '')
    anchor='''      ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    }'''
    plan=replace_once(plan, anchor, anchor+''' else {
      next_pos = ed_->points_[next_frontier];
      next_yaw = ed_->yaws_[next_frontier];
    }''')
    anchor='  taskBearingRerank(pos,vel,yaw,next_pos,next_yaw);'
    plan=replace_once(plan, anchor, '''  const bool bearing_selected = taskBearingRerank(pos,vel,yaw,next_pos,next_yaw);
  const Vector3d tour_point = next_pos;
  const double tour_yaw = next_yaw;
  bool prefer_observation = false;
  if (aspire_steps::enabled() && settled && ep_->drone_num_ == 1 && !bearing_selected &&
      !task_static_evaluation_ && next_frontier >= 0) {
    vector<Vector3d> alternatives = {ed_->points_[next_frontier]};
    vector<double> headings = {ed_->yaws_[next_frontier]};
    if (!n_yaws.empty()) {
      alternatives.insert(alternatives.end(), ed_->n_points_.front().begin(), ed_->n_points_.front().end());
      headings.insert(headings.end(), n_yaws.front().begin(), n_yaws.front().end());
    }
    const auto observation = aspire_steps::refine(*sdf_map_, *frontier_finder_->percep_utils_,
        pos, yaw[0], tour_point, tour_yaw, alternatives, headings, ed_->frontiers_[next_frontier],
        ed_->averages_[next_frontier], Vector3d(frontier_step_radius_, frontier_step_radius_, .30),
        ViewNode::vm_, ViewNode::yd_);
    const bool candidate = observation.source != aspire_steps::Source::None;
    prefer_observation = candidate && frontier_step_history_.accept(
        pos, observation.point, sdf_map_->getResolution());
    if (prefer_observation) {
      next_pos = observation.point;
      next_yaw = observation.yaw;
    }
    const char* source = "tour_fallback";
    const char* reason = (tour_point-pos).norm() <= aspire_steps::max_observation_leg ?
        "near_tour" : "no_candidate";
    if (prefer_observation) {
      source = observation.source == aspire_steps::Source::Native ? "native" : "staged";
      reason = "selected";
    } else if (candidate) {
      reason = "repeat_without_progress";
    }
    ROS_INFO("FRONTIER_STEP exploration frontier=%d source=%s changed=%d visible=%d range=%.3f tour_range=%.3f reason=%s",
        next_frontier, source, int(prefer_observation), observation.visible, (next_pos-pos).norm(),
        (tour_point-pos).norm(), reason);
  }''')
    anchor='''  if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
    return FAIL;
  }'''
    plan=replace_once(plan, anchor, '''  if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
    if (!prefer_observation) return FAIL;
    ROS_INFO("FRONTIER_STEP exploration frontier=%d source=tour_fallback reason=native_plan_failed", next_frontier);
    ed_->next_pos_ = tour_point;
    ed_->next_yaw_ = tour_yaw;
    if (planTrajToView(pos, vel, acc, yaw, tour_point, tour_yaw) == FAIL) return FAIL;
  }''')
    f.write_text(inc+s[:start]+plan+s[end:])
    f=core/'src/exploration_manager/fast_exploration_fsm.cpp'
    text=replace_once(f.read_text(), '''res = expl_manager_->planExploreMotion(
        fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);''',
        '''res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
        fd_->start_yaw_, live_execution_ && task_stop_and_plan_ && fd_->static_state_);''')
    at='  // Last observed, previously authorized roots'
    f.write_text(inc+replace_once(text,at,'''  if(aspire_steps::enabled()) {
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
