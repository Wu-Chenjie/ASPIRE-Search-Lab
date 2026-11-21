"""Thread the stopped-return request through native position and yaw planning."""
from .optimizer_overlay import replace_once


def add_argument(source, call, argument):
    """Edit one signature or call in the pinned native sources."""
    return replace_once(source, call + ')', call + ', ' + argument + ')')


def apply(core):
    include = core / 'include'
    src = core / 'src'
    header = include / 'exploration_manager/fast_exploration_fsm.h'
    header.write_text(replace_once(header.read_text(), 'int taskRecoveryPlan();',
                                  'int taskRecoveryPlan(bool fixed_return);'))
    f = src / 'exploration_manager/fast_exploration_fsm.cpp'
    s = replace_once(f.read_text(), 'int FastExplorationFSM::taskRecoveryPlan()',
                     'int FastExplorationFSM::taskRecoveryPlan(bool fixed_return)')
    s = replace_once(s, 'res=taskRecoveryPlan();', 'res=taskRecoveryPlan(fixed_return);')
    for call in ('fd_->start_yaw_,chosen,heading', 'fd_->start_yaw_,task_np_goal_,ed->next_yaw_',
                 'fd_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_'):
        s = add_argument(s, call, 'fixed_return')
    registration = '    if(res==SUCCEED)task_return_.proposed(live_counter_+1);'
    s = replace_once(s, registration, """    if (fixed_return && res == SUCCEED) {
      ROS_INFO("TASK_RETURN_ROOT id=%d root=%.9f,%.9f,%.9f yaw=%.9f", live_counter_+1,
          fd_->start_pt_.x(), fd_->start_pt_.y(), fd_->start_pt_.z(), fd_->start_yaw_[0]);
    }
""" + registration)
    s = replace_once(s, 'if(task_feasible_time_) {', 'if(task_feasible_time_ || fixed_return) {')
    s = replace_once(s, 'yaw.setUniformBspline(yp,3,yaw.getKnotSpan()*factor);',
                     'yaw.setUniformBspline(yp,3,yaw.getKnotSpan()*factor);\n'
                     '        planner_manager_->plan_data_.dt_yaw_ = yaw.getKnotSpan();')
    f.write_text(s)

    signature = 'const Vector3d& next_pos, const double& next_yaw'
    header = include / 'exploration_manager/fast_exploration_manager.h'
    header.write_text(add_argument(header.read_text(), signature, 'bool fixed_return = false'))
    f = src / 'exploration_manager/fast_exploration_manager.cpp'
    s = add_argument(f.read_text(), signature, 'bool fixed_return')
    for call in ('planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb',
                 'planExploreTraj(truncated_path, vel, acc, time_lb',
                 'pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb',
                 'planYawExplore(yaw, next_yaw, true, ep_->relax_time_'):
        s = add_argument(s, call, 'fixed_return')
    f.write_text(s)

    header = include / 'plan_manage/planner_manager.h'
    s = header.read_text()
    for declaration in ('const Eigen::Vector3d& end_vel, const double& time_lb = -1',
                        'const Eigen::Vector3d& cur_acc, const double& time_lb = -1',
                        'const double& relax_time'):
        s = add_argument(s, declaration, 'bool fixed_return = false')
    header.write_text(s)
    f = src / 'plan_manage/planner_manager.cpp'
    s = f.read_text()
    for definition in ('const Eigen::Vector3d& end_vel, const double& time_lb',
                       'const Eigen::Vector3d& cur_acc, const double& time_lb',
                       'bool lookfwd, const double& relax_time'):
        s = add_argument(s, definition, 'bool fixed_return')
    # Enforce the searched endpoint for the waypoint branch. The Kino branch
    # uses the last sampled search point, including on a partial return.
    s = replace_once(s, '  start[2] = cur_acc;', '''  start[2] = cur_acc;
  if (fixed_return) end = {tour.back(), Vector3d::Zero(), Vector3d::Zero()};''')
    for code, following in [('cost_function', '\n  vector<Eigen::Vector3d> start, end;'),
                            ('cost_func', '\n\n  vector<Vector3d> start, end;')]:
        original = f'if (pp_.min_time_) {code} |= BsplineOptimizer::MINTIME;' + following
        s = replace_once(s, original, original.replace('if (pp_.min_time_)',
                                                       'if (pp_.min_time_ && !fixed_return)'))
    for call in ('bspline_optimizers_[0]->optimize(ctrl_pts, ts, cost_function, 1, 1',
                 'bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1',
                 'bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);\n'
                 '  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1'):
        s = add_argument(s, call, 'fixed_return')
    f.write_text(s)
