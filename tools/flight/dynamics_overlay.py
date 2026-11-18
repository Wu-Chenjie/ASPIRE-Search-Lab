"""Opt-in native RACER boundary constraints and bounded time feasibility pass."""
from .optimizer_overlay import replace_once


def apply(core):
 h=core/'include/bspline_opt/bspline_optimizer.h';s=h.read_text().replace('  bool optimize_time_;','  bool optimize_time_;\n  bool task_hard_rest_ = false;');h.write_text(s)
 p=core/'src/bspline_opt/bspline_optimizer.cpp';s=p.read_text().replace('  nh.param("optimization/ld_smooth",','  nh.param("search/hard_rest_boundary", task_hard_rest_, false);\n  nh.param("optimization/ld_smooth",',1)
 anchor='  auto t1 = racer_ros::Time::now();'
 s=replace_once(s,anchor,'''  // Opt-in stop-and-plan mode: pin cubic endpoint controls before optimization.
  // All interior points still optimize with native distance / feasibility costs.
  if(task_hard_rest_ && !optimize_time_ && bspline_degree_==3 && point_num_>=6 &&
     start_state_.size()>=3 && !end_state_.empty() &&
     start_state_[1].norm()<1e-6 && start_state_[2].norm()<1e-6) {
    vector<double> lb(variable_num_, -HUGE_VAL), ub(variable_num_, HUGE_VAL);
    if(dim_!=1) for(int i=0;i<variable_num_;++i) {
      lb[i]=max(q[i]-10.,bmin[i%3]); ub[i]=min(q[i]+10.,bmax[i%3]);
    }
    bool inside=true;
    for(int j=0;j<dim_;++j) if(dim_!=1 &&
       (start_state_[0][j]<bmin[j] || start_state_[0][j]>bmax[j] ||
        end_state_[0][j]<bmin[j] || end_state_[0][j]>bmax[j]))inside=false;
    if(inside) {
      for(int i=0;i<3;++i) for(int j=0;j<dim_;++j) {
        const int a=dim_*i+j,b=dim_*(point_num_-1-i)+j;
        q[a]=lb[a]=ub[a]=start_state_[0][j];q[b]=lb[b]=ub[b]=end_state_[0][j];
      }
      opt.set_lower_bounds(lb);opt.set_upper_bounds(ub);
      ROS_INFO("TASK_HARD_REST dim=%d",dim_);
    } else ROS_WARN("TASK_HARD_REST skipped out-of-bounds endpoint");
  }

'''+anchor);p.write_text(s)
 h=core/'include/exploration_manager/fast_exploration_fsm.h';s=h.read_text();anchor='  bool task_stop_and_plan_'
 s=replace_once(s,anchor,'''  bool task_feasible_time_ = false;
  double task_v_=1.2,task_a_=1.5,task_j_=4.,task_w_=1.;
'''+anchor);h.write_text(s)
 p=core/'src/exploration_manager/fast_exploration_fsm.cpp';s=p.read_text();anchor='  nh.param("search/stop_and_plan",';s=s.replace(anchor,'''  nh.param("search/feasible_time",task_feasible_time_,false);
  nh.param("search/executor_speed",task_v_,1.2);
  nh.param("search/executor_acceleration",task_a_,1.5);
  nh.param("search/executor_jerk",task_j_,4.);
  nh.param("search/executor_yaw_rate",task_w_,1.);
'''+anchor,1)
 anchor='    auto info = &planner_manager_->local_data_;'
 s=replace_once(s,anchor,anchor+'''
    if(task_feasible_time_) {
      auto &pos=info->position_traj_;auto &yaw=info->yaw_traj_;
      auto v=pos.getDerivative(),a=v.getDerivative(),j=a.getDerivative();
      auto w=yaw.getDerivative(),alpha=w.getDerivative();
      auto bound=[](NonUniformBspline b){return b.getControlPoint().rowwise().norm().maxCoeff();};
      const double pv=bound(v),pa=bound(a),pj=bound(j),pw=bound(w);
      const double factor=std::max({1.,pv/(.95*task_v_),std::sqrt(pa/(.95*task_a_)),
                                  std::cbrt(pj/(.95*task_j_)),pw/(.95*task_w_)});
      const auto knots=pos.getKnot();const double dt=pos.getKnotSpan();
      bool uniform=dt>0.;for(int k=1;k<knots.size();++k)uniform&=fabs(knots[k]-knots[k-1]-dt)<1e-6;
      const bool rest=fd_->static_state_ && v.evaluateDeBoorT(0).norm()<1e-6 &&
        a.evaluateDeBoorT(0).norm()<1e-6 && w.evaluateDeBoorT(0).norm()<1e-6 && alpha.evaluateDeBoorT(0).norm()<1e-6;
      const bool applied=rest && uniform && std::isfinite(factor) && factor<=4. &&
        task_v_>0 && task_a_>0 && task_j_>0 && task_w_>0;
      ROS_WARN("TASK_TIME id=%d applied=%d factor=%.9f pre_v=%.9f pre_a=%.9f pre_j=%.9f pre_w=%.9f rest=%d uniform=%d",
               live_counter_+1,int(applied),factor,pv,pa,pj,pw,int(rest),int(uniform));
      if(applied && factor>1.) {
        // Native generation, before publication/authorization. Same interior
        // spatial curve, common position/yaw time scale, zero boundary jets.
        const Eigen::MatrixXd pp=pos.getControlPoint(),yp=yaw.getControlPoint();
        pos.setUniformBspline(pp,3,dt*factor);
        yaw.setUniformBspline(yp,3,yaw.getKnotSpan()*factor);
        info->velocity_traj_=pos.getDerivative();info->acceleration_traj_=info->velocity_traj_.getDerivative();
        info->yawdot_traj_=yaw.getDerivative();info->yawdotdot_traj_=info->yawdot_traj_.getDerivative();
        info->duration_=pos.getTimeSum();info->start_pos_=pos.evaluateDeBoorT(0.);
      }
    }
''');p.write_text(s)
