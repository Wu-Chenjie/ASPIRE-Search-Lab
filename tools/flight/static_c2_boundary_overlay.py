"""Opt-in native boundary constraints for fixed-knot moving C2 proposals."""
from .optimizer_overlay import replace_once


def apply(core):
    p=core/'include/bspline_opt/bspline_optimizer.h';s=p.read_text()
    s=replace_once(s,'  bool task_hard_rest_ = false;',
        '  bool task_hard_rest_ = false;\n  bool task_moving_c2_boundary_ = false;')
    p.write_text(s)
    p=core/'src/bspline_opt/bspline_optimizer.cpp';s=p.read_text()
    s=replace_once(s,'  nh.param("search/hard_rest_boundary", task_hard_rest_, false);',
        '  nh.param("search/hard_rest_boundary", task_hard_rest_, false);\n  nh.param("search/static_moving_c2", task_moving_c2_boundary_, false);')
    anchor='  auto t1 = racer_ros::Time::now();'
    s=replace_once(s,anchor,'''  if(task_moving_c2_boundary_) {
    // Soft START/END costs are not a continuity guarantee. These controls
    // encode the requested p/v/a jet exactly at the fixed cubic knot span.
    // Interior controls retain all native distance/feasibility/swarm costs.
    if(optimize_time_ || bspline_degree_!=3 || point_num_<6 ||
       start_state_.size()<3 || end_state_.empty() ||
       !std::isfinite(knot_span_) || knot_span_<=1e-6)
      throw std::runtime_error("Moving C2 requires fixed cubic knot span and valid boundary states");
    vector<double> lb(variable_num_,-HUGE_VAL),ub(variable_num_,HUGE_VAL);
    if(dim_!=1) for(int i=0;i<variable_num_;++i) {
      lb[i]=max(q[i]-10.,bmin[i%3]);ub[i]=min(q[i]+10.,bmax[i%3]);
    }
    const double dt=knot_span_;
    for(int j=0;j<dim_;++j) {
      const double p=start_state_[0][j],v=start_state_[1][j],a=start_state_[2][j];
      const double ev=end_state_.size()>1?end_state_[1][j]:0.;
      const double ea=end_state_.size()>2?end_state_[2][j]:0.;
      const double ep=end_state_[0][j];
      const double first[3]={p-dt*v+dt*dt*a/3.,p-dt*dt*a/6.,p+dt*v+dt*dt*a/3.};
      const double last[3]={ep-dt*ev+dt*dt*ea/3.,ep-dt*dt*ea/6.,ep+dt*ev+dt*dt*ea/3.};
      for(int k=0;k<3;++k) {
        if(!std::isfinite(first[k]) || !std::isfinite(last[k]) ||
           (dim_!=1 && (first[k]<bmin[j] || first[k]>bmax[j] || last[k]<bmin[j] || last[k]>bmax[j])))
          throw std::runtime_error("Moving C2 boundary controls outside native map bounds");
        const int i=dim_*k+j,e=dim_*(point_num_-3+k)+j;
        q[i]=lb[i]=ub[i]=first[k];q[e]=lb[e]=ub[e]=last[k];
      }
    }
    opt.set_lower_bounds(lb);opt.set_upper_bounds(ub);
    ROS_INFO("TASK_MOVING_C2_BOUNDARY dim=%d",dim_);
  }

'''+anchor)
    p.write_text(s)
    p=core/'include/plan_manage/planner_manager.h';s=p.read_text()
    s=replace_once(s,'  /* main planning algorithms & modules */',
        '  bool task_moving_c2_boundary_ = false;\n  /* main planning algorithms & modules */')
    p.write_text(s)
    p=core/'src/plan_manage/planner_manager.cpp';s=p.read_text()
    anchor='void FastPlannerManager::initPlanModules(racer_ros::NodeHandle& nh) {'
    s=replace_once(s,anchor,anchor+'\n  nh.param("search/static_moving_c2",task_moving_c2_boundary_,false);')
    anchor='  init.getBoundaryStates(2, 0, start, end);'
    s=replace_once(s,anchor,anchor+'''
  // Least-squares B-spline parameterization may move the requested initial
  // jet. In moving C2 mode use the actual future handoff jet, not that fit.
  if(task_moving_c2_boundary_) {
    start={start_pt,start_vel,start_acc};
  }''')
    p.write_text(s)
