"""Fail-closed native optimization and constant-signal normalization; build copy only."""

def replace_once(text, old, new):
    if text.count(old)!=1: raise ValueError('Overlay anchor mismatch: '+old[:100])
    return text.replace(old,new,1)

def apply(core):
    p=core/'src/bspline_opt/bspline_optimizer.cpp';s=p.read_text()
    s=replace_once(s,'#include <nlopt.hpp>','#include <nlopt.hpp>\n#include <stdexcept>\n#include <algorithm>\n#include <cmath>')
    begin=s.index('  if (start_state_.empty()) {',s.index('void BsplineOptimizer::optimize(Eigen::MatrixXd&'))
    end=s.index('  control_points_ = points;',begin)
    s=s[:begin]+'''  // Reject malformed inputs before any indexing or NLopt construction.
  best_variable_.clear();
  if((points.cols()!=1 && points.cols()!=3) || points.rows()<4 ||
     !points.allFinite() || !std::isfinite(dt) || dt<=1e-6 ||
     max_num_id<0 || max_num_id>=4 || max_time_id<0 || max_time_id>=4 ||
     bspline_degree_!=3 || start_state_.size()<3 || end_state_.empty())
    throw std::runtime_error("TASK_OPT invalid dimensions, interval or boundary states");
  for(const auto& v:start_state_)if(!v.allFinite())throw std::runtime_error("TASK_OPT nonfinite start");
  for(const auto& v:end_state_)if(!v.allFinite())throw std::runtime_error("TASK_OPT nonfinite end");
  if(cost_function & WAYPOINTS) {
    if(waypoints_.size()!=waypt_idx_.size())throw std::runtime_error("TASK_OPT waypoint sizes");
    for(size_t i=0;i<waypoints_.size();++i)
      if(!waypoints_[i].allFinite() || waypt_idx_[i]<0 || waypt_idx_[i]+2>=points.rows())
        throw std::runtime_error("TASK_OPT waypoint index or value");
  }
  if((cost_function & GUIDE) && guide_pts_.size()<size_t(std::max(0,int(points.rows())-6)))
    throw std::runtime_error("TASK_OPT guide size");
  if(max_iteration_num_[max_num_id]<=0 || !std::isfinite(max_iteration_time_[max_time_id]) ||
     max_iteration_time_[max_time_id]<=0)
    throw std::runtime_error("TASK_OPT invalid optimization budget");
'''+s[end:]
    s=replace_once(s,'  pt_dist_ /= double(point_num_);','''  pt_dist_ /= double(point_num_);
  if(!std::isfinite(pt_dist_))throw std::runtime_error("TASK_OPT nonfinite spacing");
  // Constant yaw is a valid signal. Native normalized jerk otherwise does
  // 0/0 here. Use unnormalized jerk only in the degenerate-spacing case.
  if(pt_dist_<=1e-9) {
    pt_dist_=1.;
    ROS_INFO("TASK_OPT constant_signal_normalization dim=%d",dim_);
  }''')
    s=replace_once(s,'  optimize();','''  try { optimize(); }
  catch(...) { start_state_.clear();time_lb_=-1;best_variable_.clear();throw; }''')
    s=replace_once(s,'  edt_environment_->sdf_map_->getBox(bmin, bmax);','''  edt_environment_->sdf_map_->getBox(bmin, bmax);
  if(!bmin.allFinite() || !bmax.allFinite() || (bmax-bmin).minCoeff()<=.002)
    throw std::runtime_error("TASK_OPT invalid map bounds");''')
    s=replace_once(s,'      lb[variable_num_ - 1] = 0.0;','      lb[variable_num_ - 1] = 1e-6; // dt=0 makes derivative costs undefined.')
    old='''  try {
    double final_cost;
    nlopt::result result = opt.optimize(q, final_cost);
  } catch (std::exception& e) {
    cout << e.what() << endl;
  }'''
    s=replace_once(s,old,'''  double final_cost=std::numeric_limits<double>::quiet_NaN();
  const nlopt::result result=opt.optimize(q,final_cost);
  // Negative NLopt results throw. Even a positive budget-limited return must
  // have a finite evaluated solution from THIS invocation, never a stale one.
  if(int(result)<=0 || !std::isfinite(final_cost) || !std::isfinite(min_cost_) ||
     best_variable_.size()!=size_t(variable_num_) ||
     !std::all_of(best_variable_.begin(),best_variable_.end(),[](double v){return std::isfinite(v);}) ||
     (optimize_time_ && best_variable_.back()<=1e-6))
    throw std::runtime_error("TASK_OPT no valid evaluated solution");''')
    s=replace_once(s,'  opt->combineCost(x, grad, cost);','''  if(x.size()!=size_t(opt->variable_num_) ||
     !std::all_of(x.begin(),x.end(),[](double v){return std::isfinite(v);}) ||
     (opt->optimize_time_ && x.back()<=1e-6))
    throw std::runtime_error("TASK_OPT invalid solver iterate");
  opt->combineCost(x, grad, cost);
  if(!std::isfinite(cost) || grad.size()!=x.size() ||
     !std::all_of(grad.begin(),grad.end(),[](double v){return std::isfinite(v);}))
    throw std::runtime_error("TASK_OPT nonfinite objective or gradient");''')
    p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_manager.cpp';s=p.read_text()
    start=s.index('int FastExplorationManager::planTrajToView(');end=s.index('\nint FastExplorationManager::updateFrontierStruct',start)
    f=s[start:end]
    f=replace_once(f,'  // Plan trajectory (position and yaw) to the next viewpoint','''  if(!pos.allFinite() || !vel.allFinite() || !acc.allFinite() || !yaw.allFinite() ||
     !next_pos.allFinite() || !std::isfinite(next_yaw) ||
     !sdf_map_->isInMap(pos) || !sdf_map_->isInMap(next_pos) ||
     (pos-next_pos).norm()<1e-4 || std::abs(yaw[0])>2*M_PI || std::abs(next_yaw)>2*M_PI) {
    ROS_WARN("TASK_NATIVE_FAIL invalid viewpoint input");return FAIL;
  }
  if(sdf_map_->getOccupancy(pos)!=SDFMap::FREE || sdf_map_->getInflateOccupancy(pos)!=0 ||
     sdf_map_->getOccupancy(next_pos)!=SDFMap::FREE || sdf_map_->getInflateOccupancy(next_pos)!=0) {
    ROS_WARN("TASK_NATIVE_FAIL viewpoint outside measured free space");return FAIL;
  }
  // An exception in yaw optimization must not leave the newly generated
  // position curve eligible for publication. Restore the last valid proposal.
  struct ProposalTransaction {
    LocalTrajData& data;std::unique_ptr<LocalTrajData> previous;bool committed=false;
    explicit ProposalTransaction(LocalTrajData& d):data(d) {
      if(d.traj_id_>0)previous=std::make_unique<LocalTrajData>(d);
    }
    ~ProposalTransaction() {
      if(!committed) {if(previous)data=*previous;else {data.traj_id_=0;data.duration_=0.;}}
    }
  } transaction(planner_manager_->local_data_);
  try {
  // Plan trajectory (position and yaw) to the next viewpoint''')
    f=replace_once(f,'  return SUCCEED;\n}', '''  auto& data=planner_manager_->local_data_;
  if(!std::isfinite(data.duration_) || data.duration_<=0 ||
     !data.position_traj_.getControlPoint().allFinite() || !data.yaw_traj_.getControlPoint().allFinite())
    throw std::runtime_error("invalid native output");
  transaction.committed=true;
  return SUCCEED;
  } catch(const std::exception& e) {
    ROS_ERROR("TASK_NATIVE_FAIL optimizer: %s",e.what());return FAIL;
  }
}''')
    s=s[:start]+f+s[end:]
    s='#include <stdexcept>\n'+s;p.write_text(s)
