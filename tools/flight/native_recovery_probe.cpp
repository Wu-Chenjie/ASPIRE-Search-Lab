// Native integration TEST ONLY: synthetic sensor observations, no actuators.
#include <racer_ros2/runtime.hpp>
#include <exploration_manager/fast_exploration_manager.h>
#include <string>
using std::string;
#include <exploration_manager/expl_data.h>
#include <exploration_manager/task_recovery_policy.hpp>
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace fast_planner;
using racer_task::Point;
static void require(bool value,const char* reason) {if(!value)throw std::runtime_error(reason);}
static Point point(const Eigen::Vector3d& p){return {p.x(),p.y(),p.z()};}
static Eigen::Vector3d eigen(const Point& p){return {p[0],p[1],p[2]};}
static void spin_for(int count) {for(int i=0;i<count;++i) {
  rclcpp::spin_some(racer_ros::node);std::this_thread::sleep_for(std::chrono::milliseconds(20));
}}
static pcl::PointCloud<pcl::PointXYZ> room(double cx,double halfx) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for(double x=-halfx;x<=halfx+.0001;x+=.1)for(double y=-2.4;y<=2.4001;y+=.1) {
    cloud.push_back(pcl::PointXYZ(cx+x,y,.3));cloud.push_back(pcl::PointXYZ(cx+x,y,3.3));
  }
  for(double y=-2.4;y<=2.4001;y+=.1)for(double z=.3;z<=3.3001;z+=.1) {
    cloud.push_back(pcl::PointXYZ(cx-halfx,y,z));cloud.push_back(pcl::PointXYZ(cx+halfx,y,z));
  }
  for(double x=-halfx;x<=halfx+.0001;x+=.1)for(double z=.3;z<=3.3001;z+=.1) {
    cloud.push_back(pcl::PointXYZ(cx+x,-2.4,z));cloud.push_back(pcl::PointXYZ(cx+x,2.4,z));
  }
  return cloud;
}
static void observe(bool disconnected=false,bool gap=false) {
  // Ordinary exact-time cloud+pose subscriptions perform ray integration,
  // obstacle inflation and ESDF. Neither occupancy nor clearance is injected.
  auto cp=racer_ros::node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_ros/cloud",10);
  auto pp=racer_ros::node->create_publisher<geometry_msgs::msg::PoseStamped>("/map_ros/pose",10);
  spin_for(15);
  for(int box=0;box<(disconnected?2:1);++box) {
    const double cx=box?5.:0.;auto cloud=room(cx,box?1.4:2.4);
    if(gap) {
      pcl::PointCloud<pcl::PointXYZ> subset;
      for(const auto& p:cloud)if(!(p.x>0 && std::abs(p.y)<1.5 && p.z>.8 && p.z<2.8))subset.push_back(p);
      cloud=std::move(subset); // Same room/sensor poses; withhold only the forward observation sector.
    }
    for(int n=0;n<8;++n) {
      sensor_msgs::msg::PointCloud2 msg;pcl::toROSMsg(cloud,msg);
      msg.header.frame_id="world";msg.header.stamp=racer_ros::node->now();
      geometry_msgs::msg::PoseStamped pose;pose.header=msg.header;
      // A sensor's own voxel remains unknown; a second observation sees it.
      pose.pose.position.x=cx+(n%2==0?-.3:.3);pose.pose.position.z=1.8;pose.pose.orientation.w=1.;
      cp->publish(msg);pp->publish(pose);spin_for(5);
    }
  }
  spin_for(10);
}
static bool measured(const SDFMap::Ptr& map,const Point& p) {
  auto v=eigen(p);return v.allFinite() && map->getOccupancy(v)==SDFMap::FREE && map->getInflateOccupancy(v)==0;
}
static void check_curve(const FastExplorationManager::Ptr& manager) {
  auto& d=manager->planner_manager_->local_data_;
  require(std::isfinite(d.duration_) && d.duration_>0 && d.duration_<60,"Invalid native duration");
  for(double t=0;t<=d.duration_+.01;t+=.01) {
    auto p=d.position_traj_.evaluateDeBoorT(std::min(t,d.duration_));
    require(measured(manager->sdf_map_,point(p)),"Native curve crossed unknown/occupied/inflated map");
    require(d.yaw_traj_.evaluateDeBoorT(std::min(t,d.duration_)).allFinite(),"Invalid yaw curve");
  }
}
static void export_evidence(const FastExplorationManager::Ptr& manager,const string& prefix) {
  auto& d=manager->planner_manager_->local_data_;std::ofstream f(prefix+"-curve.json");f<<std::setprecision(17);
  auto pos=d.position_traj_.getControlPoint();auto knots=d.position_traj_.getKnot();auto yaw=d.yaw_traj_.getControlPoint();
  f<<"{\"order\":3,\"points\":[";
  for(int i=0;i<pos.rows();++i){if(i)f<<",";f<<"["<<pos(i,0)<<","<<pos(i,1)<<","<<pos(i,2)<<"]";}
  f<<"],\"knots\":[";for(int i=0;i<knots.size();++i){if(i)f<<",";f<<knots[i];}
  f<<"],\"yaw_points\":[";for(int i=0;i<yaw.rows();++i){if(i)f<<",";f<<yaw(i,0);}
  f<<"],\"yaw_dt\":"<<d.yaw_traj_.getKnotSpan()<<"}\n";
  auto map=manager->sdf_map_;Eigen::Vector3d origin,size;map->getRegion(origin,size);const double r=map->getResolution();
  Eigen::Vector3i shape=(size/r).array().ceil().cast<int>();std::ofstream g(prefix+"-map.json");g<<std::setprecision(17);
  g<<"{\"origin\":["<<origin.x()<<","<<origin.y()<<","<<origin.z()<<"],\"size\":["<<size.x()<<","<<size.y()<<","<<size.z()<<"],\"resolution\":"<<r<<",\"shape\":["<<shape.x()<<","<<shape.y()<<","<<shape.z()<<"],\"state\":[";
  bool first=true;for(int x=0;x<shape.x();++x)for(int y=0;y<shape.y();++y)for(int z=0;z<shape.z();++z) {
    if(!first)g<<",";first=false;int o=map->getOccupancy(Eigen::Vector3i(x,y,z));g<<(o==SDFMap::FREE?0:o==SDFMap::OCCUPIED?1:-1);
  }g<<"]}\n";
}
static void optimizer_inputs(racer_ros::NodeHandle& nh,const FastExplorationManager::Ptr& manager) {
  BsplineOptimizer opt;opt.setParam(nh);opt.setEnvironment(manager->planner_manager_->edt_environment_);
  const int costs=BsplineOptimizer::SMOOTHNESS|BsplineOptimizer::START|BsplineOptimizer::END;
  auto boundary=[&](){opt.setBoundaryStates({Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()},{Eigen::Vector3d::Zero()});};
  int rejected=0;
  for(int mode=0;mode<9;++mode) {
    boundary();Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(15,1);double dt=.5;int budget=1;
    if(mode==0)controls.resize(15,2);
    if(mode==1)controls.resize(3,1);
    if(mode==2)dt=0;
    if(mode==3)dt=NAN;
    if(mode==4)controls(0,0)=NAN;
    if(mode==5)budget=4;
    if(mode==6)opt.setBoundaryStates({},{});
    if(mode==7)opt.setBoundaryStates({Eigen::Vector3d(NAN,0,0),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()},{Eigen::Vector3d::Zero()});
    if(mode==8)opt.setBoundaryStates({Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()},{});
    bool caught=false;try {opt.optimize(controls,dt,costs,budget,1);}catch(const std::exception&){caught=true;++rejected;}
    require(caught,"Malformed optimizer input was accepted");
  }
  // A successful solve followed by NLopt setup failure must not reuse best_variable_.
  boundary();Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(15,1);double dt=.5;opt.optimize(controls,dt,costs,1,1);
  require(controls.allFinite() && controls.norm()<1e-9,"Constant yaw solve failed");
  racer_ros::node->set_parameter(rclcpp::Parameter("optimization.algorithm2",-1));opt.setParam(nh);boundary();
  bool caught=false;try{opt.optimize(controls,dt,costs,1,1);}catch(const std::exception&){caught=true;}
  require(caught,"Invalid NLopt algorithm reused stale solution");
  racer_ros::node->set_parameter(rclcpp::Parameter("optimization.algorithm2",11));opt.setParam(nh);boundary();opt.optimize(controls,dt,costs,1,1);
  require(controls.allFinite(),"Optimizer did not recover after rejected call");
  racer_ros::node->set_parameter(rclcpp::Parameter("optimization.ld_smooth",NAN));opt.setParam(nh);boundary();
  caught=false;try{opt.optimize(controls,dt,costs,1,1);}catch(const std::exception&){caught=true;}
  require(caught,"Nonfinite objective was accepted");
  racer_ros::node->set_parameter(rclcpp::Parameter("optimization.ld_smooth",5.));opt.setParam(nh);boundary();
  opt.setWaypoints({Eigen::Vector3d::Zero()},{99});
  caught=false;try{opt.optimize(controls,dt,costs|BsplineOptimizer::WAYPOINTS,1,1);}catch(const std::exception&){caught=true;}
  require(caught,"Out-of-range waypoint index was accepted");
  opt.setWaypoints({},{});boundary();opt.optimize(controls,dt,costs,1,1);
  std::cout<<"PASS_INPUTS rejected="<<rejected<<" stale_solution_rejected=1 nonfinite_objective_rejected=1 waypoint_index_rejected=1 constant_yaw=1\n";
}
int main(int argc,char** argv) {
  string mode="recover",output="/tmp/native-recovery";
  for(int i=1;i+1<argc;++i){if(string(argv[i])=="--case")mode=argv[++i];else if(string(argv[i])=="--output")output=argv[++i];}
  racer_ros::init(argc,argv,"task_recovery_probe");racer_ros::NodeHandle nh("~");
  try {
    auto manager=std::make_shared<FastExplorationManager>();manager->initialize(nh);
    observe(mode=="blocked" || mode=="recover",mode=="gap");auto map=manager->sdf_map_;
    double requested_yaw=0.;nh.param("task_probe/end_yaw",requested_yaw,0.);
    const Eigen::Vector3d root(0,0,1.8),zero=Eigen::Vector3d::Zero(),unreachable(5.,0,1.8),axis(1.2,0,1.8);
    require(measured(map,point(root)),"Planning root not observed free");
    if(mode=="gap") {
      require(map->getOccupancy(axis)==SDFMap::UNKNOWN,"Withheld sector unexpectedly observed");
      require(manager->planTrajToView(root,zero,zero,zero,axis,requested_yaw)==FAIL,"Unknown sector accepted");
      std::cout<<"PASS_GAP same_target=1 withheld_observations=1 native_FAIL=1\n";rclcpp::shutdown();return 0;
    }
    if(mode=="inputs") {optimizer_inputs(nh,manager);rclcpp::shutdown();return 0;}
    if(mode=="registry") {
      require(bool(manager->task_target_registry_),"registry manager not initialized");
      manager->task_bearing_policy_.enabled=true;
      const double now=racer_ros::Time::now().toSec();
      racer_task::TargetObservation o;o.key={"uav0","native-fixture",1};o.sequence=1;o.first=now-1.;o.capture=now;
      o.frame_version="native-local-v1";o.visual="person_candidate";o.has_bearing=true;
      o.origin={root.x(),root.y(),root.z()};o.direction={1,0,0};o.half_angle=.2;o.hfov=.9;
      o.confidence=.95;o.support=5;o.support_first=now-1.;
      auto pub=racer_ros::node->create_publisher<std_msgs::msg::String>("/uav0/search/target_observations",10);spin_for(10);
      auto send=[&](){std_msgs::msg::String m;m.data="{\"schema\":1,\"kind\":\"observations\",\"observations\":["+
        racer_task::targetObservationJSON(o,0,"local_observation")+"]}";pub->publish(m);spin_for(8);};
      send();require(manager->task_target_registry_->registry.records.size()==1,"ROS observation not ingested by native manager");
      auto&memory=manager->task_bearing_policy_.discovery;require(memory.cues.size()==1,"native persistent cue missing");
      const double epoch=memory.groups.begin()->second.epoch;
      Eigen::Vector3d goal=root+Eigen::Vector3d(0,1.2,0);double gy=3.141592653589793;
      const Eigen::Vector3d baseline_goal=goal;const double baseline_yaw=gy;
      const int baseline_attempts=memory.mission_attempts;
      manager->task_bearing_policy_.enabled=false;
      require(!manager->taskBearingRerank(root,zero,zero,goal,gy) && goal.isApprox(baseline_goal) && gy==baseline_yaw,
          "disabled registry with identical accepted evidence changed native goal");
      require(memory.mission_attempts==baseline_attempts,"disabled registry consumed guidance attempts");
      manager->task_bearing_policy_.enabled=true;
      require(manager->taskBearingRerank(root,zero,zero,goal,gy),"persistent evidence did not change real native selection");
      require(measured(map,point(goal)),"persistent goal unknown or inflated");
      require(manager->planTrajToView(root,zero,zero,zero,goal,gy)==SUCCEED,"persistent native trajectory failed");check_curve(manager);export_evidence(manager,output);
      manager->taskBearingPlan(501,SUCCEED);manager->taskGuidanceFeedback(501,true,-1,false);
      require(memory.groups.begin()->second.blocked,"completed observation did not block no-progress retry");
      o.key.counter=2;o.first=o.capture=racer_ros::Time::now().toSec();o.support_first=o.capture-1.;send();
      require(memory.cues.size()==2&&memory.groups.size()==1,"ID churn lost shared failure region");
      require(memory.groups.begin()->second.epoch==epoch,"ID churn reset discovery epoch");
      goal=root+Eigen::Vector3d(0,1.2,0);gy=3.141592653589793;
      require(!manager->taskBearingRerank(root,zero,zero,goal,gy),"failed group did not fall back to original");
      // Vision ends the bearing search and requests registered depth at the observation stop.
      o.sequence++;o.capture=racer_ros::Time::now().toSec();o.visual="visually_confirmed";send();
      require(manager->task_target_registry_->registry.groupConfirmed(o.key),"confirmation record missing");
      require(manager->task_target_registry_->registry.records.size()==2,"confirmation erased passive identity");
      const auto id=manager->task_target_registry_->cueId(o.key);
      require(!memory.cues.at(id).enabled,"confirmed entity still actively guided");
      require(manager->task_depth_review_pending_,"RGB confirmation without depth released exploration");
      o.sequence++;o.capture=racer_ros::Time::now().toSec();o.has_position=true;
      o.localization="reliable";o.position={4.,0.,1.};send();
      require(!manager->task_depth_review_pending_,"registered position did not release exploration");
      std::cout<<"PASS_NATIVE_REGISTRY depth_review_released=1 confirmed_guidance_retired=1 disabled_with_same_evidence=1 actual_ROS_ingest=1 persistent_cost_selection=1 known_free_curve=1 no_progress_blocks=1 ID_churn_retains_epoch=1 ordinary_fallback=1 no_flight=1\n";rclcpp::shutdown();return 0;
    }
    if(mode=="bearing") {
      // Synthetic observed room, real native map integration, A* and trajectory.
      // No Gazebo, detector, actuator or hidden occupancy mutation.
      auto original=Eigen::Vector3d(1.2,0,1.8);double original_yaw=M_PI;
      manager->ed_->points_={original,Eigen::Vector3d(0,.9,1.8)};
      manager->ed_->yaws_={M_PI,0.};
      auto goal=original;double gy=original_yaw;
      manager->task_bearing_policy_.enabled=false;
      require(!manager->taskBearingRerank(root,zero,zero,goal,gy) && goal.isApprox(original) && gy==original_yaw,"disabled changed native goal");
      manager->task_bearing_policy_.enabled=true;const double now=racer_ros::Time::now().toSec();
      std::vector<double> packet{1,0,1,now,0,0,1.8,1,0,0,.2,.9,3,0,0,now-1.,.637};
      require(manager->task_bearing_policy_.receive(packet,now),"fixture cue rejected");
      manager->task_bearing_policy_.enabled=false;
      require(!manager->taskBearingRerank(root,zero,zero,goal,gy) && goal.isApprox(original) && gy==original_yaw,
          "disabled bearing with identical accepted evidence changed native goal");
      manager->task_bearing_policy_.enabled=true;
      require(manager->taskBearingRerank(root,zero,zero,goal,gy),"bearing did not change real native candidate selection");
      require(measured(map,point(goal)),"chosen bearing observation was not native known free");
      require(std::abs(goal.z()-root.z())<1e-9 && (goal-root).norm()<=.9001,"unbounded observation fixture goal");
      require(manager->planTrajToView(root,zero,zero,zero,goal,gy)==SUCCEED,"native bearing observation trajectory failed");
      check_curve(manager);export_evidence(manager,output);
      goal=original;gy=original_yaw;
      require(!manager->taskBearingRerank(root,zero,zero,goal,gy) && goal.isApprox(original),"forced original exploration not preserved");
      manager->task_bearing_policy_=racer_task::BearingBias();manager->task_bearing_policy_.enabled=true;
      packet[3]=now-21.;packet[15]=now-22.;require(!manager->task_bearing_policy_.receive(packet,now),"expired packet accepted");
      require(!manager->taskBearingRerank(root,zero,zero,goal,gy) && goal.isApprox(original),"expired evidence changed original goal");
      std::cout<<"PASS_NATIVE_BEARING disabled_with_same_evidence=1 measured_free=1 native_astar=1 native_trajectory=1 changed=1 disabled_equal=1 expired_equal=1 forced_original=1 no_flight=1\n";
      rclcpp::shutdown();return 0;
    }

    if(mode=="moving-boundary") {
      require(manager->planTrajToView(root,zero,zero,zero,axis,requested_yaw)==SUCCEED,"Initial native curve failed");
      check_curve(manager);export_evidence(manager,output+"-old");
      auto old=manager->planner_manager_->local_data_;
      const double boundary=.35*old.duration_;
      const auto p=old.position_traj_.evaluateDeBoorT(boundary);
      const auto v=old.velocity_traj_.evaluateDeBoorT(boundary);
      const auto a=old.acceleration_traj_.evaluateDeBoorT(boundary);
      Eigen::Vector3d yaw(old.yaw_traj_.evaluateDeBoorT(boundary)[0],old.yawdot_traj_.evaluateDeBoorT(boundary)[0],old.yawdotdot_traj_.evaluateDeBoorT(boundary)[0]);
      const auto begin=std::chrono::steady_clock::now();
      const int result=manager->planTrajToView(p,v,a,yaw,Eigen::Vector3d(1.2,.6,1.8),.3);
      const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
      std::ofstream meta(output+"-moving.json");meta<<std::setprecision(17)<<"{\"boundary\":"<<boundary<<",\"planner_wall_s\":"<<wall<<",\"result\":"<<result<<",\"succeeded\":"<<(result==SUCCEED?"true":"false")<<"}\n";meta.close();
      if(result==SUCCEED){check_curve(manager);export_evidence(manager,output+"-new");}
      std::cout<<"MOVING_NATIVE_BOUNDARY_EXPORTED result="<<result<<" wall="<<wall<<" no_actuators=1\n";
      rclcpp::shutdown();return 0;
    }

    if(mode=="axis" || mode=="rollback" || mode=="yaw-pin") {
      require(manager->planTrajToView(root,zero,zero,zero,axis,requested_yaw)==SUCCEED,"Axis trajectory failed");
      if(mode=="yaw-pin") {
        const auto fixed_position=manager->planner_manager_->local_data_.position_traj_.getControlPoint();
        racer_ros::node->set_parameter(rclcpp::Parameter("search.hard_rest_boundary",true));
        auto yaw_manager=std::make_shared<FastExplorationManager>();yaw_manager->initialize(nh);
        yaw_manager->planner_manager_->local_data_=manager->planner_manager_->local_data_;
        yaw_manager->planner_manager_->planYawExplore(zero,requested_yaw,true,1.);
        manager->planner_manager_->local_data_=yaw_manager->planner_manager_->local_data_;
        require(manager->planner_manager_->local_data_.position_traj_.getControlPoint().isApprox(fixed_position,0.),"Yaw-only comparison changed position");
        std::cout<<"PASS_YAW_ONLY position_unchanged=1 yaw_boundary_pinned=1\n";
      }
      check_curve(manager);export_evidence(manager,output);
      if(mode=="rollback") {
        auto saved=manager->planner_manager_->local_data_;
        racer_ros::node->set_parameter(rclcpp::Parameter("optimization.ld_waypt",NAN));
        auto broken=std::make_shared<FastExplorationManager>();broken->initialize(nh);observe();
        broken->planner_manager_->local_data_=saved;
        require(broken->planTrajToView(root,zero,zero,zero,axis,0.)==FAIL,"Optimizer exception falsely returned success");
        auto& d=broken->planner_manager_->local_data_;
        require(d.traj_id_==saved.traj_id_ && d.duration_==saved.duration_ &&
          d.position_traj_.getControlPoint().isApprox(saved.position_traj_.getControlPoint(),0.) &&
          d.yaw_traj_.getControlPoint().isApprox(saved.yaw_traj_.getControlPoint(),0.),"Failed proposal replaced prior valid curve");
        std::cout<<"PASS_ROLLBACK optimizer_failure=1 returned_FAIL=1 prior_curve_preserved=1\n";
      } else std::cout<<"PASS_AXIS constant_yaw=1 native_curve=1\n";
      rclcpp::shutdown();return 0;
    }
    if(mode=="invalid") {
      require(manager->planTrajToView(root,zero,zero,zero,axis,0.)==SUCCEED,"Prerequisite trajectory failed");
      const int id=manager->planner_manager_->local_data_.traj_id_;
      for(const auto& goal:std::vector<Eigen::Vector3d>{root,{NAN,0,1.8},{100,0,1.8},{INFINITY,0,1.8}})
        require(manager->planTrajToView(root,zero,zero,zero,goal,0.)==FAIL,"Invalid goal accepted");
      require(manager->planner_manager_->local_data_.traj_id_==id,"Invalid goal changed proposal");
      std::cout<<"PASS_INVALID goals=4 prior_curve_preserved=1\n";rclcpp::shutdown();return 0;
    }
    require(measured(map,point(unreachable)),"Initial disconnected goal not observed free");
    racer_task::RecoveryPolicy policy;
    for(int n=0;n<3;++n) {
      require(manager->planTrajToView(root,zero,zero,zero,unreachable,0.)==FAIL,"Disconnected initial goal unexpectedly reachable");
      policy.failed(point(unreachable),double(n));
    }
    // Controlled candidate pool from OBSERVED voxels, not room/wall geometry.
    // The production FSM supplies native frontiers/authorized breadcrumbs to
    // exactly the same selection kernel; this test does not simulate that FSM.
    std::vector<Point> candidates;std::vector<double> yaws;
    Eigen::Vector3d origin,size;map->getRegion(origin,size);const double r=map->getResolution();
    const Eigen::Vector3i shape=(size/r).array().ceil().cast<int>();
    for(int x=0;x<shape.x();++x)for(int y=0;y<shape.y();++y)for(int z=0;z<shape.z();++z) {
      Eigen::Vector3d p=origin+(Eigen::Vector3d(x,y,z)+Eigen::Vector3d::Constant(.5))*r;
      bool region=mode=="blocked"?p.x()>3.:p.x()>.8 && p.x()<1.4;
      if(region && std::abs(p.y())<1.5 && p.z()>1.4 && p.z()<2.2 && measured(map,point(p))) {
        candidates.push_back(point(p));yaws.push_back(0.);
      }
    }
    require(!candidates.empty(),"No measured candidate pool");
    int attempts=0,successes=0;
    double last_attempt=-1.;
    for(int i=0;i<5000;++i) {
      const double now=3.+i*.02;if(!policy.ready(now))continue;
      const int index=racer_task::selectGoal(policy,candidates,yaws,point(root),now,[&](const Point& p){return measured(map,p);});
      if(index<0){policy.hold(now);continue;}
      require(attempts<8 || now-last_attempt>=5.-1e-9,"Native recovery bypassed the slower retry interval");
      last_attempt=now;
      const auto goal=eigen(candidates[index]);policy.attempted(candidates[index],now);++attempts;
      int result=manager->planTrajToView(root,zero,zero,zero,goal,yaws[index]);
      std::cout<<"RECOVERY_ATTEMPT "<<attempts<<" goal="<<goal.transpose()<<" result="<<result<<"\n";
      if(result==SUCCEED){check_curve(manager);export_evidence(manager,output);++successes;break;}
      policy.failed(candidates[index],now);
    }
    require(policy.active(),"Proposal falsely counted as activation");
    if(mode=="blocked") {
      require(attempts>8 && attempts<=28 && successes==0 && manager->planner_manager_->local_data_.traj_id_==0,
              "Unreachable pool did not retry at the expected rate without a trajectory");
      std::cout<<"PASS_BLOCKED initial_failures=3 attempts="<<attempts<<" callbacks=5000 curves=0 retry_backoff=1\n";
    } else {
      require(successes==1 && attempts<=8,"No safe alternative native curve");
      std::cout<<"PASS_NATIVE_RECOVERY initial_failures=3 alternate_native_curve=1 actual_activation=0 no_flight=1\n";
    }
    rclcpp::shutdown();return 0;
  } catch(const std::exception& e) {
    std::cerr<<"PROBE_FAIL "<<e.what()<<"\n";rclcpp::shutdown();return 1;
  }
}
