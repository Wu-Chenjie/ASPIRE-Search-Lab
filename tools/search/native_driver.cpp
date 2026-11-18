// Task-owned harness around the migrated RACER manager, not another planner.
// Stop-and-plan ideal sensor rig only; no actuator commands or flight acceptance.
#include <racer_ros2/runtime.hpp>
#include <racer_ros2/messages.hpp>
#include <exploration_manager/fast_exploration_manager.h>
#include <string>
using std::string;
#include <exploration_manager/expl_data.h>
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cmath>

int main(int argc,char**argv) {
  racer_ros::init(argc,argv,"search_racer"); racer_ros::NodeHandle nh("~");
  auto manager=std::make_shared<fast_planner::FastExplorationManager>();manager->initialize(nh);
  Eigen::Vector3d pos=Eigen::Vector3d::Zero(); double yaw=0.,stamp=-1.;
  auto pub=racer_ros::node->create_publisher<racer_interfaces::msg::Bspline>("/search/trajectory",10);
  auto status=racer_ros::node->create_publisher<std_msgs::msg::String>("/search/planning_status",10);
  auto pathpub=racer_ros::node->create_publisher<nav_msgs::msg::Path>("/search/path",10);
  auto odom=racer_ros::node->create_subscription<nav_msgs::msg::Odometry>("/baseline/odom_0",20,[&](nav_msgs::msg::Odometry::SharedPtr m){
    pos<<m->pose.pose.position.x,m->pose.pose.position.y,m->pose.pose.position.z;
    auto q=m->pose.pose.orientation; yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    stamp=rclcpp::Time(m->header.stamp).seconds();
  });
  auto report=[&](const std::string &s){std_msgs::msg::String m;m.data=s;status->publish(m);};
  bool recover=false;nh.param("search/recover_frontiers",recover,false);
  int id=0;
  auto request=racer_ros::node->create_subscription<geometry_msgs::msg::PoseStamped>("/search/request",10,[&](geometry_msgs::msg::PoseStamped::SharedPtr req){
    const double now=racer_ros::Time::now().toSec();
    if(stamp<0 || now-stamp>.5 || now-stamp<-.01 || !pos.allFinite()) {report("REJECT stale odometry");return;}
    bool explore=req->header.frame_id=="explore";
    Eigen::Vector3d goal(req->pose.position.x,req->pose.position.y,req->pose.position.z);
    const auto q=req->pose.orientation;double gyaw=2*std::atan2(q.z,q.w);
    if(!explore && (req->header.frame_id!="map" || !goal.allFinite() || !std::isfinite(gyaw) ||
      manager->sdf_map_->getOccupancy(goal)!=fast_planner::SDFMap::FREE || manager->sdf_map_->getInflateOccupancy(goal)!=0)) {report("REJECT goal unknown or occupied");return;}
    manager->ed_->swarm_state_[0].pos_=pos;
    manager->ed_->swarm_state_[0].vel_.setZero();
    manager->ed_->swarm_state_[0].yaw_=yaw;
    manager->ed_->swarm_state_[0].stamp_=now;
    Eigen::Vector3d zero=Eigen::Vector3d::Zero(), ys(yaw,0,0);
    int result;
    if(explore) {manager->updateFrontierStruct(pos);result=manager->planExploreMotion(pos,zero,zero,ys);}
    else result=manager->planTrajToView(pos,zero,zero,ys,goal,gyaw);
    auto &t=manager->planner_manager_->local_data_;
    nav_msgs::msg::Path path;path.header.frame_id="map";path.header.stamp=racer_ros::node->now();
    auto audit=[&]() {
      path.poses.clear();
      double duration=t.position_traj_.getTimeSum();
      if(!std::isfinite(duration)||duration<=0||duration>60)return std::string("duration");
      Eigen::Vector3d prev=pos;
      for(double s=0;s<duration+.01;s+=.01) {
        Eigen::Vector3d p=t.position_traj_.evaluateDeBoorT(std::min(s,duration));
        std::string why;
        if(!p.allFinite())why="nonfinite";
        else if(manager->sdf_map_->getOccupancy(p)!=fast_planner::SDFMap::FREE)why="unknown_or_occupied";
        else if(manager->sdf_map_->getInflateOccupancy(p)!=0)why="inflated";
        else if((p-prev).norm()>.05)why="discontinuous";
        if(!why.empty())return why+" at t="+std::to_string(s)+" xyz="+std::to_string(p.x())+","+std::to_string(p.y())+","+std::to_string(p.z());
        geometry_msgs::msg::PoseStamped v;v.header=path.header;v.pose.position.x=p.x();v.pose.position.y=p.y();v.pose.position.z=p.z();v.pose.orientation.w=1.;path.poses.push_back(v);prev=p;
      }
      return std::string();
    };
    std::string failure=result==fast_planner::SUCCEED?audit():"planning failed";
    if(!failure.empty() && explore && recover) {
      // Audit failure is also a failed proposal; try bounded native map frontiers.
      const auto points=manager->ed_->points_;const auto yaws=manager->ed_->yaws_;
      for(size_t i=0;i<std::min(points.size(),size_t(8));++i) {
        const Eigen::Vector3d v=points[i];
        if((v-pos).norm()<.3 || manager->sdf_map_->getOccupancy(v)!=fast_planner::SDFMap::FREE || manager->sdf_map_->getInflateOccupancy(v)!=0)continue;
        report("RETRY "+failure);
        result=manager->planTrajToView(pos,zero,zero,ys,v,yaws[i]);
        failure=result==fast_planner::SUCCEED?audit():"planning failed";
        if(failure.empty())break;
      }
    }
    if(!failure.empty()){report("REJECT "+failure);return;}
    racer_interfaces::msg::Bspline b;b.order=3;b.traj_id=++id;b.start_time=racer_ros::node->now();
    auto pts=t.position_traj_.getControlPoint();for(int i=0;i<pts.rows();++i){geometry_msgs::msg::Point p;p.x=pts(i,0);p.y=pts(i,1);p.z=pts(i,2);b.pos_pts.push_back(p);}
    auto knots=t.position_traj_.getKnot();for(int i=0;i<knots.size();++i)b.knots.push_back(knots[i]);
    auto yp=t.yaw_traj_.getControlPoint();for(int i=0;i<yp.rows();++i)b.yaw_pts.push_back(yp(i,0));b.yaw_dt=t.yaw_traj_.getKnotSpan();
    pathpub->publish(path);pub->publish(b);report("ACCEPT "+std::to_string(id));
  });
  racer_ros::spin();return 0;
}
