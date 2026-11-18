// Diagnostic-only native map/frontier snapshot. No trajectory or actuator output.
#include <racer_ros2/runtime.hpp>
#include <exploration_manager/fast_exploration_manager.h>
#include <active_perception/frontier_finder.h>
#include <plan_env/sdf_map.h>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <iomanip>

int main(int argc,char**argv) {
  racer_ros::init(argc,argv,"hm3d_map_probe");racer_ros::NodeHandle nh("~");
  auto manager=std::make_shared<fast_planner::FastExplorationManager>();manager->initialize(nh);
  std::string output;nh.param("diagnostic/output",output,std::string(""));
  if(output.empty())return 2;
  auto done=racer_ros::node->create_publisher<std_msgs::msg::String>("/diagnostic/done",10);
  auto request=racer_ros::node->create_subscription<std_msgs::msg::String>("/diagnostic/snapshot",10,[&](std_msgs::msg::String::SharedPtr request){
    auto map=manager->sdf_map_;auto f=manager->frontier_finder_;
    f->searchFrontiers();f->computeFrontiersToVisit();
    if(request->data=="update"){std_msgs::msg::String m;m.data="updated";done->publish(m);return;}
    Eigen::Vector3d origin,size;map->getRegion(origin,size);double res=map->getResolution();
    Eigen::Vector3i shape=(size/res).array().ceil().cast<int>();
    std::ofstream stream(output+"/map.csv");stream<<std::setprecision(12)<<"x,y,z,occupancy,inflated,esdf,in_box\n";
    int known=0,occupied=0,inflated=0,free_safe=0;
    for(int x=0;x<shape.x();++x)for(int y=0;y<shape.y();++y)for(int z=0;z<shape.z();++z){
      Eigen::Vector3i idx(x,y,z);Eigen::Vector3d p;map->indexToPos(idx,p);
      int occ=map->getOccupancy(idx),inf=map->getInflateOccupancy(idx);known+=occ==fast_planner::SDFMap::FREE||occ==fast_planner::SDFMap::OCCUPIED;occupied+=occ==fast_planner::SDFMap::OCCUPIED;inflated+=inf==1;free_safe+=occ==fast_planner::SDFMap::FREE&&inf==0;
      stream<<p.x()<<","<<p.y()<<","<<p.z()<<","<<occ<<","<<inf<<","<<map->getDistance(idx)<<","<<map->isInBox(idx)<<"\n";
    }
    stream.close();
    std::vector<std::vector<Eigen::Vector3d>> active,dormant;f->getFrontiers(active);f->getDormantFrontiers(dormant);
    std::ofstream fronts(output+"/frontiers.csv");fronts<<"active,cluster,x,y,z\n";
    for(int type=0;type<2;++type){auto& clusters=type?active:dormant;for(size_t c=0;c<clusters.size();++c)for(auto& p:clusters[c])fronts<<type<<","<<c<<","<<p.x()<<","<<p.y()<<","<<p.z()<<"\n";}
    std::ofstream report(output+"/native-summary.json");report<<"{\"known\":"<<known<<",\"occupied\":"<<occupied<<",\"inflated\":"<<inflated<<",\"free_safe\":"<<free_safe<<",\"active_frontiers\":"<<active.size()<<",\"dormant_frontiers\":"<<dormant.size()<<"}";report.close();
    std_msgs::msg::String msg;msg.data="saved";done->publish(msg);
  });
  racer_ros::spin();return 0;
}
