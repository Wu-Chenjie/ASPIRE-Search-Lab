#include "target_registry_ros.hpp"
int main(int argc,char**argv){rclcpp::init(argc,argv);auto n=std::make_shared<rclcpp::Node>("target_registry");auto own=n->declare_parameter<std::string>("robot","uav0");racer_task::TargetRegistryROS registry(n,own);rclcpp::spin(n);rclcpp::shutdown();}
