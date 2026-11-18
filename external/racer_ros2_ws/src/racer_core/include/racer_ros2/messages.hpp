#pragma once
#include <racer_ros2/runtime.hpp>
#include <std_msgs/msg/header.hpp>
namespace std_msgs {using Header=std_msgs::msg::Header; using HeaderConstPtr=Header::ConstSharedPtr;}
#include <std_msgs/msg/empty.hpp>
namespace std_msgs {using Empty=std_msgs::msg::Empty; using EmptyConstPtr=Empty::ConstSharedPtr;}
#include <std_msgs/msg/float64_multi_array.hpp>
namespace std_msgs {using Float64MultiArray=std_msgs::msg::Float64MultiArray; using Float64MultiArrayConstPtr=Float64MultiArray::ConstSharedPtr;}
#include <std_msgs/msg/color_rgba.hpp>
namespace std_msgs {using ColorRGBA=std_msgs::msg::ColorRGBA; using ColorRGBAConstPtr=ColorRGBA::ConstSharedPtr;}
#include <geometry_msgs/msg/point.hpp>
namespace geometry_msgs {using Point=geometry_msgs::msg::Point; using PointConstPtr=Point::ConstSharedPtr;}
#include <geometry_msgs/msg/pose_stamped.hpp>
namespace geometry_msgs {using PoseStamped=geometry_msgs::msg::PoseStamped; using PoseStampedConstPtr=PoseStamped::ConstSharedPtr;}
#include <geometry_msgs/msg/pose.hpp>
namespace geometry_msgs {using Pose=geometry_msgs::msg::Pose; using PoseConstPtr=Pose::ConstSharedPtr;}
#include <geometry_msgs/msg/vector3.hpp>
namespace geometry_msgs {using Vector3=geometry_msgs::msg::Vector3; using Vector3ConstPtr=Vector3::ConstSharedPtr;}
#include <geometry_msgs/msg/quaternion.hpp>
namespace geometry_msgs {using Quaternion=geometry_msgs::msg::Quaternion; using QuaternionConstPtr=Quaternion::ConstSharedPtr;}
#include <nav_msgs/msg/odometry.hpp>
namespace nav_msgs {using Odometry=nav_msgs::msg::Odometry; using OdometryConstPtr=Odometry::ConstSharedPtr;}
#include <nav_msgs/msg/path.hpp>
namespace nav_msgs {using Path=nav_msgs::msg::Path; using PathConstPtr=Path::ConstSharedPtr;}
#include <sensor_msgs/msg/image.hpp>
namespace sensor_msgs {using Image=sensor_msgs::msg::Image; using ImageConstPtr=Image::ConstSharedPtr;}
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace sensor_msgs {using PointCloud2=sensor_msgs::msg::PointCloud2; using PointCloud2ConstPtr=PointCloud2::ConstSharedPtr;}
#include <visualization_msgs/msg/marker.hpp>
namespace visualization_msgs {using Marker=visualization_msgs::msg::Marker; using MarkerConstPtr=Marker::ConstSharedPtr;}
#include <visualization_msgs/msg/marker_array.hpp>
namespace visualization_msgs {using MarkerArray=visualization_msgs::msg::MarkerArray; using MarkerArrayConstPtr=MarkerArray::ConstSharedPtr;}
#include <racer_interfaces/msg/bspline.hpp>
namespace bspline {using Bspline=racer_interfaces::msg::Bspline; using BsplineConstPtr=Bspline::ConstSharedPtr;}
#include <racer_interfaces/msg/idx_list.hpp>
namespace plan_env {using IdxList=racer_interfaces::msg::IdxList; using IdxListConstPtr=IdxList::ConstSharedPtr;}
#include <racer_interfaces/msg/chunk_stamps.hpp>
namespace plan_env {using ChunkStamps=racer_interfaces::msg::ChunkStamps; using ChunkStampsConstPtr=ChunkStamps::ConstSharedPtr;}
#include <racer_interfaces/msg/chunk_data.hpp>
namespace plan_env {using ChunkData=racer_interfaces::msg::ChunkData; using ChunkDataConstPtr=ChunkData::ConstSharedPtr;}
#include <racer_interfaces/msg/drone_state.hpp>
namespace exploration_manager {using DroneState=racer_interfaces::msg::DroneState; using DroneStateConstPtr=DroneState::ConstSharedPtr;}
#include <racer_interfaces/msg/pair_opt.hpp>
namespace exploration_manager {using PairOpt=racer_interfaces::msg::PairOpt; using PairOptConstPtr=PairOpt::ConstSharedPtr;}
#include <racer_interfaces/msg/pair_opt_response.hpp>
namespace exploration_manager {using PairOptResponse=racer_interfaces::msg::PairOptResponse; using PairOptResponseConstPtr=PairOptResponse::ConstSharedPtr;}
#include <racer_interfaces/msg/h_grid.hpp>
namespace exploration_manager {using HGrid=racer_interfaces::msg::HGrid; using HGridConstPtr=HGrid::ConstSharedPtr;}
#include <racer_interfaces/msg/grid_tour.hpp>
namespace exploration_manager {using GridTour=racer_interfaces::msg::GridTour; using GridTourConstPtr=GridTour::ConstSharedPtr;}
#include <racer_interfaces/srv/solve_mtsp.hpp>
namespace lkh_mtsp_solver {struct SolveMTSP {using RosService=racer_interfaces::srv::SolveMTSP; using Request=RosService::Request; using Response=RosService::Response; Request request; Response response;};}
#include <racer_interfaces/srv/solve_tsp.hpp>
namespace lkh_tsp_solver {struct SolveTSP {using RosService=racer_interfaces::srv::SolveTSP; using Request=RosService::Request; using Response=RosService::Response; Request request; Response response;};}
