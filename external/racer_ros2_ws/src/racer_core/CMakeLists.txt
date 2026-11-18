cmake_minimum_required(VERSION 3.16)
project(racer_core LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(OpenCV REQUIRED)
find_library(NLOPT_LIBRARY nlopt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(racer_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(exploration_node
  src/plan_env/sdf_map.cpp
  src/plan_env/obj_predictor.cpp
  src/plan_env/edt_environment.cpp
  src/plan_env/raycast.cpp
  src/plan_env/map_ros.cpp
  src/plan_env/multi_map_manager.cpp
  src/bspline/non_uniform_bspline.cpp
  src/poly_traj/polynomial_traj.cpp
  src/path_searching/kinodynamic_astar.cpp
  src/path_searching/astar2.cpp
  src/path_searching/topo_prm.cpp
  src/active_perception/traj_visibility.cpp
  src/active_perception/heading_planner.cpp
  src/active_perception/frontier_finder.cpp
  src/active_perception/graph_node.cpp
  src/active_perception/perception_utils.cpp
  src/active_perception/uniform_grid.cpp
  src/active_perception/hgrid.cpp
  src/bspline_opt/bspline_optimizer.cpp
  src/traj_utils/planning_visualization.cpp
  src/plan_manage/planner_manager.cpp
  src/plan_manage/planner_manager_dev.cpp
  src/exploration_manager/exploration_node.cpp
  src/exploration_manager/fast_exploration_fsm.cpp
  src/exploration_manager/fast_exploration_manager.cpp
)
ament_target_dependencies(exploration_node rclcpp racer_interfaces std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs cv_bridge message_filters pcl_conversions ament_index_cpp)
target_link_libraries(exploration_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${NLOPT_LIBRARY})
file(GLOB LKH_SOURCES lkh/src/*.c)
add_executable(racer_lkh ${LKH_SOURCES} lkh/LKHmain.c)
target_include_directories(racer_lkh PRIVATE lkh/include)
target_compile_options(racer_lkh PRIVATE -fcommon)
target_link_libraries(racer_lkh m)
add_executable(mtsp_node solver.cpp)
ament_target_dependencies(mtsp_node rclcpp racer_interfaces ament_index_cpp)
add_executable(runtime_probe runtime_probe.cpp)
ament_target_dependencies(runtime_probe rclcpp std_msgs)
install(TARGETS exploration_node racer_lkh mtsp_node runtime_probe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
