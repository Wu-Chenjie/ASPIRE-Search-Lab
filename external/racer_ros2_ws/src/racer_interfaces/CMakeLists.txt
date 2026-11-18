cmake_minimum_required(VERSION 3.16)
project(racer_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  "msg/IdxList.msg"
  "msg/ChunkStamps.msg"
  "msg/ChunkData.msg"
  "msg/DroneState.msg"
  "msg/PairOpt.msg"
  "msg/PairOptResponse.msg"
  "msg/HGrid.msg"
  "msg/GridTour.msg"
  "srv/SolveMTSP.srv"
  "srv/SolveTSP.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
