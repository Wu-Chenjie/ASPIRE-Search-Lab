"""Opt-in read-only export of the actual native map after cloud fusion.

The publisher changes no occupancy, inflation, planner, or safety thresholds.
Wire v1: [version, input_stamp_s, sequence, resolution_m, origin_xyz_m,
          shape_xyz_cells, C-order native states].
States use SDFMap UNKNOWN=0/FREE=1/OCCUPIED=2, plus 4 when inflated.
"""
from .optimizer_overlay import replace_once


def apply(core):
    header = core / 'include/plan_env/map_ros.h'
    text = header.read_text()
    anchor = '#include <pcl_conversions/pcl_conversions.h>'
    text = replace_once(text, anchor, anchor + '\n#include <std_msgs/msg/float64_multi_array.hpp>\n#include <cstdint>')
    anchor = '  void processDepthImage();'
    text = replace_once(text, anchor, anchor + '''

  void publishExecutionSafetyMap(double input_stamp_seconds);
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr execution_safety_map_pub_;
  std::uint64_t execution_safety_map_seq_ = 0;
''')
    header.write_text(text)

    source = core / 'src/plan_env/map_ros.cpp'
    text = source.read_text()
    text = '#include <exploration_manager/static_causal_trace.hpp>\n' + text
    anchor = 'void MapROS::init() {'
    text = replace_once(text, anchor, anchor + '''
  int execution_safety_drone_id = 1;
  node_.param("exploration/drone_id", execution_safety_drone_id, 1);
  ROS_ASSERT(execution_safety_drone_id >= 1);
  execution_safety_map_pub_ = node_.get()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/baseline/safety_map_" + std::to_string(execution_safety_drone_id - 1),
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  executionSafetyMapTrace(); // initialize the optional journal before subscribing
''')
    start = text.index('void MapROS::cloudPoseCallback(')
    end = text.index('// void MapROS::basecoorCallback', start)
    callback = text[start:end]
    entry = '  camera_pos_(0) = pose->pose.position.x;'
    callback = replace_once(callback, entry, '''  const double capture = static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
  auto& trace = executionSafetyMapTrace();
  if (trace.enabled()) trace.record(20, 0., 0, 0, execution_safety_map_seq_ + 1, capture);
''' + entry)
    anchor = '    local_updated_ = false;\n  }\n}'
    callback = replace_once(callback, anchor, '''    local_updated_ = false;
  }
  if (trace.enabled()) trace.record(21, 0., 0, 0, execution_safety_map_seq_ + 1, capture);
  // Emit every fused callback, including unchanged/empty clouds. Timestamp is
  // the measured input stamp, not publication wall time or a synthetic clock.
  publishExecutionSafetyMap(capture);
}''')
    text = text[:start] + callback + text[end:]
    anchor = 'void MapROS::visCallback('
    start = text.index(anchor)
    text = text[:start] + '''void MapROS::publishExecutionSafetyMap(double input_stamp_seconds) {
  std_msgs::msg::Float64MultiArray message;
  const auto& shape = map_->mp_->map_voxel_num_;
  const auto& origin = map_->mp_->map_origin_;
  const std::size_t count = static_cast<std::size_t>(shape[0]) * shape[1] * shape[2];
  auto& data = message.data;
  data.resize(10 + count);
  data[0] = 1.0;
  data[1] = input_stamp_seconds;
  data[2] = static_cast<double>(++execution_safety_map_seq_);
  data[3] = map_->mp_->resolution_;
  for (int axis = 0; axis < 3; ++axis) {
    data[4 + axis] = origin[axis];
    data[7 + axis] = shape[axis];
  }
  // Same x-major, z-fastest address convention as the native SDFMap buffer.
  for (int x = 0; x < shape[0]; ++x)
    for (int y = 0; y < shape[1]; ++y)
      for (int z = 0; z < shape[2]; ++z) {
        const Eigen::Vector3i idx(x, y, z);
        data[10 + map_->toAddress(idx)] = map_->getOccupancy(idx) +
            (map_->getInflateOccupancy(idx) > 0 ? 4 : 0);
      }
  auto& trace = executionSafetyMapTrace();
  if (trace.enabled()) trace.record(22, 0., 0, 0, execution_safety_map_seq_, input_stamp_seconds);
  execution_safety_map_pub_->publish(message);
  if (trace.enabled()) trace.record(23, 0., 0, 0, execution_safety_map_seq_, input_stamp_seconds);
}

''' + text[start:]
    start = text.index('void MapROS::init() {')
    text = text[:start] + '''static racer_task::StaticCausalTrace& executionSafetyMapTrace() {
  static racer_task::StaticCausalTrace trace(std::getenv("ASPIRE_SAFETY_MAP_NATIVE_JOURNAL"));
  return trace;
}

''' + text[start:]
    source.write_text(text)
