#include <racer_ros2/runtime.hpp>
#include <exploration_manager/fast_exploration_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  racer_ros::init(argc, argv, "exploration_node");
  racer_ros::NodeHandle nh("~");

  FastExplorationFSM expl_fsm;
  expl_fsm.init(nh);

  racer_ros::Duration(1.0).sleep();
  racer_ros::spin();

  return 0;
}
