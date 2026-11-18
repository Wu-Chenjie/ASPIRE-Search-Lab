// Middleware verification executable; never publishes flight commands.
#include <racer_ros2/runtime.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
int main(int argc,char** argv) {
  racer_ros::init(argc,argv,"racer_runtime_probe");
  racer_ros::NodeHandle nh;
  double value;nh.param("probe/value",value,0.);
  auto publisher=nh.advertise<std_msgs::msg::Float64MultiArray>("/racer_port_probe",10);
  struct Counter {
    racer_ros::Publisher pub;double value;int count=0;
    void tick(const racer_ros::TimerEvent& event) {
      std_msgs::msg::Float64MultiArray m;
      m.data={double(++count),event.current_real.toSec(),value};pub.publish(m);
    }
  } counter{publisher,value};
  auto timer=nh.createTimer(racer_ros::Duration(.05),&Counter::tick,&counter);
  racer_ros::spin();return 0;
}
