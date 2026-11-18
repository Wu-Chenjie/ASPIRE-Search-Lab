#pragma once
// RACER algorithm-facing API implemented solely with rclcpp. No ROS1 runtime.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>
#include <typeindex>
#include <boost/bind/bind.hpp>

namespace racer_ros {
inline rclcpp::Node::SharedPtr node;
inline std::string parameter_name(std::string name) {
  std::replace(name.begin(), name.end(), '/', '.'); return name;
}
struct Duration {
  int64_t ns = 0;
  Duration() = default;
  explicit Duration(double s): ns(static_cast<int64_t>(s * 1e9)) {}
  double toSec() const { return ns * 1e-9; }
  void sleep() const { std::this_thread::sleep_for(std::chrono::nanoseconds(ns)); }
  operator builtin_interfaces::msg::Duration() const { return rclcpp::Duration(std::chrono::nanoseconds(ns)); }
};
struct Time {
  int64_t ns = 0;
  Time() = default;
  explicit Time(double s): ns(static_cast<int64_t>(s * 1e9)) {}
  Time(const builtin_interfaces::msg::Time &t): ns(rclcpp::Time(t).nanoseconds()) {}
  static Time now() { Time t; t.ns = node->now().nanoseconds(); return t; }
  double toSec() const { return ns * 1e-9; }
  operator builtin_interfaces::msg::Time() const { return rclcpp::Time(ns, RCL_ROS_TIME); }
  friend Duration operator-(Time a, Time b) { Duration d; d.ns=a.ns-b.ns; return d; }
  friend Time operator+(Time a, Duration b) { a.ns+=b.ns; return a; }
  friend Time operator-(Time a, Duration b) { a.ns-=b.ns; return a; }
  friend bool operator<(Time a, Time b) { return a.ns<b.ns; }
  friend bool operator>(Time a, Time b) { return a.ns>b.ns; }
};
struct WallTime {
  std::chrono::steady_clock::time_point t;
  static WallTime now() { return {std::chrono::steady_clock::now()}; }
  friend Duration operator-(WallTime a, WallTime b) { Duration d; d.ns=std::chrono::duration_cast<std::chrono::nanoseconds>(a.t-b.t).count();return d; }
};
struct TimerEvent { Time last_real, current_real; };
struct Timer { rclcpp::TimerBase::SharedPtr p; void stop(){p->cancel();} void start(){p->reset();} };
struct Subscriber { rclcpp::SubscriptionBase::SharedPtr p; void shutdown(){p.reset();} };
struct Publisher {
  rclcpp::PublisherBase::SharedPtr p;
  template<class M> void publish(const M& msg) const {
    auto typed=std::dynamic_pointer_cast<rclcpp::Publisher<M>>(p);
    if(!typed) throw std::logic_error("RACER publisher type mismatch");
    typed->publish(msg);
  }
  size_t getNumSubscribers() const {return p->get_subscription_count();}
};
struct ServiceClient {
  rclcpp::Node::SharedPtr owner;
  rclcpp::ClientBase::SharedPtr p;
  template<class S> bool call(S& service) {
    auto client=std::dynamic_pointer_cast<rclcpp::Client<typename S::RosService>>(p);
    if(!client || !client->wait_for_service(std::chrono::seconds(2))) return false;
    auto future=client->async_send_request(std::make_shared<typename S::Request>(service.request));
    auto result=rclcpp::spin_until_future_complete(owner, future, std::chrono::seconds(5));
    if(result!=rclcpp::FutureReturnCode::SUCCESS) {client->remove_pending_request(future);return false;}
    service.response=*future.get(); return service.response.empty==0;
  }
};
struct NodeHandle {
  NodeHandle(const std::string& = "") {}
  rclcpp::Node* get() {return node.get();}
  template<class T> void param(const std::string& key, T& value, const T& fallback) const {
    auto name=parameter_name(key);
    if(!node->has_parameter(name)) node->declare_parameter<T>(name,fallback);
    const auto parameter=node->get_parameter(name);
    // ROS1 accepts integer YAML scalars for double parameters. Preserve that
    // numeric promotion without weakening bool/string or fractional-int checks.
    if constexpr (std::is_floating_point_v<T>) {
      if(parameter.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER) {
        value=static_cast<T>(parameter.as_int());return;
      }
    }
    node->get_parameter(name,value);
  }
  template<class M> Publisher advertise(const std::string& topic, size_t depth, bool latch=false) {
    auto qos=rclcpp::QoS(depth);if(latch) qos.transient_local();
    return {node->create_publisher<M>(topic,qos)};
  }
  template<class M,class C> Subscriber subscribe(const std::string& topic,size_t depth,
      void(C::*fn)(const std::shared_ptr<const M>&),C* object) {
    return {node->create_subscription<M>(topic,rclcpp::QoS(depth),
      [object,fn](std::shared_ptr<const M> m){(object->*fn)(m);})};
  }
  template<class M,class F> Subscriber subscribe(const std::string& topic,size_t depth,F fn) {
    return {node->create_subscription<M>(topic,rclcpp::QoS(depth),
      [fn](std::shared_ptr<const M> m){fn(m);})};
  }
  template<class C> Timer createTimer(Duration interval,void(C::*fn)(const TimerEvent&),C* object) {
    auto last=std::make_shared<Time>(Time::now());
    return {rclcpp::create_timer(node,node->get_clock(),std::chrono::nanoseconds(interval.ns),
      [last,object,fn](){auto now=Time::now(); TimerEvent event{*last,now}; *last=now;(object->*fn)(event);})};
  }
  template<class S> ServiceClient serviceClient(const std::string& name,bool = false) {
    // Separate executor avoids waiting on the planner's mutually-exclusive callback group.
    static int count=0;
    auto options=rclcpp::NodeOptions().use_global_arguments(false);
    auto owner=std::make_shared<rclcpp::Node>(std::string(node->get_name())+"_solver_client_"+std::to_string(++count),options);
    auto client=owner->create_client<typename S::RosService>(name);
    return {owner,client};
  }
};
inline void init(int argc,char** argv,const char* name) {
  rclcpp::init(argc,argv);
  node=std::make_shared<rclcpp::Node>(name,rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
}
inline void spin() {rclcpp::spin(node);node.reset();rclcpp::shutdown();}
inline bool ok(){return rclcpp::ok();}
}
#define ROS_WARN(...) RCLCPP_WARN(racer_ros::node->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(racer_ros::node->get_logger(), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(racer_ros::node->get_logger(), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(racer_ros::node->get_logger(), __VA_ARGS__)
#define ROS_WARN_STREAM(x) RCLCPP_WARN_STREAM(racer_ros::node->get_logger(),x)
#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(racer_ros::node->get_logger(),x)
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(racer_ros::node->get_logger(),x)

#define ROS_INFO_STREAM_THROTTLE(t,x) RCLCPP_INFO_STREAM_THROTTLE(racer_ros::node->get_logger(),*racer_ros::node->get_clock(),static_cast<int64_t>((t)*1000),x)
#define ROS_WARN_THROTTLE(t,...) RCLCPP_WARN_THROTTLE(racer_ros::node->get_logger(),*racer_ros::node->get_clock(),static_cast<int64_t>((t)*1000),__VA_ARGS__)
#define ROS_INFO_THROTTLE(t,...) RCLCPP_INFO_THROTTLE(racer_ros::node->get_logger(),*racer_ros::node->get_clock(),static_cast<int64_t>((t)*1000),__VA_ARGS__)
#define ROS_ASSERT(x) do { if(!(x)) throw std::logic_error("RACER assertion: " #x); } while(false)

#define ROS_ERROR_COND(condition,...) do {if(condition) ROS_ERROR(__VA_ARGS__);} while(false)
