#include <rclcpp/rclcpp.hpp>
#include <racer_interfaces/srv/solve_mtsp.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <filesystem>
#include <stdexcept>
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include <thread>
extern char **environ;
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("mtsp_node");
  auto dir=node->declare_parameter<std::string>("exploration.mtsp_dir", "");
  int id=node->declare_parameter<int>("exploration.drone_id",1);
  int problem=node->declare_parameter<int>("exploration.problem_id",1);
  if(dir.empty() || (problem!=1 && problem!=2)) throw std::invalid_argument("Invalid solver configuration");
  auto executable=ament_index_cpp::get_package_prefix("racer_core")+"/lib/racer_core/racer_lkh";
  using S=racer_interfaces::srv::SolveMTSP;
  auto server=node->create_service<S>((problem==1?"/solve_tsp_":"/solve_acvrp_")+std::to_string(id),
    [dir,id,executable](const S::Request::SharedPtr req,S::Response::SharedPtr res) {
      res->empty=1;
      if(req->prob<1 || req->prob>3) return;
      auto path=dir+"/amtsp"+(req->prob==1?std::string(""):std::to_string(req->prob))+"_"+std::to_string(id)+".par";
      if(!std::filesystem::is_regular_file(path)) return;
      char* args[]={const_cast<char*>(executable.c_str()),const_cast<char*>(path.c_str()),nullptr};
      pid_t child;
      if(posix_spawn(&child,executable.c_str(),nullptr,nullptr,args,environ)!=0) return;
      auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(4);
      int status=0;
      while(waitpid(child,&status,WNOHANG)==0) {
        if(!rclcpp::ok() || std::chrono::steady_clock::now()>deadline) {
          kill(child,SIGKILL);waitpid(child,&status,0);return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      res->empty=WIFEXITED(status) && WEXITSTATUS(status)==0 ? 0 : 1;
    });
  RCLCPP_INFO(node->get_logger(),"RACER ROS2 solver ready");
  rclcpp::spin(node);rclcpp::shutdown();return 0;
}
