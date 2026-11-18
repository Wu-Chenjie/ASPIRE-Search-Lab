#include <gz/transport/Node.hh>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <string>
static bool echo(const gz::msgs::Boolean &req,gz::msgs::Boolean &rep){rep.set_data(req.data());return true;}
int main(int argc,char **argv){
 gz::transport::Node node;
 if(argc>1&&std::string(argv[1])=="--selftest-server"){
  if(!node.Advertise("/stage49/echo",echo))return 2;
  auto pub=node.Advertise<gz::msgs::Clock>("/stage49/clock");
  for(int i=0;i<20000;++i){gz::msgs::Clock m;m.mutable_sim()->set_sec(i/1000);m.mutable_sim()->set_nsec((i%1000)*1000000);pub.Publish(m);std::this_thread::sleep_for(std::chrono::milliseconds(1));}return 0;
 }
 std::string command;
 while(std::getline(std::cin,command)){
  bool result=false,executed=false;gz::msgs::Boolean rep;auto start=std::chrono::steady_clock::now();
  if(command=="echo"){gz::msgs::Boolean req;req.set_data(true);executed=node.Request("/stage49/echo",req,500u,rep,result);}
  else if(command=="pause"||command=="step"){gz::msgs::WorldControl req;req.set_pause(true);if(command=="step")req.set_multi_step(1);executed=node.Request("/world/annual/control",req,command=="pause"?2000u:500u,rep,result);}
  else return 3;
  auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-start).count();
  std::cout<<"{\"executed\":"<<(executed?"true":"false")<<",\"handler_result\":"<<(result?"true":"false")<<",\"data\":"<<(rep.data()?"true":"false")<<",\"elapsed_ns\":"<<elapsed<<"}"<<std::endl;
 }return 0;
}
