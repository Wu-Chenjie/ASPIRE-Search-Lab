// Host-only inbox sync benchmark; excludes association, JSON, ROS and planning.
#include "static_guidance_events.hpp"
#include <iostream>
using namespace racer_task;
int main(){
 std::cout<<"{\"scope\":\"host_native_inbox_sync_only_no_association_ros_or_solver\",\"samples\":[";
 bool comma=false;
 for(int count:{1,8,64,256}){
  TargetRegistry r;r.per_robot_limit=256;StaticGuidanceEvents q;
  for(int i=1;i<=count;++i){TargetObservation o;o.key={"uav0","bench",std::uint64_t(i)};
   o.sequence=1;o.first=1.;o.capture=1.4;o.frame_version="map";o.has_bearing=true;
   o.direction={1,0,0};o.confidence=.6;o.support=3;o.support_first=1.;
   if(!r.ingest(o,1.4).accepted)return 1;
  }
  q.sync(r,1.4);std::vector<double> times;
  for(int i=0;i<2000;++i){const auto start=std::chrono::steady_clock::now();q.sync(r,1.4);
   times.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count());}
  std::sort(times.begin(),times.end());auto s=q.stats();if(s.pending!=std::size_t(count)||s.enqueued!=std::size_t(count))return 2;
  if(comma)std::cout<<',';comma=true;
  std::cout<<"{\"entities\":"<<count<<",\"iterations\":2000,\"p50_us\":"<<times[1000]<<",\"p99_us\":"<<times[1980]<<",\"pending\":"<<s.pending<<"}";
 }std::cout<<"]}\n";
}
