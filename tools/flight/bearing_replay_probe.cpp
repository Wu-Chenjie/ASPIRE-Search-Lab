// Offline decision replay of the exact policy compiled into native RACER.
#include "bearing_bias.hpp"
#include <iostream>
#include <iomanip>
int main(){
  racer_task::BearingBias policy;char event;
  std::cout<<std::setprecision(17);
  while(std::cin>>event){
    double now;int enabled;std::cin>>now>>enabled;policy.enabled=enabled;
    if(event=='R'){
      int source;std::cin>>source;std::vector<double> packet(17);
      for(auto& value:packet)std::cin>>value;
      std::cout<<"R "<<policy.receive(packet,now,source)<<'\n';
    }else if(event=='D'){
      auto read=[](){racer_task::BearingCandidate c;int reachable;
        std::cin>>c.id>>c.point[0]>>c.point[1]>>c.point[2]>>c.yaw>>c.cost>>reachable;
        c.reachable=reachable;return c;};
      auto baseline=read();size_t n;std::cin>>n;
      std::vector<racer_task::BearingCandidate> candidates;
      for(size_t i=0;i<n;++i)candidates.push_back(read());
      auto result=policy.choose(baseline,candidates,now);
      std::cout<<"D "<<result.selected<<' '<<result.changed<<' '<<result.cue_id<<' '<<result.reason<<' '<<policy.selections<<' '<<result.scores.size();
      for(const auto& score:result.scores)std::cout<<' '<<score.id<<' '<<score.cue_id<<' '<<score.base_cost<<' '<<score.adjusted_cost<<' '<<score.utility<<' '<<score.reachable<<' '<<score.reason;
      std::cout<<'\n';
    }else return 2;
    if(!std::cin)return 3;
  }
}
