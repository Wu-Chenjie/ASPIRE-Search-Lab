#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "target_protocol.hpp"
#include <iostream>
int main(int argc,char**argv){
 racer_task::TargetRegistry registry(argc>1?argv[1]:"uav0");std::string line;std::size_t raw=0;
 while(std::getline(std::cin,line)){
  try{
   auto tree=racer_task::parseTargetTree(line);double now=tree.get<double>("now_s");
   auto kind=tree.get<std::string>("kind");
   if(kind=="sync_request"){
    auto keys=registry.sync(tree.get<std::string>("robot"),tree.get<std::string>("request_id"),racer_task::targetUnsigned(tree,"after_counter"),now);
    std::cout<<"{\"sync_records\":[";bool first=true;
    for(const auto&k:keys){if(!first)std::cout<<",";first=false;std::cout<<racer_task::targetRecordSummaryJSON(registry.records.at(k));}
    std::cout<<"]}"<<std::endl;
   }else{
    auto result=racer_task::processTargetTree(registry,tree,now);if(!result.peer)raw=std::max(raw,result.raw_tracks);
    std::cout<<racer_task::targetRegistryJSON(registry,now,raw)<<std::endl;registry.events.clear();
   }
  }catch(const std::exception&e){std::cout<<"{\"error\":"<<racer_task::reviewJsonString(e.what())<<"}"<<std::endl;}
 }
}
