#pragma once
#include <algorithm>
#include <vector>

namespace racer_port {
// Validate the single-vehicle LKH tour before converting matrix slots to IDs.
// Nodes 0 and 1 are depot and vehicle. Other nodes are tasks. -2 is the
// sentinel after the upstream parser converts the file's one-based IDs.
inline bool single_vehicle_route(const std::vector<int>& raw, int dimension,
                                 std::vector<int>& tasks) {
  tasks.clear();
  if(dimension<3 || raw.size()!=static_cast<size_t>(dimension+1) || raw.back()!=-2) return false;
  std::vector<bool> seen(dimension,false);
  for(int i=0;i<dimension;++i) {
    const int id=raw[i];
    if(id<0 || id>=dimension || seen[id]) return false;
    seen[id]=true;
  }
  const auto depot=std::find(raw.begin(),raw.end()-1,0)-raw.begin();
  // LKH may rotate its tour; the vehicle must immediately follow the depot.
  if(raw[(depot+1)%dimension]!=1) return false;
  for(int offset=2;offset<dimension;++offset) tasks.push_back(raw[(depot+offset)%dimension]-2);
  return !tasks.empty();
}
}
