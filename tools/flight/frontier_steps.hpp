#pragma once
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <Eigen/Core>

// Select a short observation leg toward a frontier RACER already chose. Only
// live frontier samples and measured occupancy are inputs. This is a proposal:
// native trajectory generation and both independent authorizers are mandatory.
namespace aspire_steps {
using V = Eigen::Vector3d;
inline bool enabled() {
  const char* p=std::getenv("ASPIRE_FRONTIER_STEPS");
  return p && std::string(p)=="1";
}
template<class Map, class Eligible>
bool select(Map& map, const V& root, const V& target, const std::vector<V>& cells,
            const V& center, Eligible eligible, V& goal, double& yaw) {
  const double res=map.getResolution();
  const V delta=target-root;
  const double horizontal=delta.head<2>().norm();
  if (!(res>0.) || !delta.allFinite() || !center.allFinite() ||
      horizontal<.6 || cells.empty()) return false;
  const V forward(delta.x()/horizontal,delta.y()/horizontal,0);
  const V side(-forward.y(),forward.x(),0);
  const double reach=std::min(1.6,horizontal);
  auto clear=[&](const V& p){return map.isInBox(p) &&
      map.getOccupancy(p)==Map::FREE && map.getInflateOccupancy(p)==0;};
  auto corridor=[&](const V& p){
    const int n=std::max(1,int(std::ceil((p-root).norm()/(.5*res))));
    for(int j=0;j<=n;++j) {
      V q=root+(p-root)*(double(j)/n);
      if(!clear(q) || !clear(q+.10*side) || !clear(q-.10*side) ||
         !clear(q+V(0,0,.10)) || !clear(q-V(0,0,.10)))return false;
    }
    return true;
  };
  // Count sampled frontier cells that an actual view could reveal, stopping
  // the line of sight at unknown or occupied intervening voxels.
  auto views=[&](const V& p,double h){
    int good=0,total=0;const size_t stride=(cells.size()+23)/24;
    for(size_t i=0;i<cells.size();i+=stride) {
      ++total;const V ray=cells[i]-p;const double r=ray.norm();
      if(r<res || r>6. || std::abs(std::remainder(std::atan2(ray.y(),ray.x())-h,2*M_PI))>.58 ||
         std::abs(ray.z())>.48*ray.head<2>().norm())continue;
      const int n=std::max(1,int(std::ceil(r/res)));bool seen=true;
      for(int j=1;j<n;++j){V q=p+ray*(double(j)/n);
        if(!map.isInBox(q) || map.getOccupancy(q)!=Map::FREE){seen=false;break;}}
      if(seen)++good;
    }
    return double(good)/total;
  };
  const double fallback_yaw=yaw;double best=-1e9;
  // Symmetric bearings about the actual frontier, not about a world heading.
  for(double range : {reach,reach*.67,reach*.45})for(int i : {0,1,-1,2,-2,3,-3}) {
    const double angle=i*M_PI/9;
    V p=root+range*(std::cos(angle)*forward+std::sin(angle)*side);
    p.z()+=(range/horizontal)*delta.z();
    const double gain=delta.norm()-(target-p).norm();
    if(gain<.20 || !eligible(p) || !corridor(p))continue;
    const V toward=center-p;
    const double h=toward.head<2>().norm()>.2?std::atan2(toward.y(),toward.x()):fallback_yaw;
    const double score=gain+1.2*views(p,h);
    if(score>best){best=score;goal=p;yaw=h;}
  }
  return best>-1e9;
}
}
