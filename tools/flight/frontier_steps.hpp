#pragma once
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>
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

template<class Visit>
bool segmentBoxes(const V& start, const V& end, double res, Visit visit) {
  const V delta = end - start;
  const int count = std::max(1, static_cast<int>(std::ceil(delta.norm() / (.5 * res))));
  for (int i = 0; i < count; ++i) {
    const V a = start + delta * (double(i) / count);
    const V b = start + delta * (double(i + 1) / count);
    if (!visit(a.cwiseMin(b), a.cwiseMax(b))) return false;
  }
  return true;
}

template<class Map, class Check>
bool voxelBox(Map& map, const V& low, const V& high, Check check) {
  Eigen::Vector3i lo, hi;
  map.posToIndex(low.array() - 1e-8, lo);
  map.posToIndex(high.array() + 1e-8, hi);
  for (int x = lo.x(); x <= hi.x(); ++x)
    for (int y = lo.y(); y <= hi.y(); ++y)
      for (int z = lo.z(); z <= hi.z(); ++z)
        if (!check(Eigen::Vector3i(x, y, z))) return false;
  return true;
}

// Cheap proposal screen on the native map. Authorization still checks the
// actual trajectory and stopping reserve against both live maps.
template<class Map>
bool measuredLeg(Map& map, const V& start, const V& end, const V& radii, double res) {
  return segmentBoxes(start, end, res, [&](const V& a, const V& b) {
    const V low = a.array() - .10;
    const V high = b.array() + .10;
    if (!map.isInBox(low) || !map.isInBox(high)) return false;
    return voxelBox(map, low - radii, high + radii, [&](const Eigen::Vector3i& idx) {
      V center;
      map.indexToPos(idx, center);
      const V gap = (low - center).cwiseMax(center - high).array() - .5 * res;
      const V outside = gap.cwiseMax(0.);
      if (outside.isZero(1e-8) && map.getInflateOccupancy(idx) != 0) return false;
      return outside.cwiseQuotient(radii).squaredNorm() >= 1. || map.getOccupancy(idx) == Map::FREE;
    });
  });
}

template<class Map>
bool liveFrontier(Map& map, const V& cell) {
  Eigen::Vector3i idx;
  map.posToIndex(cell, idx);
  if (map.getOccupancy(idx) != Map::FREE) return false;
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3i offset = Eigen::Vector3i::Unit(axis);
    const Eigen::Vector3i before = idx - offset, after = idx + offset;
    if (map.getOccupancy(before) == Map::UNKNOWN || map.getOccupancy(after) == Map::UNKNOWN)
      return true;
  }
  return false;
}

enum class Source { None, Native, Staged };
struct Observation {
  V point;
  double yaw;
  int visible = 0;
  Source source = Source::None;
};

inline constexpr double min_observation_leg = .6;
inline constexpr double max_observation_leg = 1.6;

class ReplacementHistory {
public:
  bool accept(const V& root, const V& goal, double resolution) {
    if (last_ && (goal - last_->goal).norm() <= resolution &&
        (root - last_->root).norm() < min_observation_leg) return false;
    last_ = Leg{root, goal};
    return true;
  }

private:
  struct Leg { V root; V goal; };
  std::optional<Leg> last_;
};

template<class Visit>
void stagedSamples(const V& root, const V& target, Visit visit) {
  const V delta = target - root;
  const double horizontal = delta.head<2>().norm();
  if (horizontal < min_observation_leg) return;
  const V forward(delta.x() / horizontal, delta.y() / horizontal, 0.);
  const V side(-forward.y(), forward.x(), 0.);
  // Bound the full displacement, including the interpolated altitude change.
  const double reach = horizontal * std::min(1., max_observation_leg / delta.norm());
  for (double range : {reach, reach * .67, reach * .45})
    for (int i : {0, 1, -1, 2, -2, 3, -3}) {
      const double angle = i * M_PI / 9;
      V point = root + range * (std::cos(angle) * forward + std::sin(angle) * side);
      point.z() += (range / horizontal) * delta.z();
      if (delta.norm() - (target - point).norm() >= .20) visit(point);
    }
}

template<class Map, class Camera>
Observation refine(Map& map, Camera& camera, const V& root, double root_yaw, const V& target,
    double target_yaw, const std::vector<V>& alternatives, const std::vector<double>& headings,
    const std::vector<V>& cells, const V& center, const V& radii, double speed, double yaw_rate) {
  Observation best{target, target_yaw};
  if (cells.empty() || !target.allFinite() || !center.allFinite() ||
      (target - root).norm() <= max_observation_leg) return best;
  const double res = map.getResolution();
  std::vector<V> live;
  const size_t stride = (cells.size() + 23) / 24;
  for (size_t i = 0; i < cells.size(); i += stride)
    if (liveFrontier(map, cells[i])) live.push_back(cells[i]);
  if (live.empty()) return best;
  const int min_visible = std::min<int>(3, live.size());
  double best_time = 0.;
  auto consider = [&](const V& p, double heading, Source source) {
    const double distance = (p - root).norm();
    if (!(distance >= min_observation_leg && distance <= max_observation_leg + 1e-8) ||
        !std::isfinite(heading)) return;
    camera.setPose(p, heading);
    int visible = 0;
    for (const auto& cell : live) {
      if ((cell - p).norm() < res || !camera.insideFOV(cell)) continue;
      if (segmentBoxes(p, cell, res, [&](const V& low, const V& high) {
            return voxelBox(map, low, high, [&](const Eigen::Vector3i& idx) {
              return map.getOccupancy(idx) == Map::FREE;
            });
          })) ++visible;
    }
    if (visible < min_visible) return;
    const double turn = std::abs(std::remainder(heading - root_yaw, 2 * M_PI));
    const double time = std::max(distance / speed, turn / yaw_rate);
    if (visible < best.visible || (visible == best.visible && time >= best_time) ||
        !measuredLeg(map, root, p, radii, res)) return;
    best = {p, heading, visible, source};
    best_time = time;
  };
  for (size_t i = 0; i < alternatives.size(); ++i)
    if (alternatives[i] != target || headings[i] != target_yaw)
      consider(alternatives[i], headings[i], Source::Native);
  if (best.source == Source::None) {
    stagedSamples(root, target, [&](const V& point) {
      const V look = center - point;
      const double heading = look.head<2>().norm() > .2 ? std::atan2(look.y(), look.x()) : target_yaw;
      consider(point, heading, Source::Staged);
    });
  }
  return best;
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
