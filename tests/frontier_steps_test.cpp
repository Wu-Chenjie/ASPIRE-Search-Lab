#include "frontier_steps.hpp"
#include <cassert>
#include <iostream>
#include <map>
#include <tuple>
using aspire_steps::V;
struct Map {
  enum{UNKNOWN,FREE};int sign;bool closed=false;
  double getResolution() {return .15;}
  bool isInBox(const V&p){return p.norm()<10;}
  int getInflateOccupancy(const V&p){return getOccupancy(p)==UNKNOWN;}
  int getOccupancy(const V&p){return closed|| (p.x()>.35 && sign*p.y()<.14) ? UNKNOWN:FREE;}
};

struct VoxelMap {
  enum {UNKNOWN, FREE, OCCUPIED};
  double getResolution() {return .2;}
  bool isInBox(const V& p) {return p.norm() < 10.;}
  void posToIndex(const V& p, Eigen::Vector3i& i) {
    i = (p.array() / getResolution()).floor().cast<int>();
  }
  void indexToPos(const Eigen::Vector3i& i, V& p) {
    p = (i.cast<double>().array() + .5) * getResolution();
  }
  int getOccupancy(const Eigen::Vector3i& i) {
    auto it = overrides.find({i.x(), i.y(), i.z()});
    if (it != overrides.end()) return it->second;
    if (wall && i.x() == 10) return OCCUPIED;
    return frontier && i.x() >= 15 ? UNKNOWN : FREE;
  }
  int getInflateOccupancy(const Eigen::Vector3i& i) {return getOccupancy(i) == OCCUPIED;}
  std::map<std::tuple<int, int, int>, int> overrides;
  bool frontier = true;
  bool wall = false;
};

struct Camera {
  void setPose(const V& p, double yaw) {position = p; heading = yaw;}
  bool insideFOV(const V& p) {
    const V ray = p - position;
    return ray.norm() < 4.5 && std::abs(ray.z()) < .48 * ray.head<2>().norm() &&
        std::abs(std::remainder(std::atan2(ray.y(), ray.x()) - heading, 2. * M_PI)) < .58;
  }
  V position;
  double heading;
};

void imminent_viewpoint() {
  using aspire_steps::Source;
  VoxelMap map;
  Camera camera;
  const V root(.1, .1, 1.1), tour(2.3, .1, 1.1), short_view(1.3, .1, 1.1);
  const V center(2.9, .1, 1.1), radii(.6, .6, .30);
  const std::vector<V> frontier = {center, V(2.9, -.1, 1.1), V(2.9, .3, 1.1)};
  auto refine = [&](const std::vector<V>& alternatives, const std::vector<double>& headings) {
    return aspire_steps::refine(map, camera, root, 0., tour, 0., alternatives, headings,
        frontier, center, radii, .3, .325);
  };

  // Retain a native same-frontier heading and prefer its shorter usable leg.
  auto selected = refine({V(1.5, .1, 1.1), short_view}, {0., .08});
  assert(selected.source == Source::Native && selected.point == short_view && selected.yaw == .08);
  assert(selected.visible == 3);

  // A yaw which cannot see the frontier leaves the staged measured view as the next option.
  selected = refine({short_view}, {M_PI});
  assert(selected.source == Source::Staged && selected.visible == 3);
  assert((selected.point - root).norm() <= 1.6 + 1e-8 && (selected.point - tour).norm() > .1);

  const V near_tour(.85, .1, 1.1);
  selected = aspire_steps::refine(map, camera, root, 0., near_tour, M_PI, {V(.75, .1, 1.1)}, {0.},
      frontier, center, radii, .3, .325);
  assert(selected.source == Source::None && selected.point == near_tour && selected.yaw == M_PI);

  const V boundary_tour = root + V(0., aspire_steps::max_observation_leg, 0.);
  selected = aspire_steps::refine(map, camera, root, 0., boundary_tour, M_PI, {short_view}, {0.},
      frontier, center, radii, .3, .325);
  assert(selected.source == Source::None && selected.point == boundary_tour);

  // A pocket outside the centerline consumes braking/holding clearance.
  map.overrides[{6, 2, 5}] = VoxelMap::UNKNOWN;
  assert(!aspire_steps::measuredLeg(map, root, short_view, radii, map.getResolution()));
  selected = refine({short_view}, {0.});
  assert(selected.source != Source::Native);
  map.overrides.clear();

  // A clear flight corridor cannot make a view useful through an occluder.
  map.wall = true;
  selected = refine({short_view}, {0.});
  assert(selected.source == Source::None && selected.point == tour && selected.yaw == 0.);
  map.wall = false;

  // A cluster consumed by the latest map cannot supply observation value.
  map.frontier = false;
  selected = refine({short_view}, {0.});
  assert(selected.source == Source::None && selected.visible == 0);

  aspire_steps::stagedSamples(root, V(10., 1., 8.), [&](const V& candidate) {
    assert((candidate - root).norm() <= 1.6 + 1e-8);
  });
}

void repeated_replacement() {
  aspire_steps::ReplacementHistory history;
  const V root(0., 0., 1.), first(1.2, 0., 1.), nearby(1.2, .1, 1.);
  const double resolution = .30;
  assert(history.accept(root, first, resolution));
  assert(!history.accept(V(.45, 0., 1.), nearby, resolution));
  assert(!history.accept(V(.59, 0., 1.), first, resolution));
  // Both skipped attempts preserve the first root, so this is now far enough.
  assert(history.accept(V(.60, 0., 1.), nearby, resolution));
  assert(!history.accept(V(.61, 0., 1.), first, resolution));
  assert(history.accept(V(.61, 0., 1.), V(1.2, .41, 1.), resolution));
}

void alternate_recovery_goals() {
  const std::vector<V> roots{V(-2.,0.,1.), V(-1.,0.,1.), V(0.,0.,1.)};
  auto eligible = [](const V& p) { return p.x() < 0.; };
  assert(!aspire_steps::recoveryRetreat(roots, 0, eligible));
  assert(aspire_steps::recoveryRetreat(roots, 1, eligible).value() == roots[1]);
  assert(!aspire_steps::recoveryRetreat(roots, 2, eligible));
  assert(aspire_steps::recoveryRetreat(roots, 3, [](const V& p){return p.x() < -1.;}).value() == roots[0]);
  assert(!aspire_steps::recoveryRetreat(roots, 5, [](const V&){return false;}));
}

int main(){
  alternate_recovery_goals();
  V origin(0,0,1),target(3,0,1),a,b;double ha=0,hb=0;
  std::vector<V> fa,fb;
  for(int i=0;i<24;i++){fa.emplace_back(3.2,.5+i*.03,1);fb.emplace_back(3.2,-.5-i*.03,1);}
  Map ma{1},mb{-1};auto yes=[](const V&){return true;};
  assert(aspire_steps::select(ma,origin,target,fa,fa[12],yes,a,ha));
  assert(aspire_steps::select(mb,origin,target,fb,fb[12],yes,b,hb));
  assert(a.y()>.15 && b.y()<-.15 && std::abs(a.x()-b.x())<1e-9 && std::abs(a.y()+b.y())<1e-9);
  ma.closed=true;
  assert(!aspire_steps::select(ma,origin,target,fa,fa[12],yes,a,ha));
  ma.closed=false;
  assert(!aspire_steps::select(ma,origin,target,fa,fa[12],[](const V&){return false;},a,ha));
  imminent_viewpoint();
  repeated_replacement();
  std::cout<<"PASS mirrored geometry, no measured corridor, and all candidates ineligible\n";
}
