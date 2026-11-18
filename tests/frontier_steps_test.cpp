#include "frontier_steps.hpp"
#include <cassert>
#include <iostream>
using aspire_steps::V;
struct Map {
  enum{UNKNOWN,FREE};int sign;bool closed=false;
  double getResolution() {return .15;}
  bool isInBox(const V&p){return p.norm()<10;}
  int getInflateOccupancy(const V&p){return getOccupancy(p)==UNKNOWN;}
  int getOccupancy(const V&p){return closed|| (p.x()>.35 && sign*p.y()<.14) ? UNKNOWN:FREE;}
};
int main(){
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
  std::cout<<"PASS mirrored geometry, no measured corridor, and all candidates ineligible\n";
}
