#include "recovery_policy.hpp"
#include <cassert>
#include <iostream>
using namespace racer_task;
int main() {
  const Point root={.127554,2.07747,.756059},failed={-.55,2.08,.81};
  RecoveryPolicy p;
  assert(!p.active());p.failed(failed,0);p.failed(failed,.01);assert(!p.active());
  p.failed(failed,.02);assert(p.active());assert(!p.eligible(failed,root,1));
  assert(!p.eligible({-.54,2.07,.81},root,1));assert(p.eligible({1.5,2,1.5},root,1));
  assert(!p.eligible(root,root,1));assert(!p.eligible({NAN,2,1},root,1));
  const Point previous={root[0]+.276,root[1],root[2]};
  assert(!p.eligible(previous,root,1));
  assert(p.eligible(previous,root,1,.15));
  assert(!p.eligible(root,root,1,.15));
  assert(!p.eligible(failed,root,1,.15));
  // Historical 4845 failure callbacks must reach the 5 s retry rate.
  // Simulate 50 Hz scheduling for 100 s without an environmental change.
  int attempts=0;
  double last_attempt=-1.;
  for(int i=0;i<5000;++i) {
    double now=.04+i*.02;
    if(p.ready(now)) {
      if(attempts>=8)assert(now-last_attempt>=2.-1e-9);
      last_attempt=now;
      Point goal={2.+attempts,2.,1.5};p.attempted(goal,now);p.failed(goal,now);++attempts;
    }
  }
  assert(attempts>8 && attempts<=58);assert(p.attempts()==attempts);
  p.activated();p.failed(failed,111);p.failed(failed,112);p.failed(failed,113);
  assert(p.ready(114));assert(p.attempts()==0);
  p.attempted({2,2,1.5},114);p.hold(114);
  assert(!p.ready(118.99));assert(p.ready(119.));
  // Generation alone must not clear recovery; rejection keeps it active.
  p.attempted({2,2,1.5},126);assert(p.active());p.failed({2,2,1.5},126.1);assert(p.active());
  p.activated();assert(!p.active());assert(p.failures()==0);
  p.failed(failed,200);p.failed(failed,201);p.failed(failed,202);
  assert(p.ready(203));p.hold(203);
  assert(!p.ready(207.99));assert(p.ready(208));
  assert(!p.eligible(failed,root,208));
  assert(p.eligible(failed,root,220));
  // Real v3 map-delay-10: three recovery attempts then all goals cooling
  // down. The unchanged-map hold must reopen when a blacklist expires,
  // without treating that expiry as activation or resetting attempts.
  RecoveryPolicy cooldown;
  cooldown.failed(failed,0);cooldown.failed(failed,.01);cooldown.failed(failed,.02);
  for(int i=0;i<3;++i) {
    assert(cooldown.ready(1.+i));
    cooldown.attempted({2.+i,2.,1.5},1.+i);
  }
  cooldown.hold(4.);
  assert(!cooldown.ready(8.99));
  // A remote route improvement may leave both the root and goal unchanged.
  assert(cooldown.ready(16.));
  assert(cooldown.attempts()==3 && cooldown.active());
  for(int i=0;i<5;++i) {
    assert(cooldown.ready(17.+i));
    cooldown.attempted({8.+i,2.,1.5},17.+i);
  }
  assert(cooldown.attempts()==8);
  assert(!cooldown.ready(22.99));assert(cooldown.ready(23.));
  assert(!cooldown.ready(NAN));
  // Review still uses its own age and measured endpoint checks.
  int map_reads=0;
  auto free_endpoint=[&](){++map_reads;return true;};
  assert(!reviewReady(false,true,false,10.,9.,free_endpoint));assert(map_reads==0);
  assert(!reviewReady(true,false,false,10.,9.,free_endpoint));assert(map_reads==0);
  assert(!reviewReady(true,true,true,10.,9.,free_endpoint));
  assert(reviewReady(true,true,false,10.,7.,free_endpoint)); // Old 3 s expiry no longer loses a queued request.
  assert(reviewReady(true,true,false,63.152,59.5,free_endpoint)); // Recorded first execution completed at 63 s.
  assert(reviewReady(true,true,false,69.499,59.5,free_endpoint));
  assert(reviewReady(true,true,false,80.612,69.896,free_endpoint)); // Real 10.716 s safe-boundary wait.
  assert(reviewReady(true,true,false,89.895,69.896,free_endpoint));
  assert(!reviewReady(true,true,false,89.896,69.896,free_endpoint));
  assert(!reviewReady(true,true,false,90.,69.896,free_endpoint));
  assert(!reviewReady(true,true,false,20.,0.,free_endpoint));
  assert(!reviewReady(true,true,false,10.,11.,free_endpoint));
  assert(!reviewReady(true,true,false,NAN,9.,free_endpoint));
  assert(!reviewReady(true,true,false,10.,9.,[](){return false;}));
  assert(reviewReady(true,true,false,10.,9.,free_endpoint));
  assert(cooldown.attempts()==8 && cooldown.active());
  std::cout<<"PASS: rate-limited historical failure flood, cooldown, hold, activation-only reset\n";
}
