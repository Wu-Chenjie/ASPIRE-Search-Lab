#include "return_policy.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using racer_task::ReturnPolicy;
using racer_task::Point;
int main() {
  const Point home={3.,2.,1.5}, away={0.,2.,1.5};
  ReturnPolicy normal;normal.begin(home);assert(normal.attempt(1.,false));normal.proposed(10);
  assert(!normal.attempt(10.,false)); // Pending authorization owns the proposal.
  assert(!normal.activated(9));assert(normal.activated(10));
  assert(!normal.activated(10));assert(!normal.rejected(10,1.6));
  // Planned endpoint at home is irrelevant without the matching physical proof.
  assert(!normal.observe(10,false,home,true));assert(!normal.observe(9,true,home,true));
  assert(!normal.observe(10,true,home,false));
  assert(!normal.observe(10,true,{NAN,2,1.5},true));
  assert(normal.observe(10,true,away,true));assert(normal.state()==ReturnPolicy::State::planning_home);
  assert(normal.attempt(2.,false));normal.proposed(11);normal.activated(11);
  assert(normal.observe(11,true,{3.07,2.,1.5},true));assert(normal.state()==ReturnPolicy::State::returned);
  assert(!normal.attempt(100.,true));

  ReturnPolicy recovery;recovery.begin(home);
  for(int i=0;i<3;++i)assert(recovery.attempt(i,false)); // failed primary calls
  assert(!recovery.attempt(2.1,false)); // no retry flood
  assert(recovery.attempt(3.,true));recovery.proposed(20);recovery.activated(20);
  assert(recovery.attempts()==4); // Activation preserves mission accounting.
  assert(recovery.observe(20,true,away,true));assert(recovery.home()==home);
  assert(recovery.state()==ReturnPolicy::State::planning_home);
  assert(!recovery.observe(20,true,home,true)); // duplicate old completion
  recovery.begin(away);assert(recovery.home()==home);assert(recovery.attempts()==4);
  for(int i=4;i<8;++i) {
    assert(recovery.attempt(i,i%2));recovery.proposed(20+i);recovery.activated(20+i);
    assert(recovery.observe(20+i,true,away,true));
  }
  assert(!recovery.mayPlan(8.99));
  for(int i=0;i<100;++i){recovery.begin(away);assert(!recovery.attempt(8.+i*.005,true));}
  assert(recovery.attempt(9.,false) && recovery.attempts()==9);
  recovery.proposed(40);assert(recovery.activated(40));
  assert(recovery.observe(40,true,home,true) && recovery.state()==ReturnPolicy::State::returned);

  ReturnPolicy unavailable;unavailable.begin(home);assert(unavailable.attempt(0,true));
  unavailable.noGoal(.1);assert(unavailable.state()==ReturnPolicy::State::waiting_no_goal);
  assert(!unavailable.terminal() && !unavailable.mayUseTemporary());
  assert(!unavailable.attempt(5.,false));assert(unavailable.attempt(5.1,false));
  unavailable.proposed(4);assert(unavailable.activated(4));
  assert(!unavailable.stopped(3,6.) && unavailable.executing());
  assert(unavailable.stopped(4,6.));
  assert(!unavailable.observe(4,true,home,true));
  assert(!unavailable.activated(4));
  assert(!unavailable.stopped(4,6.1));assert(unavailable.mayPlan(6.5));
  ReturnPolicy rejected;rejected.begin(home);
  for(int i=0;i<8;++i) {
    assert(rejected.attempt(i,false));
    rejected.proposed(i);assert(rejected.rejected(i,i+.1));
    assert(!rejected.rejected(i,i+1.));
  }
  assert(rejected.attempts()==8 && rejected.state()==ReturnPolicy::State::waiting_retry);
  assert(!rejected.terminal() && !rejected.attempt(9.05,false));
  assert(rejected.attempt(9.11,false));
  // Session/failsafe retirement is irreversible; fresh input cannot reflight.
  ReturnPolicy lost;lost.begin(home);lost.attempt(0,false);lost.proposed(42);lost.activated(42);lost.retire();
  lost.begin(away);assert(!lost.activated(42));assert(!lost.observe(42,true,home,true));assert(!lost.attempt(100,false));
  assert(lost.state()==ReturnPolicy::State::retired);
  ReturnPolicy temporary_home;temporary_home.begin(home);temporary_home.attempt(0,true);
  temporary_home.proposed(7);temporary_home.activated(7);temporary_home.observe(7,true,home,true);
  assert(temporary_home.state()==ReturnPolicy::State::planning_home);
  std::cout<<"PASS: fixed home, actual completion, verified stops, retry backoff and retirement\n";
}
