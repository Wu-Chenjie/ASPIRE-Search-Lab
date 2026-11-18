#include "recovery_policy.hpp"
#include "authorization_recovery.hpp"
#include <cassert>
#include <iostream>
using namespace racer_task;
int main() {
  RecoveryPolicy policy;
  const Point root={0,0,1.8}, requested={5,0,1.8}, prefix_end={1,0,1.8};
  assert(!authorizationRejected(policy,false,1,1,-1,-1,requested,0));
  assert(!authorizationRejected(policy,true,1,2,-1,-1,requested,0));
  assert(!authorizationRejected(policy,true,1,1,1,-1,requested,0));
  assert(policy.failures()==0);
  int last=-1;
  for(int proposal=1;proposal<=3;++proposal) {
    assert(authorizationRejected(policy,true,proposal,proposal,-1,last,requested,proposal));
    last=proposal;
    for(int repeat=0;repeat<100;++repeat)
      assert(!authorizationRejected(policy,true,proposal,proposal,-1,last,requested,proposal));
    assert(policy.failures()==proposal);
    assert(policy.active()==(proposal==3));
  }
  assert(!policy.eligible(requested,root,4));
  assert(policy.eligible(prefix_end,root,4)); // Blacklist the requested goal, not the native prefix end.
  int attempts=0;
  for(int i=0;i<9;++i) {
    const double now=i<8?4.+i*.5:12.5;
    assert(policy.ready(now));
    Point goal={10.+i,0,1.8};policy.attempted(goal,now);++attempts;
    assert(policy.active()); // Successful native generation is not activation.
    int proposal=4+i;
    assert(authorizationRejected(policy,true,proposal,proposal,-1,last,goal,now));last=proposal;
    assert(!policy.ready(now+.49));
    if(i>=7)assert(!policy.ready(now+4.99));
  }
  assert(attempts==9 && policy.attempts()==9 && policy.failures()==12);
  policy.activated();assert(!policy.active() && policy.attempts()==0 && policy.failures()==0);
  std::cout<<"PASS_AUTHORIZATION_RECOVERY trigger=3 recovery_attempts=9 duplicate_feedback=300 activation_only_reset=1\n";
}
