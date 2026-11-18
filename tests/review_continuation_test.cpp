#include "review_continuation.hpp"
#include "review_protocol.hpp"
#include <cassert>
#include <iostream>
using racer_task::ReviewContinuation;
int main() {
  using Phase=ReviewContinuation::Phase;
  ReviewContinuation r;
  const ReviewContinuation::Point goal{1.5,0,1.8};
  assert(!r.active() && r.begin(1234567890123456789LL,goal,1.));
  assert(r.attempt(1.) && r.proposed(7,.5) && r.activated(7,{0,0,1.8},1.));
  // A real segment completion outside .25 m retains this request and budget.
  assert(!r.complete(8,true,{.5,0,1.8},true));
  assert(!r.complete(7,false,{.5,0,1.8},true));
  assert(!r.complete(7,true,{.5,0,1.8},false));
  r.observe({.5,0,1.8},2.,true);
  assert(r.complete(7,true,{.5,0,1.8},true));
  assert(r.phase==Phase::queued && r.request_id==1234567890123456789LL && r.received==1. && r.path_used==.5);
  assert(!r.begin(1234567890123456789LL,goal,2.) && r.segment_index==1 && r.path_used==.5);
  assert(r.attempt(3.) && r.proposed(8,.5) && r.activated(8,{.5,0,1.8},3.));
  r.observe({1.,0,1.8},4.,true);
  assert(r.complete(8,true,{1.,0,1.8},true));
  assert(r.phase==Phase::queued && r.path_used==1.);
  assert(r.attempt(5.) && r.proposed(9,.5) && r.activated(9,{1.,0,1.8},5.));
  r.observe({1.251,0,1.8},6.,true);
  assert(r.complete(9,true,{1.251,0,1.8},true));
  assert(!r.active() && r.exit_state=="arrived" && r.segment_index==3);
  assert(!r.begin(1234567890123456789LL,goal,7.));
  assert(r.begin(1234567890123456790LL,goal,7.));
  for(int i=0;i<3;++i) {
    assert(r.attempt(8.+i) && r.proposed(10+i,.1) && r.activated(10+i,{0,0,1.8},8.+i));
    assert(r.complete(10+i,true,{0,0,1.8},true));
  }
  assert(!r.attempt(11.) && r.exit_state=="budget_exhausted" && r.exit_reason=="segment_budget");
  for(int i=0;i<1000;++i)assert(!r.attempt(12.+i*.01));
  assert(r.segment_index==3 && r.received==7.);
  r.finishExit();assert(!r.active());
  assert(r.begin(1234567890123456791LL,goal,20.));
  assert(r.attempt(20.) && r.proposed(20,.5) && r.activated(20,{0,0,1.8},20.));
  assert(r.requestExit("cancelled","direct_confirmed"));
  assert(r.active() && !r.requestExit("timeout","late_timeout"));
  assert(!r.complete(21,true,goal,true));
  assert(r.complete(20,true,goal,true) && r.exit_state=="cancelled");
  assert(r.begin(1234567890123456792LL,goal,30.));
  assert(r.attempt(30.) && r.proposed(21,.5) && r.activated(21,{0,0,1.8},30.));
  r.bounds(90.);assert(r.exit_state=="timeout" && r.active());
  assert(r.complete(21,true,goal,true) && !r.active());
  assert(r.begin(1234567890123456793LL,goal,100.));
  assert(r.attempt(100.) && !r.proposed(30,6.01));
  assert(r.exit_reason=="proposed_path_exceeds_remaining_budget");r.finishExit();
  assert(r.begin(1234567890123456794LL,goal,100.));
  assert(r.attempt(100.) && r.proposed(30,1.) && r.activated(30,{0,0,1.8},100.));
  r.observe({3,0,1.8},101.,true);r.observe({100,0,1.8},101.,true);r.observe({100,0,1.8},102.,false);
  assert(r.path_used==3.);r.observe({0,0,1.8},103.,true);r.bounds(103.);
  assert(r.path_used==6. && r.exit_reason=="actual_path_budget");
  r.complete(30,true,{0,0,1.8},true);assert(!r.active());
  assert(r.begin(1234567890123456795LL,goal,110.));r.bounds(109.);
  assert(r.exit_reason=="clock_reversed_or_invalid");r.finishExit();
  r.max_segments=0;assert(!r.begin(1234567890123456796LL,goal,111.));
  r.max_segments=3;r.max_duration_s=INFINITY;assert(!r.validConfig());
  ReviewContinuation hold;
  assert(hold.begin(1,{0,0,1.8},0.) && hold.attempt(0.) && hold.proposed(1,.1) && hold.activated(1,{0,0,1.8},0.));
  assert(hold.complete(1,true,{0,0,1.8},true) && hold.exit_state=="arrived");
  assert(hold.startObservation(1.) && hold.phase==Phase::holding && hold.received==0.);
  assert(hold.waitingObservation(5.999) && !hold.attempt(2.) && !hold.begin(2,goal,2.));
  assert(!hold.waitingObservation(6.) && hold.exit_state=="released");hold.finishExit();
  assert(!hold.active() && !hold.startObservation(6.));
  hold.max_duration_s=2.;
  assert(hold.begin(2,{0,0,1.8},10.) && hold.attempt(10.) && hold.proposed(2,.1) && hold.activated(2,{0,0,1.8},10.));
  assert(hold.complete(2,true,{0,0,1.8},true) && hold.startObservation(11.5));
  assert(hold.waitingObservation(11.99) && !hold.waitingObservation(12.));
  assert(hold.exit_state=="timeout" && hold.received==10.);hold.finishExit();
  std::int64_t id=0;std::string reason;
  assert(racer_task::parseReviewCancel("{\"request_id\":\"1234567890123456789\",\"reason\":\"direct_confirmed\"}",id,reason));
  assert(id==1234567890123456789LL && reason=="direct_confirmed");
  assert(racer_task::parseReviewCancel("{ \"reason\": \"target_invalid\", \"request_id\": \"12\" }",id,reason));
  for(const char* bad:{"{}","{\"request_id\":12,\"reason\":\"cancelled\"}",
      "{\"request_id\":\"1e9\",\"reason\":\"cancelled\"}",
      "{\"request_id\":\"9223372036854775808\",\"reason\":\"cancelled\"}",
      "{\"request_id\":\"1\",\"request_id\":\"2\"}",
      "{\"request_id\":\"1\",\"reason\":\"cancelled\"}garbage"})
    assert(!racer_task::parseReviewCancel(bad,id,reason));
  assert(racer_task::reviewJsonString("a\"\n")=="\"a\\\"\\u000a\"");
  std::cout<<"PASS_REVIEW_POLICY same_request_three_segments=1 budgets_no_reset=1 fresh_matching_done=1 exact_integer_identity=1 deferred_cancel=1\n";
}
