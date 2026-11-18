#pragma once
// Scheduling admission only. The caller authenticates the current proposal and
// captures its native requested goal; no map/path or execution gate is replaced.
namespace racer_task {
inline bool authorizationRejected(RecoveryPolicy& policy, bool enabled,
    int candidate, int captured_proposal, int review_proposal, int last_rejected,
    const Point& requested_goal, double now) {
  if(!enabled || candidate<0 || candidate!=captured_proposal ||
     candidate==review_proposal || candidate==last_rejected) return false;
  policy.failed(requested_goal,now);
  return true;
}
}
