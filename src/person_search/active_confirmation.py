"""Opt-in direct/active confirmation. No simulator or semantic truth inputs.

Repeated RGB classification is fallible: motion is evidence diversity, not an
independent proof of person identity. Legacy Tracker remains unchanged.
"""
from dataclasses import asdict, dataclass, field
import numpy as np
from .tracking import Track, nearest_track
from .confirmation_quality import ConfirmationQuality, SemanticCandidateMemory, QUALITY_PROXY_VERSION, mask_iou


def current_execution_stamp(execution, key, now):
    stamp=execution.get(key, -1e9)
    return stamp <= now < stamp + .5


def execution_valid(execution, now):
    return (execution.get('ready') and not execution.get('flight_session_fault')
            and current_execution_stamp(execution, 'time', now))


def review_execution_ready(execution, now):
    return execution_valid(execution, now) and execution.get('arrived')


def review_pose(target, goal):
    """Convert the selected lens position to the commanded base pose."""
    yaw = float(np.arctan2(target[1]-goal[1], target[0]-goal[0]))
    base = np.asarray(goal) - [.13233*np.cos(yaw), .13233*np.sin(yaw), .02078]
    return base, yaw


def observed_review_view(track, origin):
    goal_offset=origin-track.request_goal
    if not goal_offset @ goal_offset <= .4**2:
        return False
    baseline=origin[:2]-track.request_origin[:2]
    return baseline @ baseline >= .5**2


def suppress_same_frame_duplicates(observations, min_mask_iou=.7, max_surface_distance=.15):
    """Suppress only strongly overlapping instance support at the same surface.

    Nearby people with separate masks remain separate. This filters tracker
    input, not detector output, thresholds, RGB overlays, or stored raw masks.
    """
    observations=list(observations)
    kept=[];suppressed=[]
    for index,candidate in sorted(((i, c) for i, c in enumerate(observations) if c.semantic),
                                  key=lambda item: -item[1].confidence):
        duplicate=None
        for kept_index,other in kept:
            distance=float(np.linalg.norm(candidate.xyz-other.xyz))
            if not distance <= max_surface_distance:continue
            iou=mask_iou(candidate.pixels,other.pixels)
            if iou is not None and iou>=min_mask_iou:
                duplicate=dict(source_index=index,source_confidence=float(candidate.confidence),
                    kept_index=kept_index,kept_confidence=float(other.confidence),mask_iou=iou,surface_distance_m=distance)
                break
        if duplicate is None:kept.append((index,candidate))
        else:suppressed.append(duplicate)
    # Preserve original ordering among distinct people to minimize associations
    # changing merely because classification scores fluctuate.
    kept_indices = {index for index, _ in kept}
    return [c for i, c in enumerate(observations) if not c.semantic or i in kept_indices], suppressed


@dataclass
class ReviewTrack(Track):
    eligible_since: float | None = None
    reason: str = 'awaiting_ready_and_persistent_visual_evidence'
    hits: list = field(default_factory=list)
    request_time: float | None = None
    request_origin: list | None = None
    request_goal: list | None = None
    request_base: list | None = None
    previous_token: object = None
    moving_token: object = None
    completed_token: object = None
    completed_time: float | None = None
    post_hits: list = field(default_factory=list)
    next_attempt: float = 0.
    qualified_capture_time: float | None = None
    request_id: str | None = None
    native_status: dict = field(default_factory=dict)
    direct_hits: list = field(default_factory=list)
    quality_evidence: dict = field(default_factory=dict)
    rejection_reasons: list = field(default_factory=list)
    last_quality_good: bool = False
    confirmed_time: float | None = None
    review_closed_time: float | None = None
    review_closed_reason: str | None = None
    arrival_event_time: float | None = None


class ActiveConfirmationTracker:
    """Strict mode requires completion-matched, post-motion RGB-D evidence.

    Hybrid mode also permits reliable continuous RGB-D at one view, including
    startup observation and during a review. It uses explicit quality proxies,
    never treats missing localization as negative semantic evidence, and does
    not claim the visible-surface estimate is a calibrated person-centre bound.

    Three localized semantic captures within 5 s latch a stationary visual cue.
    It may wait at most 30 s for a safe execution boundary; this memory only
    authorizes a review request, never confirmation.
    Time limits use the ROS simulation clock. TTL begins at flight ready, not
    first startup image. Unresolved is terminal for active requests, never 'no
    person'. Hybrid passive confirmation can still use a completely fresh,
    qualified sequence after closure, without reopening any review budget.
    """
    def __init__(self, radius=.65, max_attempts=3, ttl=90., execution_timeout=25., observation_timeout=8.,
                 mode='strict', quality=None, require_request_status=False):
        if mode not in ('strict','hybrid'):raise ValueError('Unknown confirmation mode')
        self.radius=radius; self.max_attempts=max_attempts; self.ttl=ttl
        self.execution_timeout=execution_timeout; self.observation_timeout=observation_timeout
        self.tracks=[]; self.last_time=-float('inf'); self.ready_time=None
        self.events=[];self.cancellations=[];self.mode=mode;self.last_rejected_capture=-float('inf');self.flight_session_fault=None
        self.quality=quality or ConfirmationQuality();self.require_request_status=require_request_status
        self.semantic_memory=SemanticCandidateMemory()

    def event(self, track, now, reason, **extra):
        track.reason=reason
        self.events.append(dict(time=now,track=track.id,state=track.state,reason=reason,attempt=track.attempts,
                                request_id=track.request_id,**extra))

    def cancel_request(self, track, reason):
        if track.request_id is not None:
            self.cancellations.append(dict(request_id=track.request_id,reason=reason))

    def review_status(self, status, now):
        """Accept only a fresh, exact request ID and matching commanded goal."""
        request_id=status.get('request_id')
        if not isinstance(request_id,str) or not request_id.isdecimal():return False
        for t in self.tracks:
            if t.request_id!=request_id or t.state!='reviewing':continue
            stamp=status.get('time',-1e9)
            if not stamp<=now<=stamp+1. or stamp<t.native_status.get('time',-1):return False
            if status==t.native_status:return True
            try:distance=float(np.linalg.norm(np.asarray(status['goal'],float)-t.request_base))
            except (KeyError,ValueError,TypeError):return False
            if not distance<=1e-4:return False
            t.native_status=dict(status)
            self.event(t,now,'native_review_status',native_status=dict(status))
            if status.get('state') in ('cancelled','timeout','unreachable','invalid','budget_exhausted','rejected'):
                self.fail_attempt(t,now,'native_review_'+status['state'])
            return True
        return False

    def fail_attempt(self, track, now, reason):
        self.cancel_request(track,reason)
        if track.attempts >= self.max_attempts or reason=='native_review_budget_exhausted':
            self.close_review_budget(track,now,reason)
        else:
            track.state='tentative';track.next_attempt=now+3.
        track.request_time=None; track.completed_time=None; track.post_hits=[]
        self.event(track,now,reason)

    @staticmethod
    def close_review_budget(track, now, reason):
        track.state='unresolved';track.review_closed_time=now;track.review_closed_reason=reason
        # A later passive confirmation must not recycle evidence that preceded
        # the terminal active state. Counts and eligible_since stay untouched.
        track.direct_hits=[];track.last_quality_good=False;track.hits=[]

    def advance(self, now, execution):
        valid=execution_valid(execution, now)
        self.flight_session_fault=execution.get('flight_session_fault')
        if valid and self.ready_time is None:self.ready_time=now
        for t in self.tracks:
            if t.state in ('confirmed','unresolved'):continue
            if self.ready_time is not None and t.eligible_since is None:
                t.eligible_since=max(self.ready_time,t.first_seen)
            closure=None
            if self.flight_session_fault:
                closure='flight_fault'
            elif t.eligible_since is not None and now>t.eligible_since+self.ttl:
                closure='review_budget_expired'
            if closure:
                self.cancel_request(t,closure);self.close_review_budget(t,now,closure);self.event(t,now,closure);continue
            if t.state=='reviewing':
                if now>t.request_time+self.execution_timeout:
                    self.fail_attempt(t,now,'no_matched_execution_before_timeout');continue
                token=execution.get('token')
                if not valid or token is None:continue
                status=t.native_status
                plan_id=status.get('native_plan_id')
                explicit=self.require_request_status
                if explicit and (plan_id is None or execution.get('native_plan_id')!=plan_id):continue
                if not explicit and token==t.previous_token:continue
                extra={'native_plan_id':plan_id} if explicit else {}
                if not execution.get('arrived'):
                    if token!=t.moving_token:
                        t.moving_token=token
                        self.event(t,now,'new_execution_after_request_observed',token=token,**extra)
                    continue
                proof=execution.get('observation_proof')
                if not proof:continue
                if explicit:
                    arrival=status.get('time',-1)
                    if not (status.get('state')=='arrived' and proof.get('token')==token
                            and proof.get('native_plan_id')==plan_id and arrival>=t.request_time
                            and current_execution_stamp(execution,'position_stamp',now)
                            and np.linalg.norm(np.asarray(execution.get('actual_position',[1e9]*3))-t.request_base)<=.25):continue
                    extra.update(native_arrival_time=arrival,arrival_event_time=arrival,
                                 observing_started_time=now,processing_time_s=now)
                elif token!=t.moving_token:continue
                if not np.linalg.norm(np.asarray(execution.get('reference',[1e9]*3))-t.request_base)<=.25:continue
                t.completed_token=token;t.completed_time=now;t.state='observing'
                t.arrival_event_time=extra.get('arrival_event_time')
                self.event(t,now,'execution_completed_waiting_for_fresh_visual_evidence',token=token,**extra)
            elif t.state=='observing' and now>t.completed_time+self.observation_timeout:
                self.fail_attempt(t,now,'no_fresh_matched_visual_confirmation')
        return valid

    def reject_capture(self, now, reason):
        """An intervening missing/invalid RGB-D capture breaks direct evidence."""
        if self.mode!='hybrid' or now<=max(self.last_time,self.last_rejected_capture):return
        self.last_rejected_capture=now
        for t in self.tracks:
            if t.state=='confirmed':continue
            t.direct_hits=[];t.last_quality_good=False;t.rejection_reasons=[reason]
            self.event(t,now,'direct_confirmation_rejected',rejections=[reason])

    def direct_evidence(self,t,c,origin,now):
        reasons,evidence=self.quality.evaluate(c)
        if self.flight_session_fault:reasons.append('flight_session_fault')
        if now<=self.last_rejected_capture:reasons.append('capture_precedes_newer_rejection')
        if t.review_closed_time is not None and now<=t.review_closed_time:reasons.append('capture_not_after_review_closure')
        if t.direct_hits and now>t.direct_hits[-1]['time']+self.quality.max_gap_s:
            t.direct_hits=[]
        if reasons:t.direct_hits=[]
        else:
            t.direct_hits.append(dict(time=now,xyz=c.xyz.tolist(),origin=origin.tolist(),quality=evidence))
            points=np.array([h['xyz'] for h in t.direct_hits])
            median = np.median(points, axis=0)
            deviation = float(np.max(np.linalg.norm(points - median, axis=1)))
            evidence['position_deviation_m']=deviation
            if deviation>self.quality.max_position_deviation_m:
                reasons.append('position_stability_gate');t.direct_hits=t.direct_hits[-1:]
        t.last_quality_good=not reasons;t.quality_evidence=evidence;t.rejection_reasons=reasons
        if reasons:
            self.event(t,now,'direct_confirmation_rejected',rejections=reasons,evidence=evidence)
            return None
        first=t.direct_hits[0]['time']
        span=now-first
        enough=len(t.direct_hits)>=self.quality.min_frames and now>=first+self.quality.min_span_s
        if not enough:self.event(t,now,'direct_confirmation_collecting',frames=len(t.direct_hits),span_s=span,evidence=evidence)
        return median if enough else None

    def update(self, observations, origin, now, capture_context=None):
        if not np.isfinite(now) or now<self.last_time:raise ValueError('Nonmonotonic observation time')
        if now==self.last_time:return self.tracks
        self.last_time=now;origin=np.asarray(origin);used=set()
        observations,suppressed=suppress_same_frame_duplicates(observations)
        if suppressed:
            self.events.append(dict(time=now,track=0,attempt=0,state='observation_filter',
                reason='same_frame_duplicate_suppressed',suppressed_count=len(suppressed),suppressed=suppressed))
        for c in observations:
            if not c.semantic:continue  # geometry is not a person cue
            t=nearest_track(self.tracks,c.xyz,used,self.radius)
            if t is not None:
                # Confirmed coordinates and their evidence form one snapshot.
                # Later detections remain visible in semantic candidates, but
                # must not rewrite this point under an older quality claim.
                if t.state!='confirmed':t.xyz=(t.xyz*t.observations+c.xyz)/(t.observations+1)
                t.observations+=1
            else:
                t=ReviewTrack(len(self.tracks)+1,c.xyz.copy(),now,now);self.tracks.append(t)
            used.add(t.id);t.last_seen=now
            if t.state=='confirmed':
                if self.mode=='hybrid':
                    rejected,quality=self.quality.evaluate(c)
                    self.events.append(dict(time=now,track=t.id,state=t.state,attempt=t.attempts,request_id=t.request_id,
                        reason='confirmed_position_frozen',observed_xyz=c.xyz.tolist(),observed_quality=quality,
                        observed_rejections=rejected,accepted_into_confirmed_position=False,
                        confirmed_position=t.xyz.tolist(),confirmed_time=t.confirmed_time))
                continue
            if t.state!='unresolved':
                t.hits=[stamp for stamp in t.hits if now<=stamp+5.]+[now]
                if len(t.hits)>=3 and now>=t.hits[0]+.4:
                    if t.qualified_capture_time is None and t.state=='tentative':
                        self.event(t,now,'persistent_visual_cue_queued',qualified_capture_time=now,
                                   uncertainty='requires_executed_view_and_fresh_confirmation')
                    t.qualified_capture_time=now
            if all(np.linalg.norm(origin[:2]-np.asarray(p)[:2])>=.5 for p in t.semantic_views):
                t.semantic_views.append(origin.tolist())
            if self.mode=='hybrid':
                median = self.direct_evidence(t,c,origin,now)
                if median is not None:
                    prior_state=t.state;t.state='confirmed'
                    t.xyz = median
                    t.confirmed_time=now
                    arrival = t.arrival_event_time if t.arrival_event_time is not None else t.completed_time
                    fresh_post = (prior_state == 'observing' and all(
                        h['time'] > arrival and observed_review_view(t, np.asarray(h['origin']))
                        for h in t.direct_hits))
                    reason='active_multiview_confirmed' if fresh_post else 'reliable_direct_confirmed'
                    self.event(t,now,reason,confirmation_path='active_review' if fresh_post else 'direct',
                               during_review=prior_state in ('reviewing','observing'),token=t.completed_token if fresh_post else None,
                               is_before_ready=self.ready_time is None or now<self.ready_time,
                               after_review_closed=prior_state=='unresolved',review_closed_time=t.review_closed_time,
                               review_closed_reason=t.review_closed_reason,
                               capture_times=[h['time'] for h in t.direct_hits],post_capture_times=[h['time'] for h in t.direct_hits] if fresh_post else [],
                               capture_origin=origin.tolist(),goal=t.request_goal,evidence=t.direct_hits.copy(),
                               quality_thresholds=asdict(self.quality),
                               quality_proxy_version=QUALITY_PROXY_VERSION,
                               confirmed_position=t.xyz.tolist(),confirmed_time=t.confirmed_time,
                               position_representation='visible_surface_not_body_centre')
                    if fresh_post:t.post_hits=[h['time'] for h in t.direct_hits]
                    if prior_state in ('reviewing','observing'):self.cancel_request(t,reason)
                continue
            if t.state!='observing' or now<=t.completed_time:continue
            # Capture-time estimated optical pose, not command/reference pose.
            if not observed_review_view(t, origin):continue
            t.post_hits.append(now)
            if now>=t.post_hits[0]+.4:
                t.state='confirmed';t.confirmed_time=now;self.event(t,now,'active_multiview_confirmed',token=t.completed_token,
                    goal=t.request_goal,capture_origin=origin.tolist(),post_capture_times=t.post_hits.copy())
                self.cancel_request(t,'active_multiview_confirmed')
        if self.mode=='hybrid':
            for t in self.tracks:
                if t.id not in used and t.state!='confirmed':
                    before=len(t.direct_hits)
                    t.direct_hits=[];t.last_quality_good=False
                    reason=('not_observed_in_capture' if capture_context is None else 'no_semantic_candidate_in_capture' if not capture_context['semantic_candidate_count'] else 'no_localized_match_semantic_candidate_retained')
                    t.rejection_reasons=[reason]
                    if before and capture_context is not None:self.event(t,now,'direct_sequence_reset',before_frames=before,after_frames=0,reset_reason=reason,capture_context=capture_context)
        return self.tracks

    def eligible(self, track, now, execution):
        return (review_execution_ready(execution, now)
                and not any(t.state in ('reviewing','observing') for t in self.tracks)
                and track.state=='tentative' and track.attempts<self.max_attempts
                and not (self.mode=='hybrid' and track.last_quality_good and track.last_seen<=now<=track.last_seen+self.quality.max_gap_s)
                and now>=track.next_attempt and track.qualified_capture_time is not None
                and track.qualified_capture_time<=now<=track.qualified_capture_time+30.)

    def review(self, track, origin, goal, now, execution, request_id=None):
        if not self.eligible(track,now,execution):return False
        if np.linalg.norm(np.asarray(goal)[:2]-np.asarray(origin)[:2])<.8:return False
        track.attempts+=1;track.state='reviewing';track.request_time=now
        track.request_id=str(request_id) if request_id is not None else str(int(round(now*1e9)))
        track.native_status={}
        track.request_origin=np.asarray(origin).tolist();track.request_goal=np.asarray(goal).tolist()
        base, _ = review_pose(track.xyz, goal)
        track.request_base = base.tolist()
        track.previous_token=execution.get('token');track.moving_token=None
        track.completed_time=None;track.arrival_event_time=None;track.completed_token=None;track.post_hits=[]
        self.event(track,now,'visual_candidate_requested_safe_view',goal=track.request_goal,previous_token=track.previous_token,
                   request_origin=track.request_origin,request_base=track.request_base,
                   qualified_capture_time=track.qualified_capture_time,cue_age_s=now-track.qualified_capture_time,
                   uncertainty='historical_visual_position_requires_fresh_confirmation')
        return True
