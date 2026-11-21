"""Local RGB visual judgement, angular evidence and surface localization are independent.

Optional measured depth corroborates short visual associations after camera motion.
Visual IDs are short-time image associations, never verified independent people.
"""
from dataclasses import dataclass, field, asdict
from collections import OrderedDict
import numpy as np
from .confirmation_quality import (box_iou, mask_iou, quality_rejections,
                                   SURFACE_FRACTIONS, validate_thresholds)
from .bearing import bearing_camera, calibrated_bearing, completed_observation
from .perception import validate_transform
from .depth_continuity import measured_support, depth_support_matches


def association_view(k,camera_to_map,width,height):
    """Estimated RGB capture geometry only; no range or depth is inferred."""
    k=np.asarray(k,float).reshape(3,3);t=validate_transform(camera_to_map)
    if not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0 or abs(np.linalg.det(k))<1e-9:
        raise ValueError('invalid_association_intrinsics')
    return dict(k=k.copy(), map_to_pixel=k @ t[:3,:3].T, pixel_to_map=t[:3,:3] @ np.linalg.inv(k),
                rotation=t[:3,:3].copy(),origin=t[:3,3].copy(),width=int(width),height=int(height))


def duplicate_detection(a,b,box_threshold,mask_threshold):
    """Bounding boxes propose duplicates; visible masks must support the claim.

    Legacy box-only detectors retain their original NMS. Missing/invalid masks
    on a segmentation detection are not evidence that two instances coincide.
    """
    if box_iou(a.box,b.box)<box_threshold:return False
    if getattr(a,'class_id',0)!=getattr(b,'class_id',0):return False
    ma,mb=getattr(a,'mask',None),getattr(b,'mask',None)
    if ma is None and mb is None:return True
    if ma is None or mb is None:return False
    overlap = mask_iou(ma, mb)
    return overlap is not None and overlap >= mask_threshold


def rotation_projection(previous, current):
    if previous is None or current is None or previous.get('calibration_id') != current.get('calibration_id'):
        return None
    to_pixel = current.get('map_to_pixel')
    from_pixel = previous.get('pixel_to_map')
    if to_pixel is None:
        to_pixel = current['k'] @ current['rotation'].T
    if from_pixel is None:
        from_pixel = previous['rotation'] @ np.linalg.inv(previous['k'])
    return to_pixel @ from_pixel


def rotated_box(box,previous,current):
    """Rotation-only prediction. Translation can break a link, never adds range."""
    if previous is None and current is None:return tuple(box)
    projection = rotation_projection(previous, current)
    return project_box(box, current, projection) if projection is not None else None


def project_box(box,current,projection):
    x,y,w,h=box
    pixels=np.array([[x,y,1],[x+w,y,1],[x,y+h,1],[x+w,y+h,1]],float).T
    q=projection @ pixels
    if not np.isfinite(q).all() or np.any(q[2]<=1e-6):return None
    uv=(q[:2]/q[2]).T
    lo=np.maximum(uv.min(axis=0),[0,0]);hi=np.minimum(uv.max(axis=0),[current['width'],current['height']])
    if np.any(hi<=lo):return None
    return (*lo,*(hi-lo))


def projected_mask(mask, projection):
    if projection is None:
        return mask
    y,x=np.nonzero(mask);pixels=np.array([x,y,np.ones(len(x))])
    p=projection @ pixels
    valid=np.isfinite(p).all(axis=0)&(p[2]>1e-6);p=p[:,valid]
    uv=p[:2]/p[2];inside=(uv[0]>=0)&(uv[0]<mask.shape[1]-.5)&(uv[1]>=0)&(uv[1]<mask.shape[0]-.5)
    xy=np.floor(uv[:,inside]+.5).astype(int)
    result=np.zeros_like(mask);result[xy[1],xy[0]]=True
    return result


def mask_support(mask):
    if mask is None:
        return None
    mask=np.asarray(mask,bool)
    size=np.count_nonzero(mask)
    return (mask, size) if mask.ndim == 2 and size else None


def mask_overlap(previous_mask,current_mask,previous_view,current_view):
    a,b=mask_support(previous_mask),mask_support(current_mask)
    if a is None or b is None or a[0].shape!=b[0].shape:
        return None
    projection=rotation_projection(previous_view,current_view)
    if projection is None:
        return np.count_nonzero(a[0]&b[0])/min(a[1],b[1])
    image=projected_mask(a[0],projection)
    return np.count_nonzero(image&b[0])/max(1,min(np.count_nonzero(image),b[1]))


def capture_support(hits, capture, confidence, config):
    pruned = [s for s in hits if capture <= s + config.max_gap_s]
    return pruned + [capture] if confidence >= config.min_confidence else pruned


def fresh_rgb(capture, processing, max_age):
    return capture - .05 <= processing <= capture + max_age


@dataclass(frozen=True)
class NavigationConfig:
    """Measurement support only; these gates never confirm a person."""
    min_confidence: float = .50
    min_frames: int = 3
    min_span_s: float = .4
    max_gap_s: float = 2.

    def __post_init__(self):
        validate_thresholds(self, counts={'min_frames': 1}, fractions=('min_confidence',))


@dataclass(frozen=True)
class PresenceConfig:
    min_confidence: float = .70
    min_frames: int = 3
    min_span_s: float = .4
    max_gap_s: float = 2.
    association_gap_s: float = 2.
    association_iou: float = .30
    duplicate_iou: float = .60
    duplicate_mask_iou: float = .70
    lost_age_s: float = 1.5
    lost_max_translation_m: float = .50
    retire_age_s: float = 4.
    active_age_s: float = 2.
    max_capture_age_s: float = 1.5
    max_tracks: int = 128
    def __post_init__(self):
        validate_thresholds(self, counts={'min_frames': 2, 'max_tracks': 1},
            fractions=('min_confidence', 'association_iou', 'duplicate_iou', 'duplicate_mask_iou'))
        if self.retire_age_s<max(self.active_age_s,self.association_gap_s,self.lost_age_s,self.max_capture_age_s):raise ValueError('Retirement must follow all active windows')


@dataclass(frozen=True)
class LocalizationConfig:
    min_depth_pixels: int = 120
    min_valid_depth_fraction: float = .80
    max_depth_span_m: float = .45
    min_boundary_valid_fraction: float = .80
    max_boundary_near_fraction: float = .08
    max_foreground_occlusion_fraction: float = .08
    min_frames: int = 3
    min_span_s: float = .8
    max_gap_s: float = 2.
    max_position_deviation_m: float = .25
    max_age_s: float = 2.
    def __post_init__(self):
        validate_thresholds(self, counts={'min_frames': 2}, fractions=SURFACE_FRACTIONS)
    def evaluate(self,row):
        q=dict(row.get('quality',{}))
        if row.get('localization_state')!='localized_surface' or row.get('xyz') is None:return [row.get('depth_reason') or 'no_surface_measurement'],q
        reasons = quality_rejections(self, q, (
            ('depth_pixels', True), ('valid_depth_fraction', True), ('depth_span_m', False),
            ('boundary_valid_fraction', True), ('boundary_near_fraction', False),
            ('foreground_occlusion_fraction', False)), row)
        xyz=np.asarray(row['xyz'],float)
        if xyz.shape!=(3,) or not np.isfinite(xyz).all():reasons.append('nonfinite_surface_point')
        # Silhouette area and clipping are hints, not evidence that RGB presence is false.
        q['hints']={k:q.get(k) for k in ('mask_box_fraction','border_clipped','visibility_state')}
        return reasons,q


@dataclass
class PresenceTrack:
    id:int
    first_capture_s:float
    last_capture_s:float
    box:tuple
    confidence:float
    last_capture_ns:int=-1
    last_rgb_processing_s:object=None
    visual_state:str='person_candidate'
    visual_reason:str='insufficient_rgb_sequence'
    rgb_hits:list=field(default_factory=list)
    nav_hits:list=field(default_factory=list)
    captures:int=0
    confirmed_capture_s:object=None
    confirmed_processing_s:object=None
    localization_state:str='unknown'
    localization_reasons:list=field(default_factory=lambda:['no_depth_evidence'])
    localization_capture_s:object=None
    localization_processing_s:object=None
    last_localization_ns:int=-1
    localization_good:list=field(default_factory=list)
    position:object=None
    last_reliable_location:object=None
    localization_quality:dict=field(default_factory=dict)
    bearing_state:str='unknown'
    bearing_reason:str='no_capture_transform'
    bearing_capture_ns:int=-1
    bearing_geometry:object=None
    association_view:object=None
    coverage_tokens:set=field(default_factory=set)
    last_observation_event_time:object=None
    last_observation_processing_time:object=None
    lifecycle_state:str='active'
    class_id:int=0
    association_mask:object=None
    paired_association:object=None


class PresenceTracker:
    def __init__(self,visual=None,localization=None,navigation=None):
        self.visual=visual or PresenceConfig();self.localization=localization or LocalizationConfig()
        self.navigation=navigation or NavigationConfig()
        self.tracks=[];self._by_id={};self.confirmed_history={};self.next_id=1
        self.last_rgb_ns=-1;self.last_bearing_ns=-1;self.frames=OrderedDict();self.events=[]
        self.raw_candidates=0;self.frame_candidates=0;self.suppressed_duplicates=0;self.last_suppressed=0
    def event(self,t,kind,capture_s,processing_s,**details):
        self.events.append(dict(event=kind,track_id=t.id,capture_time_s=capture_s,processing_time_s=processing_s,
                               visual_state=t.visual_state,localization_state=t.localization_state,**details))
    def at(self,ident):return self._by_id[ident]
    def retire(self,now):
        """Drop expired working state; keep compact confirmed facts separately."""
        retired={t.id for t in self.tracks if now>t.last_capture_s+self.visual.retire_age_s}
        if not retired:return
        for t in self.tracks:
            if t.id not in retired:continue
            t.position=None;t.localization_state='expired';t.bearing_state='expired'
            t.lifecycle_state='archived';t.rgb_hits=[];t.nav_hits=[]
            if t.visual_state=='visually_confirmed':self.confirmed_history[t.id]=self.report_track(t,now)
            del self._by_id[t.id]
            self.event(t,'source_retired',t.last_capture_s,now,confirmed_fact_retained=t.visual_state=='visually_confirmed')
        self.tracks=[t for t in self.tracks if t.id not in retired]
        # Late geometry must never resolve a retired/reused list index.
        for ns in list(self.frames):
            self.frames[ns]=[a for a in self.frames[ns] if a['track_id'] not in retired]
    def observe_rgb(self,detections,stamp_ns,width,height,processing_s,view=None,paired=None):
        if stamp_ns<=self.last_rgb_ns:return self.frame_rows(stamp_ns,processing_s)
        capture=stamp_ns*1e-9
        if not fresh_rgb(capture,processing_s,self.visual.max_capture_age_s):
            self.events.append(dict(event='rgb_capture_ignored',capture_time_s=capture,processing_time_s=processing_s,reason='stale_or_future_rgb_capture'));return []
        self.retire(capture)
        self.last_rgb_ns=stamp_ns;self.raw_candidates=len(detections);kept=[];self.last_suppressed=0
        for i,d in sorted(enumerate(detections),key=lambda z:(-float(z[1].confidence),tuple(z[1].box))):
            box=tuple(map(float,d.box));x,y,w,h=box
            if not np.isfinite([*box,d.confidence]).all() or w<=0 or h<=0 or x+w<=0 or y+h<=0 or x>=width or y>=height:continue
            if any(duplicate_detection(d,k[1],self.visual.duplicate_iou,self.visual.duplicate_mask_iou) for k in kept):self.last_suppressed+=1;continue
            kept.append((i,d))
        self.suppressed_duplicates+=self.last_suppressed;self.frame_candidates=len(kept);used=set();associations=[]
        # Build all edges before selecting any detection. A greedy high-confidence
        # box must not steal another person's source ID during a camera turn.
        by_detection={};by_track={}
        depth_edges=set()
        masks={i:mask_support(getattr(d,'mask',None)) for i,d in kept}
        projections={}
        for t in self.tracks:
            lost=t.lifecycle_state=='lost'
            gap_ns=stamp_ns-t.last_capture_ns
            if gap_ns>(self.visual.lost_age_s if lost else self.visual.association_gap_s)*1e9:continue
            previous=t.association_view
            translated=(lost and view is not None and previous is not None and
                        np.linalg.norm(view['origin']-previous['origin'])>self.visual.lost_max_translation_m)
            if view is None and previous is None:
                projection=None
                predicted=t.box
            else:
                key=id(previous)
                if key not in projections:projections[key]=rotation_projection(previous,view)
                projection=projections[key]
                if projection is None:continue
                predicted=project_box(t.box,view,projection)
                if predicted is None:continue
            prior_mask=mask_support(t.association_mask)
            projected=None
            for raw_index,d in kept:
                if t.class_id!=getattr(d,'class_id',0) or box_iou(predicted,d.box)<self.visual.association_iou:continue
                current_mask=masks[raw_index]
                has_overlap=(prior_mask is not None and current_mask is not None
                             and prior_mask[0].shape==current_mask[0].shape)
                if has_overlap:
                    if projected is None:
                        image=projected_mask(prior_mask[0],projection)
                        projected=(image, np.count_nonzero(image)) if projection is not None else prior_mask
                    if np.count_nonzero(projected[0]&current_mask[0]) < .10*max(1,min(projected[1],current_mask[1])):continue
                if translated:
                    if paired is None or t.paired_association is None or not has_overlap:continue
                    prior,stamp,points=t.paired_association
                    depth,paired_view=paired
                    if (capture>stamp+self.visual.lost_age_s or
                            np.linalg.norm(paired_view['origin']-prior['origin'])>1.0 or
                            prior.get('calibration_id')!=paired_view.get('calibration_id') or
                            not depth_support_matches(points,d.mask,depth,paired_view)):continue
                    depth_edges.add((raw_index,t.id))
                by_detection.setdefault(raw_index,[]).append(t)
                by_track[t.id]=by_track.get(t.id,0)+1
        for raw_index,d in kept:
            matches=by_detection.get(raw_index,[])
            unique=len(matches)==1 and by_track[matches[0].id]==1
            if unique:t=matches[0]
            else:
                if len(self.tracks)>=self.visual.max_tracks:
                    self.events.append(dict(event='visual_memory_full',capture_time_s=capture,processing_time_s=processing_s));continue
                t=PresenceTrack(self.next_id,capture,capture,tuple(d.box),float(d.confidence),class_id=getattr(d,'class_id',0))
                self.next_id+=1;self.tracks.append(t);self._by_id[t.id]=t
            reason=('unique_rotation_compensated_overlap' if view is not None else 'unique_image_overlap') if unique else ('ambiguous_overlap_new_source' if matches else 'no_continuous_overlap_new_source')
            if unique and (raw_index,t.id) in depth_edges:reason='unique_paired_depth_reacquisition'
            self.event(t,'visual_association',capture,processing_s,association_reason=reason,estimated_rotation_available=view is not None)
            t.association_view=view
            t.association_mask=np.asarray(d.mask,bool).copy() if getattr(d,'mask',None) is not None else None
            if paired is not None:
                depth,paired_view=paired
                points=measured_support(t.association_mask,depth,paired_view)
                if points is not None:t.paired_association=(paired_view,capture,points)
            if t.lifecycle_state=='lost':
                t.rgb_hits=[s for s in t.rgb_hits if capture<=s+self.visual.max_gap_s]
                t.nav_hits=[s for s in t.nav_hits if capture<=s+self.navigation.max_gap_s]
                t.localization_good=[s for s in t.localization_good if capture<=s[0]+self.localization.max_gap_s]
                self.event(t,'lost_source_recovered',capture,processing_s)
            t.lifecycle_state='active'
            used.add(t.id);t.last_rgb_processing_s=processing_s;t.captures+=1;t.last_capture_s=capture;t.last_capture_ns=stamp_ns;t.box=tuple(d.box);t.confidence=float(d.confidence)
            t.nav_hits=capture_support(t.nav_hits,capture,t.confidence,self.navigation)
            associations.append(dict(track_id=t.id,box=t.box,confidence=t.confidence,raw_index=raw_index,
                nav_support_captures=len(t.nav_hits),nav_support_first_s=t.nav_hits[0] if t.nav_hits else capture))
            t.rgb_hits=capture_support(t.rgb_hits,capture,t.confidence,self.visual)
            if t.rgb_hits:
                span=t.rgb_hits[-1]-t.rgb_hits[0]
                if t.visual_state!='visually_confirmed' and len(t.rgb_hits)>=self.visual.min_frames and span+1e-9>=self.visual.min_span_s:
                    t.visual_state='visually_confirmed';t.visual_reason='continuous_trusted_rgb';t.confirmed_capture_s=capture;t.confirmed_processing_s=processing_s
                    self.event(t,'visual_confirmed',capture,processing_s,algorithm_judgement=True,identity_scope='short_time_image_association',rgb_frames=len(t.rgb_hits),rgb_span_s=span)
            elif t.visual_state!='visually_confirmed':t.visual_reason='rgb_confidence_below_gate'
        for t in self.tracks:
            if t.id not in used:
                t.lifecycle_state='lost';t.position=None
                t.localization_state='unknown';t.localization_reasons=['rgb_source_not_observed']
        self.frames[stamp_ns]=associations
        while len(self.frames)>128:self.frames.popitem(last=False)
        return self.frame_rows(stamp_ns,processing_s)
    def pending_localization(self,stamp_ns,processing_s,reason='awaiting_registered_depth'):
        for a in self.frames.get(stamp_ns,[]):
            t=self.at(a['track_id'])
            if stamp_ns<=t.last_localization_ns:continue
            t.localization_state='unknown';t.localization_reasons=[reason];t.localization_capture_s=stamp_ns*1e-9;t.localization_processing_s=processing_s;t.position=None
    def reject_localization(self,stamp_ns,processing_s,reason):
        for a in self.frames.get(stamp_ns,[]):
            t=self.at(a['track_id'])
            if stamp_ns>=t.last_localization_ns:self._reject(t,stamp_ns,processing_s,[reason],{})
    def _reject(self,t,stamp_ns,processing_s,reasons,quality):
        t.last_localization_ns=stamp_ns;t.position=None;t.localization_good=[];t.localization_capture_s=stamp_ns*1e-9;t.localization_processing_s=processing_s;t.localization_reasons=reasons;t.localization_quality=quality
        text=' '.join(reasons)
        t.localization_state=('expired' if any(k in text for k in ('stale','future','expired')) else 'unknown' if any(k in text for k in ('missing','no_depth','no_surface','awaiting')) else 'unreliable')
        self.event(t,'localization_rejected',stamp_ns*1e-9,processing_s,reasons=reasons)
    def localize(self,stamp_ns,rows,processing_s,frame='map',transform_valid=True):
        if not transform_valid or frame!='map':self.reject_localization(stamp_ns,processing_s,'capture_transform_frame_or_time_mismatch');return
        capture=stamp_ns*1e-9
        if not fresh_rgb(capture,processing_s,self.visual.max_capture_age_s):self.reject_localization(stamp_ns,processing_s,'stale_or_future_localization_capture');return
        for a in self.frames.get(stamp_ns,[]):
            t=self.at(a['track_id'])
            if stamp_ns<=t.last_localization_ns:continue
            # Surface rows preserve detector order. Box matching can swap two
            # disjoint masks with identical rectangles after mask-aware NMS.
            i=a['raw_index']
            if i>=len(rows):self._reject(t,stamp_ns,processing_s,['no_surface_measurement'],{});continue
            reasons,quality=self.localization.evaluate(rows[i])
            if reasons:self._reject(t,stamp_ns,processing_s,reasons,quality);continue
            point=np.asarray(rows[i]['xyz'],float)
            good=[p for p in t.localization_good if stamp_ns-p[0]<=self.localization.max_gap_s*1e9]+[(stamp_ns,point.copy())]
            points=np.array([p[1] for p in good]);deviation=float(np.max(np.linalg.norm(points-np.median(points,axis=0),axis=1)))
            if deviation>self.localization.max_position_deviation_m:self._reject(t,stamp_ns,processing_s,['surface_position_instability'],dict(quality,position_deviation_m=deviation));continue
            t.localization_good=good;t.last_localization_ns=stamp_ns;t.localization_capture_s=capture;t.localization_processing_s=processing_s;t.localization_quality=dict(quality,position_deviation_m=deviation,qualified_frames=len(good));t.position=None
            if len(good)>=self.localization.min_frames and stamp_ns-good[0][0]>=self.localization.min_span_s*1e9:
                t.localization_state='reliable';t.localization_reasons=[]
                t.position=dict(xyz=point.tolist(),capture_time_s=capture,processing_time_s=processing_s,frame='map',representative_point='median_visible_mask_surface_not_body_centre',quality=t.localization_quality.copy(),quality_version='surface_depth_sequence_v1')
                t.last_reliable_location=dict(t.position,history_only=True)
                self.event(t,'localization_reliable',capture,processing_s)
            else:t.localization_state='unreliable';t.localization_reasons=['insufficient_localization_sequence']
    def reject_bearing(self,stamp_ns,processing_s,reason):
        if stamp_ns<self.last_bearing_ns:return
        self.last_bearing_ns=stamp_ns
        for a in self.frames.get(stamp_ns,[]):
            t=self.at(a['track_id']);t.bearing_state='unavailable';t.bearing_reason=reason
    def update_bearing(self,detections,k,transform,width,height,stamp_ns,processing_s,execution):
        if stamp_ns<=self.last_bearing_ns:return
        self.last_bearing_ns=stamp_ns
        capture=stamp_ns*1e-9
        rows=self.frames.get(stamp_ns,())
        if not rows:return
        try:camera=bearing_camera(k,transform)
        except ValueError as error:self.reject_bearing(stamp_ns,processing_s,str(error));return
        completed=completed_observation(execution,capture)
        for a in rows:
            t=self.at(a['track_id'])
            try:geom=calibrated_bearing(a['box'],camera,width,height,validated_box=True)
            except ValueError as e:t.bearing_state='unavailable';t.bearing_reason=str(e);continue
            t.bearing_geometry=geom;t.bearing_capture_ns=stamp_ns;t.bearing_state='fresh';t.bearing_reason='capture_rgb_calibration_and_tf'
            if completed is not None:
                token,event=completed
                t.coverage_tokens.add(token);t.last_observation_event_time=event;t.last_observation_processing_time=processing_s
    def expire(self,now):
        for t in self.tracks:
            if t.localization_state=='reliable' and (t.localization_capture_s is None or not t.localization_capture_s<=now<=t.localization_capture_s+self.localization.max_age_s):
                t.localization_state='expired';t.localization_reasons=['localized_capture_expired'];t.position=None;t.localization_good=[]
            if t.bearing_state=='fresh':
                bearing_capture=t.bearing_capture_ns*1e-9
                if not bearing_capture<=now<=bearing_capture+self.visual.active_age_s:
                    t.bearing_state='expired';t.bearing_reason='rgb_bearing_capture_expired'
    def report_track(self,t,now):
        return dict(semantic_state='confirmed' if t.visual_state=='visually_confirmed' else 'candidate',identity_state='local',lifecycle_state=('lost' if t.lifecycle_state=='active' and now-t.last_capture_s>self.visual.active_age_s else t.lifecycle_state),current_surface=t.position,last_reliable_history=t.last_reliable_location,nav_support_captures=len(t.nav_hits),nav_support_first_s=t.nav_hits[0] if t.nav_hits else None,navigation_support_policy='navigation_rgb_v1',id=t.id,visual_track_id=t.id,state=t.visual_state,visual_state=t.visual_state,visual_reason=t.visual_reason,algorithm_judgement=True,identity_scope='short_time_unique_optional_rotation_iou_not_verified_person_identity',box=list(t.box),confidence=t.confidence,capture_time_s=t.last_capture_s,processing_time_s=t.last_rgb_processing_s,published_processing_time_s=now,visual_active=0<=now-t.last_capture_s<=self.visual.active_age_s,capture_count=t.captures,rgb_support_captures=len(t.rgb_hits),visual_confirmed_capture_s=t.confirmed_capture_s,visual_confirmed_processing_s=t.confirmed_processing_s,localization_state=t.localization_state,localization_reasons=list(t.localization_reasons),localization_capture_time_s=t.localization_capture_s,localization_processing_time_s=t.localization_processing_s,position=t.position,xyz=t.position['xyz'] if t.position else None,position_time=t.position['capture_time_s'] if t.position else None,position_frame='map' if t.position else None,position_representation='median_visible_mask_surface_not_body_centre',localization_quality=t.localization_quality,last_reliable_location_history=t.last_reliable_location,bearing_state=t.bearing_state,bearing_reason=t.bearing_reason,bearing_capture_time_s=t.bearing_capture_ns*1e-9 if t.bearing_capture_ns>=0 else None,bearing=t.bearing_geometry if t.bearing_state=='fresh' else None,last_observation_event_time=t.last_observation_event_time,last_observation_processing_time=t.last_observation_processing_time)
    def report(self,now):
        self.expire(now);tracks=[self.report_track(t,now) for t in self.tracks]+list(self.confirmed_history.values())
        return dict(schema_version=2,mode='decoupled_presence',confirmation_policy='decoupled',frame='map',processing_time_s=now,visual_thresholds=asdict(self.visual),navigation_thresholds=asdict(self.navigation),localization_thresholds=asdict(self.localization),counts=dict(raw_rgb_candidates_latest=self.raw_candidates,post_nms_candidates_latest=self.frame_candidates,local_visual_track_records=len(tracks),active_cache_records=len(self.tracks),archived_confirmed_records=len(self.confirmed_history),sources_created_total=self.next_id-1,visually_confirmed_track_records=sum(t.visual_state=='visually_confirmed' for t in self.tracks)+len(self.confirmed_history),reliable_localizations_current=sum(t.localization_state=='reliable' for t in self.tracks),same_frame_duplicates_suppressed_total=self.suppressed_duplicates,independent_true_people=None),count_scope='Visual track records are not independently deduplicated real people',tracks=tracks)
    def frame_rows(self,stamp_ns,now):
        self.expire(now)
        return [dict(self.report_track(self.at(a['track_id']),now),box=list(a['box']),confidence=a['confidence'],candidate_capture_time_s=stamp_ns*1e-9) for a in self.frames.get(stamp_ns,[])]
    def active_bearings(self,now):
        self.expire(now);reports=[]
        for t in self.tracks:
            if t.visual_state=='visually_confirmed' or t.bearing_state!='fresh' or t.localization_state=='reliable' or not t.nav_hits or not t.last_capture_s<=now<=t.last_capture_s+self.visual.active_age_s:continue
            persistent=len(t.nav_hits)>=self.navigation.min_frames and t.nav_hits[-1]-t.nav_hits[0]>=self.navigation.min_span_s-1e-9
            reports.append(dict(cue_id=t.id,source_uav=0,frame='map',capture_time_s=t.bearing_capture_ns*1e-9,processing_time_s=now,first_capture_s=t.first_capture_s,confidence=t.confidence,support_captures=len(t.nav_hits),support_first_s=t.nav_hits[0],support_policy='navigation_rgb_v1',persistent=persistent,observation_coverage=len(t.coverage_tokens),localization_state=t.localization_state,localization_reason=';'.join(t.localization_reasons),localized_surface_xyz=None,semantic_state=t.visual_state,visual_state=t.visual_state,count_scope='visual_angular_track_not_independent_person',**t.bearing_geometry))
        return reports


class PresenceBearings:
    def __init__(self,tracker):self.tracker=tracker
    @property
    def last_ns(self):return self.tracker.last_bearing_ns
    def update(self,*args):return self.tracker.update_bearing(*args)
    def active(self,now):return self.tracker.active_bearings(now)
