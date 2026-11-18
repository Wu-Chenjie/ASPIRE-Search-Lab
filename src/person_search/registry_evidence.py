"""Perception-to-C++ observations only; identity association remains native."""
import uuid
from collections import OrderedDict, defaultdict
from dataclasses import dataclass, field
import numpy as np
from .surface import SurfaceFusion, has_surface_measurement

@dataclass
class SourceEvidence:
    fusion: SurfaceFusion = field(default_factory=SurfaceFusion)
    capture_ns: int = -1
    state: str | None = None
    calibration: str | None = None
    identity_surface: tuple | None = None
    surface_quality: dict | None = None
    sequence: int = 0


def surface_radius(covariance):
    radius = 3 * float(np.sqrt(np.linalg.eigvalsh(np.asarray(covariance, float)).max()))
    return radius if .05 <= radius <= .4 else None


class RegistryEvidence:
    def __init__(self,robot='uav0',session=None,frame_version=None,shared_aligned=False,clock_domain='mission'):
        self.robot=robot;self.session=session or str(uuid.uuid4())
        self.frame_version=frame_version or ('local-'+robot+'-'+self.session)
        if not clock_domain:raise ValueError('Clock domain required')
        self.clock_domain=clock_domain
        self.shared_aligned=bool(shared_aligned)
        self.sources=defaultdict(SourceEvidence);self.last_qualified={};self.signatures=OrderedDict()
        self.last_visibility_ns=-1;self.visibility_cursor=0

    def capture(self,tracker,ns,rows,processing):
        self._update(tracker, ns, processing, 'pending' if rows is None else 'completed', rows)

    def reject(self,tracker,ns,processing,reason):
        self._update(tracker, ns, processing, 'unavailable', reason=reason)

    def _update(self, tracker, ns, processing, state, rows=None, reason=None):
        capture = ns * 1e-9
        for association in tracker.frames.get(ns, ()):
            ident = association['track_id']
            source = self.sources[ident]
            if ns < source.capture_ns or (ns == source.capture_ns and (
                    state == 'pending' or state == source.state == 'completed')):
                continue
            source.capture_ns = ns
            source.state = state
            source.identity_surface = source.surface_quality = None
            fusion = source.fusion
            if state == 'pending':
                fusion.current = None
                fusion.state = 'unknown'
                fusion.reasons = ['awaiting_registered_surface']
            elif state == 'unavailable':
                fusion.reject(reason, preserve_qualified=reason == 'stale_after_inference')
            else:
                index = association['raw_index']
                row = rows[index] if index < len(rows) else None
                quality = (row or {}).get('quality', {})
                calibration = quality.get('registration', {}).get('calibration_id')
                measured=has_surface_measurement(row)
                if calibration is not None or measured:
                    if fusion.last_capture >= 0 and source.calibration != calibration:
                        fusion.reject('calibration_version_changed')
                        self.last_qualified.pop(ident, None)
                    source.calibration = calibration
                fusion.observe(row, capture, processing)
                # A single qualified capture can support correspondence to an
                # existing anchor without granting navigation localization.
                if measured:
                    source.surface_quality = quality
                    radius = surface_radius(quality['covariance_m2'])
                    xyz = np.asarray(row['xyz'], float)
                    if radius is not None and xyz.shape == (3,) and np.isfinite(xyz).all():
                        source.identity_surface = (xyz.tolist(), radius)
            self._mirror(tracker.at(ident), source, processing)

    def _mirror(self, track, source, now):
        fusion = source.fusion
        fusion.expire(now)
        point = fusion.current
        rejection = None
        if point is not None:
            if 'radius_m' not in point:
                point['radius_m'] = surface_radius(point['covariance_m2'])
                if point['radius_m'] is not None:
                    self.last_qualified[track.id] = dict(point, history_only=True)
            if point['radius_m'] is None:
                rejection = 'modelled_uncertainty_too_large'
                point = None
            elif source.capture_ns != track.last_capture_ns:
                point = fusion.current = None
                fusion.state = 'unknown'
                fusion.reasons = ['awaiting_current_registered_surface']
        track.localization_state = 'unreliable' if rejection else fusion.state
        track.localization_reasons = [rejection] if rejection else list(fusion.reasons)
        track.position = point
        track.last_reliable_location = self.last_qualified.get(track.id)
        track.localization_capture_s = source.capture_ns * 1e-9 if source.state is not None else None
        track.localization_processing_s = now
        track.localization_quality = point['quality'] if point else dict(reasons=track.localization_reasons)

    def batch(self,tracker,ns,processing):
        active={t.id for t in tracker.tracks}
        for ident in (self.sources.keys() | self.last_qualified.keys()) - active:
            self.sources.pop(ident, None)
            self.last_qualified.pop(ident, None)
        rows=[]
        capture=ns*1e-9
        identity_deadline=capture+min(.75,tracker.visual.max_capture_age_s)
        identity_fresh=capture<=processing<=identity_deadline
        for a in tracker.frames.get(ns,[]):
            t=tracker.at(a['track_id'])
            source = self.sources[t.id]
            self._mirror(t, source, processing)
            p = t.position if source.capture_ns == ns else None
            localization_state = t.localization_state
            if p is None and localization_state == 'reliable':
                # A reliable point from a newer capture cannot describe this row.
                localization_state = 'expired'
            bearing=t.bearing_geometry if t.bearing_state=='fresh' and t.bearing_capture_ns==ns else None
            row=dict(robot=self.robot,session=self.session,counter=str(t.id),sequence='0',revision='0',
                     first_s=t.first_capture_s,capture_s=capture,frame='map',
                     frame_version=self.frame_version,shared_aligned=self.shared_aligned,
                     map_epoch=self.frame_version,clock_domain=self.clock_domain,
                     observation_id=f'{self.robot}/{self.session}/rgb/{ns}/{a["raw_index"]}',
                     visual_state=t.visual_state,localization_state=localization_state,
                     has_position=p is not None,has_bearing=bearing is not None,
                     confidence=a['confidence'])
            if source.capture_ns == ns:
                row.update(identity_input_state=source.state,identity_input_deadline_s=identity_deadline)
                q = source.surface_quality
                if q is not None:
                    registration = q.get('registration', {})
                    row.update(surface_quality_policy='visible_support_v1',support_pixels=q['retained_pixels'],
                        valid_depth_fraction=q['valid_depth_fraction'],component_fraction=q['component_fraction'],
                        depth_span_m=q['depth_span_m'],rgb_depth_skew_s=registration.get('skew_s',0.),
                        calibration_id=registration.get('calibration_id','unversioned-component-input'),covariance_calibrated=False)
                if source.identity_surface is not None and identity_fresh:
                    xyz, radius = source.identity_surface
                    row.update(identity_surface_policy='registered_surface_anchor_v1',
                               ix=xyz[0],iy=xyz[1],iz=xyz[2],identity_radius_m=radius)
            if p:
                row.update(px=p['xyz'][0],py=p['xyz'][1],pz=p['xyz'][2],radius_m=p['radius_m'])
            if bearing:
                o=bearing['origin'];d=bearing['direction']
                row.update(ox=o[0],oy=o[1],oz=o[2],dx=d[0],dy=d[1],dz=d[2],
                           half_angle_rad=bearing['half_angle_rad'],
                           camera_half_fov_rad=bearing['camera_half_fov_rad'],
                           support_captures=a['nav_support_captures'],
                           support_first_s=a['nav_support_first_s'],
                           support_policy='navigation_rgb_v1')
            rows.append(row)
        signature=tuple(tuple(row.items()) for row in rows)
        if self.signatures.get(ns)==signature:return None
        self.signatures[ns]=signature
        while len(self.signatures)>128:self.signatures.popitem(last=False)
        for row in rows:
            source = self.sources[int(row['counter'])]
            source.sequence += 1
            row['sequence'] = str(source.sequence)
        return dict(schema=1,kind='observations',now_s=processing,
                    raw_tracks_total=tracker.next_id-1,observations=rows)

