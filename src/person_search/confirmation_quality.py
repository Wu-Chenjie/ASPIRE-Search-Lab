"""Auditable, provisional RGB-D confirmation gates; no ground-truth access."""
from dataclasses import asdict, dataclass
import numpy as np

QUALITY_PROXY_VERSION='rgbd_mask_boundary_v1'


@dataclass(frozen=True)
class ConfirmationQuality:
    min_confidence: float = .70
    min_mask_pixels: int = 120
    min_mask_box_fraction: float = .45
    max_foreground_occlusion_fraction: float = .08
    max_boundary_near_fraction: float = .08
    min_boundary_valid_fraction: float = .80
    min_valid_depth_fraction: float = .80
    max_depth_span_m: float = .45
    min_frames: int = 3
    min_span_s: float = .8
    max_gap_s: float = 2.
    max_position_deviation_m: float = .25

    def __post_init__(self):
        validate_thresholds(self, counts={'min_frames': 2}, fractions=(
            'min_confidence', 'min_mask_box_fraction', *SURFACE_FRACTIONS))

    def evaluate(self, observation):
        q=observation.quality
        evidence=dict(q, confidence=float(observation.confidence))
        reasons = quality_rejections(self, evidence, (
            ('confidence', True), ('mask_pixels', True), ('mask_box_fraction', True), *SURFACE_CHECKS))
        if q.get('border_clipped') is not False:reasons.append('border_clipped_or_unknown')
        if not np.isfinite(observation.xyz).all():reasons.append('nonfinite_position')
        return reasons,evidence


SURFACE_CHECKS = (
    ('foreground_occlusion_fraction', False), ('boundary_near_fraction', False),
    ('boundary_valid_fraction', True), ('valid_depth_fraction', True), ('depth_span_m', False),
)
SURFACE_FRACTIONS = tuple(('min_' if minimum else 'max_') + name
                          for name, minimum in SURFACE_CHECKS if name.endswith('_fraction'))


def validate_thresholds(config, *, counts, fractions):
    values = asdict(config)
    if any(not np.isfinite(value) or value <= 0 for value in values.values()):
        raise ValueError('Thresholds must be finite and positive')
    if any(values[name] < minimum or int(values[name]) != values[name] for name, minimum in counts.items()):
        raise ValueError('Invalid count threshold')
    if any(values[name] > 1 for name in fractions):
        raise ValueError('Ratios and probabilities must be at most 1')


def quality_rejections(config, evidence, checks, fallback=None):
    reasons = []
    for name, minimum in checks:
        value = evidence.get(name) if fallback is None else evidence.get(name, fallback.get(name))
        if value is None or not np.isfinite(value):
            reasons.append(name + '_unavailable')
            continue
        limit = getattr(config, ('min_' if minimum else 'max_') + name)
        if (value < limit if minimum else value > limit):
            reasons.append(name + '_gate')
    return reasons


def mask_iou(a, b):
    a, b = np.asarray(a, bool), np.asarray(b, bool)
    if a.ndim != 2 or a.shape != b.shape:
        return None
    intersection = np.count_nonzero(a & b)
    return intersection / np.count_nonzero(a | b) if intersection else 0.


def best_box_match(box, candidates, get_box, min_iou):
    best, best_iou = None, min_iou
    for item in candidates:
        overlap = box_iou(box, get_box(item))
        if overlap >= best_iou and (best is None or overlap > best_iou):
            best, best_iou = item, overlap
    return best


def box_iou(a,b):
    ax,ay,aw,ah=a;bx,by,bw,bh=b
    area=max(0,min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by))
    return area/max(aw*ah+bw*bh-area,1.)


class SemanticCandidateMemory:
    """Short-lived image associations only, never independent people counts.

    IDs are camera/image candidates, not identities. Missing/invalid depth has
    an explicit unlocalized state; absence of later detections means unknown.
    A later valid localization of the same capture updates without a new hit.
    """
    def __init__(self):self.candidates=[];self.last_time=-float('inf')

    def update(self, rows, now):
        if now<self.last_time:return rows
        new_capture=now>self.last_time;self.last_time=now;used=set()
        for row in rows:
            c=best_box_match(row['box'], (c for c in self.candidates
                if c['id'] not in used and c['last_seen'] <= now <= c['last_seen'] + 2.),
                lambda c: c['box'], .3)
            if c is None:
                c=dict(id=len(self.candidates)+1,first_seen=now,last_seen=now,capture_count=0)
                self.candidates.append(c)
            used.add(c['id'])
            if c.get('capture_time')!=now:c['capture_count']+=1
            c.update(capture_time=now,last_seen=now,box=row['box'],confidence=row['confidence'],
                     semantic_state='person_candidate',localization_state=row['localization_state'],
                     visibility_state=row.get('visibility_state','unknown'),
                     xyz=row['xyz'],depth_reason=row['depth_reason'])
            row['semantic_candidate_id']=c['id']
        if new_capture:
            for c in self.candidates:
                if c['id'] not in used:
                    c['semantic_state']='not_currently_observed_unknown';c['visibility_state']='unknown'
        return rows
