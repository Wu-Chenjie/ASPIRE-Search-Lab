"""Cross-view de-duplication and bounded review state; no target truth."""
from dataclasses import dataclass, field
import numpy as np


@dataclass
class Track:
    id: int
    xyz: np.ndarray
    last_seen: float
    first_seen: float
    state: str = 'tentative'
    semantic_views: list = field(default_factory=list)
    attempts: int = 0
    observations: int = 1


def nearest_track(tracks, point, used, radius):
    if radius <= 0:
        return None
    best, best_distance = None, radius * radius
    for track in tracks:
        if track.id in used:
            continue
        offset = track.xyz[:2] - point[:2]
        distance = offset @ offset
        if distance < best_distance:
            best, best_distance = track, distance
    return best


class Tracker:
    def __init__(self, radius=.65, max_attempts=3, ttl=30.):
        self.radius=radius; self.max_attempts=max_attempts; self.ttl=ttl
        self.tracks=[]; self.last_time=-float('inf')

    def update(self, observations, origin, now):
        if not np.isfinite(now) or now < self.last_time: raise ValueError('Nonmonotonic observation time')
        self.last_time=now;used=set()
        for c in observations:
            t=nearest_track(self.tracks,c.xyz,used,self.radius)
            if t is not None:
                t.xyz=(t.xyz*t.observations+c.xyz)/(t.observations+1);t.observations+=1
            else:
                t=Track(len(self.tracks)+1,c.xyz.copy(),now,now);self.tracks.append(t)
            used.add(t.id);t.last_seen=now
            if c.semantic and all(np.linalg.norm(origin-np.asarray(p))>=.5 for p in t.semantic_views):
                t.semantic_views.append(np.asarray(origin).tolist())
            if len(t.semantic_views)>=2:t.state='confirmed'
        for t in self.tracks:
            if t.state not in ('confirmed','abandoned') and (now-t.first_seen>self.ttl or t.attempts>=self.max_attempts):t.state='abandoned'
        return self.tracks

    def review(self,t):
        if t.state in ('confirmed','abandoned') or t.attempts>=self.max_attempts:return False
        t.attempts+=1;t.state='reviewing';return True
