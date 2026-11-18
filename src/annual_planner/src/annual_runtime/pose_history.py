"""Timestamped estimated poses. Never extrapolate rays beyond measured motion."""
from collections import deque
import numpy as np
from aspire_camera.poses import PoseHistory


class GroundOriginWindow:
    """Average the known stationary start's EKF datum before creating a map."""
    def __init__(self, duration_s=3.):
        if not 3. <= duration_s <= 30.:
            raise ValueError('Ground calibration duration must be in [3, 30] seconds')
        self.duration_s = float(duration_s)
        self.samples = deque(maxlen=int(duration_s*101)+2)
        self.reset_key = None
        self.reason = 'awaiting_ground_samples'
        self.reset_counts = {}; self.last_reset_reason = None; self.last_input = None

    def observe(self, stamp, local_position, velocity, reset_key, grounded):
        speed=float(np.linalg.norm(velocity))
        reasons=[]
        if not grounded:reasons.append('ground_evidence_gate')
        if speed>.08:reasons.append('velocity_over_0.08')
        if reset_key!=self.reset_key:reasons.append('estimator_reset_key_changed')
        if self.samples and stamp-self.samples[-1][0] > .1+1e-9:reasons.append('source_gap_over_0.1')
        self.last_input=dict(stamp=stamp,speed_mps=speed,grounded=bool(grounded),
                            source_gap_s=stamp-self.samples[-1][0] if self.samples else None)
        if reasons:
            self.last_reset_reason=','.join(reasons)
            for reason in reasons:self.reset_counts[reason]=self.reset_counts.get(reason,0)+1
            self.samples.clear()
        self.reset_key = reset_key
        if not grounded or speed > .08:
            self.reason = 'not_stationary_or_disarmed'
            return None
        # Bound storage by elapsed time, not publisher speed. A DDS replay can
        # otherwise fill the deque before the calibration duration is reached.
        if self.samples and stamp-self.samples[-1][0] < .01-1e-9: return None
        self.samples.append((stamp, np.asarray(local_position).copy()))
        # The retained window must cover both the minimum sample count
        # and duration. A healthy slower stream can take longer to reach 50.
        while len(self.samples) > 50 and stamp-self.samples[1][0] >= self.duration_s:
            self.samples.popleft()
        positions = np.asarray([p for _,p in self.samples])
        if np.max(np.ptp(positions,axis=0)) > .12:
            self.last_reset_reason='initial_position_range_over_0.12'
            self.reset_counts[self.last_reset_reason]=self.reset_counts.get(self.last_reset_reason,0)+1
            self.samples.clear()
            self.reason = 'initial_position_unstable'
            return None
        if len(self.samples)<50 or stamp-self.samples[0][0]<self.duration_s-1e-9:
            self.reason = 'collecting_initial_position'
            return None
        self.reason = 'calibrated'
        return positions.mean(axis=0)

    def diagnostics(self):
        return dict(reason=self.reason,samples=len(self.samples),
                    source_span_s=self.samples[-1][0]-self.samples[0][0] if self.samples else 0.,
                    reset_counts=dict(self.reset_counts),last_reset_reason=self.last_reset_reason,
                    last_input=self.last_input)
