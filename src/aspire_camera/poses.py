"""Interpolate estimated poses at camera capture time."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class PoseHistory:
    def __init__(self, capacity=300, max_gap=.15):
        self.samples = deque(maxlen=capacity)
        self.max_gap = max_gap

    def add(self, stamp, position, quaternion):
        p, q = np.asarray(position, float), np.asarray(quaternion, float)
        if (p.shape != (3,) or q.shape != (4,) or
                not np.isfinite([stamp, *p, *q]).all() or
                abs(np.linalg.norm(q)-1.) > .05 or
                (self.samples and stamp <= self.samples[-1][0])):
            return False
        self.samples.append((stamp, p.copy(), Rotation.from_quat(q)))
        return True

    def _bracket(self, stamp):
        if not self.samples or not np.isfinite(stamp):
            return None
        previous = None
        for sample in self.samples:
            t = sample[0]
            if abs(t - stamp) <= 1e-9:
                return sample, sample
            if t > stamp:
                if previous is None or t - previous[0] > self.max_gap + 1e-9:
                    return None
                return previous, sample
            previous = sample
        return None

    def brackets(self, stamp):
        """Check whether an exact sample or usable measured bracket exists."""
        return self._bracket(stamp) is not None

    def at(self, stamp):
        """Return pose and measured bracket; None means not yet available/unsafe."""
        bracket = self._bracket(stamp)
        if bracket is None:
            return None
        (lo, lp, lr), (hi, hp, hr) = bracket
        if lo == hi:
            return lp.copy(), lr, (lo, hi)
        a = (stamp - lo) / (hi - lo)
        rotation = Slerp([lo, hi], Rotation.from_quat([lr.as_quat(), hr.as_quat()]))([stamp])[0]
        return (1-a)*lp+a*hp, rotation, (lo, hi)

    def passed(self, stamp):
        return bool(self.samples and self.samples[-1][0] >= stamp)
