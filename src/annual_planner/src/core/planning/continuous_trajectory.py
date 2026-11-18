"""Minimum-jerk quintic spline with hard validation.

A C4 quintic interpolant with prescribed endpoint velocity/acceleration is the
minimum integrated squared jerk curve for fixed waypoint times. SciPy's banded
B-spline interpolation solves that inner problem. This is a small reference
implementation of minimum control interpolation, not a vendored GCOPTER/MINCO
implementation.
"""
import numpy as np
from .motion_limits import motion_limits
from scipy.interpolate import make_interp_spline, PPoly


def derivative(coefficients, order):
    c = np.asarray(coefficients)
    for _ in range(order):
        c = c[1:]*np.arange(1, len(c))[:, None]
    return c


def evaluate_polynomial(c, u):
    return np.polynomial.polynomial.polyval(u, c).T


def maximum_norm(c):
    squared = np.zeros(2*len(c)-1)
    for d in range(c.shape[1]):
        term = np.polynomial.polynomial.polymul(c[:, d], c[:, d])
        squared[:len(term)] += term
    roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(squared))
    candidates = [0., 1.]+[float(r.real) for r in roots if abs(r.imag) < 1e-7 and 0 < r.real < 1]
    return float(np.sqrt(max(0., max(np.polynomial.polynomial.polyval(candidates, squared)))))


class ContinuousTrajectory:
    def __init__(self, durations, coefficients, yaw, yaw_delta, method='minimum_jerk_bspline', yaw_coefficients=None,
                 yaw_durations=None, yaw_segments=None):
        self.durations = np.asarray(durations, float); self.coefficients = np.asarray(coefficients, float)
        self.knots = np.r_[0., np.cumsum(self.durations)]; self.duration = float(self.knots[-1])
        self.yaw = float(yaw); self.yaw_delta = float(yaw_delta); self.method = method
        self.yaw_coefficients = np.array([yaw, 0., 0., 10*yaw_delta, -15*yaw_delta, 6*yaw_delta]
                                        if yaw_coefficients is None else yaw_coefficients, float)
        if (self.durations.ndim != 1 or not self.durations.size or not np.all(self.durations > 0)
                or not np.isfinite(self.duration)
                or self.coefficients.shape != (len(self.durations), 6, 3) or not np.isfinite(self.coefficients).all()
                or not np.isfinite([self.yaw, self.yaw_delta]).all() or self.yaw_coefficients.shape != (6,)
                or not np.isfinite(self.yaw_coefficients).all()):
            raise ValueError('Malformed continuous trajectory')
        self.yaw_durations = self.yaw_segments = None
        if yaw_durations is not None or yaw_segments is not None:
            self.yaw_durations = np.asarray(yaw_durations, float)
            self.yaw_segments = np.asarray(yaw_segments, float)
            if (self.yaw_durations.ndim != 1 or not len(self.yaw_durations)
                    or not np.all(self.yaw_durations > 0)
                    or self.yaw_segments.shape != (len(self.yaw_durations), 6)
                    or not np.isfinite(self.yaw_segments).all()
                    or not abs(float(sum(self.yaw_durations))-self.duration) <= 1e-7):
                raise ValueError('Malformed piecewise yaw trajectory')
            self.yaw_knots = np.r_[0., np.cumsum(self.yaw_durations)]
            for i in range(len(self.yaw_durations)-1):
                for order in range(3):
                    left = np.polynomial.polynomial.polyval(1., np.polynomial.polynomial.polyder(self.yaw_segments[i], order))/self.yaw_durations[i]**order
                    right = np.polynomial.polynomial.polyval(0., np.polynomial.polynomial.polyder(self.yaw_segments[i+1], order))/self.yaw_durations[i+1]**order
                    if abs(left-right) > 1e-5:
                        raise ValueError('Piecewise yaw violates C2 continuity')

    def yaw_sample(self, times, order=0):
        t = np.clip(np.asarray(times, float), 0., self.duration)
        if self.yaw_durations is None:
            return np.polynomial.polynomial.polyval(t/self.duration, np.polynomial.polynomial.polyder(self.yaw_coefficients, order))/self.duration**order
        i = np.minimum(len(self.yaw_durations)-1, np.searchsorted(self.yaw_knots, t, side='right')-1)
        T = self.yaw_durations[i]; u = (t-self.yaw_knots[i])/T
        c = np.stack([np.polynomial.polynomial.polyder(row, order) for row in self.yaw_segments])[i]
        value = np.zeros_like(t)
        for j in range(c.shape[-1]-1, -1, -1): value = value*u+c[..., j]
        return value/T**order

    def sample(self, t):
        t = float(np.clip(t, 0., self.duration)); i = min(len(self.durations)-1, np.searchsorted(self.knots, t, side='right')-1)
        u = (t-self.knots[i])/self.durations[i]; c = self.coefficients[i]; T = self.durations[i]
        p, v, a = [evaluate_polynomial(derivative(c, k), u)/T**k for k in range(3)]
        yaw = float(self.yaw_sample(t)); rate = float(self.yaw_sample(t, 1))
        return p, v, a, yaw, rate

    def sample_many(self, times):
        """Evaluate the same piecewise polynomial at a batch of times."""
        t=np.clip(np.asarray(times,float).reshape(-1),0.,self.duration)
        segments=np.minimum(len(self.durations)-1,np.searchsorted(self.knots,t,side='right')-1)
        durations=self.durations[segments];u=(t-self.knots[segments])/durations
        values=[]
        for order in range(3):
            coefficients=np.stack([derivative(c,order) for c in self.coefficients])[segments]
            value=np.zeros((len(t),3))
            for coefficient in np.moveaxis(coefficients[:,::-1],1,0):value=value*u[:,None]+coefficient
            values.append(value/durations[:,None]**order)
        heading=self.yaw_sample(t); rate=self.yaw_sample(t, 1)
        return (*values,heading,rate)

    def limits(self):
        return {name: max(maximum_norm(derivative(c, k))/T**k for c, T in zip(self.coefficients, self.durations))
                for k, name in [(1, 'speed'), (2, 'acceleration'), (3, 'jerk')]}

    def jerk_cost(self):
        cost = 0.
        for c, T in zip(self.coefficients, self.durations):
            j = derivative(c, 3)
            for axis in range(3):
                poly = np.polynomial.polynomial.polymul(j[:, axis], j[:, axis])
                cost += float(sum(poly/np.arange(1, len(poly)+1)))/T**5
        return cost

    def yaw_acceleration(self, t):
        return float(self.yaw_sample(t, 2))

    def yaw_rate_limit(self):
        if self.yaw_durations is not None:
            return max(maximum_norm(np.polynomial.polynomial.polyder(c)[:, None])/T
                       for c, T in zip(self.yaw_segments, self.yaw_durations))
        return maximum_norm(np.polynomial.polynomial.polyder(self.yaw_coefficients)[:, None])/self.duration

    def path(self, dt=.08):
        return self.sample_many(np.linspace(0,self.duration,max(2,int(np.ceil(self.duration/dt))+1)))[0]

    def to_dict(self):
        packet = dict(schema='annual.trajectory/1', durations=self.durations.tolist(), coefficients=self.coefficients.tolist(),
                    yaw=self.yaw, yaw_delta=self.yaw_delta, method=self.method, duration=self.duration,
                    yaw_coefficients=self.yaw_coefficients.tolist(), yaw_rate_limit=self.yaw_rate_limit(),
                    limits=self.limits(), jerk_integral=self.jerk_cost())
        if self.yaw_durations is not None:
            packet.update(schema='annual.trajectory/2', yaw_durations=self.yaw_durations.tolist(), yaw_segments=self.yaw_segments.tolist())
        return packet

    @classmethod
    def from_dict(cls, packet):
        if packet.get('schema') not in ('annual.trajectory/1', 'annual.trajectory/2'):
            raise ValueError('Unsupported trajectory schema')
        extra = {} if packet['schema']=='annual.trajectory/1' else dict(yaw_durations=packet['yaw_durations'], yaw_segments=packet['yaw_segments'])
        result = cls(packet['durations'], packet['coefficients'], packet['yaw'], packet['yaw_delta'], packet['method'], packet.get('yaw_coefficients'), **extra)
        if not motion_limits().accepts(result):
            raise ValueError('Trajectory exceeds executor dynamics envelope')
        return result


def interpolate(points, durations, yaw, delta, start_velocity=None, start_acceleration=None, start_yaw_rate=0., start_yaw_acceleration=0.):
    times = np.r_[0., np.cumsum(durations)]; zero = np.zeros(3)
    velocity = zero if start_velocity is None else np.asarray(start_velocity, float)
    acceleration = zero if start_acceleration is None else np.asarray(start_acceleration, float)
    spline = make_interp_spline(times, points, k=5, bc_type=([(1, velocity), (2, acceleration)], [(1, zero), (2, zero)]))
    coefficients = np.zeros((len(durations), 6, 3))
    for axis in range(3):
        p = PPoly.from_spline((spline.t, spline.c[:, axis], spline.k))
        for i, (start, T) in enumerate(zip(times, durations)):
            j = np.searchsorted(p.x, start+T*.5)-1
            coefficients[i, :, axis] = p.c[:, j][::-1]*T**np.arange(6)
    T = times[-1]; c = np.array([yaw, start_yaw_rate*T, .5*start_yaw_acceleration*T*T, 0., 0., 0.])
    c[3:] = np.linalg.solve([[1, 1, 1], [3, 4, 5], [6, 12, 20]],
                            [delta-c[1]-c[2], -c[1]-2*c[2], -2*c[2]])
    return ContinuousTrajectory(durations, coefficients, yaw, delta, yaw_coefficients=c)
