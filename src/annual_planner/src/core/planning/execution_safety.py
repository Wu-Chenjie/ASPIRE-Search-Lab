"""Measured-map execution certificates and finite C2 stopping contingencies.

No occupancy inference: native states come from the native post-fusion callback.
The native inflation field AND the mapper's existing physical envelope must pass.
Polynomial Bernstein hulls enclose the continuous centerline; no point sampling
is treated as a continuous certificate. Unknown and out-of-map fail closed.
"""
from bisect import bisect_left, bisect_right
import math
import os
import numpy as np
from core.exploration.collision_geometry import CLEARANCE_TOLERANCE_M
from .continuous_trajectory import ContinuousTrajectory, maximum_norm
from .motion_limits import motion_limits
from .bspline_trajectory import BRAKE_DURATIONS, join_brake, native_prefix, stopping_polynomial

TRACKING_MARGIN = .25
# Start the finite stop while comfortable margin remains before the 0.25 m tube limit.
TRACKING_BRAKE_TRIGGER = .15
MAX_MAP_AGE = 2.0
STATIC_REFERENCE_JUMP_M = .12
SITL_RELAXED_PROFILE = 'sitl_relaxed'
SITL_FRONTIER_PROFILE = 'sitl_frontier'
SITL_FRONTIER_PADDING_M = .03
EXECUTION_PROFILE_ENV = 'ASPIRE_EXECUTION_SAFETY_PROFILE'


def relaxed_sitl_execution():
    return os.environ.get(EXECUTION_PROFILE_ENV) in (SITL_RELAXED_PROFILE, SITL_FRONTIER_PROFILE)


def frontier_sitl_execution():
    return os.environ.get(EXECUTION_PROFILE_ENV) == SITL_FRONTIER_PROFILE


def static_reference_jump_limit():
    return .15 if relaxed_sitl_execution() else STATIC_REFERENCE_JUMP_M


def stopped_planning_reference(report, active_id, completed, now):
    """Serialize the fixed executor reference once it is ready and stopped."""
    if (report.get('ready') is not True or report.get('arrived') is not True
            or report.get('pending_token') is not None):
        return None
    if active_id < 0:
        if (report.get('epoch') != 0 or report.get('token') is not None
                or report.get('native_plan_id') is not None):
            return None
    elif not completed or report.get('native_plan_id') != active_id:
        return None

    try:
        stamp = float(report['time'])
        if not 0. <= now-stamp <= .5 or not 0. <= now-float(report['position_stamp']) <= .5:
            return None
        position, actual, velocity, acceleration = np.asarray(
            [report['reference'], report['actual_position'],
             report['reference_velocity'], report['reference_acceleration']], dtype=float)
        yaw, yaw_rate = float(report['reference_yaw']), float(report['reference_yaw_rate'])
        if (position.shape != (3,) or not np.isfinite([position, actual, velocity, acceleration]).all()
                or not np.isfinite([yaw, yaw_rate]).all()
                or np.linalg.norm(position-actual) > TRACKING_MARGIN
                or np.linalg.norm(velocity)+np.linalg.norm(acceleration)+abs(yaw_rate) > 1e-5):
            return None
    except (KeyError, TypeError, ValueError):
        return None
    return [stamp, *position.tolist(), yaw]


def tracking_brake_trigger():
    return .18 if relaxed_sitl_execution() else TRACKING_BRAKE_TRIGGER


class NativeSafetyMap:
    def __init__(self):
        self.stamp = -np.inf
        self.sequence = -1
        self.state = None

    def ingest(self, data):
        a = np.asarray(data, float)
        if a.ndim != 1 or len(a) < 10 or a[0] != 1 or not np.isfinite(a[1:10]).all():
            raise ValueError('Invalid native safety snapshot')
        stamp, seq, resolution = a[1:4]
        shape = tuple(map(int, a[7:10]))
        vals = a[10:]
        if (stamp < 0 or seq != int(seq) or resolution <= 0 or
                any(size < 1 or size != supplied for size, supplied in zip(shape, a[7:10])) or
                len(a) != 10+math.prod(shape) or not np.isfinite(vals).all() or
                not np.all(vals == np.floor(vals)) or
                not np.all((vals >= 0) & (vals <= 6) & (vals != 3))):
            raise ValueError('Malformed native safety geometry')
        if seq <= self.sequence or stamp < self.stamp:
            return False
        origin = a[4:7]
        if self.state is not None and (self.shape != tuple(shape) or self.resolution != resolution or not np.array_equal(self.origin, origin)):
            raise ValueError('Native safety geometry changed during execution')
        state = a[10:].astype(np.uint8).reshape(shape)
        self.stamp, self.sequence, self.resolution = float(stamp), int(seq), float(resolution)
        self.origin, self.shape, self.state = origin.copy(), shape, state
        return True

    def fresh(self, now):
        return self.state is not None and -.05 <= now-self.stamp <= MAX_MAP_AGE

    def box_safe(self, low, high, *, ignore_inflation=False, strict_box=None):
        if self.state is None:
            return False
        ox, oy, oz = self.origin
        res = self.resolution
        sx, sy, sz = self.shape
        lo0 = math.floor((low[0] - ox) / res)
        lo1 = math.floor((low[1] - oy) / res)
        lo2 = math.floor((low[2] - oz) / res)
        hi0 = math.floor((high[0] - ox) / res)
        hi1 = math.floor((high[1] - oy) / res)
        hi2 = math.floor((high[2] - oz) / res)
        if lo0 < 0 or lo1 < 0 or lo2 < 0 or hi0 >= sx or hi1 >= sy or hi2 >= sz:
            return False
        cells = self.state[lo0:hi0 + 1, lo1:hi1 + 1, lo2:hi2 + 1]
        if bool(np.all(cells == 1)):
            return True
        if not ignore_inflation:
            return False
        # Bit 4 encodes inflation. Raw unknown and occupied still block.
        if not bool(np.all((cells & 3) == 1)):
            return False
        if strict_box is not None:
            rlo, rhi = strict_box
            rlo0 = math.floor((rlo[0] - ox) / res)
            rlo1 = math.floor((rlo[1] - oy) / res)
            rlo2 = math.floor((rlo[2] - oz) / res)
            rhi0 = math.floor((rhi[0] - ox) / res)
            rhi1 = math.floor((rhi[1] - oy) / res)
            rhi2 = math.floor((rhi[2] - oz) / res)
            if rlo0 < 0 or rlo1 < 0 or rlo2 < 0 or rhi0 >= sx or rhi1 >= sy or rhi2 >= sz:
                return False
            return bool(np.all(self.state[rlo0:rhi0 + 1, rlo1:rhi1 + 1, rlo2:rhi2 + 1] == 1))
        return True


def polynomial_box(coefficients, lo, hi):
    """Convex hull of degree-five Bernstein controls on [lo, hi]."""
    c = np.asarray(coefficients)
    q = np.zeros_like(c)
    for k in range(6):
        for j in range(k, 6):
            q[k] += c[j]*math.comb(j,k)*lo**(j-k)*(hi-lo)**k
    b = np.zeros_like(c)
    for i in range(6):
        for k in range(i+1): b[i] += q[k]*math.comb(i,k)/math.comb(5,k)
    return b.min(axis=0), b.max(axis=0)


def reference_step_distance(curve, progress, seconds=.04):
    """Conservative reference displacement over the executor's maximum step.

    This is headroom for advancing the reference, not a prediction of actual
    vehicle motion. Measured tracking and the final candidate remain guarded.
    """
    origin = curve.sample(progress)[0]
    end = min(curve.duration, progress+seconds)
    if end < progress:
        return 0.
    knots = curve.knots
    # Include segments touching either end of the closed reference interval.
    first = bisect_left(knots, progress, lo=1) - 1
    last = bisect_right(knots, end, hi=len(curve.durations))
    bound = 0.
    for i in range(first, last):
        c, duration = curve.coefficients[i], curve.durations[i]
        a, b = max(progress, knots[i]), min(end, knots[i+1])
        low, high = polynomial_box(c, (a-knots[i])/duration, (b-knots[i])/duration)
        bound = max(bound, float(np.linalg.norm(np.maximum(abs(low-origin), abs(high-origin)))))
    return bound


def _ellipsoid_distance(delta, rx, rz):
    """Exact Euclidean distance from nonnegative box gaps to E(rx, rx, rz)."""
    if abs(rx - rz) <= 1e-12:
        return np.maximum(np.linalg.norm(delta, axis=1) - rx, 0.)
    rho = np.hypot(delta[:, 0], delta[:, 1])
    z = delta[:, 2]
    dist = np.zeros(len(delta), dtype=float)
    out = (rho / rx) ** 2 + (z / rz) ** 2 > 1.
    if np.any(out):
        ro, zo = rho[out], z[out]
        rx2, rz2 = rx * rx, rz * rz
        t = np.maximum(np.maximum(rx * ro - rx2, rz * zo - rz2), 0.)
        for _ in range(10):
            a, b = rx2 + t, rz2 + t
            fa, fb = (rx * ro / a) ** 2, (rz * zo / b) ** 2
            step = (fa + fb - 1.) / (2. * (fa / a + fb / b))
            t = np.maximum(t + step, 0.)
            if float(np.max(np.abs(step))) <= 1e-12:
                break
        dist[out] = np.hypot(ro - rx2 * ro / (rx2 + t), zo - rz2 * zo / (rz2 + t))
    return dist


class ExecutionCorridor:
    def __init__(self, runtime, native, margin=TRACKING_MARGIN, *, relaxed_sitl=False, frontier_sitl=False):
        self.runtime, self.native = runtime, native
        self.frontier_sitl = frontier_sitl
        self.margin = SITL_FRONTIER_PADDING_M if frontier_sitl else float(margin)
        self.relaxed_sitl = relaxed_sitl or frontier_sitl

    def box_safe(self, low, high):
        raw_low, raw_high = np.asarray(low), np.asarray(high)
        low, high = raw_low - self.margin, raw_high + self.margin
        m = self.runtime
        if low[2] < m.flight_limits[0] or high[2] > m.flight_limits[1]: return False
        # Unknown and occupied cells block across the full tracking tube; in
        # strict mode, also enforce native obstacle inflation (bit 4) on the
        # reference AABB without double-stacking margin onto the 0.60 m kernel.
        if not self.native.box_safe(
            low, high, ignore_inflation=True,
            strict_box=None if self.relaxed_sitl else (raw_low, raw_high),
        ):
            return False
        # Conservative Minkowski test between the enclosing reference AABB
        # + Euclidean L2 tracking ball and every occupied/unknown voxel box.
        center, half = (low+high)/2, (high-low)/2
        radius = m.clearance+np.linalg.norm((half+m.resolution/2)*m.metric)
        ids = m.obstacle_tree.query_ball_point(center*m.metric, radius)
        if len(ids):
            obstacles = m.obstacle_centers[ids]
            delta = np.maximum(abs(center-obstacles)-half-m.resolution/2, 0.)
            near = np.linalg.norm(delta*m.metric, axis=1) < m.clearance-CLEARANCE_TOLERANCE_M
            if np.any(near):
                if self.margin <= 0.:
                    return False
                raw_center, raw_half = (raw_low+raw_high)/2, (raw_high-raw_low)/2
                raw_delta = np.maximum(abs(raw_center-obstacles[near])-raw_half-m.resolution/2, 0.)
                rx = m.clearance / m.metric[0]
                rz = m.clearance / m.metric[2]
                if np.any(_ellipsoid_distance(raw_delta, rx, rz) < self.margin-CLEARANCE_TOLERANCE_M):
                    return False
        if m.reserved_points is not None:
            if m.reserved_points.query(center)[0] < m.reserved_radius+np.linalg.norm(half): return False
        return True

    def safe_path(self, path):
        # Existing physical full-path check remains mandatory. AABB subdivision
        # additionally encloses each straight interpolation, including endpoints.
        p = np.asarray(path, float)
        if not self.runtime.safe_path(p): return False
        if len(p) == 1: return self.box_safe(p[0],p[0])
        def segment(a,b,depth=0):
            if self.box_safe(np.minimum(a,b),np.maximum(a,b)): return True
            if depth >= 8 or np.linalg.norm(b-a) < .005: return False
            mid=(a+b)/2
            return segment(a,mid,depth+1) and segment(mid,b,depth+1)
        return all(segment(a,b) for a,b in zip(p[:-1],p[1:]))

    def safe_curve(self, curve, start=0.):
        if not 0 <= start <= curve.duration: return False
        if self.relaxed_sitl and not self.frontier_sitl:
            endpoint = curve.sample(curve.duration)[0]
            if not ExecutionCorridor(self.runtime, self.native, self.margin).box_safe(endpoint, endpoint):
                return False

        def span(c,lo,hi,T,begin,depth=0):
            low,high=polynomial_box(c,max(lo,begin),hi)
            if self.box_safe(low,high): return True
            if depth >= 12 or (hi-lo)*T <= .01: return False
            # Keep the admission subdivision when checking a suffix. Shifting
            # every midpoint can create a failing box on an unchanged map.
            mid=(lo+hi)/2
            if begin < mid and not span(c,lo,mid,T,begin,depth+1): return False
            return span(c,mid,hi,T,begin,depth+1)
        for i,(c,T) in enumerate(zip(curve.coefficients,curve.durations)):
            if curve.knots[i+1] < start: continue
            begin=min(1.,max(0.,(start-curve.knots[i])/T))
            if not span(c,0.,1.,T,begin): return False
        return True


def _yaw_toward_capture(coefficients, duration, bearing, max_rate):
    # Smoothstep changes only the terminal heading, preserving the C2 jet at
    # both ends. Test the true polynomial peak with the existing yaw envelope.
    error=math.remainder(bearing-sum(coefficients),2*math.pi)
    step=np.array([0.,0.,0.,10.,-15.,6.])*error
    for fraction in (1.,.75,.50,.25,.125):
        trial=coefficients+fraction*step
        derivative=np.polynomial.polynomial.polyder(trial)[:,None]
        if maximum_norm(derivative)/duration<=max_rate:
            return trial
    return coefficients


def certified_brake(curve, progress, corridor, *, accept_endpoint=None, measured_bearing=None):
    """Stop from the exact current reference jet, without a reference jump.

    Finite candidates only, original dynamics limits retained. The old prefix is
    kept to preserve the execution clock and native feedback's boundary meaning.
    The new stop receives no exploration reward and is not an escape planner.
    """
    if not 0 <= progress < curve.duration: raise ValueError('No moving brake boundary')
    p,v,a,h,r=curve.sample(progress); alpha=curve.yaw_acceleration(progress)
    limits=motion_limits()
    prefix = None
    for T in BRAKE_DURATIONS:
        c = stopping_polynomial(p, v, a, T)
        y = stopping_polynomial(h, r, alpha, T)
        if measured_bearing is not None:
            y=_yaw_toward_capture(y,T,measured_bearing,limits.yaw_rate)
        try:
            brake=ContinuousTrajectory([T],[c],h,float(sum(y)-h),'measured_map_C2_brake',yaw_coefficients=y)
            # Constructor checks geometry/C2; retain the exact dynamics gate.
            # Packet statistics/serialization are not admission checks.
            if not limits.accepts(brake): continue
            if accept_endpoint is not None and not accept_endpoint(brake.sample(T)[0], T):continue
            if not corridor.safe_curve(brake): continue
            if progress == 0: return brake
        except ValueError:
            continue
        if prefix is None:
            # Peak dynamics of the joined curve are the span maxima. The
            # brake passed independently, and the retained prefix is fixed.
            try:
                prefix = native_prefix(curve, progress)
                if not limits.accepts(prefix):
                    break
            except ValueError:
                break
        try:
            return join_brake(prefix, brake, 'native_prefix_with_safety_brake')
        except ValueError:
            continue
    raise ValueError('No certified C2 stop in latest measured corridor')

# Includes one full maximum allowed measured-map age, plus execution/check delay.
# These are experiment acceptance budgets, not a promise about unseen obstacles.
EXECUTION_DELAY_BUDGET = .20
REACTION_HORIZON = MAX_MAP_AGE + EXECUTION_DELAY_BUDGET
RECHECK_PERIOD = .10


def stopping_reserve(curve, progress, corridor, horizon=REACTION_HORIZON):
    """Certify continuing for the latency budget AND then coming to rest.

    A full original trajectory with a safe terminal stop is insufficient: the
    brake at an intermediate point can leave its corridor. This separate finite
    contingency must exist before each continuing execution grant.
    """
    if not np.isfinite(horizon) or horizon < REACTION_HORIZON:
        raise ValueError('Stopping reserve must cover the full latency budget')
    boundary=min(curve.duration,progress+horizon)
    if boundary >= curve.duration:
        end=curve.sample(curve.duration)
        if (np.linalg.norm(end[1])+np.linalg.norm(end[2])+abs(end[4])+abs(curve.yaw_acceleration(curve.duration))>1e-5 or
                not corridor.safe_curve(curve,progress)):
            raise ValueError('No certified terminal stopping reserve')
        return curve,boundary
    reserve=certified_brake(curve,boundary,corridor)
    if not corridor.safe_curve(reserve,progress):
        raise ValueError('Latency interval before stopping is blocked')
    return reserve,boundary


def certified_stopped_reference(curve, progress, reference, actual, corridor):
    """Only physical/reference eligibility. Never proves an observed view."""
    endpoint,v,a,yaw,rate=curve.sample(curve.duration)
    reference,actual=np.asarray(reference,float),np.asarray(actual,float)
    if (reference.shape!=(3,) or actual.shape!=(3,) or
            not np.isfinite([progress,*reference,*actual]).all() or
            abs(progress-curve.duration)>1e-9 or np.linalg.norm(reference-endpoint)>1e-9):
        raise ValueError('terminal_reference_not_at_fixed_endpoint')
    if np.linalg.norm(v)+np.linalg.norm(a)+abs(rate)+abs(curve.yaw_acceleration(curve.duration))>1e-5:
        raise ValueError('terminal_reference_not_stopped')
    if np.linalg.norm(actual-reference)>TRACKING_MARGIN:raise ValueError('tracking_tube_exceeded')
    if not corridor.box_safe(reference,reference):raise ValueError('terminal_root_no_longer_certified')
    return dict(fixed_endpoint=endpoint.tolist(),reference_stopped=True,observation_complete=False)


def certified_terminal_hold(curve, progress, reference, actual, corridor, now, since, wait_s):
    state=certified_stopped_reference(curve,progress,reference,actual,corridor)
    if (not np.isfinite([now,since,wait_s]).all() or wait_s<=0 or now<since or now>since+wait_s):
        raise ValueError('terminal_observation_wait_deadline_exceeded')
    return dict(state,started_at=float(since),absolute_deadline=float(since+wait_s),wait_budget_s=float(wait_s))
