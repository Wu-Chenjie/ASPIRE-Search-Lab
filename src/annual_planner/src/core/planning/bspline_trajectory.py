"""Exact native B-spline conversion; no endpoint refit or time scaling."""
import math
import os
import numpy as np
from scipy.interpolate import BSpline
from .continuous_trajectory import ContinuousTrajectory
from .motion_limits import motion_limits


BRAKE_DURATIONS = (.4, .6, .8, 1., 1.25, 1.5, 2., 2.5, 3., 4.)


def stopping_polynomial(value, rate, acceleration, duration):
    """Normalized quartic with zero terminal rate and acceleration."""
    T = duration
    return np.stack((value, rate*T, .5*acceleration*T*T,
                     -rate*T-(2./3)*acceleration*T*T,
                     .5*rate*T+.25*acceleration*T*T, np.zeros_like(value)))


def join_brake(prefix, brake, method):
    """Join a C2 brake to an exact native prefix with piecewise yaw."""
    duration = brake.duration
    end_yaw = float(sum(brake.yaw_coefficients))
    return ContinuousTrajectory(
        np.r_[prefix.durations, duration], np.concatenate([prefix.coefficients, brake.coefficients]),
        prefix.yaw, end_yaw-prefix.yaw, method,
        yaw_durations=np.r_[prefix.yaw_durations, duration],
        yaw_segments=np.vstack([prefix.yaw_segments, brake.yaw_coefficients]))


def polynomial_spans(knots, points, degree):
    knots=np.asarray(knots, float); points=np.asarray(points, float)
    if (type(degree) is not int or not 3 <= degree <= 5 or points.ndim != 2
            or len(points) <= degree or len(knots) != len(points)+degree+1
            or not np.isfinite(knots).all() or not np.isfinite(points).all()
            or np.any(np.diff(knots)<0)):
        raise ValueError('Malformed native B-spline')
    start,end=knots[degree],knots[-degree-1]
    if end<=start: raise ValueError('Empty native B-spline domain')
    breaks=np.unique(knots[(knots>=start)&(knots<=end)])
    durations=np.diff(breaks); spline=BSpline(knots,points,degree,extrapolate=False)
    coefficients=np.zeros((len(durations),6,points.shape[1]))
    for i,(t,T) in enumerate(zip(breaks[:-1],durations)):
        for k in range(degree+1): coefficients[i,k]=spline(t,nu=k)*T**k/math.factorial(k)
    for i in range(len(durations)-1):
        for k in range(3):
            left=sum(coefficients[i,j]*math.factorial(j)/math.factorial(j-k) for j in range(k,6))/durations[i]**k
            right=coefficients[i+1,k]*math.factorial(k)/durations[i+1]**k
            if np.linalg.norm(left-right)>1e-5: raise ValueError('Native position spline violates C2')
    return durations,coefficients


def native_trajectory(packet):
    duration,coefficients=polynomial_spans(packet['knots'],packet['points'],packet['order'])
    yaw=np.asarray(packet['yaw_points'],float); dt=float(packet['yaw_dt'])
    if yaw.ndim!=1 or len(yaw)<4 or not np.isfinite(dt) or dt<=0:
        raise ValueError('Malformed native yaw spline')
    yd,yc=polynomial_spans(np.arange(-3,len(yaw)+1)*dt,yaw[:,None],3)
    duration_delta=sum(duration)-sum(yd)
    if abs(duration_delta)>1e-6: raise ValueError('Native position/yaw domains differ')
    # Roundoff in knot generation must not create an uncovered tail.
    yd[-1]+=duration_delta
    start=float(yc[0,0,0]); end=float(sum(yc[-1,:,0]))
    result=ContinuousTrajectory(duration,coefficients,start,end-start,'native_racer_bspline',
                                yaw_durations=yd,yaw_segments=yc[:,:,0])
    if not motion_limits().accepts(result):
        raise ValueError('Trajectory exceeds executor dynamics envelope')
    return result


def _brake_candidates(curve, runtime, diagnostics):
    p,v,a,h,r=curve.sample(curve.duration);alpha=curve.yaw_acceleration(curve.duration)
    if max(np.linalg.norm(v),np.linalg.norm(a),abs(r),abs(alpha))<1e-5:
        yield curve, None
        return
    limits=motion_limits()
    if not limits.accepts(curve):
        return
    for T in BRAKE_DURATIONS:
        c = stopping_polynomial(p, v, a, T)
        y = stopping_polynomial(h, r, alpha, T)
        try:
            brake=ContinuousTrajectory([T],[c],h,float(sum(y)-h),yaw_coefficients=y)
            if not limits.accepts(brake):
                raise ValueError('Trajectory exceeds executor dynamics envelope')
            if not runtime.safe_path(brake.path()):
                if diagnostics is not None: diagnostics.append((T,'brake_path'))
                continue
            result = join_brake(curve, brake, 'native_racer_bspline_with_brake')
        except ValueError as error:
            if diagnostics is not None: diagnostics.append((T,str(error)))
            continue
        # Peak dynamics are the maxima across spans. Both parts have passed
        # those checks, and construction checks the joined yaw's continuity.
        yield result, brake


def append_safe_brake(curve, runtime, diagnostics=None):
    """Preserve the native prefix and append a sampled-safe C2 stop."""
    for stopped, _ in _brake_candidates(curve, runtime, diagnostics):
        return stopped
    raise ValueError('No known-free C2 terminal brake for native trajectory')


def native_prefix(curve, end):
    """Cut position and independent yaw polynomials without changing their jets."""
    if not 0 < end <= curve.duration:
        raise ValueError('Invalid native prefix duration')
    def cut(durations, coefficients):
        knots=np.r_[0.,np.cumsum(durations)]
        i=min(len(durations)-1,int(np.searchsorted(knots,end,side='left')-1))
        result=np.array(coefficients[:i+1],copy=True)
        last=end-knots[i]
        result[-1]*=(last/durations[i])**np.arange(6).reshape((6,)+(1,)*(result.ndim-2))
        return np.r_[durations[:i],last],result
    durations,coefficients=cut(curve.durations,curve.coefficients)
    yd,yc=cut(curve.yaw_durations,curve.yaw_segments)
    return ContinuousTrajectory(durations,coefficients,curve.yaw,float(curve.yaw_sample(end)-curve.yaw),
        'native_racer_bspline_prefix',yaw_durations=yd,yaw_segments=yc)


def frontier_end_times(curve, runtime):
    """Bound the search to sixteen nearby stops plus geometric fallback cuts."""
    yield curve.duration
    if curve.duration <= .1:
        return
    left,right=0.,curve.duration
    for _ in range(8):
        middle=(left+right)/2
        if runtime.safe_curve(native_prefix(curve,middle)):
            left=middle
        else:
            right=middle
    for end in np.r_[np.maximum(np.arange(left,left-3.2,-.2), .1),left/2,left/4,.1]:
        if .1<=end<curve.duration:
            yield float(end)
        if end==.1:
            break


def safe_native_execution(curve, runtime, *, validate_stop=None, reserve_horizon=None, diagnostics=None):
    """Authorize the longest tested native prefix with a known-free stop.

    If the reserve validator only depends on a fixed reaction horizon, pass
    that horizon to avoid rerunning it for each brake after the same prefix.
    """
    frontier = getattr(runtime, 'frontier_sitl', False)
    stepped = frontier or os.environ.get('ASPIRE_FRONTIER_STEPS') == '1' and hasattr(runtime, 'safe_curve')
    if frontier:
        ends = frontier_end_times(curve,runtime)
    elif stepped:
        # Take a conservative stop inside measured space instead of repeatedly
        # testing nearly identical long candidates against an aging snapshot.
        ends = [curve.duration]
        while ends[-1] > .5:
            ends.append(max(.5, ends[-1] * .5))
    else:
        ends = np.r_[curve.duration,np.arange(curve.duration-.25,.49,-.25)]
    for end in ends:
        prefix=curve if end==curve.duration else native_prefix(curve,float(end))
        status = dict(prefix_s=float(end))
        if diagnostics is not None: diagnostics.append(status)
        if stepped and not runtime.safe_curve(prefix):
            status['stage']='continuous_prefix'
            continue
        if not frontier and not runtime.safe_path(prefix.path()):
            status['stage']='sampled_prefix'
            continue
        attempts = [] if diagnostics is not None else None
        if attempts is not None: status['brake_attempts']=attempts
        status['stage']='no_legal_brake'
        reserve_before_brake = reserve_horizon is not None and prefix.duration > reserve_horizon
        for stopped, brake in _brake_candidates(prefix,runtime,attempts):
            brake_time = brake.duration if brake is not None else 0.
            if stepped and brake is not None and not runtime.safe_curve(brake):
                status['stage']='continuous_brake'
                status.pop('detail',None)
                if attempts is not None: attempts.append((brake_time,'continuous_brake'))
                continue
            if not frontier and brake is not None and not runtime.safe_path(stopped.path()):
                status['stage']='sampled_stopped_path'
                if attempts is not None: attempts.append((brake_time,'sampled_stopped_path'))
                continue
            if frontier and validate_stop is not None:
                try:
                    validate_stop(stopped)
                except ValueError as error:
                    status.update(stage='stopping_reserve', detail=str(error))
                    if attempts is not None: attempts.append((brake_time,'stopping_reserve'))
                    if reserve_before_brake: break
                    continue
            status['stage']='accepted'
            status.pop('detail',None)
            return stopped,float(end)
    raise ValueError('No known-free native prefix with C2 stop')
