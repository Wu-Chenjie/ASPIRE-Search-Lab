"""Exact curve boundary checks; no controller or reservation side effects."""
import math
import hashlib
import time
import numpy as np


def future_boundary_time(curve, progress, lead):
    """Earliest future boundary with enough planning/authorization lead.

    Actual old-service gain is checked separately before adoption. Choosing a
    boundary does not authorize leaving, release a lease, or complete a task.
    """
    if not np.isfinite([progress,lead]).all() or progress<0 or lead<=0:
        raise ValueError('Invalid handoff progress or lead')
    boundary=progress+lead
    return float(boundary) if boundary<curve.duration-.4 else None


def moving_boundary_time(curve, progress, lead):
    """Latest feasible boundary before longitudinal deceleration, never clip lead.

    Preparing near the end of cruise gives actual old-service frames time to
    arrive. When preparation is already late, preserve the existing feasible
    boundary during deceleration. Never manufacture a window by clipping lead.
    """
    earliest = future_boundary_time(curve, progress, lead)
    if earliest is None:
        return None
    times = np.linspace(earliest, curve.duration-.4, max(2, int(curve.duration/.05)+1))
    _, velocities, accelerations, _, _ = curve.sample_many(times)
    speeds = np.linalg.norm(velocities, axis=1)
    increasing = np.sum(velocities*accelerations, axis=1) >= -1e-4
    candidates = times[(speeds >= .1) & increasing]
    return float(candidates[-1]) if len(candidates) else earliest


def continuity(old, new, progress):
    left = old.sample(progress); right = new.sample(0.)
    return dict(position=float(np.linalg.norm(left[0]-right[0])),
                velocity=float(np.linalg.norm(left[1]-right[1])),
                acceleration=float(np.linalg.norm(left[2]-right[2])),
                yaw=abs(math.atan2(math.sin(left[3]-right[3]), math.cos(left[3]-right[3]))),
                yaw_rate=abs(left[4]-right[4]),
                yaw_acceleration=abs(old.yaw_acceleration(progress)-new.yaw_acceleration(0.)))


def validate_handoff(old, new, progress, current_progress=0.):
    if not current_progress+.05 < progress < old.duration:
        raise ValueError('Handoff is outside the unconsumed curve')
    errors = continuity(old, new, progress)
    if any(not 0 <= value <= 1e-5 for value in errors.values()):
        raise ValueError('Handoff violates position / C2 / yaw boundary continuity')
    return errors


def curve_fingerprint(curve):
    """Geometry/version identity without the costly diagnostic to_dict metrics."""
    digest = hashlib.sha256()
    for name, value in (('durations', curve.durations), ('coefficients', curve.coefficients),
                        ('yaw_coefficients', curve.yaw_coefficients), ('yaw_durations', curve.yaw_durations),
                        ('yaw_segments', curve.yaw_segments)):
        digest.update(name.encode())
        if value is None:
            digest.update(b'none')
        else:
            array = np.ascontiguousarray(value, dtype='<f8')
            digest.update(str(array.shape).encode())
            digest.update(array.tobytes())
    return digest.hexdigest()


class HandoffBudgetExceeded(ValueError):
    """Candidate validation exceeded its control-cycle window."""


def check_handoff_budget(now, native_stamp, mapper_stamp, started, clock=time.monotonic):
    from .execution_safety import EXECUTION_DELAY_BUDGET, MAX_MAP_AGE
    elapsed = clock()-started
    if not 0 <= elapsed <= EXECUTION_DELAY_BUDGET:
        raise HandoffBudgetExceeded('handoff_safety_check_delay_budget_exceeded')
    for stamp in (native_stamp, mapper_stamp):
        age = now-stamp
        if not (0 <= age and age+elapsed <= MAX_MAP_AGE):
            raise ValueError('handoff_measured_map_stale')
    return elapsed


def validate_safe_handoff(old, new, boundary, current_progress, *, expected_old,
                          now, native_stamp, mapper_stamp, corridor, started,
                          phase='admission', new_progress=0., expected_new=None,
                          old_safety_revision=0, reference_step=None, clock=time.monotonic):
    """Check the actual curves at admission or immediately before transition.

    `started` is the caller's original safety-cycle start, not a reset stopwatch.
    No trajectory, reference, authorization or observation state is mutated.
    """
    from .execution_safety import stopping_reserve
    from .motion_limits import motion_limits

    def budget():
        return check_handoff_budget(now, native_stamp, mapper_stamp, started, clock)

    budget()
    if old.yaw_durations is None or new.yaw_durations is None:
        raise ValueError('handoff_requires_native_piecewise_yaw')
    if type(old_safety_revision) is not int or old_safety_revision != 0:
        raise ValueError('handoff_predecessor_safety_revision_changed')
    if not isinstance(expected_old, str) or curve_fingerprint(old) != expected_old:
        raise ValueError('handoff_predecessor_curve_changed')
    new_identity = curve_fingerprint(new)
    if expected_new is not None and new_identity != expected_new:
        raise ValueError('handoff_candidate_curve_changed')
    if not np.isfinite([boundary, current_progress, new_progress]).all():
        raise ValueError('handoff_progress_invalid')
    if phase == 'admission':
        if new_progress != 0.:
            raise ValueError('handoff_unstarted_curve_progress')
        errors = validate_handoff(old, new, boundary, current_progress)
    elif phase == 'transition':
        # The current reference has not yet crossed the boundary; next_time may
        # cross it during this tick. The admission lead is no longer applicable.
        if not 0 <= current_progress <= boundary < old.duration:
            raise ValueError('handoff_boundary_already_consumed')
        if not 0 <= new_progress < new.duration:
            raise ValueError('handoff_new_progress_invalid')
        if (type(reference_step) not in (int, float)
                # Subtracting two absolute trajectory times has roundoff.
                # This tolerance is arithmetic only; the caller still clamps
                # the commanded controller step to the original .04 seconds.
                or not 0 <= reference_step <= .04+1e-12 or
                abs(boundary-current_progress+new_progress-reference_step) > 1e-9):
            raise ValueError('handoff_reference_step_mismatch')
        errors = continuity(old, new, boundary)
        if any(not 0 <= value <= 1e-5 for value in errors.values()):
            raise ValueError('Handoff violates position / C2 / yaw boundary continuity')
    else:
        raise ValueError('Unknown handoff validation phase')
    budget()
    if not motion_limits().accepts(new):
        raise ValueError('handoff_dynamics_envelope_exceeded')
    endpoint = new.sample(new.duration)
    if (np.linalg.norm(endpoint[1])+np.linalg.norm(endpoint[2])+abs(endpoint[4])+
            abs(new.yaw_acceleration(new.duration)) > 1e-5):
        raise ValueError('handoff_endpoint_not_at_rest')
    budget()
    if not corridor.safe_curve(new, new_progress):
        raise ValueError('handoff_new_curve_not_known_free')
    budget()
    stopping_reserve(new, new_progress, corridor)
    delay = budget()
    return dict(continuity=errors, old_curve_sha256=expected_old,
                new_curve_sha256=new_identity, check_wall_s=delay,
                checked_at=now, phase=phase, new_progress=new_progress)
