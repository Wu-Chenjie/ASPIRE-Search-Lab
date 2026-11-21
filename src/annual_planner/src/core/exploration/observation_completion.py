"""A processed native sensor frame proves an endpoint observation, never a forecast."""
from collections import deque
import hashlib
import json
import math
import numpy as np


class ObservationCompletion:
    def __init__(self, source, max_age_s=3.):
        self.source = source; self.max_age_s = max_age_s
        self.frames = deque(maxlen=16); self.session = None; self.sequence = -1
        self.retired_sessions = set(); self.last_time = -math.inf
        self.rejected_reason = None
        self.wait_reason = 'no_integrated_frame'

    def ingest(self, packet, now, grid=None):
        try:
            session = packet['sensor_session']; sequence = packet['sequence']; stamp = float(packet['time'])
            point = np.asarray(packet['sensor_position'], float); yaw = float(packet['sensor_yaw'])
            pose_time = float(packet['pose_time']); version = packet['map_version']
            shape = packet['shape']; origin = np.asarray(packet['origin'], float); resolution = float(packet['resolution'])
            if (packet['source'] != self.source or packet.get('sensor') != 'gz_depth' or
                    packet.get('integration_completed') is not True or packet.get('ray_cell_count', 0) <= 0 or
                    packet.get('point_count', 0) <= 0 or not isinstance(session, str) or not session or
                    type(sequence) is not int or sequence < 1 or type(version) is not int or version < 0 or
                    point.shape != (3,) or origin.shape != (3,) or len(shape) != 3 or
                    any(type(v) is not int or v <= 0 for v in shape) or resolution <= 0 or
                    not np.isfinite([*point, *origin, yaw, pose_time, stamp, resolution]).all() or
                    abs(pose_time-stamp) > .15 or not -.1 <= now-stamp <= self.max_age_s):
                raise ValueError('invalid_sensor_frame')
            digest = hashlib.sha256(json.dumps([shape, origin.tolist(), resolution], separators=(',', ':')).encode()).hexdigest()
            if grid and digest != grid:
                raise ValueError('sensor_grid_mismatch')
            if session in self.retired_sessions or stamp <= self.last_time or (session == self.session and sequence <= self.sequence):
                raise ValueError('duplicate_or_late_frame')
            if session == self.session and self.frames and version < self.frames[-1]['map_version']:
                raise ValueError('map_version_regressed')
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.rejected_reason = str(exc); return False
        if self.session is not None and self.session != session:
            self.retired_sessions.add(self.session); self.frames.clear()
        self.session = session; self.sequence = sequence; self.last_time = stamp
        # Copy only the small completion proof, not thousands of map cells.
        self.frames.append(dict(sensor_session=session, sequence=sequence, time=stamp,
                                position=point.tolist(), yaw=yaw, map_version=version,
                                shape=shape, origin=origin.tolist(), resolution=resolution,
                                grid=digest, ray_cell_count=packet['ray_cell_count']))
        self.rejected_reason = None; return True

    def proof(self, now, arrived_at, goal, yaw, grid=None):
        if arrived_at is None:
            self.wait_reason = 'not_at_endpoint'
            return None
        try:
            goal = np.asarray(goal, float)
            if goal.shape != (3,) or not np.isfinite([*goal, now, arrived_at, yaw]).all():
                raise ValueError('Invalid goal')
        except (ValueError, TypeError):
            self.wait_reason = 'invalid_view_geometry'
            return None
        self.wait_reason = 'no_integrated_frame'
        for index, frame in enumerate(reversed(self.frames)):
            age = now - frame['time']
            if frame['time'] + .35 < arrived_at:
                reason = 'awaiting_post_arrival_integrated_frame'
                earlier_frames_ineligible = True
            elif not 0 <= age <= self.max_age_s:
                reason = 'integrated_frame_stale'
                # A frame in the future can still have an eligible predecessor.
                earlier_frames_ineligible = age >= 0
            else:
                earlier_frames_ineligible = False
                angle = abs(math.atan2(math.sin(frame['yaw']-yaw), math.cos(frame['yaw']-yaw)))
                reason = ('sensor_position_outside_view' if np.linalg.norm(np.asarray(frame['position'])-goal) >= .15 else
                          'sensor_yaw_outside_view' if angle >= .20 else
                          'sensor_grid_mismatch' if grid and frame['grid'] != grid else None)
            if reason is None:
                self.wait_reason = None
                return dict(frame, completion_reason='post_arrival_integrated_sensor_frame', arrived_at=arrived_at)
            if index == 0:
                self.wait_reason = reason
            if earlier_frames_ineligible:
                break
        return None


def completed_execution_proof(packet):
    """Recognize a latched endpoint receipt, not an ephemeral reason string.

    This checks the report's identity/time binding. Independent replay must also
    match its (session, sequence) against the recorded integration packet.
    """
    proof = packet.get('observation_proof')
    if packet.get('flight_session_fault') or not isinstance(proof, dict): return False
    try:
        times = [float(proof[k]) for k in ('arrived_at', 'time', 'completed_at')]
        return bool(packet.get('arrived') is True and packet.get('token') and
                    type(packet.get('epoch')) is int and packet['epoch'] > 0 and
                    proof['token'] == packet['token'] and proof['epoch'] == packet['epoch'] and
                    proof['completion_reason'] == 'post_arrival_integrated_sensor_frame' and
                    isinstance(proof['sensor_session'], str) and proof['sensor_session'] and
                    type(proof['sequence']) is int and proof['sequence'] > 0 and
                    proof['ray_cell_count'] > 0 and all(math.isfinite(t) for t in times) and
                    times[0] - .35 - 1e-6 <= times[1] <= times[2])
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def failed_observation_proof(packet, processing_time=None):
    """Recognize an authenticated failed task, never successful observation.

    The caller independently checks its latest maps before releasing native
    intent. This packet does not cancel/change the executor's fixed reference.
    """
    failure=packet.get('observation_failure');hold=packet.get('terminal_hold')
    if (not isinstance(failure,dict) or not isinstance(hold,dict) or
            packet.get('arrived') or packet.get('observation_proof') is not None or
            packet.get('safety_fault') or packet.get('flight_session_fault') or not packet.get('ready')):return False
    try:
        now=float(packet['time']);processing=now if processing_time is None else float(processing_time)
        closed=float(failure['closed_at']);deadline=float(failure['observation_deadline'])
        since=float(failure['started_at']);checked=float(hold['checked_at'])
        ref=np.asarray(packet['reference'],float);actual=np.asarray(packet['actual_position'],float)
        jet=np.linalg.norm(packet['reference_velocity'])+np.linalg.norm(packet['reference_acceleration'])+abs(packet['reference_yaw_rate'])
        wait_s=float(hold.get('wait_budget_s',3.))
        return bool(packet.get('token') and failure['token']==packet['token'] and
            type(packet['epoch']) is int and packet['epoch']>0 and failure['epoch']==packet['epoch'] and
            failure['outcome']=='unobserved' and failure['reason']=='terminal_observation_wait_deadline_exceeded' and
            np.isfinite([now,closed,deadline,since,checked,*ref,*actual,jet,wait_s]).all() and
            -.05<=processing-now<=.5 and -.1<=processing-float(packet['position_stamp'])<=.5 and
            0<=now-checked<=.5 and deadline<closed<=checked and 0<wait_s<=5. and abs(deadline-since-wait_s)<1e-6 and
            type(failure['mission_failure_count']) is int and failure['mission_failure_count']>=1 and
            ref.shape==(3,) and actual.shape==(3,) and np.linalg.norm(ref-actual)<=.25 and jet<=1e-5 and
            abs(packet['trajectory_time']-packet['trajectory_duration'])<1e-9 and
            hold['reference_stopped'] is True and hold['observation_complete'] is False and hold['task_state']=='failed' and
            np.linalg.norm(ref-np.asarray(hold['fixed_endpoint']))<=1e-9)
    except (KeyError,TypeError,ValueError,OverflowError):return False
