"""Opt-in, bounded, passive control-chain observations; never a control input."""
import hashlib
import gzip
import json
import math
import os
import queue
import threading
import time
from pathlib import Path


def freeze(value):
    """Detach mutable messages/arrays before the writer thread sees them."""
    if isinstance(value, dict): return {str(k): freeze(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [freeze(v) for v in value]
    if hasattr(value, 'tolist'): return freeze(value.tolist())
    if isinstance(value, float) and not math.isfinite(value): return str(value)
    if value is None or isinstance(value, (str, bool, int, float)): return value
    if hasattr(value, 'item'): return freeze(value.item())
    return str(value)


class ObservationLog:
    def __init__(self, directory, drone, role, *, capacity=8192, max_bytes=128*1024*1024, compress=False):
        self.context = {}; self.counts = {}; self.enqueued = 0; self.written = 0
        self.dropped = 0; self.write_errors = 0; self.bytes = 0; self.closed = False
        self.max_bytes = max_bytes; self.queue = queue.Queue(maxsize=capacity)
        self.meta = dict(schema=1, session=os.environ.get('RACER_OBSERVATION_SESSION', 'unspecified'), drone=int(drone), role=role)
        self.compress=bool(compress)
        base=f'drone-{drone}-{role}-observation'
        self.path = Path(directory)/(base+'.jsonl'+('.gz' if self.compress else ''))
        self.status_path = Path(directory)/(base+'.status.json'); self.file = None
        self.thread = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.file = (gzip.open(self.path,'xt',encoding='utf-8',compresslevel=1) if self.compress
                         else self.path.open('x', buffering=65536))
            self.thread = threading.Thread(target=self._write, daemon=True); self.thread.start()
        except OSError:
            self.write_errors += 1

    def event(self, kind, **fields):
        if self.closed: return
        self.counts[kind] = self.counts.get(kind, 0)+1
        row = freeze(dict(self.meta, event=kind, event_seq=self.counts[kind], wall_ns=time.monotonic_ns(), **fields))
        if self.file is None:
            self.dropped += 1; return
        try: self.queue.put_nowait(row); self.enqueued += 1
        except queue.Full: self.dropped += 1

    def _write(self):
        while True:
            row = self.queue.get()
            try:
                if row is None: return
                line = json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n'
                size = len(line.encode())
                if self.bytes+size > self.max_bytes:
                    self.dropped += 1; continue
                self.file.write(line); self.bytes += size; self.written += 1
            except (OSError, ValueError, TypeError): self.write_errors += 1
            finally: self.queue.task_done()

    def close(self):
        if self.closed: return
        self.closed = True
        if self.thread is not None:
            try: self.queue.put(None, timeout=2.)
            except queue.Full: self.write_errors += 1
            self.thread.join(timeout=5.)
        alive = bool(self.thread and self.thread.is_alive())
        if self.file is not None and not alive:
            try: self.file.close()
            except OSError: self.write_errors += 1
        try:
            stored_bytes = self.path.stat().st_size
        except OSError:
            stored_bytes = 0
        status = dict(self.meta, enqueued=self.enqueued, written=self.written, dropped=self.dropped,
                      write_errors=self.write_errors, bytes=self.bytes, counts=self.counts,
                      writer_alive=alive, pending=self.queue.qsize(), max_bytes=self.max_bytes,
                      compression='gzip' if self.compress else 'none', byte_budget_basis='uncompressed_utf8',
                      stored_bytes=stored_bytes)
        try: self.status_path.write_text(json.dumps(status, indent=2)+'\n')
        except OSError: pass  # An observation failure must never change control.


def record(node, kind, supplier):
    """Lazy snapshots; no work except lookup when disabled. Never raise to control."""
    sink = node.control_timing.observation
    if sink is None: return
    try:
        fields = supplier()
        sink.event(kind, sim_ns=int(node.get_clock().now().nanoseconds), **fields)
    except Exception as error:
        try:
            sink.event('observation_error', source_event=kind, error=repr(error))
        except Exception:
            sink.write_errors += 1


def xyz(v): return [float(v.x), float(v.y), float(v.z)]


def reference_payload(m):
    p = m.points[0]; q = p.transforms[0].rotation
    return dict(frame=m.header.frame_id, position=xyz(p.transforms[0].translation),
                quaternion=[q.x, q.y, q.z, q.w], velocity=xyz(p.velocities[0].linear),
                acceleration=xyz(p.accelerations[0].linear), yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                yaw_rate=float(p.velocities[0].angular.z))


BRIDGE_STATE_FIELDS = (
    'origin', 'position', 'position_sample_us', 'resets',
    'attitude_reset', 'attitude_sample', 'reset_pending', 'alignment_fault',
)


def reference_record(node, m, *, accepted=False):
    payload = reference_payload(m)
    stamp = int(m.header.stamp.sec)*1_000_000_000+int(m.header.stamp.nanosec)
    key = dict(stamp_ns=stamp, payload_sha256=hashlib.sha256(json.dumps(freeze(payload), sort_keys=True, separators=(',', ':')).encode()).hexdigest())
    sink = node.control_timing.observation
    if accepted:
        sink.context['reference_key'] = key
    # Schema 1 also wrote these fields as null for executor events.
    state = bridge_state(node) if sink.meta['role'] == 'bridge' else dict.fromkeys(BRIDGE_STATE_FIELDS)
    return dict(reference_key=key, enu=payload, **state)


def bridge_state(node):
    values = (node.origin, node.position, node.last_sample, node.resets,
              node.attitude_reset, node.attitude_sample, node.position_reset_pending, node.alignment_fault)
    return dict(zip(BRIDGE_STATE_FIELDS, values))


def setpoint_record(node, m):
    return dict(reference_key=node.control_timing.observation.context.get('reference_key'),
                target_timestamp_us=int(m.timestamp), ned=dict(position=list(m.position), velocity=list(m.velocity),
                acceleration=list(m.acceleration), yaw=float(m.yaw), yaw_rate=float(m.yawspeed)), **bridge_state(node))


def local_record(node, m):
    keys=('timestamp','timestamp_sample','x','y','z','vx','vy','vz','z_deriv',
          'xy_valid','z_valid','v_xy_valid','v_z_valid','xy_reset_counter','z_reset_counter',
          'vxy_reset_counter','vz_reset_counter','heading_reset_counter','delta_xy','delta_z','delta_vxy','delta_vz','delta_heading')
    return dict(raw_ned={k:getattr(m,k) for k in keys if hasattr(m,k)}, **bridge_state(node))


def odometry_record(node, m):
    return dict(stamp_ns=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec), frame=m.header.frame_id,
                child_frame=m.child_frame_id, position=xyz(m.pose.pose.position),
                quaternion=[getattr(m.pose.pose.orientation,k) for k in ('x','y','z','w')],
                body_velocity=xyz(m.twist.twist.linear))


def executor_state(node):
    sink=node.control_timing.observation
    return dict(token=node.token, native_id=node.native_plan_id, epoch=node.epoch,
                curve_time=node.curve_time, curve_duration=node.curve.duration if node.curve else None,
                previous_tick=node.last, actual_position=node.positions.get(node.id),
                position_stamp=node.stamps.get(node.id), used_odometry=sink.context.get('odometry'),
                reference=node.ref, reference_velocity=node.velocity, reference_acceleration=node.acceleration,
                reference_yaw=node.heading, yaw_rate=node.yaw_rate, reason=node.reason,
                safety_fault=node.safety_fault, stopping=node.safety_stopping, stop_deadline=node.safety_stop_deadline,
                native_map_stamp=node.native_safety.stamp, native_map_sequence=node.native_safety.sequence,
                mapper_stamp=node.safety_stamp)


def executor_odometry(node, m):
    value=odometry_record(node,m);node.control_timing.observation.context['odometry']=freeze(value);return value
