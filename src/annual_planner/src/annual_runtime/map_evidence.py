"""Passive mapper/safety capture wiring. No diagnostic value is a policy input."""
import functools,json,time
import numpy as np
from core.planning.execution_safety import ExecutionCorridor, TRACKING_MARGIN
from .trigger_evidence import optional_ring


def bind_capture(node,role):
    # Called from both node constructors, so every evidence attribute exists
    # for the lifetime of the node instead of being probed at each call site.
    node.map_evidence=optional_ring(node.id,role,pre_records=12 if role=='mapper' else 48,post_records=2 if role=='mapper' else 4)
    node.map_evidence_pub=None
    node._map_evidence_received={}
    node._map_evidence_receipt_ordinal=0
    node._map_evidence_query=None
    node._map_evidence_original=None
    sink=node.map_evidence
    if sink is None:return
    try:
        from std_msgs.msg import String
        topic=f'/drone_{node.id}/diagnostics/map_evidence_trigger'
        node.map_evidence_pub=node.create_publisher(String,topic,10)
        node.create_subscription(String,topic,lambda m:sink.receive_trigger(m.data),10)
    except (ImportError,RuntimeError):
        sink.write_errors+=1;sink.close();node.map_evidence=None


def close_capture(node):
    sink=node.map_evidence
    if sink is not None:sink.close()


def mapper_received(node,message):
    """Bounded receipt metadata; never influences queue admission or interpolation."""
    sink=node.map_evidence
    if sink is None:return
    try:
        records=node._map_evidence_received
        ordinal=node._map_evidence_receipt_ordinal+1
        node._map_evidence_receipt_ordinal=ordinal
        records[id(message)]=dict(receipt_ordinal=ordinal,receipt_sim_ns=node.get_clock().now().nanoseconds,
            receipt_wall_ns=time.monotonic_ns(),pending_before=len(node.pending))
        while len(records)>8:records.pop(next(iter(records)))
    except (KeyError,TypeError,ValueError):sink.write_errors+=1


def mapper_before(node,message,stamp,pose):
    sink=node.map_evidence
    if sink is None:return None
    started=time.monotonic()
    try:
        position,attitude,bracket=pose;cal=node.calibration
        receipt=node._map_evidence_received.pop(id(message),None)
        endpoints=[(t,p,r.as_quat()) for t,p,r in node.poses.samples if any(abs(t-b)<=1e-9 for b in bracket)]
        context=dict(metadata=dict(receipt=receipt,consume_sim_ns=node.get_clock().now().nanoseconds,
                    consume_wall_ns=time.monotonic_ns(),pose_endpoint_samples_complete=len(endpoints)==len(set(bracket)),sensor_session=node.session,sequence_before=node.sequence,sensor_stamp_s=stamp,
                    depth_header_stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                    depth_frame=message.header.frame_id,depth_encoding=message.encoding,depth_step=message.step,
                    depth_bigendian=bool(message.is_bigendian),width=message.width,height=message.height,
                    calibration_stamp_ns=cal.header.stamp.sec*10**9+cal.header.stamp.nanosec,
                    calibration_frame=cal.header.frame_id,calibration_width=cal.width,calibration_height=cal.height,
                    pose_bracket=list(bracket),mapper_version_before=node.map.version,
                    map_origin=node.map.origin.tolist(),resolution=node.map.resolution,shape=list(node.map.shape)),
                    arrays=dict(raw_depth=np.frombuffer(bytes(message.data),np.uint8),k=np.asarray(cal.k),
                    estimated_position=np.asarray(position),estimated_quaternion=attitude.as_quat(),
                    mapper_before=node.map.state.copy(),belief_before=node.belief.copy(),
                    ego_cells_before=np.asarray(sorted(node.ego_cells),dtype=np.int64).reshape(-1,3),
                    pose_endpoint_stamps=np.asarray([e[0] for e in endpoints]),
                    pose_endpoint_positions=np.asarray([e[1] for e in endpoints]).reshape(-1,3),
                    pose_endpoint_quaternions=np.asarray([e[2] for e in endpoints]).reshape(-1,4)))
        context['capture_before_wall_s']=time.monotonic()-started
        return context
    except (KeyError,TypeError,ValueError):sink.write_errors+=1;return None


def mapper_after(node,context,packet,points,hits,origin,world):
    sink=node.map_evidence
    if sink is None or context is None:return
    started=time.monotonic()
    try:
        context['metadata'].update(sequence=packet['sequence'],map_version_after=packet['map_version'],
            integration_completed=True,point_count=packet['point_count'],ray_cell_count=packet['ray_cell_count'],
            full_snapshot=packet['full_snapshot'],native_frame_published='native_frame' in packet)
        context['arrays'].update(mapper_after=node.map.state,belief_after=node.belief,points_sensor=points,
            endpoint_hits=hits,sensor_origin=origin,world_points=world,
            packet_indices=np.asarray(packet['indices'],dtype=np.int64),packet_values=np.asarray(packet['values'],dtype=np.int8),
            measured_indices=np.asarray(packet['measured_indices'],dtype=np.int64),measured_values=np.asarray(packet['measured_values'],dtype=np.int8),
            ego_cleared=np.asarray(packet['ego_cleared_indices'],dtype=np.int64).reshape(-1,3))
        sink.record('mapper_consumed_input',packet['time'],context['metadata'],context['arrays'])
    except (KeyError,TypeError,ValueError):sink.write_errors+=1
    finally:sink.hook_max_wall_s=max(sink.hook_max_wall_s,context['capture_before_wall_s']+time.monotonic()-started)


class QueryTraceCorridor(ExecutionCorridor):
    def __init__(self,base):
        super().__init__(base.runtime, base.native, base.margin,
                         relaxed_sitl=base.relaxed_sitl, frontier_sitl=base.frontier_sitl)
        self.query_count=0;self.reject_count=0;self.first_rejects=[];self.trace_errors=0
    def box_safe(self,low,high):
        result=super().box_safe(low,high)
        try:
            self.query_count+=1
            if not result:
                self.reject_count+=1
                if len(self.first_rejects)<8:self.first_rejects.append(dict(low=np.asarray(low).tolist(),high=np.asarray(high).tolist()))
        except (TypeError,ValueError):self.trace_errors+=1
        return result


def evidence_corridor(node,base):
    if node.map_evidence is None:return base
    try:
        traced=QueryTraceCorridor(base);node._map_evidence_query=traced
        node._map_evidence_original=node.curve.to_dict() if node.curve else None
        return traced
    except (TypeError,ValueError):
        node.map_evidence.write_errors+=1;return base


def _capture_safety_result(node,t,start,sink,new_fault,cpu_start):
    q=node._map_evidence_query;fault=node.safety_fault
    if q is None and not fault:return
    m=node.safety_replica.map;n=node.native_safety
    if n.state is None:return
    meta=dict(token=node.token,epoch=node.epoch,native_plan_id=node.native_plan_id,
        processing_sim_s=t,mapper_stamp=node.safety_stamp,mapper_version=m.version,
        native_stamp=n.stamp,native_sequence=n.sequence,origin=m.origin.tolist(),resolution=m.resolution,
        clearance=m.clearance,flight_limits=list(m.flight_limits),tracking_margin=TRACKING_MARGIN,
        corridor_margin=q.margin if q else None, relaxed_sitl=q.relaxed_sitl if q else None,
        frontier_sitl=q.frontier_sitl if q else None,
        position_stamp=node.stamps.get(node.id),actual_yaw=node.actual_yaw,reference_yaw=node.heading,
        reference_yaw_rate=node.yaw_rate,query_trace_errors=q.trace_errors if q else 0,
        query_original=node._map_evidence_original or (node.curve.to_dict() if node.curve else None),
        progress=node.curve_time,query_count=q.query_count if q else 0,
        rejected_boxes=q.reject_count if q else 0,first_rejected_boxes=q.first_rejects if q else [],
        final_curve=node.curve.to_dict() if node.curve else None,fault=fault,
        safety_stopping=node.safety_stopping,stop_deadline=node.safety_stop_deadline,
        observation_failure=node.observation_failure,policy_check_wall_s=time.monotonic()-start,
        policy_check_thread_cpu_s=time.thread_time()-cpu_start)
    arrays=dict(mapper_state=m.state,native_state=n.state,reference=node.ref,actual=node.positions[node.id],
                native_origin=n.origin,native_resolution=np.asarray(n.resolution),reference_velocity=node.velocity,reference_acceleration=node.acceleration)
    sink.record('safety_query',t,meta,arrays)
    if fault and new_fault:
        trigger=sink.trigger('execution_safety_fault',t,details=dict(token=node.token,fault=fault))
        publisher=node.map_evidence_pub
        if trigger and publisher:
            from std_msgs.msg import String
            publisher.publish(String(data=json.dumps(trigger,separators=(',',':'))))

def capture_safety(method):
    @functools.wraps(method)
    def call(node,t):
        sink=node.map_evidence
        if sink is None:return method(node,t)
        had_fault=bool(node.safety_fault)
        if had_fault and sink.pending_window is None:return method(node,t)
        node._map_evidence_query=None;node._map_evidence_original=None;start=time.monotonic();cpu_start=time.thread_time()
        try:return method(node,t)
        finally:
            capture_started=time.monotonic()
            try:
                _capture_safety_result(node,t,start,sink,not had_fault,cpu_start)
            except Exception:
                # Capture runs in finally. Diagnostic failures must preserve the policy outcome.
                sink.write_errors+=1
            sink.hook_max_wall_s=max(sink.hook_max_wall_s,time.monotonic()-capture_started)
    return call
