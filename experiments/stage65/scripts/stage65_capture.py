"""Explicit passive diagnostics only. No policy/mapper/planner function changes.

Installed by stage65-node-entry.py before the unchanged node is imported.
Snapshots report current objects, including absent native state/curve. They are
never safety-query results, never a source of occupancy or authorization.
"""
import functools,json,time
import numpy as np
from annual_runtime.trigger_evidence import optional_ring


def snapshot(node,role,*,extra=None):
    rep=getattr(node,'safety_replica',None) if role=='executor' else getattr(node,'replica',None)
    m=getattr(node,'map',None) if role=='mapper' else getattr(rep,'map',None)
    native=getattr(node,'native_safety',None)
    state=getattr(native,'state',None)
    clock=node.get_clock().now().nanoseconds
    meta=dict(role=role,callback_sim_ns=int(clock),callback_wall_ns=time.monotonic_ns(),
              diagnostic_only=True,policy_query_performed=False,
              mapper_state_available=m is not None,native_state_available=state is not None,
              curve_present=getattr(node,'curve',None) is not None,
              current_intent_present=bool(getattr(node,'intent',None)),pending_intent_present=bool(getattr(node,'pending',None)),
              flight_fault_reason=getattr(getattr(node,'flight_fault',None),'reason',None),safety_enabled=bool(getattr(node,'safety_enabled',False)),
              token=getattr(node,'token',None),native_plan_id=getattr(node,'native_plan_id',None),
              mapper_stamp=getattr(node,'safety_stamp',getattr(node,'last',None)),
              native_stamp=float(native.stamp) if state is not None else None,
              native_sequence=int(native.sequence) if state is not None else None)
    arrays={}
    if m is not None:
        meta.update(mapper_origin=m.origin.tolist(),mapper_resolution=float(m.resolution),
                    mapper_version=int(m.version),mapper_shape=list(m.state.shape),
                    mapper_clearance=float(m.clearance),flight_limits=list(m.flight_limits),
                    mapper_collision_profile=m.collision_profile,mapper_metric=np.asarray(m.metric).tolist(),
                    reserved_context_present=getattr(m,'reserved_points',None) is not None,
                    mapper_recovery_yaw=getattr(m,'recovery_yaw',None),
                    mapper_cache_matches_state=getattr(m,'_built_signature',(None,))[0]==m.state.tobytes())
        arrays['mapper_state']=m.state
        if meta['reserved_context_present']:
            arrays['mapper_reserved_points']=np.asarray(m.reserved_points.data)
            meta['mapper_reserved_radius']=float(m.reserved_radius)
    if state is not None:
        meta.update(native_origin=native.origin.tolist(),native_resolution=float(native.resolution),native_shape=list(state.shape))
        arrays['native_state']=state
    positions=getattr(node,'positions',{})
    actual=positions.get(getattr(node,'id',None)) if isinstance(positions,dict) else None
    actual=getattr(node,'pose',None) if actual is None else actual
    if actual is not None:arrays['actual_estimated_position']=np.asarray(actual)
    reference=getattr(node,'ref',None)
    if reference is not None:arrays['reference']=np.asarray(reference)
    if extra:meta.update(extra)
    return clock*1e-9,meta,arrays


def immediate(sink,kind,t,meta,arrays,window=None,enqueue=True):
    """Use the original immutable/capped queue without spending post-query credit."""
    pending=sink.pending_window;before=sink.records;sink.pending_window=None
    try:sink.record(kind,t,meta,arrays)
    finally:sink.pending_window=pending
    if sink.records!=before+1:return False
    frame=sink.ring[-1]
    if not enqueue:return True
    if window is None:
        if not sink.windows:return True # retained ring/explicit close tail
        window=sink.windows[-1]
    return sink._enqueue(frame,window,'probe' if kind=='probe_current_state' else 'decision')


def receive(node,sink,role,data):
    before=len(sink.windows)
    # Gateway has no execution-fault query stream; reserve its two windows for
    # startup and bounded rejection context. Executor/mapper retain old handling.
    if role=='gateway':
        try:
            x=json.loads(data)
            if x.get('reason')!='bounded_startup_capture_probe':return
        except Exception:
            sink.write_errors+=1;return
    sink.receive_trigger(data)
    if len(sink.windows)==before:return # wrong session/drone, duplicate or cap
    window=sink.windows[-1]
    if window['trigger']['reason']!='bounded_startup_capture_probe':return
    started=time.monotonic()
    try:
        t,meta,arrays=snapshot(node,role,extra=dict(probe_id=window['trigger']['id']))
        immediate(sink,'probe_current_state',t,meta,arrays,window)
    except Exception:sink.write_errors+=1
    finally:sink.hook_max_wall_s=max(sink.hook_max_wall_s,time.monotonic()-started)


def bind_capture(node,role):
    options=dict(pre_records=12 if role=='mapper' else 48,post_records=2 if role=='mapper' else 4)
    if role=='gateway':options=dict(pre_records=1,post_records=0)
    sink=optional_ring(node.id,role,**options);node.map_evidence=sink
    if sink is None:return
    try:
        from std_msgs.msg import String
        topic=f'/drone_{node.id}/diagnostics/map_evidence_trigger'
        node.map_evidence_pub=node.create_publisher(String,topic,10)
        node.create_subscription(String,topic,lambda m:receive(node,sink,role,m.data),10)
    except Exception:
        sink.write_errors+=1;sink.close();node.map_evidence=None


def capture_rejection(node,row,reason):
    sink=getattr(node,'map_evidence',None)
    if sink is None:return
    count=getattr(node,'_stage65_rejections_seen',0)+1;node._stage65_rejections_seen=count
    if count>3:return # fixed first three, not chosen by outcome/identity
    started=time.monotonic()
    try:
        t,meta,arrays=snapshot(node,'gateway',extra=dict(rejection_ordinal=count,
            rejected_proposal=row,rejection_reason=str(reason),all_rejections_seen=count,
            execution_feedback=getattr(node,'execution',None),last_sensor_stamp=getattr(node,'last_sensor',None)))
        before=sink.records;window=getattr(node,'_stage65_rejection_window',None)
        immediate(sink,'gateway_rejected_proposal',t,meta,arrays,window,enqueue=window is not None)
        if sink.records!=before+1:return
        if window is None:
            trigger=sink.trigger('bounded_first_three_rejections',t,trigger_id='stage65-rejections')
            if trigger:node._stage65_rejection_window=sink.windows[-1]
    except Exception:sink.write_errors+=1
    finally:sink.hook_max_wall_s=max(sink.hook_max_wall_s,time.monotonic()-started)


def install_gateway(cls):
    original_init,original_reject,original_destroy=cls.__init__,cls.reject,cls.destroy_node
    @functools.wraps(original_init)
    def init(self,*args,**kwargs):
        original_init(self,*args,**kwargs);bind_capture(self,'gateway')
    @functools.wraps(original_reject)
    def reject(self,row,reason):
        try:return original_reject(self,row,reason)
        finally:capture_rejection(self,row,reason)
    @functools.wraps(original_destroy)
    def destroy(self,*args,**kwargs):
        sink=getattr(self,'map_evidence',None)
        if sink is not None:sink.close()
        return original_destroy(self,*args,**kwargs)
    cls.__init__,cls.reject,cls.destroy_node=init,reject,destroy
