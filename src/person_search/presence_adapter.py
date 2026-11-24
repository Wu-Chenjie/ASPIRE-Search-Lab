"""ROS adapter for decoupled presence; original mapping/control depth is untouched."""
from bisect import bisect_left
import json
import numpy as np
from .node import matrix,rgb_pixels,newest_bracketed_capture,capture_age_valid
from .segmentation import localize_masks
from aspire_camera.rgbd_alignment import require_registered_pair,stamp_ns


def capture_world(node,frame,ns,history,Time,*,pose_bracketed=False):
    if node.require_pose and not pose_bracketed and not history.brackets(ns*1e-9):raise ValueError('missing_capture_pose_bracket')
    if not node.fresh_capture(ns):raise ValueError('stale_or_future_capture')
    world=node.buffer.lookup_transform('map',frame,Time(nanoseconds=ns))
    if world.header.frame_id!='map' or world.child_frame_id!=frame:raise ValueError('capture_transform_frame_mismatch')
    tf_ns=stamp_ns(world.header)
    if tf_ns not in (0,ns):raise ValueError('capture_transform_time_mismatch')
    return matrix(world.transform)


def nearest_depth_stamp(stamps,ns,max_skew_ns):
    pos=bisect_left(stamps,ns)
    nearest=min(stamps[max(0,pos-1):pos+1],key=lambda s:(abs(s-ns),s),default=None)
    return nearest if nearest is not None and abs(nearest-ns)<=max_skew_ns else None


def geometry_pair(node,queues,ns):
    if ns not in queues['rgb'] or ns not in queues['ri']:return None
    if node.rgbd_registration_mode=='strict':
        if ns not in queues['depth'] or ns not in queues['di']:return None
        depth_ns=ns
    else:
        depth_ns=nearest_depth_stamp(sorted(queues['depth'].keys() & queues['di'].keys()),ns,node.rgbd_max_skew_s*1e9)
        if depth_ns is None:return None
    return (queues['rgb'][ns],queues['depth'][depth_ns],queues['ri'][ns],queues['di'][depth_ns])


def prepare_geometry(node,images,history,Time):
    from aspire_camera.rgbd_registration import decode_depth,camera_info_model,CaptureTransform,register_depth
    rgb,depth,ri,di=images;ns=stamp_ns(rgb.header);dns=stamp_ns(depth.header)
    if node.rgbd_registration_mode=='strict':
        transform=capture_world(node,depth.header.frame_id,ns,history,Time)
        registration=node.buffer.lookup_transform(rgb.header.frame_id,depth.header.frame_id,Time(nanoseconds=ns))
        require_registered_pair(rgb,depth,ri,di,registration)
        data=decode_depth(depth.data,depth.width,depth.height,depth.step,depth.encoding,depth.is_bigendian)
        import hashlib
        calibration=dict(width=ri.width,height=ri.height,frame=ri.header.frame_id,
            k=list(ri.k),p=list(ri.p),r=list(ri.r),d=list(ri.d))
        identity=hashlib.sha256(json.dumps(calibration,sort_keys=True,allow_nan=False).encode()).hexdigest()
        return data,di.k,transform,dict(rgb_stamp_ns=ns,depth_stamp_ns=dns,skew_s=0.,mode='strict',calibration_id=identity)
    if dns!=ns:
        if not capture_age_valid(dns,node.get_clock().now().nanoseconds,node.max_age):raise ValueError('stale_registration_capture')
        if node.require_pose and not history.brackets(dns*1e-9):raise ValueError('missing_depth_capture_pose_bracket')
    rc,rid=camera_info_model(ri,rgb,node.rgbd_image_mode)
    dc,did=camera_info_model(di,depth,node.rgbd_image_mode)
    transform=capture_world(node,rc.frame_id,ns,history,Time)
    # tf2 full queries bind BOTH acquisition times through a fixed frame. A
    # changed output header alone cannot compensate camera motion.
    registered=node.buffer.lookup_transform_full(rc.frame_id,Time(nanoseconds=ns),dc.frame_id,Time(nanoseconds=dns),'map')
    if registered.header.frame_id!=rc.frame_id or registered.child_frame_id!=dc.frame_id or stamp_ns(registered.header) not in (0,ns):
        raise ValueError('cross_time_transform_provenance_mismatch')
    motion=CaptureTransform(matrix(registered.transform),dc.frame_id,rc.frame_id,dns,ns)
    data=decode_depth(depth.data,depth.width,depth.height,depth.step,depth.encoding,depth.is_bigendian)
    data,quality=register_depth(data,dc,rc,motion,dns,ns,max_skew_s=node.rgbd_max_skew_s,static_scene=node.rgbd_static_scene)
    quality.update(mode='reproject',calibration_id=rid+':'+did)
    return data,rc.k,transform,quality


def rgb_capture_intrinsics(infos,ns,frame,width,height,max_age):
    """Causal calibration for fixed-intrinsics RGB; never reuse a camera pose.

    CameraInfo/RGB DDS callbacks can arrive separately. A preceding same-frame
    calibration within the original capture-age window is valid for this fixed
    camera; depth registration still requires its original exact capture pair.
    """
    stamp = max((s for s, info in infos.items() if 0<=ns-s<=max_age*1e9
                 and info.header.frame_id == frame and (info.width, info.height) == (width, height)),
                default=None)
    if stamp is None:raise ValueError('missing_causal_rgb_calibration')
    return infos[stamp].k, stamp


def bind_identity_context(node,ns,rgb,queues,history,Time):
    """Capture-only association context. Never a current bearing/XYZ authority."""
    from tf2_ros import TransformException
    try:
        if node.rgbd_registration_mode=='reproject':
            from aspire_camera.rgbd_registration import camera_info_model
            info=queues['ri'].get(ns)
            if info is None:raise ValueError('missing_exact_rgb_calibration')
            model,calibration_id=camera_info_model(info,rgb,node.rgbd_image_mode)
            k=model.k;calibration_ns=ns
        else:
            k,calibration_ns=rgb_capture_intrinsics(queues['ri'],ns,rgb.header.frame_id,
                rgb.width,rgb.height,node.max_age)
            calibration_id=None
        transform=capture_world(node,rgb.header.frame_id,ns,history,Time)
        from .presence import association_view
        view=association_view(k,transform,rgb.width,rgb.height)
        view['calibration_id']=calibration_id
        return dict(view=view,k=np.array(k,copy=True),transform=transform,
            calibration_ns=calibration_ns,bound_sim_ns=node.get_clock().now().nanoseconds),None
    except (ValueError,TransformException) as error:return None,str(error)

def snapshot_inputs(node):
    from copy import copy
    with node.input_lock:
        queues = {k: q.copy() for k, q in node.queues.items()}
        history = copy(node.pose_history)
        history.samples = node.pose_history.samples.copy()
    return queues, history


def select_presence_capture(node, queues, history, now_ns):
    last_rgb = node.tracker.last_rgb_ns
    fresh = {ns for ns in queues['rgb'] if ns > last_rgb}
    due = {ns for ns in fresh if last_rgb < 0 or ns - last_rgb >= node.min_rgb_period_ns}
    collecting_depth = node.registry_mode and any(
        t.visual_state == 'visually_confirmed' and t.last_reliable_location is None
        and t.last_capture_ns == last_rgb for t in node.tracker.tracks)
    # During review also use registered pairs between normal RGB inference ticks.
    candidates = ((fresh if collecting_depth else due) |
                  (node.tracker.frames.keys() & queues['rgb'].keys())) & queues['ri'].keys()
    depth_stamps = queues['depth'].keys() & queues['di'].keys()
    if node.rgbd_registration_mode=='strict':
        paired=[ns for ns in candidates & depth_stamps if ns>node.last_ns]
    else:
        depth_stamps=sorted(depth_stamps)
        paired=[ns for ns in candidates if ns>node.last_ns and nearest_depth_stamp(
            depth_stamps,ns,node.rgbd_max_skew_s*1e9) is not None]
    geo_ns = newest_bracketed_capture(paired, history if node.require_pose else None, now_ns, node.max_age)
    if geo_ns is not None:
        return geo_ns, geo_ns, None
    ns = newest_bracketed_capture(due, None, now_ns, node.max_age)
    return ns, None, max(paired, default=None)


def discard_superseded_rgb(node,ns):
    """Newest-first sampling: retire un-observed older frames instead of queueing them.

    A capture older than the selected one can only be observed later, which the
    tracker already rejects. Frames the tracker did observe stay while their depth
    pass may still be pending, then retire silently once they leave the pairing
    window so they are not counted twice.

    The caller must be the single inference thread: only then can a just
    selected capture not be retired while it is being inferred.
    """
    frames=node.tracker.frames
    horizon=ns-int(float(getattr(node,'max_age',1.5))*1e9)
    with node.input_lock:
        older=[s for s in node.queues['rgb'] if s<ns]
        dead=[s for s in older if s not in frames]
        expired=[s for s in older if s in frames and s<horizon]
        for stale in dead+expired:
            del node.queues['rgb'][stale]
            getattr(node,'received_wall',{}).pop(stale,None)
        node.rgb_skipped_total=getattr(node,'rgb_skipped_total',0)+len(dead)
    return len(dead)+len(expired)


def process_presence(node,queues,history):
    from rclpy.time import Time
    from tf2_ros import TransformException
    from cv2 import error as OpenCVError
    now_ns = node.get_clock().now().nanoseconds
    now = now_ns * 1e-9
    tracker = node.tracker
    tracker.expire(now)
    # Complete a paired capture first; otherwise RGB does not depend on depth/TF.
    ns, geo_ns, stale_pair_ns = select_presence_capture(node, queues, history, now_ns)
    if ns is not None and discard_superseded_rgb(node,ns):
        queues=snapshot_inputs(node)[0]
    geo_pair=None
    prepared=None
    preparation_error=None
    if ns is not None:
        rgb=queues['rgb'][ns]
        try:
            previous_rgb=tracker.last_rgb_ns
            if ns>previous_rgb:
                context,context_error=bind_identity_context(node,ns,rgb,queues,history,Time)
            detections=node.detect_semantics(ns,rgb)
            # Inference runs outside the input lock. Its initial queue/pose
            # snapshot can be obsolete by completion: retrying the SAME copy
            # previously split IDs and published navigation before ready depth.
            # Refresh once, with no sleep and no delay of RGB confirmation.
            queues, history = snapshot_inputs(node)
            if queues['rgb'].get(ns) is not rgb:
                queues['rgb'].pop(ns,None)
            geo_pair=geometry_pair(node,queues,ns) if ns>node.last_ns else None
            if geo_pair is None:
                geo_ns=None
            elif geo_ns is None and capture_age_valid(ns,node.get_clock().now().nanoseconds,node.max_age):
                if not node.require_pose or history.brackets(ns*1e-9):geo_ns=ns
            if ns<=previous_rgb:
                node.timing(ns,'rgb_presence_geometry_only')
            else:
                # Keep the context bound before inference when it was available.
                if context is None:
                    context,context_error=bind_identity_context(node,ns,rgb,queues,history,Time)
                view=context['view'] if context is not None else None
                if context is not None:
                    if node.evidence_dir:
                        node.save_evidence(str(ns)+'-rgb-association.npz',
                            k=context['k'],camera_to_map=context['transform'],
                            calibration_stamp_ns=context['calibration_ns'],
                            identity_context_bound_sim_ns=context['bound_sim_ns'],
                            identity_only_not_current_navigation=True)
                else:node.timing(ns,'rgb_association_image_only',context_error)
                paired=None
                if node.review_depth_continuity and view is not None and geo_ns==ns:
                    from .presence import association_view
                    try:
                        prepared=prepare_geometry(node,geo_pair,history,Time)
                        depth,intrinsics,transform,registration=prepared
                        paired_view=association_view(intrinsics,transform,rgb.width,rgb.height)
                        paired_view['calibration_id']=registration['calibration_id']
                        paired=(depth,paired_view)
                    except ValueError as error:
                        if prepared is None:preparation_error=error
                    except TransformException:
                        pass  # TF delivery may complete before the localization step.
                tracker.observe_rgb(detections,ns,rgb.width,rgb.height,node.get_clock().now().nanoseconds*1e-9,view=view,paired=paired)
                if any(event['event']=='visual_confirmed' for event in tracker.events):
                    publish_presence_events(node)
                    if node.registry_mode:
                        processing = node.get_clock().now().nanoseconds*1e-9
                        node.registry_evidence.capture(tracker,ns,None,processing)
                        publish_registry_observations(node,ns,processing)
                if tracker.last_rgb_ns>previous_rgb:
                    node.rgb_consumed_total+=1;node.last_rgb_consumed_ns=tracker.last_rgb_ns
                    node.timing(ns,'rgb_presence_consumed')
                else:node.timing(ns,'rgb_presence_rejected','outside_original_capture_age_gate')
            node.last_semantic_ns=max(node.last_semantic_ns,ns)
        except (ValueError,RuntimeError,OSError,OpenCVError) as e:
            node.timing(ns,'rgb_rejected',str(e));publish_presence(node);return
        # Same ray path/native packet as before, with no confirmed-state exclusion.
        node.bearing_capture(ns,queues,history)
    trace_input=None
    if geo_ns is not None and geo_ns in tracker.frames:
        try:
            if preparation_error is not None:raise preparation_error
            data,intrinsics,transform,registration_quality=(prepared if prepared is not None else
                prepare_geometry(node,geo_pair,history,Time))
            node.timing(geo_ns,'surface_start')
            indices=[a['raw_index'] for a in tracker.frames[geo_ns]]
            kept=[detections[i] for i in indices]
            if node.registry_mode:
                from .surface import localize_surfaces
                kept_rows,supports=localize_surfaces(kept,data,intrinsics,transform,registration_quality=registration_quality)
            else:kept_rows,_=localize_masks(kept,data,intrinsics,transform)
            # Keep the detector indices used by the tracks and evidence files.
            rows=[None]*len(detections)
            for i,row in zip(indices,kept_rows):rows[i]=row
            node.timing(geo_ns,'surface_complete')
            if not node.fresh_capture(geo_ns):raise ValueError('stale_after_inference')
            if node.registry_mode:node.registry_evidence.capture(tracker,geo_ns,rows,node.get_clock().now().nanoseconds*1e-9)
            else:tracker.localize(geo_ns,rows,node.get_clock().now().nanoseconds*1e-9)
            if node.registry_mode and node.static_visibility_config is not None:
                from .visibility_evidence import visibility_batch
                node.pending_visibility=visibility_batch(node.registry_evidence,tracker,geo_ns,data,intrinsics,transform,
                    node.get_clock().now().nanoseconds*1e-9,node.static_visibility_config,node.static_visibility_model)
            node.processed+=1;node.timing(geo_ns,'accepted')
            if node.trace_detector is not None:
                trace_input=rgb,data,intrinsics,transform,registration_quality
            if node.registry_mode and node.evidence_dir:
                indexed_supports=np.zeros((len(detections),*data.shape),dtype=bool)
                if indices:indexed_supports[indices]=supports
                node.save_evidence(str(geo_ns)+'-supports.npz',supports=indexed_supports)
            if node.evidence_dir:node.save_evidence(str(geo_ns)+'-geometry.npz',stamp_ns=geo_ns,depth=data,k=intrinsics,camera_to_map=transform,registration_json=json.dumps(registration_quality))
        except (ValueError,RuntimeError,OSError,OpenCVError,TransformException) as e:
            node.rejected+=1;tracker.reject_localization(geo_ns,node.get_clock().now().nanoseconds*1e-9,str(e));node.timing(geo_ns,'localization_rejected',str(e))
            if node.registry_mode:node.registry_evidence.reject(tracker,geo_ns,node.get_clock().now().nanoseconds*1e-9,str(e))
        node.last_ns=geo_ns
    elif ns is not None and ns>node.last_ns:
        tracker.pending_localization(ns,node.get_clock().now().nanoseconds*1e-9,'missing_registered_capture_pair_or_pose')
        if node.registry_mode:node.registry_evidence.capture(tracker,ns,None,node.get_clock().now().nanoseconds*1e-9)
    elif ns is None and stale_pair_ns in tracker.frames:
        tracker.reject_localization(stale_pair_ns, now, 'stale_or_unbracketed_localization_capture')
    node.publish_bearings(node.get_clock().now().nanoseconds*1e-9)
    publish_presence(node)
    if trace_input is not None and node.fresh_capture(geo_ns):
        from std_msgs.msg import String
        rgb,data,intrinsics,transform,registration_quality=trace_input
        try:
            detections=node.trace_detector.detect(rgb_pixels(rgb))
            if not node.fresh_capture(geo_ns):return
            hints=node.trace_pipeline.observe(detections,data,intrinsics,transform,geo_ns,
                registration_quality=registration_quality,now_ns=node.get_clock().now().nanoseconds)
            if node.fresh_capture(geo_ns):
                node.trace_pub.publish(String(data=json.dumps(dict(schema=1,kind='search_hints',hints=hints))))
        except Exception as error:
            node.get_logger().warning(f'Trace observations unavailable: {error!r}', throttle_duration_sec=5.)


def presence_markers(report,stamp):
    from visualization_msgs.msg import Marker,MarkerArray
    from geometry_msgs.msg import Point
    arr=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;arr.markers.append(clear)
    for t in report['tracks']:
        point=t['position'] if t['localization_state']=='reliable' else None
        geometry=t['bearing'] if t['bearing_state']=='fresh' else None
        if point:
            m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns='reliable_visible_surface';m.id=t['id'];m.type=Marker.SPHERE;m.action=Marker.ADD;m.pose.orientation.w=1.
            m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,point['xyz']);m.scale.x=m.scale.y=m.scale.z=.2;m.color.a=.9;m.color.b=1.;m.color.g=.7;arr.markers.append(m);anchor=np.array(point['xyz'])
        elif geometry:
            origin=np.array(geometry['origin']);direction=np.array(geometry['direction']);m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns='rgb_bearing_no_range';m.id=t['id'];m.type=Marker.LINE_LIST;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=.025;m.color.a=.8;m.color.g=1. if t['visual_state']=='visually_confirmed' else .5;m.color.r=.2
            # Fixed 2m display length is not a person range or navigation endpoint.
            for direction in [direction,*map(np.array,geometry['corner_rays'])]:
                for p in (origin,origin+direction*2.):m.points.append(Point(x=float(p[0]),y=float(p[1]),z=float(p[2])))
            arr.markers.append(m);anchor=origin
        else:continue # Image/status retain history; do not invent a world point.
        label=Marker();label.header.frame_id='map';label.header.stamp=stamp;label.ns='presence_state';label.id=t['id'];label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.;label.pose.position.x,label.pose.position.y,label.pose.position.z=map(float,anchor+[0,0,.2]);label.scale.z=.12;label.color.a=1.;label.color.r=label.color.g=label.color.b=1.
        label.text=f"V{t['id']} {t['visual_state']} / loc:{t['localization_state']}"+(' / ray display only, no range' if not point else ' / visible surface');arr.markers.append(label)
    return arr


def publish_presence_events(node):
    from std_msgs.msg import String
    while node.tracker.events:
        event=node.tracker.events.pop(0);node.presence_events.publish(String(data=json.dumps(event)))
        if node.evidence_dir:node.record_evidence(dict(kind='presence_event',**event))


def publish_registry_observations(node, ns, processing):
    from std_msgs.msg import String
    batch=node.registry_evidence.batch(node.tracker,ns,processing)
    if batch is not None:
        node.registry_pub.publish(String(data=json.dumps(batch,separators=(',',':'))))
        if node.evidence_dir:node.record_evidence(dict(kind='registry_observations',batch=batch))


def publish_presence(node):
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    now=node.get_clock().now();processing=now.nanoseconds*1e-9;report=node.tracker.report(processing);report['stamp_ns']=node.tracker.last_rgb_ns
    node.output.publish(String(data=json.dumps(report)))
    if not node.registry_mode:node.markers.publish(presence_markers(report,now.to_msg()))
    ns=node.tracker.last_rgb_ns
    if node.registry_mode and ns>=0:
        publish_registry_observations(node,ns,processing)
    if node.registry_mode and node.pending_visibility is not None:
        # Same ordered publisher, after positive observations from this capture.
        visibility=node.pending_visibility;node.pending_visibility=None
        node.registry_pub.publish(String(data=json.dumps(visibility,separators=(',',':'))))
        if node.evidence_dir:node.record_evidence(dict(kind='registry_visibility',batch=visibility))
    if ns>=0:
        rows=node.tracker.frame_rows(ns,processing)
        semantic=dict(schema_version=2,stamp_ns=ns,processing_time_s=processing,published_sim_ns=now.nanoseconds,backend=node.backend,counts=report['counts'],candidates=rows)
        node.semantic_output.publish(String(data=json.dumps(semantic)))
        if node.evidence_dir:node.record_evidence(dict(kind='semantic',**semantic))
        with node.input_lock:rgb=node.queues['rgb'].get(ns)
        if rgb is not None:
            import cv2
            pixels=rgb_pixels(rgb)
            for r in rows:
                x,y,w,h=map(int,r['box']);color=(40,220,70) if r['visual_state']=='visually_confirmed' else (240,190,30);cv2.rectangle(pixels,(x,y),(x+w,y+h),color,1)
                cv2.putText(pixels,f"V{r['id']} {r['visual_state']} loc:{r['localization_state']}",(max(0,x-2),max(12,y-3)),cv2.FONT_HERSHEY_SIMPLEX,.28,color,1)
            m=Image();m.header=rgb.header;m.height=rgb.height;m.width=rgb.width;m.encoding='rgb8';m.step=rgb.width*3;m.data=pixels.tobytes();node.annotated_output.publish(m)
    publish_presence_events(node)
