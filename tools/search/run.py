#!/usr/bin/env python3
"""Closed-loop ideal camera rig: real Gazebo RGB-D -> RACER -> new RGB-D.

This is NOT flight. Model pose follows sampled native curves without dynamics.
Only the scene setup can read fixture truth; decision code sees RGB-D and a
commanded ideal camera pose. No named viewpoint or target position is consulted.
"""
import argparse,json,os,sys,time,subprocess,math,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import BSpline
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
from tools.prepare_sim import prepare,model
from tools.rgbd_scene.cameras import tf_arguments
from tools.racer_ros2.light.processes import terminate
from annual_runtime.rgbd_alignment import require_registered_pair,stamp_ns
from person_search.perception import PeopleDetector,candidates_from_cloud,cloud
from person_search.tracking import Tracker
from person_search.policy import ObservedVolume,choose_view
from tools.search.configuration import configuration
from tools.racer_ros2.configuration import native_node_command


def request_ideal_pose(pose, angle):
    req=f'name: "annual_x500_0" position {{ x: {pose[0]} y: {pose[1]} z: {pose[2]} }} orientation {{ w: {math.cos(angle/2)} z: {math.sin(angle/2)} }}'
    try:
        reply=subprocess.run(['gz','service','-s','/world/annual/set_pose','--reqtype','gz.msgs.Pose',
            '--reptype','gz.msgs.Boolean','--timeout','3000','--req',req],capture_output=True,text=True,timeout=5)
    except subprocess.TimeoutExpired:
        return 'Ideal rig pose request timed out'
    if reply.returncode or not any(line.strip()=='data: true' for line in reply.stdout.splitlines()):
        return 'Ideal rig pose request was not acknowledged'
    return None


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--native',type=Path,required=True);p.add_argument('--strategy',choices=['racer','lateral','active'],default='active')
    p.add_argument('--seconds',type=float,default=60);p.add_argument('--negative',action='store_true');p.add_argument('--no-frontier-recovery',action='store_true')
    p.add_argument('--start',choices=['partial','hidden','visible'],default='partial');p.add_argument('--height',type=float,default=2.2)
    a=p.parse_args()
    if not math.isfinite(a.seconds) or a.seconds<0:p.error('--seconds must be finite and nonnegative')
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    prepare(ROOT/'tools/rgbd_scene/map.json',out,rgbd_test_scene=True)
    import xml.etree.ElementTree as ET
    tree=ET.parse(out/'scene.sdf');world=tree.find('world')
    if a.negative:world.remove(world.find('include[name="stationary_person"]'))
    # Indoor enclosure supplies measured finite ranges; missing depth stays unknown.
    for name,center,size in [('back',[-3.1,0,2],[.2,8,4]),('front',[7.1,0,2],[.2,8,4]),('left',[2,-4.1,2],[10,.2,4]),('right',[2,4.1,2],[10,.2,4]),('ceiling',[2,0,4.1],[10,8,.2])]:
        model(world,'room_'+name,center,('box',{'size':' '.join(map(str,size))}))
    # Fixed initial condition, never consumed as target truth by decisions.
    y={'partial':-.9,'hidden':0.,'visible':-2.}[a.start]
    origin=np.array([0.,y,a.height]);yaw={'partial':.2213144423,'hidden':0.,'visible':.463647609}[a.start]
    def model_pose(o,angle):return [o[0]-.13233*math.cos(angle),o[1]-.13233*math.sin(angle),o[2]-.26078,0.,0.,angle]
    world.find('include[name="annual_x500_0"]/pose').text=' '.join(map(str,model_pose(origin,yaw)))
    tree.write(out/'scene.sdf')
    config=json.loads((out/'runtime.json').read_text())
    os.environ.update(DISPLAY=':119',LIBGL_ALWAYS_SOFTWARE='1',GZ_PARTITION='person_search_'+out.name,ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/'roslog'),GZ_SIM_RESOURCE_PATH=os.pathsep.join([str(out/'models'),config['models'],*config['additional_model_paths']]))
    children=[];logs=[];events=[];report={'scope':'Ideal camera sensor-in-loop, native RACER curves, no flight dynamics or PX4; commanded rig pose is not estimated odometry.','strategy':a.strategy,'negative_fixture':a.negative,'start':a.start,'height':a.height,'budget_wall_s':a.seconds,'discovered':False,'path_m':0.,'aligned_pairs':0,'planning_requests':0,'accepted_curves':0}
    report['code_sha256']={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for folder in (ROOT/'tools/search',ROOT/'src/person_search') for f in folder.rglob('*') if f.is_file() and f.suffix in ('.py','.cpp','.hpp','.sh')}
    report['native_binary_sha256']=hashlib.sha256((a.native/'install/racer_core/lib/racer_core/search_racer').read_bytes()).hexdigest()
    report['native_overlay_sha256']=hashlib.sha256((a.native/'task-overlay.json').read_bytes()).hexdigest()
    report['frontier_recovery']=a.strategy!='racer' and not a.no_frontier_recovery
    def start(name,args):
        f=(out/(name+'.log')).open('w');logs.append(f);children.append(subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
    import rclpy,cv2
    from sensor_msgs.msg import Image,CameraInfo,PointCloud2,PointField
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker,MarkerArray
    from racer_interfaces.msg import Bspline
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer,TransformListener,TransformException
    rclpy.init();node=rclpy.create_node('person_search_sensor_loop');node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    queues={k:{} for k in ('rgb','depth','ri','di')};subs=[]
    def receive(key,m):
        queues[key][stamp_ns(m.header)]=m
        while len(queues[key])>80:del queues[key][min(queues[key])]
    for key,kind,topic in [('rgb',Image,'rgb/image'),('depth',Image,'depth/image'),('ri',CameraInfo,'rgb/camera_info'),('di',CameraInfo,'depth/camera_info')]:subs.append(node.create_subscription(kind,'/drone_0/'+topic,lambda m,k=key:receive(k,m),qos_profile_sensor_data))
    pubcloud=node.create_publisher(PointCloud2,'/baseline/cloud_0',20);pubpose=node.create_publisher(PoseStamped,'/baseline/sensor_0',20);pubodom=node.create_publisher(Odometry,'/baseline/odom_0',20);request=node.create_publisher(PoseStamped,'/search/request',10)
    markers=node.create_publisher(MarkerArray,'/search/targets',10)
    curves=[];statuses=[]
    subs.append(node.create_subscription(Bspline,'/search/trajectory',curves.append,10));subs.append(node.create_subscription(String,'/search/planning_status',lambda m:statuses.append(m.data),10))
    detector=PeopleDetector();tracker=Tracker(ttl=a.seconds);volume=ObservedVolume();previous=[]
    last_ns=0;ignore_until=0;last_detection=-1e9;last_request=-1e9;pending=False;native_started=False;motion=None;pending_pose=None;motion_start=0;movement_last=0;seq=0;start_wall=time.monotonic();active_start=None;current_track=None
    def transform():
        t=np.eye(4);t[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]]);t[:3,3]=origin;return t
    try:
        start('xvfb',['Xvfb',':119','-screen','0','1024x768x24','-nolisten','tcp']);time.sleep(.7)
        start('gazebo',['gz','sim','-r','-s',str(out/'scene.sdf')]);start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(out/'bridge.yaml')])
        for i,t in enumerate(config['camera_transforms']):start('tf-'+str(i),['ros2','run','tf2_ros','static_transform_publisher',*tf_arguments(t)])
        while a.seconds>0:
            now=time.monotonic();rclpy.spin_once(node,timeout_sec=.01)
            if any(c.poll() is not None for c in children):raise RuntimeError('Owned sensor/planner process exited')
            if not rclpy.ok():raise KeyboardInterrupt
            if active_start is None and now-start_wall>60 and 'startup_wait' not in report:
                report['startup_wait']='Waiting for registered RGB-D and native search startup'
                print(report['startup_wait'],file=sys.stderr)
            if active_start is not None and now-active_start>=a.seconds:break
            if motion is not None:
                spline,yspline,duration=motion
                if now-movement_last>.12:
                    if pending_pose is None:
                        t=min(now-motion_start,duration)
                        pending_pose=t,spline(t),float(yspline(t)),now
                    t,new,nyaw,requested_at=pending_pose
                    move_error=request_ideal_pose(model_pose(new,nyaw),nyaw)
                    movement_last=time.monotonic()
                    if move_error:
                        events.append({'event':'pose_unacknowledged','reason':move_error,'wall_s':movement_last-active_start})
                        continue
                    motion_start+=movement_last-requested_at
                    pending_pose=None
                    report['path_m']+=float(np.linalg.norm(new-origin));origin[:]=new;yaw=nyaw
                    if t>=duration:
                        motion=None;ignore_until=max(max(q,default=0) for q in queues.values())+800_000_000
                        previous.append(origin.copy());events.append({'event':'arrived','origin':origin.tolist(),'wall_s':movement_last-active_start})
                continue # Never project an image while the rig's pose may be unsettled.
            common=set.intersection(*(set(q) for q in queues.values()))
            common=[n for n in common if n>max(last_ns,ignore_until)]
            if not common:continue
            ns=max(common);rgb,dep,ri,di=[queues[k][ns] for k in ('rgb','depth','ri','di')]
            try:tf=buffer.lookup_transform(rgb.header.frame_id,dep.header.frame_id,Time(nanoseconds=ns))
            except TransformException:continue
            last_ns=ns
            try:require_registered_pair(rgb,dep,ri,di,tf)
            except ValueError as error:
                report['rejected_pairs']=report.get('rejected_pairs',0)+1
                report['last_pair_error']=str(error)
                continue
            report['aligned_pairs']+=1
            pixels=np.ndarray((rgb.height,rgb.width,3),np.uint8,bytes(rgb.data),strides=(rgb.step,3,1)).copy()
            if rgb.encoding=='bgr8':pixels=pixels[:,:,::-1]
            depth=np.ndarray((dep.height,dep.width),np.dtype('>f4' if dep.is_bigendian else '<f4'),bytes(dep.data),strides=(dep.step,4)).copy()
            T=transform();points=cloud(depth,di.k,T);valid=np.isfinite(depth)&(depth>.25)&(depth<8)
            pc=points[::3,::3][valid[::3,::3]].astype('<f4')
            m=PointCloud2();m.header.stamp=dep.header.stamp;m.header.frame_id='map';m.height=1;m.width=len(pc);m.fields=[PointField(name=k,offset=4*i,datatype=7,count=1) for i,k in enumerate('xyz')];m.point_step=12;m.row_step=len(pc)*12;m.data=pc.tobytes();m.is_dense=True;pubcloud.publish(m)
            pose=PoseStamped();pose.header=m.header;pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,origin)
            q=Rotation.from_matrix(T[:3,:3]).as_quat();pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,q);pubpose.publish(pose)
            od=Odometry();od.header=m.header;od.child_frame_id='ideal_rig';od.pose.pose.position=pose.pose.position;od.pose.pose.orientation.w=math.cos(yaw/2);od.pose.pose.orientation.z=math.sin(yaw/2);pubodom.publish(od)
            if not native_started:
                specs=configuration(out,di.k)
                for s in sorted(specs,key=lambda s:s['executable']=='search_racer'):
                    if s['executable']=='search_racer':s['parameters']['search.recover_frontiers']=a.strategy!='racer' and not a.no_frontier_recovery
                    start(s['name'],native_node_command(s,out,a.native/'install/racer_core/lib/racer_core'))
                native_started=True;active_start=now;last_request=now
            if now-last_detection>1.:
                volume.add(depth, di.k, T, points=points)
                cs = candidates_from_cloud(pixels, depth, points, detector)
                tracker.update(cs, origin, now-active_start)
                last_detection = now
                frame={'event':'observation','wall_s':now-active_start,'stamp_ns':ns,'origin':origin.tolist(),'camera_to_map':T.tolist(),'k':list(di.k),'candidates':[{'xyz':c.xyz.tolist(),'semantic':c.semantic,'depth_m':c.depth_m,'box':c.box} for c in cs],'tracks':[{'id':t.id,'xyz':t.xyz.tolist(),'state':t.state,'attempts':t.attempts,'semantic_views':len(t.semantic_views)} for t in tracker.tracks]};events.append(frame)
                arr=MarkerArray()
                for t in tracker.tracks:
                    mark=Marker();mark.header.frame_id='map';mark.header.stamp=dep.header.stamp;mark.ns='ideal_rig_candidates';mark.id=t.id;mark.type=Marker.SPHERE;mark.action=Marker.ADD;mark.pose.orientation.w=1.;mark.pose.position.x,mark.pose.position.y,mark.pose.position.z=map(float,t.xyz);mark.scale.x=mark.scale.y=mark.scale.z=.25;mark.color.a=.9;mark.color.r=0. if t.state=='confirmed' else 1.;mark.color.g=1. if t.state=='confirmed' else .5;arr.markers.append(mark)
                markers.publish(arr)
                if cs or seq==0:
                    cv2.imwrite(str(out/f'frame-{seq:03}.png'),pixels[:,:,::-1]);np.save(out/f'depth-{seq:03}.npy',depth);(out/f'frame-{seq:03}.json').write_text(json.dumps(frame,indent=2));seq+=1
                if any(t.state=='confirmed' for t in tracker.tracks):
                    report['discovered']=True;report['latency_wall_s']=now-active_start;break
            if curves:
                b=curves.pop(0);pending=False
                pts=np.array([[p.x,p.y,p.z] for p in b.pos_pts]);knots=np.array(b.knots);duration=knots[-b.order-1]-knots[b.order]
                spline=BSpline(knots-knots[b.order],pts,b.order)
                # Upstream non-uniform cubic yaw knots, same de Boor convention.
                yk=(np.arange(len(b.yaw_pts)+4)-3)*b.yaw_dt
                yspline=BSpline(yk,np.array(b.yaw_pts),3)
                if np.linalg.norm(spline(0)-origin)>.12:events.append({'event':'reject_start_mismatch'});continue
                report['accepted_curves']+=1;motion=(spline,yspline,float(duration));motion_start=now
                events.append({'event':'trajectory','id':b.traj_id,'duration':duration,'points':pts.tolist(),'knots':knots.tolist(),'wall_s':now-active_start})
                continue
            if statuses:
                for status in statuses:events.append({'event':'planner_status','status':status,'wall_s':now-active_start})
                if any(s.startswith('REJECT') for s in statuses):pending=False
                statuses.clear()
            if pending and now-last_request>12:
                pending=False;events.append({'event':'planning_timeout'})
            if not pending and now-last_request>3 and request.get_subscription_count():
                goal=None;current_track=None
                if a.strategy!='racer':
                    for t in tracker.tracks:
                        if t.state not in ('confirmed','abandoned'):
                            goal=choose_view(t,origin,volume,previous,strategy=a.strategy)
                            if goal is not None:current_track=t;break
                req=PoseStamped();req.header.stamp=dep.header.stamp;req.pose.orientation.w=1.
                if goal is not None:
                    if not tracker.review(current_track):continue
                    previous.append(goal.copy())
                    req.header.frame_id='map';req.pose.position.x,req.pose.position.y,req.pose.position.z=map(float,goal)
                    angle=math.atan2(current_track.xyz[1]-goal[1],current_track.xyz[0]-goal[0]);req.pose.orientation.w=math.cos(angle/2);req.pose.orientation.z=math.sin(angle/2)
                else:req.header.frame_id='explore'
                request.publish(req);pending=True;last_request=now;report['planning_requests']+=1
                events.append({'event':'request','mode':req.header.frame_id,'goal':None if goal is None else goal.tolist(),'wall_s':now-active_start})
        report['status']='COMPLETED_WINDOW';report['tracks']=[{'id':t.id,'xyz':t.xyz.tolist(),'state':t.state,'attempts':t.attempts,'semantic_views':len(t.semantic_views)} for t in tracker.tracks]
    except Exception as e:
        report.update(status='FAIL',error=str(e));raise
    finally:
        report['elapsed_wall_s']=time.monotonic()-(active_start or start_wall);report['cleanup']=terminate(children)
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n');(out/'events.json').write_text(json.dumps(events,indent=2)+'\n')
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown()
        print(json.dumps(report,indent=2))

if __name__=='__main__':main()
