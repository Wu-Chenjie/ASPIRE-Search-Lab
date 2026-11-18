#!/usr/bin/env python3
"""Local SITL only: original PX4 EKF2/guards/executor, native ROS2 RACER.

No model pose service, oracle pose, or scene geometry enters decision nodes.
Scene truth is written separately solely for subsequent evaluation.
"""
import argparse,json,os,sys,time,subprocess,math,hashlib,tarfile
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
ROOT=Path('/tmp/racer-stage45-src')
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
from tools.prepare_sim import prepare,model
from tools.rgbd_scene.cameras import tf_arguments,set_text
from tools.flight.gazebo_shutdown import terminate_flight
from tools.flight.cpu_profile import CPUProfile
from tools.flight.gazebo_guard import configuration as guard_configuration,verify_loaded as verify_guard_loaded
from tools.flight.run_safety import BootstrapTrackingWatchdog,StartupDeadline
from core.planning.motion_limits import motion_environment
from core.exploration.voxel_config import voxel_environment


def scene(out,negative,layout="original",performance="legacy"):
    # Initial position and room bounds only are provided to the planning runtime.
    spec=dict(bounds=[[-3,-4,0],[7,4,4]],search_starts=[[0,{'original':0,'visible-left':-2.,'partial-left':-.9,'visible-right':2.,'partial-right':.9}[layout],2.2]],obstacles=[])
    (out/'map.json').write_text(json.dumps(spec));prepare(out/'map.json',out,motion_profile='legacy')
    tree=ET.parse(out/'scene.sdf');world=tree.find('world')
    if not negative:
        inc=ET.SubElement(world,'include')
        for key,value in [('uri','model://person_standing'),('name','stationary_person'),('static','true'),('pose','4 0 0 0 0 -1.5707963267948966')]:ET.SubElement(inc,key).text=value
    model(world,'board',[2,0,1.1],('box',{'size':'.08 .9 2.2'}))
    for name,center,size in [('back',[-3.1,0,2],[.2,8,4]),('front',[7.1,0,2],[.2,8,4]),('left',[2,-4.1,2],[10,.2,4]),('right',[2,4.1,2],[10,.2,4]),('ceiling',[2,0,4.1],[10,8,.2])]:model(world,'room_'+name,center,('box',{'size':' '.join(map(str,size))}))
    ET.SubElement(world,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
    tree.write(out/'scene.sdf')
    basepath=out/'models/annual_base_0/model.sdf';base=ET.parse(basepath);link=base.find('model/link[@name="base_link"]');sensor=ET.SubElement(link,'sensor',name='evaluation_contact',type='contact');set_text(sensor,'always_on','true');set_text(sensor,'update_rate','20');set_text(sensor,'topic','/evaluation/contacts');contact=ET.SubElement(sensor,'contact')
    for c in link.findall('collision'):ET.SubElement(contact,'collision').text=c.get('name')
    base.write(basepath)

    cfg=json.loads((out/'runtime.json').read_text());cfg.update(research_planner='racer_ros2',native_frame_source='mapper_observation',racer_planning_scale=.5,racer_flight_limits=[.7,3.],control_timing_diagnostics=True,bootstrap_altitudes=[1.75,2.65,2.2])
    cfg.update(layout=layout,performance_profile=performance)
    if performance!='legacy':cfg.update(motion_limits=dict(speed=1.,acceleration=1.2,jerk=4.,yaw_rate=.9),racer_planning_limits=dict(speed=.3,acceleration=.4,jerk=4.,yaw_rate=.325),racer_min_time=performance!='consistent')
    if performance=='bounded-fast':cfg.update(motion_limits=dict(speed=1.2,acceleration=1.5,jerk=4.,yaw_rate=1.),racer_planning_limits=dict(speed=.45,acceleration=.6,jerk=4.,yaw_rate=.4),racer_min_time=False,racer_feasible_time=True)
    (out/'runtime.json').write_text(json.dumps(cfg,indent=2));(out/'evaluation-truth.json').write_text(json.dumps(dict(person=None if negative else [4,0,0],board=[2,0,1.1],use='independent evaluation only')))
    assert not cfg.get('manual_camera_test')
    assert ET.parse(out/'models/annual_x500_0/model.sdf').findtext('model/static')!='true'
    return cfg


def paced_completion(process,out):
    if process.poll() is None:return False
    data=json.loads((out/'pacer-result.json').read_text())
    if process.returncode!=0 or not data.get('completed') or data.get('errors') or data.get('counts',{}).get('queue_drop'):
        raise RuntimeError('Bounded real IMU pacing failed: '+json.dumps(data))
    return True

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--px4-build',type=Path,help='Isolated verified PX4 SITL build directory');p.add_argument('--sim-real-time-factor',type=float,help='Diagnostic pacing only; physics timestep stays unchanged');p.add_argument('--wall-watchdog-seconds',type=float,help='Explicit harness wall budget for slow simulation; never changes flight guards');p.add_argument('--external',type=Path,default=Path('/planner/external'));p.add_argument('--seconds',type=float,default=240);p.add_argument('--budget-clock',choices=['wall','sim'],default='wall');p.add_argument('--negative',action='store_true');p.add_argument('--active-review',action='store_true');p.add_argument('--active-confirmation',action='store_true',help='Opt-in confirmation only after a measured RACER review and fresh RGB-D evidence');p.add_argument('--confirmation-mode',choices=['legacy','strict','hybrid','decoupled','registry'],default='',help='Explicit confirmation mode, independent of active review for fair comparisons');p.add_argument('--registry-cue-fixture',action='store_true',help='Explicit negative SITL-only target-observation ID churn/dropout fixture');p.add_argument('--bearing-bias',action='store_true',help='Opt-in bounded RGB bearing utility in native RACER candidate ranking');p.add_argument('--review-continuation',action='store_true',help='Opt-in bounded native continuation of the same review request');p.add_argument('--stop-and-plan',action='store_true',help='Wait for measured trajectory completion before replanning, independently of perception');p.add_argument('--recover-no-path',action='store_true');p.add_argument('--layout',choices=['original','visible-left','partial-left','visible-right','partial-right'],default='original');p.add_argument('--performance',choices=['legacy','moderate','consistent','bounded-fast'],default='legacy');p.add_argument('--rviz',action='store_true');p.add_argument('--record-video',action='store_true');p.add_argument('--prepared',type=Path,help='Generated flyable scene; sensor truth files are not read by decision nodes');p.add_argument('--no-perception',action='store_true');p.add_argument('--vertical-samples',action='store_true',help='Opt-in native viewpoint altitude sampling');p.add_argument('--detector',choices=['hog','yolo11n-seg'],default='hog');p.add_argument('--segmentation-weights',default='');p.add_argument('--detector-python-path',default='');p.add_argument('--fault-injection',choices=['flags','position','dds'],help='Explicit SITL-only application/message or owned-agent interruption fixture');p.add_argument('--map-delay-fixture',action='store_true',help='Explicit SITL-only bounded interruption of the owned depth mapper');a=p.parse_args()
    if (a.active_confirmation or a.confirmation_mode in ('strict','hybrid','decoupled','registry')) and a.no_perception:p.error('confirmation mode requires perception')
    if a.confirmation_mode in ('decoupled','registry') and (a.active_review or a.detector!='yolo11n-seg'):p.error('decoupled requires YOLO RGB and native bearing guidance without legacy XYZ review')
    if a.active_confirmation and a.confirmation_mode:p.error('Use either legacy --active-confirmation or explicit --confirmation-mode')
    if a.map_delay_fixture and (os.environ.get('RACER_EXECUTION_SAFETY')!='1' or a.fault_injection):p.error('map-delay fixture requires execution safety and no other fault fixture')
    if a.registry_cue_fixture and not (a.negative and a.confirmation_mode=='registry' and os.environ.get('RACER_EXECUTION_SAFETY')=='1'):
        p.error('registry cue fixture requires explicit negative registry SITL with execution safety')
    a.active_review=a.active_review or a.active_confirmation
    if a.review_continuation and not a.active_review:p.error('--review-continuation requires --active-review')
    if a.active_review and a.confirmation_mode in ('strict','hybrid') and not a.review_continuation:p.error('Explicit confirmation modes with active review require --review-continuation for request/plan status binding')
    cpu_profile=CPUProfile(os.environ.get('RACER_CPU_PROFILE','inherit'))
    cpu_profile.install_parent()
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    if a.prepared:
        import shutil
        for name in ('models','scene.sdf','runtime.json','bridge.yaml','scene-audit.json','evaluation-truth.json'):
            src=a.prepared/name
            if src.is_dir():shutil.copytree(src,out/name)
            else:shutil.copyfile(src,out/name)
        cfg=json.loads((out/'runtime.json').read_text())
        if cfg.get('manual_camera_test'):raise ValueError('Refuse static sensor rig for flight')
        a.layout=cfg['layout'];a.performance=cfg['performance_profile']
    else:cfg=scene(out,a.negative,a.layout,a.performance)
    if a.sim_real_time_factor is not None:
        if not 0<a.sim_real_time_factor<=1:raise ValueError('Expected slow diagnostic pacing in (0,1]')
        tree=ET.parse(out/'scene.sdf');physics=tree.find('world/physics')
        if physics is None:raise ValueError('Scene has no physics settings')
        set_text(physics,'real_time_factor',str(a.sim_real_time_factor));tree.write(out/'scene.sdf')
    if os.environ.get('RACER_EXECUTION_SAFETY')=='1':
        physics=ET.parse(out/'scene.sdf').find('world/physics')
        rtf=float(physics.findtext('real_time_factor','1.0')) if physics is not None else 1.0
        if not 0<rtf<=1.: raise ValueError('Execution safety wall delay bound requires physics RTF in (0,1]')
        cfg['execution_safety_timing_contract']=dict(physics_real_time_factor=rtf,
            max_sim_to_wall_rate=1.,check_clock='monotonic_wall_upper_bound_at_rtf_le_1')
    if (a.performance=='bounded-fast' or a.recover_no_path) and not (a.active_review or a.stop_and_plan):raise ValueError('bounded-fast requires static stop-and-plan mode (--active-review)')
    cfg.update(bearing_bias=a.bearing_bias,confirmation_mode=a.confirmation_mode or ('strict' if a.active_confirmation else 'legacy'),review_continuation=a.review_continuation,map_delay_fixture=a.map_delay_fixture,execution_safety=os.environ.get("RACER_EXECUTION_SAFETY")=="1",active_confirmation=a.active_confirmation,active_review=a.active_review and not a.no_perception,stop_and_plan=a.active_review or a.stop_and_plan,no_path_recovery=a.recover_no_path,vertical_samples=a.vertical_samples,recover_frontiers=a.active_review or a.stop_and_plan);(out/'runtime.json').write_text(json.dumps(cfg,indent=2));ext=a.external.resolve()
    guard_env,guard_report=guard_configuration(out/'scene.sdf')
    build=a.px4_build.resolve() if a.px4_build else ext/'PX4-Autopilot/build/px4_sitl_default';binary=build/'bin/px4';agent=ext/'agent_install/bin/MicroXRCEAgent'
    if not binary.is_file() or not agent.is_file():raise RuntimeError('Existing local SITL dependencies unavailable')
    os.environ.update(DISPLAY=':119',LIBGL_ALWAYS_SOFTWARE='1',GZ_PARTITION='person_flight_'+out.name,ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/'roslog'),PYTHONDONTWRITEBYTECODE='1',GZ_SIM_RESOURCE_PATH=os.pathsep.join([str(out/'models'),cfg['models'],str(ROOT/'external/rgbd_scene_assets')]),PX4_GZ_STANDALONE='1',PX4_GZ_WORLD='annual',HEADLESS='1')
    os.environ.update(motion_environment(cfg));os.environ.update(voxel_environment(cfg))
    from ament_index_python.packages import get_package_prefix
    native_binary=Path(get_package_prefix('racer_core'))/'lib/racer_core/exploration_node'
    native_hash=hashlib.sha256(native_binary.read_bytes()).hexdigest()
    source_files=[f for folder in (ROOT/'tools',ROOT/'src') for f in folder.rglob('*') if f.is_file() and f.suffix in ('.py','.cpp','.hpp','.sh')]
    source_hashes={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in source_files}
    source_hashes['tools/flight/run.py']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    measurement_files=[Path(x) for x in ['/tmp/stage68-bootstrap/stage68-pacer.py','/tmp/stage68-bootstrap/stage68_publication.py','/tmp/stage62_public_rpc.py','/tmp/stage58-observer.py','/tmp/stage65-measurement/stage65_capture.py','/tmp/stage65-measurement/stage65-node-entry.py','/tmp/stage66-measurement/stage66-startup-probe.py']]
    measurement_hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in measurement_files}
    (out/'measurement-stack.json').write_text(json.dumps(measurement_hashes,indent=2)+'\n')
    with tarfile.open(out/'runtime-source.tar.gz','x:gz') as archive:
        for f in source_files:archive.add(Path(__file__) if f==ROOT/'tools/flight/run.py' else f,arcname=str(f.relative_to(ROOT)))
        for f in measurement_files:archive.add(f,arcname='measurement-layer/'+f.name)
    children=[];child_names={};logs=[];fault=None;map_delay=None;unexpected_exits=[];bootstrap_data=None
    def start(name,args,env=None,cwd=None):
        f=(out/(name+'.log')).open('x');logs.append(f);proc=subprocess.Popen(cpu_profile.command(name,args),env=dict(os.environ,**(env or {})),cwd=cwd or ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(proc);child_names[proc.pid]=name;cpu_profile.verify(name,proc);return proc
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image,CameraInfo
    from annual_runtime.pose_history import PoseHistory
    from annual_runtime.rgbd_alignment import stamp_ns
    import numpy as np,cv2
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    rclpy.init();node=rclpy.create_node('flight_evidence');node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)]);tf=TransformBroadcaster(node)
    pose_history=PoseHistory();frames={k:{} for k in ('rgb','depth','ri','di')};last_frame=-1e9;frame_count=0
    def image_received(key,msg):
        q=frames[key];q[stamp_ns(msg.header)]=msg
        while len(q)>30:del q[min(q)]
    for key,kind,topic in [('rgb',Image,'rgb/image'),('depth',Image,'depth/image'),('ri',CameraInfo,'rgb/camera_info'),('di',CameraInfo,'depth/camera_info')]:node.create_subscription(kind,'/drone_0/'+topic,lambda m,k=key:image_received(k,m),qos_profile_sensor_data)
    evidence=(out/'telemetry.jsonl').open('x',buffering=1);started=time.monotonic();counts={};last={};previous=None;path=0.;max_z=0.;last_log=-1.;failure=None;budget_end_sim=0.;bootstrap_watchdog=BootstrapTrackingWatchdog();startup_watchdog=StartupDeadline();deferred_started=False
    def record(kind,row):
        counts[kind]=counts.get(kind,0)+1;last[kind]=row
        evidence.write(json.dumps(dict(kind=kind,wall_s=time.monotonic()-started,data=row))+'\n')
    def odom(m):
        nonlocal previous,path,max_z,last_log
        pos=m.pose.pose.position;q=m.pose.pose.orientation
        pose_history.add(stamp_ns(m.header)*1e-9,[pos.x,pos.y,pos.z],[q.x,q.y,q.z,q.w])
        # PX4 bridge provides ENU estimate, calibrated at known initial ground datum.
        t=TransformStamped();t.header=m.header;t.header.frame_id='map';t.child_frame_id='drone_0/base_link';t.transform.translation.x=pos.x;t.transform.translation.y=pos.y;t.transform.translation.z=pos.z;t.transform.rotation=q;tf.sendTransform(t)
        v=[pos.x,pos.y,pos.z];max_z=max(max_z,pos.z)
        if previous is not None:path+=math.dist(v,previous)
        previous=v
        now=time.monotonic()
        if now-last_log>.2:record('estimated_odometry',dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,xyz=v,quaternion=[q.x,q.y,q.z,q.w]));last_log=now
    node.create_subscription(Odometry,'/drone_0/estimated_odometry',odom,qos_profile_sensor_data)
    from px4_msgs.msg import EstimatorStatusFlags
    from annual_runtime.px4_bridge_node import px4_topic
    node.create_subscription(EstimatorStatusFlags,px4_topic('/px4_1/fmu','out','estimator_status_flags',EstimatorStatusFlags),lambda m:record('estimator_fusion',{k:bool(getattr(m,k)) for k in ('cs_gnss_pos','cs_gnss_vel','cs_gps_hgt','cs_ev_pos','cs_ev_hgt','cs_ev_yaw','cs_baro_hgt') if hasattr(m,k)}),qos_profile_sensor_data)
    for kind,topic,qos in [('flight','/drone_0/flight_status',qos_profile_sensor_data),('execution','/drone_0/view_execution',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),('tracks','/search/tracks',10),('semantic','/search/semantic_candidates',10),('perception_timing','/search/perception_timing',10),('active_confirmation','/search/active_confirmation',10),('presence_event','/search/presence_events',30),('review_status','/search/review_status',10),('review_cancel','/search/review_cancel',10),('bearing','/search/bearings',30),('bearing_decision','/search/bearing_decision',100),('person_records','/uav0/search/person_records',100),('target_registry_events','/uav0/search/target_registry_events',100),('target_summary','/search/target_summary',100),('target_sync','/search/target_sync_request',100)]:node.create_subscription(String,topic,lambda m,k=kind:record(k,json.loads(m.data)),qos)
    registry_fixture=None;registry_fixture_pub=None
    if a.registry_cue_fixture:
        from tools.flight.registry_cue_fixture import ChurningCueFixture
        registry_fixture=ChurningCueFixture()
        registry_fixture_pub=node.create_publisher(String,'/uav0/search/target_observations',20)
    try:
        start('xvfb',['Xvfb',':119','-screen','0','1280x900x24','-nolisten','tcp']);time.sleep(.7)
        if a.rviz:
            rviz_cfg=yaml.safe_load((ROOT/'config/search.rviz').read_text())
            # Bound visualization work on the software-rendered local container.
            # This changes only the display, never mapper input or occupancy.
            rviz_cfg['Visualization Manager']['Global Options']['Frame Rate']=5
            for display in rviz_cfg['Visualization Manager']['Displays']:
                if a.confirmation_mode=='registry' and display.get('Topic',{}).get('Value')=='/search/targets':
                    display['Topic']['Value']='/uav0/search/targets';display['Name']='Persistent presence and visible-surface status'
                if display['Class']=='rviz_default_plugins/PointCloud2':
                    display['Decay Time']=.3
                    display['Style']='Points'
            view=rviz_cfg['Visualization Manager']['Views']['Current'];start_xyz=cfg['fleet_starts'][0]
            view['Focal Point']=dict(zip(('X','Y','Z'),map(float,start_xyz)));view['Distance']=22. if a.prepared else 11.
            if a.detector=='yolo11n-seg' and not a.no_perception:
                for display in rviz_cfg['Visualization Manager']['Displays']:
                    if display['Class']=='rviz_default_plugins/Image':
                        display['Name']='YOLO masks and boxes';display['Topic']['Value']='/search/detection_image';display['Topic']['Reliability Policy']='Reliable'
            rviz_path=out/'search.rviz';rviz_path.write_text(yaml.safe_dump(rviz_cfg))
            start('rviz',['rviz2','-d',str(rviz_path),'--ros-args','-p','use_sim_time:=true'])
            start('x11vnc',['x11vnc','-display',':119','-localhost','-rfbport','5901','-nopw','-forever','-shared'])
            start('websockify',['websockify','--web=/usr/share/novnc','127.0.0.1:18680','127.0.0.1:5901'])
            start('display',[sys.executable,'-m','tools.flight.visualization'])
        # Expensive perception/video/native mapping start only after genuine EKF origin and odometry.
        start('evaluation',[sys.executable,'-m','tools.flight.evaluation_recorder',str(out)])
        start('world_tf',['ros2','run','tf2_ros','static_transform_publisher','--frame-id','map','--child-frame-id','world'])
        imu_observer=start('imu_capture',[sys.executable,'/tmp/stage58-observer.py',str(out)])
        deadline=time.monotonic()+5
        while not (out/'observer-ready.json').exists():
            if imu_observer.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Independent raw observer not ready')
            time.sleep(.02)
        gazebo_proc=start('gazebo',['gz','sim','-s',str(out/'scene.sdf'),'--seed','123'],env=guard_env)
        verify_guard_loaded(gazebo_proc,guard_report)
        agent_proc=start('agent',[str(agent),'udp4','-p','8888'],{'LD_LIBRARY_PATH':str(ext/'agent_install/lib')+':'+os.environ.get('LD_LIBRARY_PATH','')})
        start('sensor_bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(out/'bridge.yaml')])
        for i,t in enumerate(cfg['camera_transforms']):start('tf-'+str(i),['ros2','run','tf2_ros','static_transform_publisher',*tf_arguments(t)])
        v=cfg['vehicles'][0];work=out/'px4_1';work.mkdir();script=(build/'etc/init.d-posix/rcS').read_text();needle='uxrce_dds_client start -t udp -p $uxrce_dds_port $uxrce_dds_ns'
        if script.count(needle)!=1:raise ValueError('Review changed PX4 startup')
        if os.environ.get('RACER_PX4_LOGGER_TOPICS'):
            from tools.flight.logger_configuration import install_logger_topics
            install_logger_topics(work,build,os.environ['RACER_PX4_LOGGER_TOPICS'])
        startup=work/'startup.sh';startup.write_text(script.replace(needle,'param set UXRCE_DDS_SYNCT 0\n'+needle)+'\nparam set COM_RC_IN_MODE 4\nparam set NAV_DLL_ACT 0\nparam set COM_OF_LOSS_T 0.5\nparam set COM_OBL_RC_ACT 4\n')
        time.sleep(3)
        start('px4',[str(binary),'-i','1','-d','-w',str(work),'-s',str(startup),str(build/'etc')],dict(PX4_SYS_AUTOSTART='4002',PX4_SIM_MODEL='gz_x500_depth',PX4_GZ_MODEL_NAME=v['model'],PX4_PARAM_EKF2_GPS_DELAY='0',PX4_PARAM_SENS_EN_MAGSIM='1',PX4_PARAM_EKF2_MAG_TYPE='6'),str(work))
        params=dict(use_sim_time=True,drone_id=0,px4_instance=1,bounds=json.dumps(cfg['bounds']),fleet_starts=json.dumps(cfg['fleet_starts']),start=json.dumps(v['start']),spawn=json.dumps(v['spawn']),initial_position=json.dumps(v['initial_position']),origin_calibration_s=3.,require_depth_odometry=False,sitl_auto_arm=True,publish_native_frames=True,bootstrap_altitudes=json.dumps(cfg['bootstrap_altitudes']),control_timing_dir=str(out/'timing'),safety_flight_limits=json.dumps(cfg.get('racer_flight_limits')),execution_safety_dir=str(out/'execution-safety'))
        paramfile=out/'nodes.yaml';paramfile.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        remaps=[]
        if a.fault_injection:
            from tools.flight.fault_injection import FaultSchedule
            from px4_msgs.msg import VehicleLocalPosition
            fault=FaultSchedule(a.fault_injection,out,agent_proc)
            start('fault_observer',[sys.executable,'-m','tools.flight.fault_injection','--mode',a.fault_injection,'--output',str(out)])
            if a.fault_injection!='dds':
                cls,topic,target=(EstimatorStatusFlags,'estimator_status_flags','flags') if a.fault_injection=='flags' else (VehicleLocalPosition,'vehicle_local_position','raw_position')
                remaps=['-r',px4_topic('/px4_1/fmu','out',topic,cls)+':=/fault_fixture/'+target]
        deadline=time.monotonic()+10
        while 'Startup script returned successfully' not in (out/'px4.log').read_text(errors='replace'):
            if time.monotonic()>deadline:raise RuntimeError('PX4 startup failed while genuine world paused at zero')
            time.sleep(.02)
        if a.budget_clock!='sim' or a.seconds!=180 or a.wall_watchdog_seconds!=1260:raise ValueError('Stage55 fixed budget contract')
        remaining=a.wall_watchdog_seconds-(time.monotonic()-started)
        if remaining<=0:raise RuntimeError('No remaining original wall budget')
        deadline=time.monotonic()+15
        while not (out/'camera-ready.json').exists():
            if imu_observer.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Actual zero-sim RGB/depth renderer readiness missing')
            time.sleep(.02)
        remaining=a.wall_watchdog_seconds-(time.monotonic()-started)
        if remaining<=0:raise RuntimeError('No remaining original wall budget')
        imu_pacer=start('imu_pacer',[sys.executable,'/tmp/stage68-bootstrap/stage68-pacer.py',str(out),str(a.seconds),str(remaining)],env={'STAGE58_REQUIRE_PARENT_RELEASE':'1'})
        deadline=time.monotonic()+10
        while not (out/'initial-bootstrap-ready.json').exists():
            if imu_pacer.poll() is not None:raise RuntimeError('Grounded real-input preparation failed before any bridge/arming graph')
            if time.monotonic()>deadline:raise RuntimeError('Bounded grounded preparation did not finish')
            rclpy.spin_once(node,timeout_sec=.01)
        bootstrap_data=json.loads((out/'initial-bootstrap-ready.json').read_text())
        if bootstrap_data.get('physical_sim_us')!=4000 or len(bootstrap_data.get('commander_guards',[]))!=2 or not all(x.get('disarmed') and x.get('no_offboard') and x.get('no_failsafe') for x in bootstrap_data.get('commander_guards',[])):
            raise RuntimeError('Missing actual grounded initialization guards')
        bootstrap_data['owned_children_before_bridge']=[child_names[c.pid] for c in children]
        if any(name in bootstrap_data['owned_children_before_bridge'] for name in ['px4_bridge_node','view_executor_node','racer_adapter','person_search']):raise RuntimeError('Actuation graph present during grounded initialization')
        for name in ('px4_bridge_node',):
            runtime_proc=start(name,[sys.executable,'-m','annual_runtime.'+name,'--ros-args','--params-file',str(paramfile)]+(remaps if name=='px4_bridge_node' else []))
            if name=='pointcloud_mapping_node' and a.map_delay_fixture:
                from tools.flight.map_delay_fixture import MeasuredMapDelay
                map_delay=MeasuredMapDelay(runtime_proc,out)
        bootstrap_data['bridge_started_monotonic_ns']=time.monotonic_ns()
        (out/'allow-pacing.json').write_text(json.dumps(dict(reason='Original bridge started after genuine4ms raw/grounded proof',bridge_pid=runtime_proc.pid,bootstrap_file_sha256=hashlib.sha256((out/'initial-bootstrap-ready.json').read_bytes()).hexdigest()))+'\n')
        def start_search_graph():
            for name in ('pointcloud_mapping_node','view_executor_node'):
                runtime_proc=start(name,[sys.executable,'/tmp/stage65-measurement/stage65-node-entry.py','annual_runtime.'+name,'--ros-args','--params-file',str(paramfile)])
                if name=='pointcloud_mapping_node' and a.map_delay_fixture:
                    nonlocal map_delay
                    from tools.flight.map_delay_fixture import MeasuredMapDelay
                    map_delay=MeasuredMapDelay(runtime_proc,out)
            start('racer_adapter',[sys.executable,'/tmp/stage65-measurement/stage65-node-entry.py','tools.research.frontier_baselines.live_ros2','--runtime',str(out/'runtime.json'),'--output',str(out/'flight'),'--transport','dds'])
            if not a.no_perception:start('person_search',[sys.executable,'-m','person_search.node','--ros-args','-p','use_sim_time:=true','-p','require_estimated_pose:=true','-p','active_review:='+str(a.active_review).lower(),'-p','active_confirmation:='+str(a.active_confirmation).lower(),*(['-p','confirmation_mode:='+a.confirmation_mode] if a.confirmation_mode else []),'-p','detector_backend:='+a.detector,'-p','segmentation_weights:='+a.segmentation_weights,'-p','evidence_dir:='+str(out/'perception')],env={'PYTHONPATH':a.detector_python_path+os.pathsep+os.environ.get('PYTHONPATH','')} if a.detector_python_path else None)
            if a.record_video:start('continuous_rgb',[sys.executable,'-m','tools.flight.video_recorder',str(out)])
        while True:
            elapsed=time.monotonic()-started
            budget_end_sim=node.get_clock().now().nanoseconds*1e-9
            if (elapsed if a.budget_clock=='wall' else budget_end_sim)>=a.seconds:
                # World stays paused at target; finish actual final readonly snapshots within original wall budget.
                if not paced_completion(imu_pacer,out):
                    if elapsed>a.wall_watchdog_seconds:raise RuntimeError('Simulation budget wall watchdog exceeded')
                    rclpy.spin_once(node,timeout_sec=.01)
                    continue
                if map_delay:map_delay.validate()
                if fault and fault.state!='complete':raise RuntimeError('Fault fixture did not complete within budget')
                break
            if a.budget_clock=='sim' and elapsed>(a.wall_watchdog_seconds if a.wall_watchdog_seconds is not None else max(120.,a.seconds*3+60)):raise RuntimeError('Simulation budget wall watchdog exceeded')
            rclpy.spin_once(node,timeout_sec=.01)
            startup_error=startup_watchdog.observe(budget_end_sim,elapsed,last.get('estimated_odometry'),
                                                   last.get('flight',{}),last.get('execution',{}))
            if startup_error:raise RuntimeError(startup_error)
            if startup_watchdog.has_odometry and not deferred_started:
                record('startup',dict(event='validated_odometry_start_deferred_graph',**startup_watchdog.report()))
                start_search_graph();deferred_started=True
            if registry_fixture:
                packet=registry_fixture.step(budget_end_sim,last.get('execution',{}).get('ready',False))
                if packet:
                    registry_fixture_pub.publish(String(data=json.dumps(packet,separators=(',',':'))))
                    record('registry_fixture',dict(explicit_synthetic_observation=True,batch=packet))
            if map_delay:map_delay.step(budget_end_sim,last.get('flight',{}),last.get('execution',{}))
            if fault and fault.step(budget_end_sim,last.get('flight',{}),last.get('execution',{})):break
            safety_fault=last.get('execution',{}).get('safety_fault')
            if safety_fault: raise RuntimeError('Execution safety fault: '+safety_fault)
            flight_fault=last.get('flight',{}).get('flight_session_fault')
            # Record expected failsafe through restoration; controller/session guards remain active.
            if flight_fault and not (fault and fault.allows_observation(flight_fault)):
                raise RuntimeError('Latched flight fault: '+flight_fault)
            if a.prepared and 'execution' in last:
                safety_error=bootstrap_watchdog.observe(last['execution'])
                if safety_error:raise RuntimeError(safety_error)
            common=set.intersection(*(set(q) for q in frames.values()))
            if common and time.monotonic()-last_frame>5.:
                ns=max(common);pose=pose_history.at(ns*1e-9)
                if pose is not None:
                    rgb,depth,ri,di=[frames[k][ns] for k in ('rgb','depth','ri','di')]
                    rgb_data=np.ndarray((rgb.height,rgb.width,3),np.uint8,bytes(rgb.data),strides=(rgb.step,3,1)).copy()
                    depth_data=np.ndarray((depth.height,depth.width),np.dtype('>f4' if depth.is_bigendian else '<f4'),bytes(depth.data),strides=(depth.step,4)).copy()
                    np.savez_compressed(out/f'rgbd-{frame_count:03}.npz',rgb=rgb_data,depth=depth_data,k=np.array(di.k),rgb_k=np.array(ri.k),stamp_ns=ns,base_position=pose[0],base_quaternion=pose[1].as_quat(),pose_bracket=pose[2])
                    cv2.imwrite(str(out/f'rgbd-{frame_count:03}.png'),rgb_data[:,:,::-1] if rgb.encoding=='rgb8' else rgb_data)
                    frame_count+=1;last_frame=time.monotonic()
            if (out/'observer-status.json').exists():
                capture=json.loads((out/'observer-status.json').read_text())
                if capture.get('errors') or capture.get('counts',{}).get('dropped'):raise RuntimeError('Independent IMU capture integrity failure')
            unexpected_exits=[dict(name=child_names[c.pid],pid=c.pid,returncode=c.returncode) for c in children if c.poll() is not None and not (c is imu_pacer and paced_completion(imu_pacer,out))]
            if unexpected_exits:raise RuntimeError('Owned process exited: '+json.dumps(unexpected_exits))
    except (Exception,KeyboardInterrupt) as e:
        import traceback
        traceback.print_exc() # diagnostic only; failure handling/deadlines unchanged
        failure=str(e) or type(e).__name__
    finally:
        if map_delay:map_delay.close(budget_end_sim)
        if fault:fault.close(budget_end_sim)
        cleanup=terminate_flight(children,child_names,out,timeout=8)
        allowed_exit={0,-2,-15,130,143}
        abnormal=[dict(name=child_names[x['pid']],phase='cleanup',**x)
                  for x in cleanup['direct_returncodes'] if x['returncode'] not in allowed_exit]
        unexpected_exits.extend(abnormal)
        if abnormal and failure is None:failure='Owned process abnormal exit during cleanup: '+json.dumps(abnormal)
        # A cleanup crash must never hide outside the run archive.
        core=ROOT/'core'
        if core.is_file() and core.stat().st_mtime>=time.time()-(time.monotonic()-started)-1:
            import shutil
            (out/'cores').mkdir(exist_ok=True);shutil.move(str(core),str(out/'cores/core'))
        report=dict(measurement_stack_sha256=measurement_hashes,grounded_initialization=bootstrap_data,gazebo_shutdown_guard=guard_report,cpu_profile=cpu_profile.report(),startup=startup_watchdog.report(),deferred_graph_started=deferred_started,registry_cue_fixture=registry_fixture.report() if registry_fixture else None,bearing_bias=a.bearing_bias,confirmation_mode=cfg['confirmation_mode'],review_continuation=a.review_continuation,map_delay_fixture=a.map_delay_fixture,map_delay_state=map_delay.state if map_delay else None,map_delay_brake_seen=map_delay.brake_seen if map_delay else None,execution_safety=cfg['execution_safety'],fault_injection=a.fault_injection,fault_fixture_state=fault.state if fault else None,px4_build=str(build),requested_sim_real_time_factor=a.sim_real_time_factor,harness_wall_watchdog_seconds=a.wall_watchdog_seconds,scope='PX4 SITL dynamics, EKF2 estimated odometry, original guarded executor and native RACER FSM; GNSS-aided simulation, not indoor localization acceptance',wall_s=time.monotonic()-started,budget_clock=a.budget_clock,budget_seconds=a.seconds,budget_end_sim=budget_end_sim,counts=counts,last=last,estimated_path_m=path,max_estimated_height_m=max_z,error=failure,unexpected_exits=unexpected_exits,cleanup=cleanup,layout=a.layout,performance_profile=a.performance,negative=a.negative,active_confirmation=a.active_confirmation,active_review=cfg['active_review'],stop_and_plan=cfg['stop_and_plan'],no_path_recovery=a.recover_no_path,record_video=a.record_video,flight_fixture=os.environ.get('RACER_FLIGHT_FIXTURE','none'),no_perception=a.no_perception,detector=a.detector,detector_python_path=a.detector_python_path,segmentation_weights_sha256=hashlib.sha256(Path(a.segmentation_weights).read_bytes()).hexdigest() if a.segmentation_weights else None,vertical_samples=a.vertical_samples,prepared_scene=str(a.prepared) if a.prepared else None,captured_pairs=frame_count,code_sha256=source_hashes,native_binary_sha256=native_hash,px4_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
        (out/'result.json').write_text(json.dumps(report,indent=2));evidence.close()
        for f in logs:f.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        print(json.dumps({k:v for k,v in report.items() if k not in ('last','code_sha256')}))
    if failure:raise SystemExit(1)
if __name__=='__main__':main()
