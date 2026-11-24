#!/usr/bin/env python3
"""Local SITL only: original PX4 EKF2/guards/executor, native ROS2 RACER.

No model pose service, oracle pose, or scene geometry enters decision nodes.
Scene truth is written separately solely for subsequent evaluation.
"""
import argparse,json,os,sys,time,subprocess,math,hashlib,tarfile
from contextlib import suppress
from io import BytesIO
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
from aspire_interfaces import topics
from person_search.detector_spec import MASK_DETECTORS
from tools.flight.scene_bundle import scene
from aspire_camera.gazebo import tf_arguments,set_text
from tools.flight.gazebo_shutdown import terminate_flight
from tools.flight.cpu_profile import CPUProfile
from tools.flight.gazebo_guard import configuration as guard_configuration,verify_loaded as verify_guard_loaded
from tools.flight.run_safety import BootstrapTrackingWatchdog,StartupDeadline
from core.planning.motion_limits import motion_environment
from core.planning.execution_safety import EXECUTION_PROFILE_ENV, SITL_RELAXED_PROFILE, SITL_FRONTIER_PROFILE
from core.exploration.voxel_config import voxel_environment


def validate_flight_scene(out, config):
    if config.get('manual_camera_test'):
        raise ValueError('Refuse static sensor rig for flight')
    world = ET.parse(out/'scene.sdf').find('world')
    if world is None:
        return  # Scene pacing validation reports a missing world.
    vehicles = {vehicle['model'] for vehicle in config['vehicles'] if 'model' in vehicle}
    static = {}
    for model in world.findall('model'):
        if model.get('name') in vehicles:
            static[model.get('name')] = model.findtext('static', 'false')
    for include in world.findall('include'):
        uri = include.findtext('uri', '')
        name = include.findtext('name') or uri.rstrip('/').rsplit('/', 1)[-1]
        if name not in vehicles:
            continue
        override = include.findtext('static')
        if override is not None:
            static[name] = override
        elif uri.startswith('model://'):
            model_path = out/'models'/uri.removeprefix('model://')/'model.sdf'
            if model_path.resolve().is_relative_to((out/'models').resolve()) and model_path.is_file():
                static[name] = ET.parse(model_path).findtext('model/static', 'false')
    if any(value.strip().lower() in ('true', '1') for value in static.values()):
        raise ValueError('Refuse static aircraft model for flight')


def scene_real_time_factor(path, requested, execution_safety):
    """Return the requested and effective rates after configuring the output SDF."""
    tree=ET.parse(path)
    world=tree.find('world')
    if world is None:
        raise ValueError('Scene has no world')
    physics=world.find('physics')
    if requested is not None and physics is None:
        physics=ET.SubElement(world,'physics',name='default_physics',type='ode')
        set_text(physics,'max_step_size','0.001')
    factor=physics.find('real_time_factor') if physics is not None else None
    requested_rtf=1. if requested is None else requested
    if requested is None and factor is not None:
        value=factor.text or ''
        try:
            if len(factor):
                raise ValueError
            requested_rtf=float(value)
        except ValueError as error:
            raise ValueError('Expected a numeric physics real_time_factor') from error
    if not math.isfinite(requested_rtf):
        raise ValueError('Physics real_time_factor must be finite')

    effective_rtf=requested_rtf
    if execution_safety and physics is not None:
        step=float(physics.findtext('max_step_size', '0.001'))
        # Harmonic disables pacing below 1e-9 and stores its pacing period as
        # signed int nanoseconds. Use a period that can bound wall-time checks.
        if not 1.e-9<=step<2**31/1.e9:
            raise ValueError('Physics max_step_size is outside the supported pacing range')
        if not 1.e-9<=requested_rtf<=1. or int(step*1.e9)/requested_rtf>=2**31:
            effective_rtf=1.
    if requested is not None or effective_rtf!=requested_rtf:
        set_text(physics,'real_time_factor',effective_rtf)
        tree.write(path)
    return requested_rtf,effective_rtf


OPTIONAL_PROCESSES = frozenset({
    'rviz', 'x11vnc', 'websockify', 'display', 'map_display',
    'evaluation', 'continuous_rgb', 'person_search',
})


def main():
    p=argparse.ArgumentParser()
    profile = p.add_mutually_exclusive_group()
    profile.add_argument('--sitl-frontier-execution', action='store_true',
                         help='SITL frontier geometry and finer certified stop candidates')
    profile.add_argument('--sitl-relaxed-execution', action='store_true',
                   help='Compare relaxed in-flight geometry and tracking triggers using PX4 SITL')
    p.add_argument('--scene-spec', type=Path, help='D-owned indoor scene JSON')
    p.add_argument('--detector-manifest', type=Path,
                   help='Local JSON manifest for the YOLO segmentation weights and runtime')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--static-guidance-settings',type=Path,help='Validated common static settings JSON for both arms; --bearing-bias selects guided');p.add_argument('--px4-build',type=Path,help='Isolated verified PX4 SITL build directory');p.add_argument('--sim-real-time-factor',type=float,help='Diagnostic pacing only; physics timestep stays unchanged');p.add_argument('--wall-watchdog-seconds',type=float,help='Explicit harness wall budget for slow simulation; never changes flight guards');p.add_argument('--external',type=Path,default=Path('/planner/external'));p.add_argument('--seconds',type=float,default=240);p.add_argument('--budget-clock',choices=['wall','sim'],default='wall');p.add_argument('--negative',action='store_true');p.add_argument('--active-review',action='store_true');p.add_argument('--active-confirmation',action='store_true',help='Opt-in confirmation only after a measured RACER review and fresh RGB-D evidence');p.add_argument('--confirmation-mode',choices=['legacy','strict','hybrid','decoupled','registry'],default='',help='Explicit confirmation mode, independent of active review for fair comparisons');p.add_argument('--registry-cue-fixture',action='store_true',help='Explicit negative SITL-only target-observation ID churn/dropout fixture');p.add_argument('--bearing-bias',action='store_true',help='Opt-in bounded RGB bearing utility in native RACER candidate ranking');p.add_argument('--review-continuation',action='store_true',help='Opt-in bounded native continuation of the same review request');p.add_argument('--stop-and-plan',action='store_true',help='Wait for measured trajectory completion before replanning, independently of perception');p.add_argument('--recover-no-path',action='store_true');p.add_argument('--layout', default='original', help='Named start from the scene specification');p.add_argument('--performance',choices=['legacy','moderate','consistent','bounded-fast'],default='legacy');p.add_argument('--rviz',action='store_true');p.add_argument('--record-video',action='store_true');p.add_argument('--record-debug-evidence',action='store_true');p.add_argument('--prepared',type=Path,help='Generated flyable scene; sensor truth files are not read by decision nodes');p.add_argument('--no-perception',action='store_true');p.add_argument('--vertical-samples',action='store_true',help='Opt-in native viewpoint altitude sampling');p.add_argument('--detector',choices=['hog',*MASK_DETECTORS],default='hog');p.add_argument('--segmentation-weights',default='');p.add_argument('--detector-python-path',default='');p.add_argument('--min-rgb-period-s',type=float,default=.2,help='Minimum spacing between inferred RGB captures; 0 keeps every newest frame only');p.add_argument('--map-projection-stride',type=int,default=8,help='Depth projection pixel stride for the mapper; 4 densifies narrow obstacles');p.add_argument('--fault-injection',choices=['flags','position','dds'],help='Explicit SITL-only application/message or owned-agent interruption fixture');p.add_argument('--map-delay-fixture',action='store_true',help='Explicit SITL-only bounded interruption of the owned depth mapper');a=p.parse_args()
    if a.prepared and a.scene_spec:
        p.error("choose either --prepared or --scene-spec")
    if a.detector_manifest is not None and (a.no_perception or a.detector != 'yolo11n-seg' or not a.segmentation_weights):
        p.error('--detector-manifest requires enabled perception, --detector yolo11n-seg and --segmentation-weights')
    bundle = None
    if a.prepared and (a.prepared / "bundle.json").is_file():
        bundle = json.loads((a.prepared / "bundle.json").read_text())
        if bundle.get("version") != 1 or type(bundle.get("negative")) is not bool:
            p.error("unsupported prepared scene manifest")
        if a.negative and not bundle["negative"]:
            p.error("--negative conflicts with prepared scene manifest")
        a.negative = bundle["negative"]
    for flag, seconds in (('--seconds', a.seconds), ('--wall-watchdog-seconds', a.wall_watchdog_seconds)):
        if seconds is not None and (not math.isfinite(seconds) or seconds < 0):
            p.error(flag+' must be finite and nonnegative')
    if not math.isfinite(a.min_rgb_period_s) or a.min_rgb_period_s < 0:
        p.error('--min-rgb-period-s must be finite and nonnegative')
    if not 1 <= a.map_projection_stride <= 16:
        p.error('--map-projection-stride must be 1..16')
    diagnostics={}
    def warn(kind, reason):
        if kind not in diagnostics:
            diagnostics[kind]=str(reason)
            with suppress(OSError):print(f'{kind}: {reason}', file=sys.stderr)

    def evidence_hash(path, kind):
        try:
            return hashlib.sha256(path.read_bytes()).hexdigest()
        except OSError as error:
            warn(kind,error)

    execution_safety=os.environ.get('RACER_EXECUTION_SAFETY')=='1'
    if a.active_confirmation and a.confirmation_mode:
        warn('active_confirmation','Using the explicit confirmation mode')
        a.active_confirmation=False
    if a.no_perception and (a.confirmation_mode or a.active_confirmation or a.active_review):
        warn('perception','Confirmation and review disabled without perception')
        a.confirmation_mode='';a.active_confirmation=False;a.active_review=False
    if a.confirmation_mode in ('decoupled','registry'):
        if a.detector not in MASK_DETECTORS:
            warn('confirmation','Selected mode requires person instance segmentation; using legacy confirmation')
            a.confirmation_mode='legacy'
        elif a.active_review:
            warn('active_review','Legacy XYZ review disabled for the selected confirmation mode')
            a.active_review=False
    if a.map_delay_fixture and (not execution_safety or a.fault_injection):
        warn('map_delay_fixture','Skipped without execution safety or alongside another fault fixture')
        a.map_delay_fixture=False
    if a.registry_cue_fixture and not (a.negative and a.confirmation_mode=='registry' and execution_safety):
        warn('registry_cue_fixture','Skipped outside negative registry SITL with execution safety')
        a.registry_cue_fixture=False
    a.active_review=a.active_review or a.active_confirmation
    if (a.sitl_relaxed_execution or a.sitl_frontier_execution) and (a.active_review or not execution_safety):
        warn('execution_profile','Requested profile unavailable without execution safety and inactive review')
        a.sitl_frontier_execution=a.sitl_relaxed_execution=False
    os.environ.pop(EXECUTION_PROFILE_ENV, None)
    if a.sitl_frontier_execution:
        os.environ[EXECUTION_PROFILE_ENV] = SITL_FRONTIER_PROFILE
    elif a.sitl_relaxed_execution:
        os.environ[EXECUTION_PROFILE_ENV] = SITL_RELAXED_PROFILE
    if a.review_continuation and not a.active_review:
        warn('review_continuation','Disabled because active review is unavailable')
        a.review_continuation=False
    if a.active_review and a.confirmation_mode in ('strict','hybrid'):
        a.review_continuation=True
    manifest_sha256 = None
    if a.detector_manifest is not None:
        # Child processes start in the repository. Resolve model paths from the caller.
        a.detector_manifest = a.detector_manifest.expanduser().absolute()
        a.segmentation_weights = str(Path(a.segmentation_weights).expanduser().absolute())
        manifest_sha256 = evidence_hash(a.detector_manifest, 'detector_manifest')
        if manifest_sha256 is None:
            p.error(f'cannot read --detector-manifest {a.detector_manifest}')
    weights_sha256=evidence_hash(Path(a.segmentation_weights),'segmentation_weights') if a.segmentation_weights else None
    requested_cpu_profile=os.environ.get('RACER_CPU_PROFILE','inherit')
    try:
        cpu_profile=CPUProfile(requested_cpu_profile)
    except (OSError,ValueError) as error:
        warn('cpu_profile',error)
        cpu_profile=CPUProfile()
    try:
        cpu_profile.install_parent()
    except (OSError,RuntimeError) as error:
        warn('cpu_parent_affinity',error)
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    if a.prepared:
        import shutil
        shutil.copytree(a.prepared/'models',out/'models')
        for name in ('scene.sdf','runtime.json','bridge.yaml'):
            shutil.copyfile(a.prepared/name,out/name)
        for name in ('scene-audit.json','evaluation-truth.json','scene-spec.json','bundle.json'):
            src=a.prepared/name
            if src.is_file():
                try:
                    shutil.copyfile(src,out/name)
                except OSError as error:
                    warn(name,error)
        cfg=json.loads((out/'runtime.json').read_text())
        if bundle is not None:
            cfg['models'] = str(ROOT / 'external/PX4-gazebo-models/models')
        a.layout=cfg['layout'];a.performance=cfg['performance_profile']
    elif a.scene_spec:
        cfg = scene(out, a.negative, a.layout, a.performance, spec_path=a.scene_spec)
    else:
        cfg = scene(out, a.negative, a.layout, a.performance)
    validate_flight_scene(out,cfg)
    requested_rtf,rtf=scene_real_time_factor(out/'scene.sdf',a.sim_real_time_factor,execution_safety)
    if requested_rtf!=rtf:
        warn('sim_real_time_factor',f'Using {rtf:g}x for execution safety timing after a request for {requested_rtf:g}x')
    wall_watchdog_seconds = a.wall_watchdog_seconds
    startup_watchdog = StartupDeadline(real_time_factor=rtf if a.budget_clock == 'sim' else 1.)
    if execution_safety:
        cfg['execution_safety_timing_contract']=dict(physics_real_time_factor=rtf,
            max_sim_to_wall_rate=1.,check_clock='monotonic_wall_upper_bound_at_rtf_le_1')
    if a.performance=='bounded-fast' or a.recover_no_path or (execution_safety and len(cfg['vehicles'])==1):
        a.stop_and_plan=True
    cfg.update(bearing_bias=a.bearing_bias,confirmation_mode=a.confirmation_mode or ('strict' if a.active_confirmation else 'legacy'),review_continuation=a.review_continuation,map_delay_fixture=a.map_delay_fixture,execution_safety=execution_safety,active_confirmation=a.active_confirmation,active_review=a.active_review and not a.no_perception,stop_and_plan=a.active_review or a.stop_and_plan,no_path_recovery=a.recover_no_path,vertical_samples=a.vertical_samples,recover_frontiers=a.active_review or a.stop_and_plan)
    if os.environ.get('ASPIRE_NATIVE_MAP_DIRECT_DDS')=='1' or cfg.get('native_frame_source')=='mapper_dds':
        if execution_safety and len(cfg['vehicles'])==1 and os.environ.get('RACER_RECORD_INPUT')!='1':
            cfg['native_frame_source']='mapper_dds'
        else:
            warn('native_map_dds','Using mapper observations because direct DDS prerequisites are unavailable')
            cfg['native_frame_source']='mapper_observation'
    # Select the profile only from this simulator entry point, never from a
    # reused prepared-scene JSON file.
    cfg.pop('execution_safety_profile', None)
    if a.sitl_frontier_execution:
        cfg['execution_safety_profile'] = SITL_FRONTIER_PROFILE
        cfg['voxel_map'] = dict(cfg.get('voxel_map', {}), legacy_clearance_m=.50)
    elif a.sitl_relaxed_execution:
        cfg['execution_safety_profile'] = SITL_RELAXED_PROFILE
    if a.static_guidance_settings:
        from aspire_planning.static_pair import apply_static_settings
        try:
            if not a.prepared or weights_sha256 is None:
                raise ValueError('Static paired runs require a prepared scene and explicit detector weights')
            settings=json.loads(a.static_guidance_settings.read_text())
            perception_contract = dict(detector=a.detector, weights_sha256=weights_sha256)
            if manifest_sha256 is not None:
                perception_contract['detector_manifest_sha256'] = manifest_sha256
            paired = dict(cfg, static_perception_contract=perception_contract)
            cfg=apply_static_settings(paired,settings,guided=a.bearing_bias,seconds=a.seconds,budget_clock=a.budget_clock)
        except (OSError,ValueError,KeyError,TypeError) as error:
            warn('static_pair',error)
            cfg.pop('static_pair',None)
            cfg.update(static_event_replan=False,static_cancel_enabled=False,static_moving_c2=False)
    from tools.racer_ros2.configuration import static_feature_options
    session=str(int.from_bytes(os.urandom(6),'big') or 1) if cfg.get('static_cancel_enabled') is True else ''
    options=static_feature_options(dict(cfg,static_cancel_session=session))
    static_switches=('static_event_audit','static_event_replan','static_weak_enabled','static_cancel_enabled','static_moving_c2')
    disabled=[key for key in static_switches if cfg.get(key) and not options[key]]
    if disabled:
        warn('static_features','Unavailable: '+', '.join(disabled))
        cfg.pop('static_pair',None)
    for key in cfg.keys() & options.keys():
        cfg[key]=options[key]
    cfg.pop('static_cancel_session',None)
    if cfg.get('static_pair'):
        from aspire_planning.static_pair import common_runtime_digest
        cfg['static_pair']['common_runtime_sha256']=common_runtime_digest(cfg)
    if options['static_cancel_session']:
        session=options['static_cancel_session']
        os.environ['RACER_STATIC_CANCEL_SESSION']=session
        try:
            (out/'static-cancellation-session.json').write_text(json.dumps(dict(session=session,scope='per-run namespace, not authorization')))
        except OSError as error:
            warn('static_cancel_session_record',error)
    else:os.environ.pop('RACER_STATIC_CANCEL_SESSION',None)
    if options['static_moving_c2']:
        os.environ['RACER_STATIC_MOVING_C2']='1'
    else:os.environ.pop('RACER_STATIC_MOVING_C2',None)
    (out/'runtime.json').write_text(json.dumps(cfg,indent=2));ext=a.external.resolve()
    from aspire_interfaces.visibility import visibility_ros_arguments
    try:
        visibility_args=visibility_ros_arguments(cfg)
    except (ValueError,TypeError,KeyError) as error:
        warn('visibility_configuration',error)
        visibility_args=[]
    try:
        guard_env,guard_report=guard_configuration(out/'scene.sdf')
    except (ValueError,OSError,ET.ParseError) as error:
        guard_env,guard_report={},dict(enabled=False,error=str(error))
        warn('gazebo_guard_configuration',error)
    build=a.px4_build.resolve() if a.px4_build else ext/'PX4-Autopilot/build/px4_sitl_default';binary=build/'bin/px4';agent=ext/'agent_install/bin/MicroXRCEAgent'
    if not binary.is_file() or not agent.is_file():raise RuntimeError('Existing local SITL dependencies unavailable')
    os.environ.update(DISPLAY=':119',LIBGL_ALWAYS_SOFTWARE='1',GZ_PARTITION='person_flight_'+out.name,ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/'roslog'),PYTHONDONTWRITEBYTECODE='1',GZ_SIM_RESOURCE_PATH=os.pathsep.join([str(out/'models'),cfg['models'],str(ROOT/'external/rgbd_scene_assets')]),PX4_GZ_STANDALONE='1',PX4_GZ_WORLD='annual',HEADLESS='1')
    os.environ.update(motion_environment(cfg));os.environ.update(voxel_environment(cfg))
    from ament_index_python.packages import get_package_prefix
    native_binary=Path(get_package_prefix('racer_core'))/'lib/racer_core/exploration_node'
    native_hash=evidence_hash(native_binary,'native_binary_hash')
    source_hashes={}
    if a.record_debug_evidence:
        try:
            with tarfile.open(out/'runtime-source.tar.gz','x:gz') as archive:
                for folder in (ROOT/'tools',ROOT/'src'):
                    for path in folder.rglob('*'):
                        if path.suffix not in ('.py','.cpp','.hpp','.sh') or not path.is_file():continue
                        name=str(path.relative_to(ROOT));raw=path.read_bytes()
                        info=archive.gettarinfo(str(path),arcname=name)
                        if info.isfile():info.size=len(raw)
                        archive.addfile(info,BytesIO(raw) if info.isfile() else None)
                        source_hashes[name]=hashlib.sha256(raw).hexdigest()
        except (OSError,tarfile.TarError) as error:
            source_hashes.clear()
            warn('runtime_source_archive',error)
    children=[];child_names={};fault=None;map_delay=None;unexpected_exits=[]
    observed_exits=set()
    def start(name,args,env=None,cwd=None):
        try:
            log=(out/(name+'.log')).open('x')
        except OSError as error:
            warn(name+'_log',error)
            log=subprocess.DEVNULL
        try:
            proc=subprocess.Popen(cpu_profile.best_effort_command(name,args),
                env=dict(os.environ,**(env or {})),cwd=cwd or ROOT,stdout=log,
                stderr=subprocess.STDOUT,start_new_session=True)
        except OSError as error:
            if name not in OPTIONAL_PROCESSES:raise
            warn(name+'_spawn',error)
            return None
        finally:
            if log != subprocess.DEVNULL:
                try:log.close()
                except OSError as error:warn(name+'_log',error)
        children.append(proc);child_names[proc.pid]=name
        try:
            cpu_profile.verify(name,proc)
        except (OSError,RuntimeError) as error:
            warn(name+'_affinity',error)
        return proc

    def check_child_exits():
        for child in children:
            status=child.poll()
            if status is None or child.pid in observed_exits:continue
            observed_exits.add(child.pid)
            name=child_names[child.pid]
            exit_report=dict(name=name,pid=child.pid,returncode=status)
            unexpected_exits.append(exit_report)
            if name in OPTIONAL_PROCESSES:
                warn(name+'_exit',status)
            else:
                raise RuntimeError('Owned process exited: '+json.dumps(exit_report))
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image,CameraInfo
    from aspire_camera.poses import PoseHistory
    from aspire_camera.rgbd_alignment import stamp_ns
    import numpy as np
    cv2=None
    if a.record_debug_evidence:
        try:
            import cv2
        except (ImportError,OSError) as error:
            warn('rgbd_snapshot',error)
    from std_msgs.msg import Float64MultiArray,String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    rclpy.init();node=rclpy.create_node('flight_evidence');node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)]);tf=TransformBroadcaster(node)
    pose_history=PoseHistory() if a.record_debug_evidence else None
    frames={k:{} for k in ('rgb','depth','ri','di')};last_frame=-1e9;frame_count=0
    if a.record_debug_evidence:
        def image_received(key,msg):
            q=frames[key];q[stamp_ns(msg.header)]=msg
            while len(q)>30:del q[min(q)]
        for key, kind, stream, channel in [('rgb', Image, 'rgb', 'image'), ('depth', Image, 'depth', 'image'),
                                           ('ri', CameraInfo, 'rgb', 'camera_info'), ('di', CameraInfo, 'depth', 'camera_info')]:
            topic = topics.camera_topic(0, stream, channel)
            node.create_subscription(kind, topic, lambda m,k=key:image_received(k,m), qos_profile_sensor_data)
    try:
        evidence=(out/'telemetry.jsonl').open('x',buffering=1)
    except OSError as error:
        warn('telemetry_write',error)
        evidence=None
    started=time.monotonic();counts={};last={};previous=None;path=0.;max_z=0.;last_log=-1.;failure=None;mission_failure=None;budget_exhausted=False;budget_end_sim=0.;bootstrap_watchdog=BootstrapTrackingWatchdog();deferred_started=False
    def record(kind,row):
        counts[kind]=counts.get(kind,0)+1;last[kind]=row
        if evidence is not None and 'telemetry_write' not in diagnostics:
            try:
                evidence.write(json.dumps(dict(kind=kind,wall_s=time.monotonic()-started,data=row))+'\n')
            except OSError as error:
                warn('telemetry_write',error)
    def record_message(kind,message):
        try:
            row=json.loads(message.data)
            if kind in ('flight','execution') and not isinstance(row,dict):raise ValueError('Expected a telemetry object')
        except ValueError as error:
            warn(kind+'_telemetry',error)
            return
        record(kind,row)
    def odom(m):
        nonlocal previous,path,max_z,last_log
        pos=m.pose.pose.position;q=m.pose.pose.orientation
        if pose_history is not None:
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
    for kind,topic,qos in [('flight','/drone_0/flight_status',qos_profile_sensor_data),('execution',topics.view_execution(0),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),('tracks','/search/tracks',10),('semantic','/search/semantic_candidates',10),('perception_timing','/search/perception_timing',10),('active_confirmation','/search/active_confirmation',10),('presence_event','/search/presence_events',30),('review_status',topics.REVIEW_STATUS,10),('review_cancel',topics.REVIEW_CANCEL,10),('bearing','/search/bearings',30),('bearing_decision','/search/bearing_decision',100),('person_records','/uav0/search/person_records',100),('target_registry_events','/uav0/search/target_registry_events',100),('target_summary','/search/target_summary',100),('target_sync','/search/target_sync_request',100)]:node.create_subscription(String,topic,lambda m,k=kind:record_message(k,m),qos)
    if os.environ.get('ASPIRE_SAFETY_MAP_TIMING')=='1':
        for kind in ('static_cancel','static_cancel_ack','execution_feedback'):
            topic='feedback' if kind=='execution_feedback' else kind
            node.create_subscription(Float64MultiArray,f'/baseline/{topic}_0',
                lambda m,k=kind:record(k,dict(data=list(m.data),sim_ns=node.get_clock().now().nanoseconds)),100)
        node.create_subscription(String,topics.target_observations('uav0'),
            lambda m:record_message('target_observations',m),100)
    registry_fixture=None;registry_fixture_pub=None
    if a.registry_cue_fixture:
        from tools.flight.registry_cue_fixture import ChurningCueFixture
        registry_fixture=ChurningCueFixture()
        registry_fixture_pub=node.create_publisher(String,topics.target_observations('uav0'),20)
    def start_viewer():
        rviz_cfg=yaml.safe_load((ROOT/'config/search.rviz').read_text())
        # Bound visualization work on the software-rendered local container.
        # This changes only the display, never mapper input or occupancy.
        manager = rviz_cfg['Visualization Manager']
        manager['Global Options']['Frame Rate'] = 5
        manager['Tools'] = [{'Class': 'rviz_default_plugins/MoveCamera'}]
        displays = manager['Displays']
        live_depth = next(d for d in displays if d['Name'] == 'Measured depth')
        live_depth.update({'Decay Time': 0.3, 'Style': 'Points'})
        if cfg['execution_safety']:
            live_depth['Enabled'] = False
            displays.insert(1, {
                'Class': 'rviz_default_plugins/PointCloud2', 'Name': 'Accumulated occupancy',
                'Enabled': True, 'Style': 'Points', 'Size (Pixels)': 4,
                'Color Transformer': 'AxisColor', 'Axis': 'Z', 'Decay Time': 0,
                'Topic': {'Value': '/search/mapped_cloud', 'Reliability Policy': 'Reliable',
                          'Durability Policy': 'Transient Local', 'Depth': 1},
            })
        for display in rviz_cfg['Visualization Manager']['Displays']:
            if a.confirmation_mode=='registry' and display.get('Topic',{}).get('Value')=='/search/targets':
                display['Topic']['Value']='/uav0/search/targets';display['Name']='Persistent presence and visible-surface status'
        view=rviz_cfg['Visualization Manager']['Views']['Current'];start_xyz=cfg['fleet_starts'][0]
        view['Focal Point']=dict(zip(('X','Y','Z'),map(float,start_xyz)));view['Distance']=22. if a.prepared else 11.
        if a.detector in MASK_DETECTORS and not a.no_perception:
            raw_rgb = next(d for d in displays if d['Name'] == 'Actual RGB')
            displays.append(dict(raw_rgb, Name='Person masks and boxes', Enabled=False,
                                 Topic=dict(raw_rgb['Topic'], Value='/search/detection_image',
                                            **{'Reliability Policy': 'Reliable'})))
        rviz_path=out/'search.rviz';rviz_path.write_text(yaml.safe_dump(rviz_cfg))
        viewer_display = os.environ.get('ASPIRE_RVIZ_DISPLAY')
        rviz_args = ['rviz2','-d',str(rviz_path),'--ros-args','-p','use_sim_time:=true']
        if viewer_display:
            # Gazebo remains on the original Xvfb display.
            start('rviz', rviz_args, env={'DISPLAY': viewer_display})
        else:
            # WSLg exports WAYLAND_DISPLAY even when this run uses Xvfb.
            # Keep the browser viewer on its private X11 display.
            start('rviz', rviz_args, env={'QT_QPA_PLATFORM': 'xcb'})
            start('x11vnc',['env','-u','WAYLAND_DISPLAY','-u','WAYLAND_SOCKET',
                            'x11vnc','-display',':119','-localhost','-rfbport','5901',
                            '-nopw','-forever','-shared'])
            start('websockify',['websockify','--web=/usr/share/novnc','127.0.0.1:18680','127.0.0.1:5901'])
        start('display',[sys.executable,'-m','tools.flight.visualization'])
        if cfg['execution_safety']:
            start('map_display',[sys.executable,'-m','tools.flight.map_display'])
    try:
        start('xvfb',['Xvfb',':119','-screen','0','1280x900x24','-nolisten','tcp']);time.sleep(.7)
        if a.rviz:
            try:
                start_viewer()
            except (OSError,ValueError,KeyError,TypeError,StopIteration,yaml.YAMLError) as error:
                warn('viewer_configuration',error)
        # Expensive perception/video/native mapping start only after genuine EKF origin and odometry.
        start('evaluation',[sys.executable,'-m','tools.flight.evaluation_recorder',str(out)])
        start('world_tf',['ros2','run','tf2_ros','static_transform_publisher','--frame-id','map','--child-frame-id','world'])
        gazebo_proc=start('gazebo',['gz','sim','-r','-s',str(out/'scene.sdf'),'--seed','123'],env=guard_env)
        if guard_env:
            try:
                verify_guard_loaded(gazebo_proc,guard_report)
            except RuntimeError as error:
                guard_report.update(loaded_verified=False,error=str(error))
                warn('gazebo_guard_load',error)
        agent_proc=start('agent',[str(agent),'udp4','-p','8888'],{'LD_LIBRARY_PATH':str(ext/'agent_install/lib')+':'+os.environ.get('LD_LIBRARY_PATH','')})
        start('sensor_bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(out/'bridge.yaml')])
        for i,t in enumerate(cfg['camera_transforms']):start('tf-'+str(i),['ros2','run','tf2_ros','static_transform_publisher',*tf_arguments(t)])
        v=cfg['vehicles'][0];work=out/'px4_1';work.mkdir();script=(build/'etc/init.d-posix/rcS').read_text();needle='uxrce_dds_client start -t udp -p $uxrce_dds_port $uxrce_dds_ns'
        if script.count(needle)!=1:raise ValueError('Review changed PX4 startup')
        if os.environ.get('RACER_PX4_LOGGER_TOPICS'):
            from tools.flight.logger_configuration import install_logger_topics
            try:
                install_logger_topics(work,build,os.environ['RACER_PX4_LOGGER_TOPICS'])
            except (OSError,ValueError) as error:
                warn('px4_logger_configuration',error)
        startup=work/'startup.sh';startup.write_text(script.replace(needle,'param set UXRCE_DDS_SYNCT 0\n'+needle)+'\nparam set COM_RC_IN_MODE 4\nparam set NAV_DLL_ACT 0\nparam set COM_OF_LOSS_T 0.5\nparam set COM_OBL_RC_ACT 4\n')
        time.sleep(3)
        start('px4',[str(binary),'-i','1','-d','-w',str(work),'-s',str(startup),str(build/'etc')],dict(PX4_SYS_AUTOSTART='4002',PX4_SIM_MODEL='gz_x500_depth',PX4_GZ_MODEL_NAME=v['model'],PX4_PARAM_EKF2_GPS_DELAY='0',PX4_PARAM_SENS_EN_MAGSIM='1',PX4_PARAM_EKF2_MAG_TYPE='6'),str(work))
        params=dict(use_sim_time=True,drone_id=0,px4_instance=1,bounds=json.dumps(cfg['bounds']),fleet_starts=json.dumps(cfg['fleet_starts']),start=json.dumps(v['start']),spawn=json.dumps(v['spawn']),initial_position=json.dumps(v['initial_position']),origin_calibration_s=1.5,require_depth_odometry=False,sitl_auto_arm=True,publish_native_frames=True,bootstrap_altitudes=json.dumps(cfg['bootstrap_altitudes']),control_timing_dir=str(out/'timing') if a.record_debug_evidence else '',safety_flight_limits=json.dumps(cfg.get('racer_flight_limits')),execution_safety_dir=str(out/'execution-safety') if a.record_debug_evidence else '')
        params['publish_native_dds']=cfg.get('native_frame_source')=='mapper_dds'
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
        start('px4_bridge_node',[sys.executable,'-m','annual_runtime.px4_bridge_node',
                                 '--ros-args','--params-file',str(paramfile),*remaps])
        def start_search_graph():
            for name in ('pointcloud_mapping_node','view_executor_node'):
                runtime_proc=start(name,[sys.executable,'-m','annual_runtime.'+name,'--ros-args','--params-file',str(paramfile),
                    *(['-p','projection_stride:='+str(a.map_projection_stride)] if name=='pointcloud_mapping_node' else [])])
                if name=='pointcloud_mapping_node' and a.map_delay_fixture:
                    nonlocal map_delay
                    from tools.flight.map_delay_fixture import MeasuredMapDelay
                    try:
                        map_delay=MeasuredMapDelay(runtime_proc,out)
                    except OSError as error:
                        warn('map_delay_fixture',error)
            start('racer_adapter',[sys.executable,'-m','tools.research.frontier_baselines.live_ros2','--runtime',str(out/'runtime.json'),'--output',str(out/'flight'),'--transport','dds'])
            if not a.no_perception:
                detector_args = ['-p', 'detector_backend:='+a.detector,
                                 '-p', 'segmentation_weights:='+a.segmentation_weights]
                if a.detector_manifest is not None:
                    detector_args += ['-p', f'detector_manifest:={a.detector_manifest}']
                start('person_search', [
                    sys.executable, '-m', 'person_search.node', '--ros-args',
                    '-p', 'use_sim_time:=true', '-p', 'require_estimated_pose:=true',
                    '-p', 'active_review:='+str(a.active_review).lower(),
                    '-p', 'active_confirmation:='+str(a.active_confirmation).lower(),
                    *(['-p', 'confirmation_mode:='+a.confirmation_mode] if a.confirmation_mode else []),
                    *detector_args, *(['-p', 'evidence_dir:='+str(out/'perception')] if a.record_debug_evidence else []), *visibility_args,
                    '-p','min_rgb_period_s:='+repr(a.min_rgb_period_s),
                ], env={'PYTHONPATH': a.detector_python_path+os.pathsep+os.environ.get('PYTHONPATH','')}
                    if a.detector_python_path else None)
            if a.record_video:start('continuous_rgb',[sys.executable,'-m','tools.flight.video_recorder',str(out)])
        while True:
            elapsed=time.monotonic()-started
            budget_end_sim=node.get_clock().now().nanoseconds*1e-9
            check_child_exits()
            if (elapsed if a.budget_clock=='wall' else budget_end_sim)>=a.seconds:
                budget_exhausted=True
                break
            if wall_watchdog_seconds is not None and elapsed>=wall_watchdog_seconds:
                raise RuntimeError('Explicit wall watchdog exceeded')
            rclpy.spin_once(node,timeout_sec=.01)
            if not rclpy.ok():raise KeyboardInterrupt
            startup_error=startup_watchdog.observe(budget_end_sim,elapsed,last.get('estimated_odometry'),
                                                   last.get('flight',{}),last.get('execution',{}))
            if startup_error:
                warn('startup_ready' if startup_watchdog.has_odometry else 'startup_odometry',startup_error)
            if startup_watchdog.has_odometry and not deferred_started:
                record('startup',dict(event='validated_odometry_start_deferred_graph',**startup_watchdog.report()))
                start_search_graph();deferred_started=True
            if registry_fixture:
                packet=registry_fixture.step(budget_end_sim,last.get('execution',{}).get('ready',False))
                if packet:
                    registry_fixture_pub.publish(String(data=json.dumps(packet,separators=(',',':'))))
                    record('registry_fixture',dict(explicit_synthetic_observation=True,batch=packet))
            if map_delay and 'map_delay_fixture' not in diagnostics:
                try:
                    map_delay.step(budget_end_sim,last.get('flight',{}),last.get('execution',{}))
                except (OSError,RuntimeError) as error:
                    warn('map_delay_fixture',error)
                    try:
                        map_delay.resume(budget_end_sim,'fixture_error')
                    except OSError as error:
                        warn('map_delay_fixture_resume',error)
            fixture_complete=fault and fault.step(budget_end_sim,last.get('flight',{}),last.get('execution',{}))
            safety_fault=last.get('execution',{}).get('safety_fault')
            if safety_fault:
                warn('execution_safety_fault',safety_fault)
                mission_failure=mission_failure or f'Execution safety fault: {safety_fault}'
            flight_fault=last.get('flight',{}).get('flight_session_fault')
            if flight_fault and not (fault and fault.allows_observation(flight_fault)):
                warn('flight_session_fault',flight_fault)
                mission_failure=mission_failure or f'Latched flight fault: {flight_fault}'
            if fixture_complete:break
            if a.prepared and 'execution' in last:
                safety_error=bootstrap_watchdog.observe(last['execution'])
                if safety_error:warn('bootstrap_tracking',safety_error)
            if (a.record_debug_evidence and cv2 is not None and time.monotonic()-last_frame>5. and
                    (common := frames['rgb'].keys() & frames['depth'].keys() & frames['ri'].keys() & frames['di'].keys())):
                ns=max(common);pose=pose_history.at(ns*1e-9)
                if pose is not None:
                    last_frame=time.monotonic()
                    try:
                        rgb,depth,ri,di=[frames[k][ns] for k in ('rgb','depth','ri','di')]
                        if rgb.encoding not in ('rgb8','bgr8') or depth.encoding!='32FC1':
                            raise ValueError(f'Unsupported snapshot encoding {rgb.encoding}/{depth.encoding}')
                        rgb_data=np.ndarray((rgb.height,rgb.width,3),np.uint8,bytes(rgb.data),strides=(rgb.step,3,1)).copy()
                        depth_data=np.ndarray((depth.height,depth.width),np.dtype('>f4' if depth.is_bigendian else '<f4'),bytes(depth.data),strides=(depth.step,4)).copy()
                        np.savez_compressed(out/f'rgbd-{frame_count:03}.npz',rgb=rgb_data,depth=depth_data,k=np.array(di.k),rgb_k=np.array(ri.k),stamp_ns=ns,base_position=pose[0],base_quaternion=pose[1].as_quat(),pose_bracket=pose[2])
                        if not cv2.imwrite(str(out/f'rgbd-{frame_count:03}.png'),rgb_data[:,:,::-1] if rgb.encoding=='rgb8' else rgb_data):
                            raise OSError('Could not write RGB snapshot')
                    except (OSError,TypeError,ValueError,cv2.error) as error:
                        warn('rgbd_snapshot',error)
                    else:
                        frame_count+=1
    except (Exception,KeyboardInterrupt) as e:failure=str(e) or type(e).__name__
    finally:
        if a.map_delay_fixture and not (map_delay and map_delay.state=='complete' and map_delay.brake_seen):
            reason='Map-delay fixture did not complete with a reported certified brake'
            warn('map_delay_fixture',reason)
            mission_failure=mission_failure or reason
        if a.fault_injection and (fault is None or fault.state!='complete'):
            reason='Fault fixture did not complete within budget'
            warn('fault_fixture',reason)
            mission_failure=mission_failure or reason
        for name,fixture in (('map_delay_fixture',map_delay),('fault_fixture',fault)):
            if fixture:
                try:
                    fixture.close(budget_end_sim)
                except (OSError,RuntimeError) as error:
                    warn(name+'_cleanup',error)
        cleanup=terminate_flight(children,child_names,out,timeout=8)
        allowed_exit={0,-2,-15,130,143}
        abnormal=[dict(name=child_names[x['pid']],phase='cleanup',**x)
                  for x in cleanup['direct_returncodes'] if x['returncode'] not in allowed_exit]
        unexpected_exits.extend(x for x in abnormal if x['pid'] not in observed_exits)
        critical=[x for x in abnormal if x['name'] not in OPTIONAL_PROCESSES]
        if critical and failure is None:
            if budget_exhausted:
                warn('cleanup_abnormal_exit', json.dumps(critical))
            else:
                failure='Owned process abnormal exit during cleanup: '+json.dumps(critical)
        if budget_exhausted and a.seconds>0 and not startup_watchdog.ready:
            reason=('Run budget expired before measured flight readiness' if deferred_started
                    else 'Run budget expired before valid estimated odometry and search startup')
            warn('startup_incomplete',reason)
            mission_failure=mission_failure or reason
        failure=failure or mission_failure
        # A cleanup crash must never hide outside the run archive.
        core=ROOT/'core'
        if core.is_file() and core.stat().st_mtime>=time.time()-(time.monotonic()-started)-1:
            import shutil
            try:
                (out/'cores').mkdir(exist_ok=True);shutil.move(str(core),str(out/'cores/core'))
            except OSError as error:
                warn('core_archive',error)
        report=dict(gazebo_shutdown_guard=guard_report,cpu_profile=cpu_profile.report(),startup=startup_watchdog.report(),deferred_graph_started=deferred_started,registry_cue_fixture=registry_fixture.report() if registry_fixture else None,bearing_bias=a.bearing_bias,confirmation_mode=cfg['confirmation_mode'],review_continuation=a.review_continuation,map_delay_fixture=a.map_delay_fixture,map_delay_state=map_delay.state if map_delay else None,map_delay_brake_seen=map_delay.brake_seen if map_delay else None,execution_safety=cfg['execution_safety'],fault_injection=a.fault_injection,fault_fixture_state=fault.state if fault else None,px4_build=str(build),requested_sim_real_time_factor=requested_rtf,effective_sim_real_time_factor=rtf,harness_wall_watchdog_seconds=a.wall_watchdog_seconds,scope='PX4 SITL dynamics, EKF2 estimated odometry, original guarded executor and native RACER FSM; GNSS-aided simulation, not indoor localization acceptance',wall_s=time.monotonic()-started,budget_clock=a.budget_clock,budget_seconds=a.seconds,budget_end_sim=budget_end_sim,counts=counts,last=last,estimated_path_m=path,max_estimated_height_m=max_z,error=failure,unexpected_exits=unexpected_exits,cleanup=cleanup,layout=a.layout,performance_profile=a.performance,negative=a.negative,active_confirmation=a.active_confirmation,active_review=cfg['active_review'],stop_and_plan=cfg['stop_and_plan'],no_path_recovery=a.recover_no_path,record_video=a.record_video,flight_fixture=os.environ.get('RACER_FLIGHT_FIXTURE','none'),no_perception=a.no_perception,detector=a.detector,detector_python_path=a.detector_python_path,segmentation_weights_sha256=weights_sha256,vertical_samples=a.vertical_samples,prepared_scene=str(a.prepared) if a.prepared else None,captured_pairs=frame_count,code_sha256=source_hashes,native_binary_sha256=native_hash,px4_binary_sha256=evidence_hash(binary,'px4_binary_hash'))
        report['effective_harness_wall_watchdog_seconds'] = wall_watchdog_seconds
        report['detector_manifest_sha256'] = manifest_sha256
        report['requested_cpu_profile']=requested_cpu_profile
        if evidence is not None:
            try:
                evidence.close()
            except OSError as error:
                warn('telemetry_write',error)
        report['diagnostics']=diagnostics
        try:
            (out/'result.json').write_text(json.dumps(report,indent=2))
        except OSError as error:
            warn('result_write',error)
        finally:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
        with suppress(OSError):print(json.dumps({k:v for k,v in report.items() if k not in ('last','code_sha256')}))
    if failure:raise SystemExit(1)
if __name__=='__main__':main()
