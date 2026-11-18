#!/usr/bin/env python3
"""Native RGB/depth/TF manual-view acceptance, with no flight controller or planner."""
import argparse
from contextlib import suppress
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src/annual_planner/src'))
from tools.prepare_sim import prepare
from tools.rgbd_scene.cameras import tf_arguments
from tools.rgbd_scene.view import switch
from tools.racer_ros2.light.processes import terminate
from annual_runtime.rgbd_alignment import require_registered_pair,stamp_ns


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--interactive',action='store_true',help='Keep rig running; type view names to switch and capture, q to stop')
    p.add_argument('--prepared',type=Path,help='Pre-generated static sensor scene; source directory remains untouched');p.add_argument('--seconds',type=float,default=300);a=p.parse_args()
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    if a.prepared:
        import shutil
        for name in ('models','scene.sdf','runtime.json','bridge.yaml','viewpoints.json','scene-audit.json'):
            src=a.prepared/name
            if src.is_dir():shutil.copytree(src,out/name)
            else:shutil.copyfile(src,out/name)
    else:prepare(ROOT/'tools/rgbd_scene/map.json',out,rgbd_test_scene=True)
    config=json.loads((out/'runtime.json').read_text())
    if not config.get('manual_camera_test'):raise ValueError('Prepared scene must be static sensor-only')
    partition='annual_rgbd_'+out.name
    os.environ.update(DISPLAY=':109',LIBGL_ALWAYS_SOFTWARE='1',GZ_PARTITION=partition,ROS_DOMAIN_ID='198',ROS_LOCALHOST_ONLY='1',
        GZ_SIM_RESOURCE_PATH=os.pathsep.join([str(out/'models'),config['models'],*config['additional_model_paths']]))
    children=[];report=dict(scope='Manual static camera views; no PX4, no planner, no flight acceptance',gz_partition=partition,ros_domain=198,
        asset_lock=json.loads((ROOT/'tools/rgbd_scene/assets.lock.json').read_text()),views={})
    def warn(name,error):
        report.setdefault('diagnostics',{})[name]=str(error)
    def start(name,args):
        log=None
        try:
            log=(out/(name+'.log')).open('x')
        except OSError as error:
            warn(name+'_log',error)
        try:
            child=subprocess.Popen(args,stdout=log if log is not None else subprocess.DEVNULL,
                                   stderr=subprocess.STDOUT,start_new_session=True)
            children.append((name,child))
        finally:
            if log is not None:
                with suppress(OSError):log.close()
    import rclpy
    from sensor_msgs.msg import Image,CameraInfo
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import Buffer,TransformListener,TransformException
    from rclpy.time import Time
    import cv2
    rclpy.init();node=rclpy.create_node('rgbd_acceptance');buffer=Buffer();listener=TransformListener(buffer,node)
    queues={key:{} for key in ('rgb','depth','rgb_info','depth_info')};counts={k:0 for k in queues}
    def received(key,m):
        queues[key][stamp_ns(m.header)]=m;counts[key]+=1
        if len(queues[key])>200:del queues[key][min(queues[key])]
    for key,kind,topic in [('rgb',Image,'rgb/image'),('depth',Image,'depth/image'),('rgb_info',CameraInfo,'rgb/camera_info'),('depth_info',CameraInfo,'depth/camera_info')]:
        node.create_subscription(kind,'/drone_0/'+topic,lambda m,k=key:received(k,m),qos_profile_sensor_data)
    def pump():
        rclpy.spin_once(node,timeout_sec=.02)
        for name,child in children:
            status=child.poll()
            if status is None:continue
            if name!='collision_probe':raise RuntimeError(f'A sensor/TF process exited: {name} ({status})')
            warn('collision_probe_exit',status)
    def collect(name):
        before=max(queues['depth'],default=0)
        moved=switch(out,name);accepted=0;used=set();deadline=time.monotonic()+45
        while accepted<30 and time.monotonic()<deadline:
            pump()
            common=(queues['rgb'].keys() & queues['depth'].keys() & queues['rgb_info'].keys() & queues['depth_info'].keys())-used
            for ns in sorted(common):
                if ns<before+500_000_000:continue
                rgb,depth,ri,di=[queues[k][ns] for k in ('rgb','depth','rgb_info','depth_info')]
                stamp=Time(nanoseconds=ns)
                try:
                    tf=buffer.lookup_transform(rgb.header.frame_id,depth.header.frame_id,stamp)
                    require_registered_pair(rgb,depth,ri,di,tf)
                    if accepted==29:
                        extrinsic=buffer.lookup_transform('drone_0/base_link',depth.header.frame_id,stamp).transform
                except TransformException:
                    continue
                except ValueError as exc:
                    failures=report.setdefault('alignment_failures',{}).setdefault(name,{'count':0})
                    failures.update(count=failures['count']+1,stamp_ns=ns,error=str(exc))
                    used.add(ns)
                    continue
                used.add(ns)
                if accepted==0:first_stamp=ns
                accepted+=1
                if accepted==30:break
        if accepted<30:
            raise TimeoutError(f'No 30 registered pairs for {name}; accepted {accepted}, received {counts}')
        folder=out/name;folder.mkdir(exist_ok=False)
        rgb_bytes,depth_bytes=bytes(rgb.data),bytes(depth.data)
        (folder/'rgb.bin').write_bytes(rgb_bytes);(folder/'depth.bin').write_bytes(depth_bytes)
        pixels=np.ndarray((rgb.height,rgb.width,3),dtype=np.uint8,buffer=rgb_bytes,strides=(rgb.step,3,1))
        if not cv2.imwrite(str(folder/'rgb.png'),pixels[:,:,::-1] if rgb.encoding=='rgb8' else pixels):raise RuntimeError('PNG encoding failed')
        raw=np.ndarray((depth.height,depth.width),dtype=np.dtype('>f4' if depth.is_bigendian else '<f4'),buffer=depth_bytes,strides=(depth.step,4))
        np.save(folder/'depth.npy',raw)
        def calibration(info):return dict(frame=info.header.frame_id,stamp_ns=stamp_ns(info.header),width=info.width,height=info.height,k=list(info.k),p=list(info.p),r=list(info.r),d=list(info.d),distortion_model=info.distortion_model)
        t=tf.transform.translation;q=tf.transform.rotation
        record=dict(switch=moved,pairs_checked=accepted,first_stamp_ns=first_stamp,last_stamp_ns=ns,
            max_rgb_depth_stamp_difference_ns=0,rgb=calibration(ri),depth=calibration(di),
            rgb_encoding=rgb.encoding,depth_encoding=depth.encoding,rgb_step=rgb.step,depth_step=depth.step,
            depth_to_rgb=dict(parent=tf.header.frame_id,child=tf.child_frame_id,xyz=[t.x,t.y,t.z],xyzw=[q.x,q.y,q.z,q.w]),
            depth_to_base=dict(xyz=[getattr(extrinsic.translation,k) for k in 'xyz'],xyzw=[getattr(extrinsic.rotation,k) for k in 'xyzw']),
            direct_index_contract='PASS for these exact paired messages',rgb_sha256=hashlib.sha256(rgb_bytes).hexdigest(),depth_sha256=hashlib.sha256(depth_bytes).hexdigest())
        (folder/'frame.json').write_text(json.dumps(record,indent=2)+'\n');report['views'][name]=record
        with suppress(OSError):print('CAPTURED '+name+' registered_pairs='+str(accepted),flush=True)
    try:
        start('xvfb',['Xvfb',':109','-screen','0','1024x768x24','-nolisten','tcp']);time.sleep(1)
        if a.prepared:
            try:start('collision_probe',[sys.executable,'-m','tools.hm3d.contact_probe',str(out)])
            except OSError as error:warn('collision_probe_spawn',error)
        start('gazebo',['gz','sim','-r','-s',str(out/'scene.sdf')])
        start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(out/'bridge.yaml')])
        for i,t in enumerate(config['camera_transforms']):start('tf-'+str(i),['ros2','run','tf2_ros','static_transform_publisher',*tf_arguments(t)])
        deadline=time.monotonic()+120
        while not all(queues.values()):
            pump()
            if time.monotonic()>deadline:raise TimeoutError('No RGB/depth/CameraInfo: '+str(counts))
        if a.interactive:
            import select
            end=time.monotonic()+a.seconds
            print('Type visible / partial / hidden / revealed to switch and save; q to quit.',flush=True)
            while time.monotonic()<end:
                pump()
                if select.select([sys.stdin],[],[],0)[0]:
                    command=sys.stdin.readline().strip()
                    if command in ('q',''):break
                    if command in ('visible','partial','hidden','revealed'):collect(command)
        else:
            for name in ('visible','partial','hidden','revealed'):collect(name)
        for key,command in (('topics',['ros2','topic','list','--no-daemon','--spin-time','2']),
                            ('gazebo_version',['gz','sim','--version'])):
            try:report[key]=subprocess.run(command,capture_output=True,text=True,timeout=10,check=True).stdout
            except (OSError,subprocess.SubprocessError) as error:warn(key,error)
        report['ros_distro']=os.environ.get('ROS_DISTRO');report['status']='CAPTURED'
    except Exception as e:report.update(status='FAIL',error=str(e));raise
    finally:
        report['received_counts']=counts;report['cleanup']=terminate([child for _,child in children])
        node.destroy_node();rclpy.shutdown()
        (out/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
