"""Fixed measured RGB-D/pose replay into native diagnostic map; no flight.

Flight captures use saved estimated poses. Static captures use their recorded
manual camera poses and must never be reported as autonomous motion.
"""
import argparse,hashlib,json,subprocess,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from tools.racer_ros2.configuration import configuration


def frames(capture):
    from annual_runtime.depth_projection import project_depth,SENSOR_TRANSLATION
    capture=Path(capture)
    recorded=capture/'native-racer/measured-input.jsonl.gz'
    if recorded.is_file():
        import gzip
        k=json.loads((capture/'native-racer/protocol.json').read_text())['inputs']['intrinsics']
        with gzip.open(recorded,'rt') as stream:
            for i,line in enumerate(stream):
                row=json.loads(line)
                if row['type']=='frame':yield dict(file=str(recorded)+':'+str(i),origin=row['origin'],points=row['points'],k=k,stamp=row['time'])
        return
    files=sorted(capture.glob('rgbd-*.npz'))
    if files:
        for file in files:
            d=np.load(file);r=Rotation.from_quat(d['base_quaternion']);origin=d['base_position']+r.apply(SENSOR_TRANSLATION)
            points,hits=project_depth(d['depth'],d['k']);points[~hits]*=4.51/4.5
            yield dict(file=str(file),origin=origin.tolist(),points=(r.apply(points)+origin).tolist(),k=d['k'].tolist())
    else:
        for name in ['visible','partial','hidden','revealed']:
            folder=capture/name;f=json.loads((folder/'frame.json').read_text());pose=f['switch']['pose']
            # Camera pose was deliberately fixed by the sensor-only runner.
            r=Rotation.from_euler('xyz',pose[3:]);base=np.array(pose[:3])+[0,0,.24]
            origin=base+r.apply(SENSOR_TRANSLATION);points,hits=project_depth(np.load(folder/'depth.npy'),f['depth']['k']);points[~hits]*=4.51/4.5
            yield dict(file=str(folder/'depth.npy'),origin=origin.tolist(),points=(r.apply(points)+origin).tolist(),k=f['depth']['k'])


def wait_until(ready, spin, seconds, reason):
    deadline=time.monotonic()+seconds
    while not ready():
        remaining=deadline-time.monotonic()
        if remaining<=0:raise TimeoutError(reason)
        spin(min(.05,remaining))


def main():
    p=argparse.ArgumentParser();p.add_argument('capture');p.add_argument('output');p.add_argument('--resolution',type=float,default=.3);p.add_argument('--vertical',action='store_true');p.add_argument('--until',type=float,default=float('inf'));p.add_argument('--incremental',action='store_true');p.add_argument('--repeats',type=int,default=5);a=p.parse_args()
    out=Path(a.output);out.mkdir(exist_ok=False);capture=Path(a.capture)
    cfg=json.loads((capture/'runtime.json').read_text());samples=[s for s in frames(capture) if s.get('stamp',0)<=a.until]
    stream=out/'measured-input.jsonl';stream.write_text(''.join(json.dumps(f,separators=(',',':'))+'\n' for f in samples))
    hello=dict(bounds=cfg['bounds'],starts=cfg['fleet_starts'],intrinsics=samples[0]['k'],flight_limits=cfg['racer_flight_limits'],
        motion_limits=cfg['motion_limits'],planning_limits=cfg['racer_planning_limits'],planning_scale=.5,min_time=False,voxel_resolution=a.resolution,body_clearance=.6,vertical_samples=a.vertical)
    import rclpy
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from builtin_interfaces.msg import Time
    from ament_index_python.packages import get_package_share_directory,get_package_prefix
    from tools.racer_ros2.transport import NativeDDSTransport
    rclpy.init();transport=NativeDDSTransport(out/'transport',1)
    clock=transport.create_publisher(Clock,'/clock',10);request=transport.create_publisher(String,'/diagnostic/snapshot',10);done=[]
    transport.create_subscription(String,'/diagnostic/done',done.append,10)
    config=next(c for c in configuration(hello,out,Path(get_package_share_directory('racer_core'))/'config/single_drone_planner.xml') if c['executable']=='exploration_node')
    config['parameters']['diagnostic.output']=str(out);param=out/'params.yaml';param.write_text(yaml.safe_dump({'hm3d_map_probe':{'ros__parameters':config['parameters']}}))
    binary=Path(get_package_prefix('racer_core'))/'lib/racer_core/hm3d_map_probe'
    args=[str(binary),'--ros-args','--params-file',str(param)]
    for src,dst in config['remappings']:args+=['-r',src+':='+dst]
    log=(out/'native.log').open('w');child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(transport,timeout_sec=.02)
            if child.poll() is not None:raise RuntimeError('Native diagnostic exited')
    try:
        cloud=transport.publishers_by_kind['cloud',0]
        wait_until(lambda: request.get_subscription_count()>0 and cloud.get_subscription_count()>0,
                   spin,20,'DDS diagnostic discovery')
        def update_frontiers(kind):
            before=len(done);request.publish(String(data=kind))
            wait_until(lambda: len(done)>before,spin,30,'Native frontier update')
        t=1.
        for sample in samples:
            for rep in range(a.repeats):
                t+=.2;ns=round(t*1e9);clock.publish(Clock(clock=Time(sec=ns//10**9,nanosec=ns%10**9)));spin(.04)
                transport.publish_row(dict(type='frame',drone=0,time=t,origin=sample['origin'],points=sample['points']));spin(.08)
                if a.incremental:update_frontiers('update')
        for _ in range(10):
            t+=.1;ns=round(t*1e9);clock.publish(Clock(clock=Time(sec=ns//10**9,nanosec=ns%10**9)));spin(.05)
        update_frontiers('snapshot')
        (out/'protocol.json').write_text(json.dumps(dict(scope='Fixed-input diagnostic only; no trajectory/executor/PX4. Recorded native stream when available, otherwise sparse RGB-D reconstruction; simulated replay scheduling is not original online timing',capture=str(capture),frames=len(samples),repeats=a.repeats,incremental=a.incremental,hello=hello,input_sha256=hashlib.sha256(stream.read_bytes()).hexdigest(),native_sha256=hashlib.sha256(binary.read_bytes()).hexdigest()),indent=2))
    finally:
        child.terminate()
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        log.close();transport.close();transport.destroy_node();rclpy.shutdown()
    print((out/'native-summary.json').read_text())

if __name__=='__main__':main()
