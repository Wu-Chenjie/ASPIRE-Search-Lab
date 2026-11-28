"""ROS component evidence test only. No PX4, Gazebo, new scene or flight."""
import json,os,time
from pathlib import Path
from types import SimpleNamespace as N
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from sensor_msgs.msg import Image,CameraInfo
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy
from std_msgs.msg import Header
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from annual_runtime.pointcloud_mapping_node import PointCloudMapper
from core.exploration.voxel_mapping import VoxelMap
from annual_runtime.trigger_evidence import EvidenceRing
from tools.flight.map_display import MapDisplay

def test_real_ros_trigger_and_raw_mapper_capture_exactly_replay(tmp_path,monkeypatch):
    monkeypatch.setenv('RACER_MAP_EVIDENCE_DIR',str(tmp_path/'evidence'))
    monkeypatch.setenv('RACER_OBSERVATION_SESSION','stage36-component')
    bounds=[[-3,-4,0],[7,4,4]];param=tmp_path/'params.json'
    param.write_text(json.dumps(dict(pointcloud_mapper=dict(ros__parameters=dict(bounds=json.dumps(bounds),publish_native_frames=True,projection_stride=4)))))
    rclpy.init(args=['--ros-args','--params-file',str(param)],domain_id=186)
    node=PointCloudMapper();peer=rclpy.create_node('capture_trigger_fixture');sink=node.map_evidence
    pub=peer.create_publisher(String,'/drone_0/diagnostics/map_evidence_trigger',10)
    fixture=os.environ.get('RACER_EVIDENCE_REPLAY_FIXTURE')
    if fixture:
        a=np.load(fixture);depth=a['depth'].astype(np.float32);k=a['k'].reshape(-1).tolist();position=a['base_position'];quat=a['base_quaternion'];stamp=float(a['stamp_ns'])*1e-9
    else:
        depth=np.full((240,320),2.8,np.float32);k=[216.,0,160.,0,216.,120.,0,0,1.];position=np.array([0.,-2.,1.5]);quat=np.array([0.,0.,0.,1.]);stamp=10.
        depth[:, :32] = 6.
        depth[:8, :8] = np.nan
    info=CameraInfo();info.width=depth.shape[1];info.height=depth.shape[0];info.k=k;info.header.frame_id='depth_optical';node.camera_info(info)
    def image(t):
        m=Image();ns=round(t*1e9);m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9;m.header.frame_id='depth_optical';m.width=depth.shape[1];m.height=depth.shape[0];m.encoding='32FC1';m.step=m.width*4;m.data=depth.tobytes();return m
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01);rclpy.spin_once(peer,timeout_sec=.01)
    try:
        node.poses.add(stamp-.01,position,quat);node.poses.add(stamp+.01,position,quat)
        node.integrate(image(stamp),stamp,(position,Rotation.from_quat(quat),(stamp-.01,stamp+.01)))
        end=time.monotonic()+4
        while not pub.get_subscription_count():spin(.05);assert time.monotonic()<end
        pub.publish(String(data=json.dumps(dict(schema=1,kind='map_evidence_trigger',id='component-one',drone=0,session=sink.session,sim_s=stamp,reason='component_test'))))
        while not sink.windows:spin(.05);assert time.monotonic()<end
        for t in [stamp+.4,stamp+.8]:node.integrate(image(t),t,(position,Rotation.from_quat(quat),(t-.01,t+.01)))
    finally:
        node.destroy_node();peer.destroy_node();rclpy.shutdown()
    status=json.loads((sink.folder/'status.json').read_text());assert status['write_errors']==status['queue_dropped']==0
    assert status['windows'][0]['post_captured']==2 and status['written']==3
    with np.load(sorted(sink.folder.glob('*.npz'))[0]) as capture:
        meta=json.loads(bytes(capture['metadata_json']).decode())['record'];assert meta['pose_endpoint_samples_complete']
        np.testing.assert_allclose(capture['pose_endpoint_positions'],[position,position]);assert meta['receipt'] is None
        m=image(meta['sensor_stamp_s']);m.data=capture['raw_depth'].tobytes()
        info.k=capture['k'].tolist();state=capture['mapper_before'];replay=VoxelMap(bounds);replay.state=state.copy();replay.shape=state.shape;replay.version=meta['mapper_version_before']
        packets=[]; surfaces=[]
        fake=N(id=0,session=meta['sensor_session'],calibration=info,map=replay,projection_stride=meta['projection_stride'],
               belief=capture['belief_before'].copy(),ego_cells=set(map(tuple,capture['ego_cells_before'])),
               full_requested=False,publish_native_frames=True,sequence=meta['sequence_before'],last=-10.,
               map_evidence=None,map_timing=None,publish_native_dds=False,
               surface_pub=N(get_subscription_count=lambda:1, publish=surfaces.append),
               pub=N(publish=lambda msg:packets.append(json.loads(msg.data))))
        PointCloudMapper.integrate(fake,m,meta['sensor_stamp_s'],(capture['estimated_position'],Rotation.from_quat(capture['estimated_quaternion']),meta['pose_bracket']))
        np.testing.assert_array_equal(fake.map.state,capture['mapper_after']);np.testing.assert_array_equal(fake.belief,capture['belief_after'])
        np.testing.assert_array_equal(np.array(packets[0]['indices']),capture['packet_indices']);np.testing.assert_array_equal(np.array(packets[0]['values']),capture['packet_values'])
        # The direct DDS path must carry the same measured geometry and clock.
        legacy=packets[0]['native_frame'];clouds=[];poses=[]
        fake.publish_native_dds=True
        fake.native_cloud_pub=N(publish=clouds.append);fake.native_pose_pub=N(publish=poses.append)
        PointCloudMapper.integrate(fake,m,meta['sensor_stamp_s'],(capture['estimated_position'],Rotation.from_quat(capture['estimated_quaternion']),meta['pose_bracket']))
        cloud,pose=clouds[0],poses[0]
        assert cloud.header.stamp==pose.header.stamp==m.header.stamp
        assert cloud.header.frame_id==pose.header.frame_id=='world'
        assert 'native_frame' not in packets[1]
        np.testing.assert_array_equal(np.frombuffer(cloud.data,dtype='<f4').reshape(-1,3),np.asarray(legacy['points'],dtype='<f4'))
        np.testing.assert_array_equal([pose.pose.position.x,pose.pose.position.y,pose.pose.position.z],legacy['origin'])
        for measured in surfaces:
            assert measured.header.stamp == m.header.stamp and measured.header.frame_id == 'map'
            expected = capture['world_points'][capture['endpoint_hits']].astype(np.float32)
            np.testing.assert_array_equal(read_points_numpy(measured), expected)


def test_surface_display_keeps_first_measured_point_across_frames():
    published = []
    display = N(surfaces={}, surface_header=None, surface_publisher=N(publish=published.append))
    def send(sec, points):
        header = Header(frame_id='map')
        header.stamp.sec = sec
        cloud = create_cloud_xyz32(header, np.asarray(points, np.float32).reshape(-1, 3))
        MapDisplay.receive_hits(display, cloud)
    send(1, [[-.01, 0, 2], [.001, 0, 2]])
    send(2, [[.04, 0, 2], [.06, 0, 2]])
    send(3, [])
    MapDisplay.publish_surfaces(display)
    send(4, [[.07, 0, 2]])
    MapDisplay.publish_surfaces(display)
    assert len(published) == 1 and published[0].header.stamp.sec == 2
    expected = np.array([[-.01, 0, 2], [.001, 0, 2], [.06, 0, 2]], np.float32)
    np.testing.assert_array_equal(read_points_numpy(published[0]), expected)

# Reuse the real callback harness, not a running aircraft.
from test_execution_safety_ros import executor,activate,snapshot,observation,set_odometry

def test_real_safety_callback_preserves_rejection_and_captures_query(executor,tmp_path):
    node=executor.node;activate(executor);ring=EvidenceRing(tmp_path,0,'executor',post_records=4);node.map_evidence=ring
    node.map_evidence_pub=N(publish=lambda message:None)
    index=node.safety_replica.map.indices(node.ref).tolist();observation(executor,[index],sequence=99)
    node.safety_dirty=True;node.check_execution_safety(executor.now())
    assert node.safety_fault and not node.arrived
    for i in range(4):node.check_execution_safety(executor.now()+(i+1)*.02)
    assert len(ring.windows)==1 and ring.windows[0]['post_captured']==4
    ring.close();rows=sorted(ring.folder.glob('*.npz'));assert len(rows)==5
    with np.load(rows[0]) as z:
        meta=json.loads(bytes(z['metadata_json']).decode())['record'];assert meta['query_count']>0 and meta['rejected_boxes']>0 and meta['fault']==node.safety_fault
        np.testing.assert_array_equal(z['mapper_state'],node.safety_replica.map.state)
