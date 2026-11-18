"""The ideal camera can only advance its recorded pose on a service acknowledgment."""
import json
import os
import sys
from types import SimpleNamespace as NS

import cv2
import pytest

from tools.search import run


def test_pose_timeout_and_negative_reply_allow_an_acknowledged_retry(monkeypatch):
    replies = iter([
        run.subprocess.TimeoutExpired('gz', 5),
        NS(returncode=0, stdout='data: false\n'),
        NS(returncode=1, stdout='data: true\n'),
        NS(returncode=0, stdout='Response: \n  data: true\n'),
    ])
    commands = []

    def send(command, **kwargs):
        commands.append(command)
        reply = next(replies)
        if isinstance(reply, Exception):
            raise reply
        return reply

    monkeypatch.setattr(run.subprocess, 'run', send)
    pose = (1., 2., 3.)
    assert 'timed out' in run.request_ideal_pose(pose, .5)
    assert 'not acknowledged' in run.request_ideal_pose(pose, .5)
    assert 'not acknowledged' in run.request_ideal_pose(pose, .5)
    assert run.request_ideal_pose(pose, .5) is None
    assert len(commands) == 4 and all(command == commands[0] for command in commands)


@pytest.mark.parametrize('seconds', [0., 5.])
def test_ideal_search_entry_waits_for_same_pose_ack_before_advancing(tmp_path, monkeypatch, seconds):
    native = tmp_path / 'native'
    binary = native / 'install/racer_core/lib/racer_core/search_racer'
    binary.parent.mkdir(parents=True)
    binary.touch()
    (native / 'task-overlay.json').write_text('{}')
    output = tmp_path / 'output'
    monkeypatch.setattr(run, 'ROOT', tmp_path)
    monkeypatch.setattr(sys, 'argv', ['run.py', '--native', str(native), '--output', str(output),
                                    '--seconds', str(seconds)])
    monkeypatch.setattr(os, 'environ', {})
    monkeypatch.setattr(cv2, 'imwrite', lambda *_: True)

    def prepare(map_file, out, rgbd_test_scene):
        (out / 'scene.sdf').write_text('<sdf><world><include><name>annual_x500_0</name>'
                                      '<pose>0 0 0 0 0 0</pose></include></world></sdf>')
        (out / 'runtime.json').write_text(json.dumps(dict(models=str(out), additional_model_paths=[],
                                                        camera_transforms=[])))

    monkeypatch.setattr(run, 'prepare', prepare)
    monkeypatch.setattr(run, 'terminate', lambda _: {})
    monkeypatch.setattr(run, 'configuration', lambda *_: [])
    monkeypatch.setattr(run, 'PeopleDetector', object)
    monkeypatch.setattr(run, 'candidates_from_cloud', lambda *_: [])

    clock = NS(wall=0., ticks=0)
    subscriptions = {}
    cloud_publish_times = []
    pose_requests = []

    def sleep(seconds):
        clock.wall += seconds

    monkeypatch.setattr(run, 'time', NS(monotonic=lambda: clock.wall, sleep=sleep))

    def send(command, **kwargs):
        pose_requests.append((command, len(cloud_publish_times)))
        if len(pose_requests) == 1:
            sleep(.9)
            raise run.subprocess.TimeoutExpired('gz', 5)
        return NS(returncode=0, stdout='data: true\n')

    monkeypatch.setattr(run.subprocess, 'run', send)
    monkeypatch.setattr(run.subprocess, 'Popen', lambda *a, **k: NS(poll=lambda: None))

    def check_pair(rgb, depth, ri, di, tf):
        if rgb.header.stamp.sec == 1:
            raise ValueError('mismatched RGB-D pair')

    monkeypatch.setattr(run, 'require_registered_pair', check_pair)

    def header():
        return NS(frame_id='rgb', stamp=NS(sec=clock.ticks, nanosec=0))

    def pose():
        return NS(position=NS(x=0., y=0., z=0.), orientation=NS(w=1., x=0., y=0., z=0.))

    def subscribe(_type, topic, callback, _qos):
        subscriptions[topic] = callback

    def publisher(_type, topic, _qos):
        publish = cloud_publish_times.append if topic == '/baseline/cloud_0' else lambda _: None
        return NS(publish=publish, get_subscription_count=lambda: 0)

    def spin(node, timeout_sec):
        sleep(.2)
        clock.ticks += 1
        for topic, encoding, data in [('rgb', 'rgb8', b'\0'*3), ('depth', '32FC1', b'\0'*4)]:
            subscriptions[f'/drone_0/{topic}/image'](NS(header=header(), height=1, width=1,
                step=len(data), encoding=encoding, data=data, is_bigendian=False))
            subscriptions[f'/drone_0/{topic}/camera_info'](NS(header=header(), k=[1.,0.,0.,0.,1.,0.,0.,0.,1.]))
        if clock.ticks == 2:
            subscriptions['/search/trajectory'](NS(traj_id=1, order=3, yaw_dt=1/3, yaw_pts=[0.]*4,
                pos_pts=[NS(x=x, y=-.9, z=2.2) for x in (0., 1/3, 2/3, 1.)],
                knots=[0.]*4+[1.]*4))

    def install(name, **attrs):
        monkeypatch.setitem(sys.modules, name, NS(**attrs))

    node=NS(create_subscription=subscribe, create_publisher=publisher, set_parameters=lambda _: None,
            destroy_node=lambda: None)
    install('rclpy', init=lambda: None, create_node=lambda _: node, spin_once=spin,
            ok=lambda: True, shutdown=lambda: None, parameter=NS(Parameter=lambda *a, **k: None))
    install('rclpy.qos', qos_profile_sensor_data=0)
    install('rclpy.time', Time=NS)
    install('sensor_msgs.msg', Image=object, CameraInfo=object,
            PointCloud2=lambda: NS(header=header()), PointField=NS)
    install('geometry_msgs.msg', PoseStamped=lambda: NS(header=header(), pose=pose()))
    install('nav_msgs.msg', Odometry=lambda: NS(pose=NS(pose=pose())))
    install('std_msgs.msg', String=object)
    install('visualization_msgs.msg', Marker=object, MarkerArray=lambda: NS(markers=[]))
    install('racer_interfaces.msg', Bspline=object)
    install('tf2_ros', Buffer=lambda: NS(lookup_transform=lambda *_: None),
            TransformListener=lambda *_: None, TransformException=RuntimeError)

    run.main()

    report = json.loads((output / 'result.json').read_text())
    assert report['status'] == 'COMPLETED_WINDOW'
    if seconds == 0.:
        assert not pose_requests and clock.ticks == 0 and report['path_m'] == 0.
        return

    assert report['rejected_pairs'] == 1 and report['accepted_curves'] == 1
    assert len(pose_requests) >= 3 and pose_requests[0] == pose_requests[1]
    assert pose_requests[2][0] != pose_requests[1][0]
    assert pose_requests[0][1] == 1
    assert report['path_m'] == pytest.approx(1.)
    events = json.loads((output / 'events.json').read_text())
    order = [event['event'] for event in events]
    assert order.index('pose_unacknowledged') < order.index('arrived')
    assert next(event['origin'] for event in events if event['event']=='arrived') == [1., -.9, 2.2]
