"""Exercise the static camera entry point without starting sensor processes."""
import json
import os
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest

from tools.rgbd_scene import run


@pytest.mark.parametrize('final_pair', [True, False])
def test_static_capture_recovers_after_bad_pair_and_late_tf(tmp_path, monkeypatch, final_pair):
    subscriptions = {}
    state = NS(wall=0., slot=None, stamp=0, tf_retries=0, stopped=False)

    def prepare(_map, out, **_options):
        (out/'runtime.json').write_text(json.dumps(dict(
            manual_camera_test=True, models=str(out/'models'),
            additional_model_paths=[], camera_transforms=[])))

    def switch(_out, name):
        state.slot = 0
        return name

    def spin(_node, timeout_sec):
        slot = state.slot
        state.wall += .02 if slot == 30 else 44.99/30
        if slot is not None:
            state.slot += 1
        if slot == 30 and not final_pair:
            return
        state.stamp += 1
        for camera, encoding, step in [('rgb','rgb8',3), ('depth','32FC1',4)]:
            header = NS(frame_id=camera, stamp=NS(sec=state.stamp,nanosec=0))
            image = NS(header=header,width=1,height=1,encoding=encoding,
                       step=step,data=bytes(step),is_bigendian=False)
            calibration = NS(header=header,width=1,height=1,k=[1.]*9,r=[],p=[],d=[],
                             distortion_model='plumb_bob',binning_x=0,binning_y=0,
                             roi=NS(x_offset=0,y_offset=0,width=0,height=0,do_rectify=False))
            if slot == 0 and camera == 'depth':
                calibration.k[0] = 2.
            subscriptions[f'/drone_0/{camera}/image'](image)
            subscriptions[f'/drone_0/{camera}/camera_info'](calibration)

    class TransformException(Exception):
        pass

    def lookup(target, source, stamp):
        if state.slot == 2 and stamp == state.stamp * 1_000_000_000:
            state.tf_retries += 1
            raise TransformException
        sec,nanosec=divmod(stamp,1_000_000_000)
        return NS(header=NS(frame_id=target,stamp=NS(sec=sec,nanosec=nanosec)),child_frame_id=source,
                  transform=NS(translation=NS(x=0.,y=0.,z=0.),
                               rotation=NS(x=0.,y=0.,z=0.,w=1.)))

    def shutdown():
        state.stopped = True

    def subscribe(_message, topic, callback, _qos):
        subscriptions[topic] = callback

    node = NS(create_subscription=subscribe,destroy_node=lambda:None)
    for module, attrs in {
        'rclpy': dict(init=lambda:None,create_node=lambda _:node,spin_once=spin,shutdown=shutdown),
        'rclpy.qos': dict(qos_profile_sensor_data=0),
        'rclpy.time': dict(Time=lambda *,nanoseconds:nanoseconds),
        'sensor_msgs.msg': dict(Image=object,CameraInfo=object),
        'tf2_ros': dict(Buffer=lambda:NS(lookup_transform=lookup),TransformListener=lambda *_:None,
                        TransformException=TransformException),
        'cv2': dict(imwrite=lambda *_:True),
    }.items():
        monkeypatch.setitem(sys.modules, module, NS(**attrs))
    monkeypatch.setattr(os, 'environ', {})
    monkeypatch.setattr(run,'prepare',prepare)
    monkeypatch.setattr(run,'switch',switch)
    monkeypatch.setattr(run,'terminate',lambda _:[])
    monkeypatch.setattr(run,'time',NS(monotonic=lambda:state.wall,sleep=lambda _:None))
    def metadata_command(command, **kwargs):
        if command[0] == 'ros2':
            raise subprocess.TimeoutExpired(command, kwargs['timeout'])
        return NS(stdout='Gazebo version')

    monkeypatch.setattr(run.subprocess, 'Popen', lambda *a,**k:NS(poll=lambda:None))
    monkeypatch.setattr(run.subprocess, 'run', metadata_command)
    out = tmp_path/'capture'
    monkeypatch.setattr(sys,'argv',['run','--output',str(out)])

    if final_pair:
        run.main()
    else:
        with pytest.raises(TimeoutError, match='No 30 registered pairs'):
            run.main()

    report = json.loads((out/'acceptance.json').read_text())
    assert report['alignment_failures']['visible']['count'] == 1
    assert 'calibration k differs' in report['alignment_failures']['visible']['error']
    assert state.tf_retries and state.stopped
    if final_pair:
        assert report['status'] == 'CAPTURED'
        assert 'timed out' in report['diagnostics']['topics']
        assert report['gazebo_version'] == 'Gazebo version'
        assert set(report['views']) == {'visible','partial','hidden','revealed'}
        for capture in report['views'].values():
            assert capture['pairs_checked'] == 30
            assert capture['last_stamp_ns'] - capture['first_stamp_ns'] == 29_000_000_000
    else:
        assert report['status'] == 'FAIL' and not report['views']
