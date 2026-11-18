"""Run the recorder callbacks with deterministic ROS messages and a real codec."""
import json
import sys
from types import SimpleNamespace as NS

import pytest

from tools.flight import video_recorder


@pytest.fixture
def recorder(tmp_path, monkeypatch):
    subscriptions, messages = {}, []
    node = NS(set_parameters=lambda _:None, destroy_node=lambda:None,
              get_clock=lambda:NS(now=lambda:NS(nanoseconds=42)))
    node.create_subscription = lambda _type,topic,callback,_qos: subscriptions.update({topic:callback})

    def spin(_node, timeout_sec):
        topic, msg = messages.pop(0)
        subscriptions[topic](msg)

    for module, attrs in {
        'rclpy': dict(init=lambda:None, shutdown=lambda:None, create_node=lambda _:node,
                      spin_once=spin, ok=lambda:bool(messages), parameter=NS(Parameter=lambda *a,**k:None)),
        'rclpy.qos': dict(qos_profile_sensor_data=0),
        'sensor_msgs.msg': dict(Image=object),
        'std_msgs.msg': dict(String=object),
    }.items():
        monkeypatch.setitem(sys.modules,module,NS(**attrs))
    monkeypatch.setattr(video_recorder,'signal',NS(SIGINT=2,SIGTERM=15,signal=lambda *_:None))
    monkeypatch.setattr(sys,'argv',['video_recorder',str(tmp_path)])
    return messages


def rgb(stamp, encoding='rgb8', step=50, data=None, width=16):
    return ('/drone_0/rgb/image', NS(header=NS(stamp=NS(sec=1,nanosec=stamp)), encoding=encoding,
        step=step, width=width, height=12, data=bytes(12*step) if data is None else data))


@pytest.mark.parametrize('fps', [.5, 240.])
def test_rejects_only_bad_messages_and_encodes_supported_time_base(tmp_path, monkeypatch, recorder, fps):
    cv2 = pytest.importorskip('cv2')
    probe = cv2.VideoWriter(str(tmp_path/'probe.avi'),cv2.VideoWriter_fourcc(*'MJPG'),fps,(16,12))
    if not probe.isOpened():
        pytest.skip(f'MJPEG backend cannot encode {fps:g} FPS on this host')
    probe.release()
    monkeypatch.setenv('RACER_VIDEO_FPS',str(fps))
    overlay = '/search/semantic_candidates'
    recorder[:] = [rgb(3), (overlay,NS(data='{')), rgb(4,encoding='mono8'), rgb(4,step=5),
                   rgb(4,data=b''), rgb(4,width=10), (overlay,NS(data='[]')),
                   (overlay,NS(data='{"candidates":[]}')), rgb(3), rgb(4,'bgr8')]
    video_recorder.main()
    meta=json.loads((tmp_path/'continuous-rgb-metadata.json').read_text())
    assert meta['complete'] and meta['frames']==2
    assert meta['encoded_fps']==fps and meta['rejected_non_increasing']==1
    assert meta['rejected_invalid_images']==4 and meta['rejected_invalid_overlays']==2
    assert len((tmp_path/'continuous-overlays.jsonl').read_text().splitlines())==1
    stamps=[json.loads(line) for line in (tmp_path/'continuous-rgb-stamps.jsonl').read_text().splitlines()]
    assert [s['stamp_ns'] for s in stamps]==[1_000_000_003,1_000_000_004]
    cap=cv2.VideoCapture(str(tmp_path/'continuous-rgb.avi'))
    try:
        assert cap.isOpened() and cap.get(cv2.CAP_PROP_FPS)==pytest.approx(fps)
        assert cap.read()[0] and cap.read()[0] and not cap.read()[0]
    finally:
        cap.release()


@pytest.mark.parametrize('write_failure', [False, True])
def test_encoder_failure_ends_recording(tmp_path, monkeypatch, recorder, write_failure):
    cv2 = pytest.importorskip('cv2')

    def fail_write(_pixels):
        raise OSError('AVI write failed')

    monkeypatch.setattr(cv2,'VideoWriter',lambda *a:NS(
        isOpened=lambda:write_failure, write=fail_write, release=lambda:None))
    recorder.append(rgb(1))
    error = OSError if write_failure else RuntimeError
    with pytest.raises(error,match='AVI write failed|MJPEG recorder unavailable'):
        video_recorder.main()
    meta=json.loads((tmp_path/'continuous-rgb-metadata.json').read_text())
    assert not meta['complete'] and meta['frames']==0 and meta['error'].startswith(error.__name__)
