"""Read-only RGB recorder: one encoded frame per increasing source timestamp.

RACER_VIDEO_FPS controls the AVI time base, not the sensor. JSONL source/receipt
stamps are authoritative. No interpolation, repeated output frames or controls.
"""
import hashlib
import json
import math
import os
import signal
import sys
import time
from pathlib import Path


def main():
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    out = Path(sys.argv[1])
    fps = float(os.environ.get('RACER_VIDEO_FPS', '15'))
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError('RACER_VIDEO_FPS must be finite and positive')
    rclpy.init()
    node = rclpy.create_node('flight_continuous_rgb_recorder')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    writer = None
    count = rejected = invalid_images = invalid_overlays = 0
    last_stamp = -1
    stopped = False
    stamps = (out / 'continuous-rgb-stamps.jsonl').open('x', buffering=1)
    overlays = (out / 'continuous-overlays.jsonl').open('x', buffering=1)
    rejects = (out / 'continuous-rgb-rejected.jsonl').open('x', buffering=1)
    metadata = dict(encoded_fps=fps, capture_topic='/drone_0/rgb/image',
                    frame_policy='one encoded frame per increasing source stamp; no interpolation',
                    started_monotonic_ns=time.monotonic_ns(), complete=False)
    meta_path = out / 'continuous-rgb-metadata.json'
    meta_path.write_text(json.dumps(metadata, indent=2))

    def reject_frame(ns, receipt, reason):
        rejects.write(json.dumps(dict(stamp_ns=ns, last_stamp_ns=last_stamp,
                                      receipt_monotonic_ns=receipt, reason=reason))+'\n')

    def receive(msg):
        nonlocal writer, count, last_stamp, rejected, invalid_images
        receipt = time.monotonic_ns()
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if ns <= last_stamp:
            reject_frame(ns, receipt, 'non_increasing_source_stamp')
            rejected += 1
            return
        try:
            if msg.encoding not in ('rgb8', 'bgr8'):
                raise ValueError('unsupported_image_encoding')
            raw = bytes(msg.data)
            if min(msg.height, msg.width) <= 0 or msg.step < msg.width*3 or len(raw) < msg.step*msg.height:
                raise ValueError('invalid_image_layout')
            size = msg.width, msg.height
            if writer is not None and size != (metadata['width'], metadata['height']):
                raise ValueError('image_dimensions_changed')
            rgb = np.ndarray((msg.height, msg.width, 3), np.uint8, raw, strides=(msg.step, 3, 1))
        except (ValueError, TypeError, OverflowError) as error:
            reject_frame(ns, receipt, str(error))
            invalid_images += 1
            return
        if writer is None:
            writer = cv2.VideoWriter(str(out / 'continuous-rgb.avi'),
                                     cv2.VideoWriter_fourcc(*'MJPG'), fps, size)
            if not writer.isOpened():
                raise RuntimeError('MJPEG recorder unavailable')
            metadata.update(width=msg.width, height=msg.height, encoding=msg.encoding)
        writer.write(rgb[:, :, ::-1] if msg.encoding == 'rgb8' else rgb)
        stamps.write(json.dumps(dict(frame=count, stamp_ns=ns,
                                     receipt_monotonic_ns=receipt,
                                     encode_done_monotonic_ns=time.monotonic_ns(),
                                     receipt_sim_ns=node.get_clock().now().nanoseconds,
                                     source_pixels_sha256=hashlib.sha256(rgb.tobytes()).hexdigest()))+'\n')
        last_stamp = ns
        count += 1

    def overlay(kind, msg):
        nonlocal invalid_overlays
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            data = None
        if not isinstance(data, dict):
            invalid_overlays += 1
            return
        overlays.write(json.dumps(dict(kind=kind,
            receipt_monotonic_ns=time.monotonic_ns(),
            receipt_sim_ns=node.get_clock().now().nanoseconds, data=data))+'\n')

    def stop(*_):
        nonlocal stopped
        stopped = True

    node.create_subscription(Image, '/drone_0/rgb/image', receive, qos_profile_sensor_data)
    for kind, topic in [('semantic', '/search/semantic_candidates'),
                        ('tracks', '/search/tracks'), ('flight', '/drone_0/flight_status'),
                        ('execution', '/drone_0/view_execution'),
                        ('active_confirmation', '/search/active_confirmation')]:
        node.create_subscription(String, topic, lambda m, k=kind: overlay(k, m), 10)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    clean_shutdown = False
    try:
        while not stopped and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
        clean_shutdown = True
    except Exception as exc:
        metadata['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        if writer is not None:
            writer.release()
        stamps.close(); overlays.close(); rejects.close()
        metadata.update(frames=count, rejected_non_increasing=rejected,
                        rejected_invalid_images=invalid_images, rejected_invalid_overlays=invalid_overlays,
                        stopped_monotonic_ns=time.monotonic_ns(), complete=clean_shutdown)
        meta_path.write_text(json.dumps(metadata, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
