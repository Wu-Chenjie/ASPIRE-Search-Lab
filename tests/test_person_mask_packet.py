from base64 import b64decode
from types import SimpleNamespace

import cv2
import numpy as np

from person_search.segmentation import PersonMask, person_mask_packet


def test_original_pixel_masks_preserve_overlap_and_empty_captures():
    rgb = SimpleNamespace(header=SimpleNamespace(frame_id='rgb_optical'), width=12, height=8)
    left = np.zeros((8, 12), dtype=bool)
    left[2:7, 1:6] = True
    right = np.roll(left, 3, axis=1)
    detections = [PersonMask((1, 2, 5, 5), .9, left), PersonMask((4, 2, 5, 5), .8, right)]
    packet = person_mask_packet(12345, rgb, detections, [(1, 23), (0, 42)])
    assert (packet['stamp_ns'], packet['frame_id'], packet['width'], packet['height']) == (12345, 'rgb_optical', 12, 8)
    for row, index, track in zip(packet['instances'], [1, 0], [23, 42], strict=True):
        assert (row['instance_id'], row['track_id'], row['confidence']) == (index, track, detections[index].confidence)
        im = cv2.imdecode(np.frombuffer(b64decode(row['mask_png_base64']), np.uint8), cv2.IMREAD_UNCHANGED)
        np.testing.assert_array_equal(im, detections[index].mask.astype(np.uint8) * 255)
    assert person_mask_packet(12346, rgb, [], [])['instances'] == []
