"""RF-DETR-Seg Nano person masks in the original RGB camera coordinates."""
from hashlib import sha256
from importlib.metadata import version
from pathlib import Path

import numpy as np

from .segmentation import PersonMask


CHECKPOINT_SHA256 = 'a44613a4ecd6b5ba61a62002c600b0b6cb7a9da2936a45317ec4b62c635fb99b'
# Intake threshold selected on the 625 calibration frames of the Depot 07 capture.
PERSON_CONFIDENCE = 0.2918183505535126


class RFDETRPersonDetector:
    def __init__(self, weights):
        import torch
        from rfdetr import ClassThresholds, RFDETRSegNano

        path = Path(weights).expanduser().resolve(strict=True)
        if sha256(path.read_bytes()).hexdigest() != CHECKPOINT_SHA256:
            raise ValueError('Expected the RF-DETR-Seg Nano COCO checkpoint')
        torch.set_num_threads(2)
        self.thresholds = ClassThresholds({'person': PERSON_CONFIDENCE}, default=float('inf'))
        model = RFDETRSegNano(pretrain_weights=str(path), device='cuda')
        self.predictor = model.inference(compile=True, compile_backend='cudagraph',
                                         batch_size=1, dtype=torch.float32)
        self.detect(np.zeros((240, 320, 3), np.uint8))
        torch.cuda.synchronize()
        self.device_name = torch.cuda.get_device_name()

    def runtime_diagnostics(self):
        return dict(detector='rfdetr-seg-nano', device=self.device_name, precision='fp32',
                    compile_backend='cudagraph', batch_size=1, confidence=PERSON_CONFIDENCE,
                    rfdetr_version=version('rfdetr'), warmup_image_discarded=True)

    def detect(self, rgb):
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('Expected an RGB uint8 image')
        result = self.predictor.predict(rgb, threshold=self.thresholds, include_source_image=False)
        if not len(result):
            return []
        if result.mask is None or result.mask.shape != (len(result), *rgb.shape[:2]):
            raise ValueError('RF-DETR masks must match the original RGB image')
        people = []
        for box, confidence, label, mask in zip(result.xyxy, result.confidence,
                                                result.data['class_name'], result.mask):
            if label == 'person' and mask.any():
                x1, y1, x2, y2 = map(float, box)
                people.append(PersonMask((x1, y1, x2 - x1, y2 - y1), float(confidence),
                                         mask.astype(bool, copy=False)))
        return people
