"""Validated local instance segmentation with an explicit artifact contract. No model auto-download or cloud input."""
import importlib.metadata
import os
import time
import numpy as np
from .segmentation import PersonMask

VERSION='8.3.228'
WEIGHT_SHA256='55ed65c56c91713d23e8402371c6c49a6fd84f257f7dce452e8d70e41dcbe152'
MASK_CANDIDATE_CAP=64


class YoloPersonDetector:
    def __init__(self,weights,confidence=.35,spec=None,*,preserve_mask_candidates=False):
        from .detector_spec import DetectorSpec
        self.spec=spec or DetectorSpec(model_sha256=WEIGHT_SHA256)
        if not 0<=confidence<=1:raise ValueError('Invalid confidence threshold')
        path=self.spec.verify(weights)
        os.environ['YOLO_AUTOINSTALL']='false'
        os.environ['YOLO_OFFLINE']='true'
        os.environ.setdefault('YOLO_CONFIG_DIR','/tmp/racer-yolo-settings')
        import torch
        from ultralytics import YOLO,settings
        settings.update({'sync':False,'hub':False,'wandb':False,'comet':False,'mlflow':False,'clearml':False})
        self.spec.verify_device(torch)
        torch.set_num_threads(2)
        self.predict_options=dict(device=self.spec.device,imgsz=self.spec.input_size,half=self.spec.precision=='fp16',classes=list(self.spec.class_map),retina_masks=True,verbose=False,save=False,batch=1)
        if spec is not None:self.predict_options['rect']=False
        if preserve_mask_candidates:
            # Keep overlapping boxes until PresenceTracker can compare their masks.
            self.predict_options.update(iou=1.0,max_det=MASK_CANDIDATE_CAP)
        self.model=YOLO(str(path),task='segment')
        self.confidence=float(confidence)
        self._torch=torch
        # Lazy backend setup can reset CPU threads. Warm on a discarded
        # black image before requesting the preferred setting again.
        started = time.monotonic()
        self.model.predict(source=np.zeros((240,320,3),np.uint8),conf=self.confidence,**self.predict_options)
        if preserve_mask_candidates and self.model.predictor.model.end2end:
            raise ValueError('Mask-aware deduplication requires a detector without embedded NMS')
        names=self.model.names
        if any(names.get(k)!=v for k,v in self.spec.class_map.items()):raise ValueError('Model class map differs from DetectorSpec')
        backend_threads = torch.get_num_threads()
        torch.set_num_threads(2)
        self.thread_diagnostics = dict(backend_threads_after_warmup=backend_threads,
            requested_threads=2, warmup_wall_s=time.monotonic()-started,
            warmup_image_discarded=True)
    def runtime_diagnostics(self):
        return dict(self.thread_diagnostics,actual_threads=self._torch.get_num_threads(),detector_spec=self.spec.report(),
            box_nms_iou=self.predict_options.get('iou'),mask_candidate_cap=self.predict_options.get('max_det'))

    def detect(self,rgb):
        if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3:
            raise ValueError('Expected RGB uint8 image')
        # Ultralytics ndarray input is BGR; mask output must match original pixels.
        result=self.model.predict(source=np.ascontiguousarray(rgb[:,:,::-1]),conf=self.confidence,**self.predict_options)[0]
        if result.masks is None:return []
        masks=result.masks.data.cpu().numpy()
        if masks.ndim!=3 or masks.shape[1:]!=rgb.shape[:2]:
            raise ValueError('Segmentation output is not registered to original RGB pixels')
        boxes=result.boxes.xyxy.cpu().numpy()
        scores=result.boxes.conf.cpu().numpy()
        classes=result.boxes.cls.cpu().numpy()
        count=len(masks)
        if (boxes.shape!=(count,4) or scores.shape!=(count,) or classes.shape!=(count,)
                or not np.isfinite(boxes).all() or not ((scores>=0)&(scores<=1)).all()):
            raise ValueError('Invalid segmentation result')
        try:
            labels=[self.spec.class_map[c] for c in classes]
        except KeyError:
            raise ValueError('Unexpected output class') from None
        # Convert masks only after the metadata is known to be usable.
        return [PersonMask((float(x),float(y),float(r-x),float(b-y)),float(score),mask,class_id=int(c),semantic_class=label)
                for (x,y,r,b),score,mask,c,label in zip(boxes,scores,masks>.5,classes,labels)]
