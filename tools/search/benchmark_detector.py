"""Bounded local image/inference timing; this does not measure ROS/flight latency."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
from person_search.detector_spec import DetectorSpec
from person_search.yolo import YoloPersonDetector

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',required=True,type=Path)
    parser.add_argument('--weights',required=True,type=Path)
    parser.add_argument('--images',required=True,type=Path)
    parser.add_argument('--max-frames',type=int,default=100)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if not 1<=args.max_frames<=1000:parser.error('max-frames must be 1..1000')
    if args.output.exists():parser.error('output already exists')
    paths=sorted(p for p in args.images.iterdir() if p.suffix.lower() in ('.jpg','.jpeg','.png'))[:args.max_frames]
    if not paths:parser.error('No local images')
    model=YoloPersonDetector(args.weights,spec=DetectorSpec.load(args.manifest))
    from PIL import Image
    frames=[]
    for path in paths:
        started=time.perf_counter()
        with Image.open(path) as im:rgb=np.asarray(im.convert('RGB'))
        loaded=time.perf_counter();detections=model.detect(rgb)
        if model.spec.device!='cpu':model._torch.cuda.synchronize(model.spec.device)
        done=time.perf_counter()
        frames.append(dict(image=path.name,decode_s=loaded-started,inference_s=done-loaded,total_s=done-started,instances=len(detections)))
    report=dict(scope='local_image_decode_and_single_worker_inference_only',batch=1,
                runtime=model.runtime_diagnostics(),frames=frames,
                inference_quantiles_s=dict(zip(('p50','p95','p99'),np.quantile([f['inference_s'] for f in frames],[.5,.95,.99]).tolist())))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2)

if __name__=='__main__':main()
