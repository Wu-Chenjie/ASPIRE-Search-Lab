"""Bounded local image/inference timing; this does not measure ROS/flight latency."""
import argparse
import json
from dataclasses import replace
from pathlib import Path
import sys
import time
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
from person_search.detector_spec import DetectorSpec
from person_search.yolo import YoloPersonDetector

def quantiles(values):
    return dict(zip(('p50','p95','p99'),np.quantile(values,[.5,.95,.99]).tolist()))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',required=True,type=Path)
    parser.add_argument('--weights',required=True,type=Path)
    parser.add_argument('--images',required=True,type=Path)
    parser.add_argument('--max-frames',type=int,default=100)
    parser.add_argument('--repeats',type=int,default=1,help='Cycles through the images so short sets still fill the sample')
    parser.add_argument('--input-size',type=int,default=0,help='Square inference size override; 0 keeps the manifest value')
    parser.add_argument('--warmup-frames',type=int,default=2,help='Discarded inferences before timing')
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if not 1<=args.max_frames<=1000:parser.error('max-frames must be 1..1000')
    if not 1<=args.repeats<=100:parser.error('repeats must be 1..100')
    if args.input_size and (args.input_size%32 or args.input_size<32):parser.error('input-size must be a positive multiple of 32')
    if not 0<=args.warmup_frames<=100:parser.error('warmup-frames must be 0..100')
    if args.output.exists():parser.error('output already exists')
    paths=sorted(p for p in args.images.iterdir() if p.suffix.lower() in ('.jpg','.jpeg','.png'))[:args.max_frames]
    if not paths:parser.error('No local images')
    spec=DetectorSpec.load(args.manifest)
    if args.input_size:spec=replace(spec,input_size=args.input_size)
    model=YoloPersonDetector(args.weights,spec=spec)
    from PIL import Image
    loaded=[]
    for path in paths:
        started=time.perf_counter()
        with Image.open(path) as im:rgb=np.asarray(im.convert('RGB'))
        loaded.append((path.name,rgb,time.perf_counter()-started))
    gpu=model.spec.device!='cpu'
    for _,rgb,_ in (loaded*args.repeats)[:args.warmup_frames]:
        model.detect(rgb)
        if gpu:model._torch.cuda.synchronize(model.spec.device)
    frames=[]
    for name,rgb,decode_s in loaded*args.repeats:
        started=time.perf_counter();detections=model.detect(rgb)
        if gpu:model._torch.cuda.synchronize(model.spec.device)
        done=time.perf_counter()
        frames.append(dict(image=name,decode_s=decode_s,inference_s=done-started,total_s=done-started+decode_s,
                           instances=len(detections),input_pixels=[int(rgb.shape[1]),int(rgb.shape[0])],
                           mask_pixels=int(sum(int(d.mask.sum()) for d in detections))))
    elapsed=sum(f['inference_s'] for f in frames)
    report=dict(scope='local_image_decode_and_single_worker_inference_only',batch=1,
                runtime=model.runtime_diagnostics(),frames=frames,
                inference_quantiles_s=quantiles([f['inference_s'] for f in frames]),
                inference_fps=len(frames)/elapsed if elapsed>0 else None,
                input_size=model.spec.input_size,precision=model.spec.precision,device=model.spec.device)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2)

if __name__=='__main__':main()
