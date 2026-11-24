"""Wall-clock RGB throughput of the decoupled sampling loop; local, no ROS.

Replays local frames at a fixed input rate through the same newest-first
selection and backlog pruning the registry node uses, with the real detector.
It measures sampling/completion/latency, not flight or DDS timing.
"""
import argparse
import json
from dataclasses import replace
from collections import OrderedDict
from pathlib import Path
import sys
import threading
import time
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[2]),str(Path(__file__).resolve().parents[2]/'src')]
from person_search.detector_spec import DetectorSpec
from person_search.yolo import YoloPersonDetector
from person_search.presence_adapter import select_presence_capture, discard_superseded_rgb
from person_search.node import claim_rgb, enqueue_rgb, rgb_pixels

class Tracker:
    def __init__(self):self.last_rgb_ns=-1;self.frames=OrderedDict();self.tracks=[]

class Node:
    """Only the attributes the selection path reads."""
    registry_mode=True;last_ns=-1;rgbd_registration_mode='strict';rgbd_max_skew_s=0.
    require_pose=False;max_age=1.5;input_queue_limit=40
    def __init__(self,period_ns):
        self.queues={k:{} for k in ('rgb','depth','ri','di')}
        self.tracker=Tracker();self.input_lock=threading.RLock();self.rgb_receipts={}
        self.min_rgb_period_ns=period_ns;self.rgb_skipped_total=0;self.rgb_ingest_dropped_total=0

def quantiles(values):
    return dict(zip(('p50','p95','p99'),np.quantile(values,[.5,.95,.99]).tolist())) if values else {}

def annotate(pixels,detections):
    """Same per-frame drawing cost as publish_semantics, minus the ROS publish."""
    import cv2
    for detection in detections:
        color=np.array([30,210,255])
        pixels[detection.mask]=(pixels[detection.mask]*.65+color*.35).astype(np.uint8)
        x,y,w,h=map(int,detection.box);cv2.rectangle(pixels,(x,y),(x+w,y+h),color.tolist(),1)
    return pixels

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',required=True,type=Path)
    parser.add_argument('--weights',required=True,type=Path)
    parser.add_argument('--images',required=True,type=Path)
    parser.add_argument('--input-fps',type=float,default=30.)
    parser.add_argument('--seconds',type=float,default=60.,help='Wall-clock load duration')
    parser.add_argument('--min-rgb-period-s',type=float,default=0.)
    parser.add_argument('--annotate',action='store_true',help='Also pay the evidence-image drawing cost')
    parser.add_argument('--input-size',type=int,default=0,help='Square inference size override; 0 keeps the manifest value')
    parser.add_argument('--warmup',type=int,default=3)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if not 1.<=args.input_fps<=240.:parser.error('input-fps must be 1..240')
    if not 5.<=args.seconds<=900.:parser.error('seconds must be 5..900')
    if not np.isfinite(args.min_rgb_period_s) or args.min_rgb_period_s<0:parser.error('min-rgb-period-s must be nonnegative')
    if args.input_size and (args.input_size%32 or args.input_size<32):parser.error('input-size must be a positive multiple of 32')
    if args.output.exists():parser.error('output already exists')
    paths=sorted(p for p in args.images.iterdir() if p.suffix.lower() in ('.jpg','.jpeg','.png'))
    if not paths:parser.error('No local images')
    from PIL import Image
    frames=[np.asarray(Image.open(p).convert('RGB')) for p in paths]
    spec=DetectorSpec.load(args.manifest)
    if args.input_size:spec=replace(spec,input_size=args.input_size)
    model=YoloPersonDetector(args.weights,spec=spec)
    for i in range(args.warmup):model.detect(frames[i%len(frames)])
    node=Node(round(args.min_rgb_period_s*1e9))
    stop=threading.Event();wake=threading.Event();state=dict(index=0,inferred=0)
    def producer():
        period=1./args.input_fps;next_at=time.monotonic();index=0
        while not stop.is_set():
            now=time.monotonic()
            if now<next_at:
                stop.wait(min(next_at-now,.05));continue
            stamp=time.monotonic_ns()
            frame=frames[index%len(frames)];index+=1
            with node.input_lock:
                enqueue_rgb(node, stamp, frame)
                for key in ('depth','ri','di'):
                    queue = node.queues[key]
                    queue[stamp] = None
                    if len(queue) > node.input_queue_limit:
                        del queue[min(queue)]
                state['index']=index
            next_at+=period
            if next_at<now:next_at=now+period
            wake.set()
    samples=[]
    def consume():
        while not stop.is_set():
            wake.wait(timeout=.05);wake.clear()
            with node.input_lock:
                ns,_,_=select_presence_capture(node,node.queues,None,time.monotonic_ns())
                if ns is None:continue
                discard_superseded_rgb(node,ns)
                frame=node.queues['rgb'][ns]
                receipt=claim_rgb(node,ns,frame)
            started=time.perf_counter();pixels=rgb_pixels(type('I',(),{'encoding':'rgb8','data':frame.tobytes(),
                'height':frame.shape[0],'width':frame.shape[1],'step':frame.shape[1]*3})())
            converted=time.perf_counter();detections=model.detect(pixels);inferred_at=time.perf_counter()
            if args.annotate:annotate(pixels.copy(),detections)
            published=time.perf_counter()
            latency=time.monotonic()-receipt.received_wall_s
            node.last_ns=ns;node.tracker.last_rgb_ns=ns;node.tracker.frames[ns]=[];state['inferred']+=1
            while len(node.tracker.frames)>128:node.tracker.frames.popitem(last=False)
            samples.append(dict(stamp_ns=ns,convert_s=converted-started,inference_s=inferred_at-converted,
                frame_s=inferred_at-started,publish_s=published-inferred_at,
                latency_s=latency,instances=len(detections)))
    started=time.monotonic()
    threads=[threading.Thread(target=producer),threading.Thread(target=consume)]
    for thread in threads:thread.daemon=True;thread.start()
    time.sleep(args.seconds);stop.set();wake.set()
    for thread in threads:thread.join(timeout=10.)
    elapsed=time.monotonic()-started
    received=state['index'];inferred=len(samples);skipped=node.rgb_skipped_total;dropped=node.rgb_ingest_dropped_total
    report=dict(scope='local_sampling_loop_with_real_detector_no_ros',wall_s=elapsed,
        input_fps=args.input_fps,min_rgb_period_s=args.min_rgb_period_s,annotate=args.annotate,
        input_size=model.spec.input_size,precision=model.spec.precision,device=model.spec.device,
        rgb_received_total=received,rgb_inferred_total=inferred,rgb_skipped_total=skipped,
        rgb_ingest_dropped_total=dropped,effective_inference_fps=inferred/elapsed,
        new_frame_completion_rate=inferred/received if received else None,
        drop_rate=(received-inferred)/received if received else None,
        inference_quantiles_s=quantiles([s['inference_s'] for s in samples]),
        frame_quantiles_s=quantiles([s['frame_s'] for s in samples]),
        publish_quantiles_s=quantiles([s['publish_s'] for s in samples]),
        latency_quantiles_s=quantiles([s['latency_s'] for s in samples]))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k!='scope'},indent=2))

if __name__=='__main__':main()
