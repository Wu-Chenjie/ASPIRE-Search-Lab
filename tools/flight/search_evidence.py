"""Offline summary and annotated inference-frame replay; never commands flight."""
import argparse,bisect,collections,hashlib,json
from pathlib import Path
import numpy as np


def rows(path):return [json.loads(s) for s in Path(path).read_text().splitlines()]
def stats(a):
    a=list(a)
    return dict(count=len(a),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(max(a))) if a else None


def summarize(run):
    from tools.flight.evaluate import evaluate
    run=Path(run);tel=rows(run/'telemetry.jsonl');obs=rows(run/'perception/observations.jsonl')
    sem={r['stamp_ns']:r for r in obs if r['kind']=='semantic'}
    timing=[r for r in obs if r['kind']=='timing'];accepted=[r for r in timing if r['status']=='accepted']
    first2d=min((ns for ns,r in sem.items() if r['candidates']),default=None)
    first3d=min((ns for ns,r in sem.items() if any(c['state']=='localized' for c in r['candidates'])),default=None)
    confirmed=[r['data'] for r in tel if r['kind']=='tracks' and any(t['state']=='confirmed' for t in r['data']['tracks'])]
    j=evaluate(run);j.update(semantic_frames=len(sem),positive_semantic_frames=sum(bool(r['candidates']) for r in sem.values()),
        first_2d_sim_s=first2d*1e-9 if first2d else None,first_3d_sim_s=first3d*1e-9 if first3d else None,
        first_confirmed_sim_s=confirmed[0]['stamp_ns']*1e-9 if confirmed else None,
        inference_wall_s=stats(r['inference_wall_s'] for r in sem.values() if r.get('inference_wall_s') is not None),
        accepted_capture_age_sim_s=stats(r['age_s'] for r in accepted),
        accepted_receipt_age_wall_s=stats(r['received_age_wall_s'] for r in accepted if 'received_age_wall_s' in r),
        timing_status_counts=dict(collections.Counter(r['status']+':'+str(r['reason']) for r in timing)),
        unlocalized_reason_counts=dict(collections.Counter(c['depth_reason'] for r in sem.values() for c in r['candidates'] if c['state']=='unlocalized')),
        unlocalized_has_no_position=all(c['xyz'] is None for r in sem.values() for c in r['candidates'] if c['state']=='unlocalized'),
        raw_inference_files=len(list((run/'perception').glob('*-rgb-masks.npz'))),
        annotation_source='Recorded model boxes/masks only; positions/states from actual online messages; no truth drawn',
        truth_use='Only offline scoring against evaluation-truth.json; online modules do not read it')
    text=json.dumps(j,indent=2);(run/'search-audit.json').write_text(text);print(text);return j


def video(run,out,speed=4.):
    import cv2
    run,out=Path(run),Path(out);out.mkdir(parents=True,exist_ok=False)
    obs=rows(run/'perception/observations.jsonl');sem={r['stamp_ns']:r for r in obs if r['kind']=='semantic'}
    files=sorted((run/'perception').glob('*-rgb-masks.npz'),key=lambda p:int(p.name.split('-')[0]));stamps=[int(p.name.split('-')[0]) for p in files]
    tel=rows(run/'telemetry.jsonl');tracks=sorted([r['data'] for r in tel if r['kind']=='tracks'],key=lambda r:r['stamp_ns']);ts=[r['stamp_ns'] for r in tracks]
    poses=sorted([r['data'] for r in tel if r['kind']=='estimated_odometry'],key=lambda r:r['stamp']);pt=[r['stamp']*1e9 for r in poses]
    xy=np.array([p['xyz'][:2] for p in poses]);lo=xy.min(0)-.8;hi=xy.max(0)+.8;span=max(hi-lo)
    writer=cv2.VideoWriter(str(out/'search-replay.mp4'),cv2.VideoWriter_fourcc(*'avc1'),10.,(1000,600))
    if not writer.isOpened():raise RuntimeError('H264 writer unavailable')
    index=[];last_file=None;frame_data=None
    def point(v):return tuple(np.rint([820,420]+(np.asarray(v)[:2]-(lo+hi)/2)*[260/span,-260/span]).astype(int))
    for n,shown in enumerate(np.arange(stamps[0],stamps[-1]+1,speed*.1e9)):
        fi=max(0,bisect.bisect_right(stamps,shown)-1);ns=stamps[fi]
        if last_file!=fi:frame_data=np.load(files[fi]);last_file=fi
        d=frame_data;rgb=d['rgb'].copy();entry=sem.get(ns,{});candidates=entry.get('candidates',[])
        for i,(mask,box,score) in enumerate(zip(d['masks'],d['boxes'],d['scores'])):
            color=np.array([255,210,30]) if i>=len(candidates) or candidates[i]['state']=='unlocalized' else np.array([30,210,255])
            rgb[mask]=(rgb[mask]*.65+color*.35).astype(np.uint8)
            x,y,w,h=map(int,box);cv2.rectangle(rgb,(x,y),(x+w,y+h),color.tolist(),1)
            cv2.putText(rgb,f'person {score:.2f}',(x,max(12,y-3)),cv2.FONT_HERSHEY_SIMPLEX,.3,color.tolist(),1)
        frame=np.full((600,1000,3),24,np.uint8);frame[70:550,:640]=cv2.resize(rgb[:,:,::-1],(640,480),interpolation=cv2.INTER_NEAREST)
        ti=bisect.bisect_right(ts,ns)-1;tr=tracks[ti]['tracks'] if ti>=0 else []
        pi=bisect.bisect_right(pt,ns);path=np.array([point(p['xyz']) for p in poses[:pi]],np.int32)
        if len(path)>1:cv2.polylines(frame,[path],False,(200,150,60),2)
        if len(path):cv2.circle(frame,tuple(path[-1]),5,(255,255,255),-1)
        for t in tr:
            c=(40,220,70) if t['state']=='confirmed' else (0,170,255);p=point(t['xyz']);cv2.circle(frame,p,7,c,2)
            cv2.putText(frame,f"#{t['id']} {t['state']}",(p[0]+8,p[1]),cv2.FONT_HERSHEY_SIMPLEX,.43,c,1)
        texts=[(15,25,f'INFERENCE-FRAME LOG REPLAY {speed:g}x | not live | gaps held, no interpolation'),
               (15,53,f'RGB capture {ns*1e-9:.3f}s | shown {shown*1e-9:.3f}s | no truth overlays'),
               (655,100,f'2D detections: {len(candidates)}'),
               (655,135,f"3D localized: {sum(c['state']=='localized' for c in candidates)}"),
               (655,170,f"Confirmed tracks: {sum(t['state']=='confirmed' for t in tr)}"),
               (655,225,'Map: estimated XY path'),(655,250,'and online target markers'),
               (15,581,'Yellow: 2D only | Cyan: localized | Green map marker: confirmed across viewpoints')]
        for x,y,text in texts:cv2.putText(frame,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.47,(225,225,225),1,cv2.LINE_AA)
        if candidates and candidates[0]['state']=='unlocalized':
            reason=candidates[0]['depth_reason'] or 'pending depth';cv2.putText(frame,reason[:40],(655,195),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,170,255),1)
        writer.write(frame);index.append(dict(frame=n,shown_sim_ns=int(shown),source_capture_ns=ns,source=str(files[fi].relative_to(run))))
    writer.release();(out/'frame-index.json').write_text(json.dumps(index))
    meta=dict(scope='Actual online inference frames replay, not continuous recording; 4x simulated time with held frame gaps, no interpolation',
        run=str(run),frames=len(index),fps=10,speed=speed,source_boxes_masks='Recorded exact model arrays',source_markers='Actual /search/tracks messages',
        no_truth_drawn=True,raw_continuous_rgb_available=(run/'continuous-rgb.avi').exists(),
        sha256=hashlib.sha256((out/'search-replay.mp4').read_bytes()).hexdigest())
    (out/'video-source.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--video-output',type=Path);a=p.parse_args()
    summarize(a.run)
    if a.video_output:video(a.run,a.video_output)
