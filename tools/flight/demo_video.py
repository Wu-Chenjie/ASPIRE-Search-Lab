"""Faithful offline demo from continuous RGB, actual detections and estimate logs.

Each selected recorded RGB frame appears exactly once. No interpolation or frame
holds are used to inflate cadence. Detection overlays appear only on their own
saved inference frame, never on a newer live-source image. No truth is loaded.
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import numpy as np
from tools.flight.demo_video_audit import rows, sha256, timing_report


def asof(events, receipt_ns):
    """Only messages actually received by the recorder at this instant."""
    stamps=[r['receipt_monotonic_ns'] for r in events]
    i=bisect.bisect_right(stamps,receipt_ns)-1
    return events[i] if i>=0 else None


def first_result_receipts(events):
    """A source may publish 2D then localized status; count it only once."""
    seen=set();receipts=[]
    for event in sorted(events,key=lambda r:r['receipt_monotonic_ns']):
        ns=event['data'].get('stamp_ns')
        if ns is None or ns in seen:continue
        seen.add(ns);receipts.append(event['receipt_monotonic_ns'])
    return receipts


def unique_result_rate(receipts, now_ns, window_ns=5_000_000_000):
    count=bisect.bisect_right(receipts,now_ns)-bisect.bisect_right(receipts,now_ns-window_ns)
    return count/(window_ns*1e-9)


def select_frames(records, start=None, end=None):
    chosen=[r for r in records if (start is None or r['stamp_ns']>=start*1e9)
            and (end is None or r['stamp_ns']<=end*1e9)]
    if len(chosen)<2:raise ValueError('Need at least two source frames in interval')
    if not all(b['stamp_ns']>a['stamp_ns'] for a,b in zip(chosen,chosen[1:])):
        raise ValueError('Non-increasing source stamps; cannot claim unique frame replay')
    if not all(b['frame']==a['frame']+1 for a,b in zip(chosen,chosen[1:])):
        raise ValueError('Frame index is not contiguous')
    return chosen


def map_extent(poses, track_events):
    """Fixed offline axes cover recorded estimates and online tracks, no truth."""
    points=[p['xyz'][:2] for p in poses]
    points.extend(t['xyz'][:2] for event in track_events
                  for t in event['data'].get('tracks',[]) if t.get('xyz') is not None)
    xy=np.asarray([p for p in points if len(p)==2 and np.isfinite(p).all()])
    if not len(xy):xy=np.array([[0.,0.]])
    lo,hi=xy.min(0)-1.,xy.max(0)+1.
    return (lo+hi)/2.,float(max(hi-lo))


def review_label(tracks_event, review_event):
    if tracks_event and tracks_event['data'].get('mode')=='legacy':
        return 'disabled (legacy confirmation)'
    if review_event:
        d=review_event['data']
        return f"track #{d.get('track','?')} {d.get('state','')}: {d.get('reason','')}"
    tracks=tracks_event['data'].get('tracks',[]) if tracks_event else []
    if tracks:
        t=tracks[-1]
        return f"latest track #{t['id']}: {t.get('reason',t.get('state','unknown'))}"
    return 'no 3D track; see 2D candidate status'


def compose(run,out,start=None,end=None,speed=1.,title='Recorded simulation',expected_hz=None):
    import cv2
    run,out=Path(run),Path(out)
    if not math.isfinite(speed) or speed<=0:raise ValueError('Positive finite speed required')
    if out.resolve().is_relative_to(Path(__file__).resolve().parents[2]):
        raise ValueError('Video output must be outside the code tree')
    records=select_frames(rows(run/'continuous-rgb-stamps.jsonl'),start,end)
    if not all('receipt_monotonic_ns' in r for r in records):
        raise ValueError('This faithful overlay requires the new recorder receipt timestamps')
    overlay=rows(run/'continuous-overlays.jsonl')
    groups={k:sorted([r for r in overlay if r['kind']==k],key=lambda r:r['receipt_monotonic_ns'])
            for k in ['semantic','tracks','flight','execution','active_confirmation']}
    # Pre-index receipt times; never use capture stamps as publication times.
    group_times={k:[r['receipt_monotonic_ns'] for r in v] for k,v in groups.items()}
    inference_receipts=first_result_receipts(groups['semantic'])
    def latest(k,receipt,source_ns):
        i=bisect.bisect_right(group_times[k],receipt)-1
        if k == 'semantic':
            while i >= 0 and groups[k][i]['data'].get('stamp_ns', 0) > source_ns:
                i -= 1
        return groups[k][i] if i>=0 else None
    poses=sorted([r['data'] for r in rows(run/'telemetry.jsonl') if r['kind']=='estimated_odometry'],key=lambda r:r['stamp'])
    pt=[r['stamp']*1e9 for r in poses]
    center,span=map_extent(poses,groups['tracks'])
    def point(v):return tuple(np.rint(np.array([1030,545])+(np.array(v[:2])-center)*[200/span,-200/span]).astype(int))
    source_span=(records[-1]['stamp_ns']-records[0]['stamp_ns'])*1e-9
    fps=len(records)/source_span*speed
    out.mkdir(parents=True,exist_ok=False)
    video=out/'demo.mp4';writer=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'avc1'),fps,(1280,720))
    if not writer.isOpened():raise RuntimeError('H264 encoder unavailable; no substitute silently selected')
    cap=cv2.VideoCapture(str(run/'continuous-rgb.avi'))
    if not cap.isOpened():raise ValueError('Cannot open source video')
    index=(out/'frame-index.jsonl').open('x')
    # Sequential decode avoids codec seek ambiguity when selecting a clip.
    for _ in range(records[0]['frame']):
        if not cap.read()[0]:raise ValueError('Source ends before selected clip')
    infer_ns=None;inference=None;written=0
    def text(canvas,s,x,y,scale=.47,color=(235,235,235)):
        cv2.putText(canvas,str(s),(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
    try:
        for n,row in enumerate(records):
            ok,bgr=cap.read()
            if not ok:raise ValueError('Source video has fewer frames than timestamp index')
            ns=row['stamp_ns'];receipt=row['receipt_monotonic_ns']
            current={k:latest(k,receipt,ns) for k in groups}
            sem=current['semantic'];tracks=current['tracks'];ex=current['execution'];fl=current['flight']
            semdata=sem['data'] if sem else {};det_ns=semdata.get('stamp_ns')
            if det_ns!=infer_ns:
                inference=None;infer_ns=det_ns
                f=run/'perception'/f'{det_ns}-rgb-masks.npz'
                if det_ns is not None and f.exists():
                    with np.load(f) as d:
                        inference=d['rgb'][:,:,::-1].copy()
                        for box,score in zip(d['boxes'],d['scores']):
                            x,y,w,h=map(int,box)
                            cv2.rectangle(inference,(x,y),(x+w,y+h),(0,210,255),1)
                            cv2.putText(inference,f'person {score:.2f}',(x,max(12,y-3)),cv2.FONT_HERSHEY_SIMPLEX,.32,(0,210,255),1)
            canvas=np.full((720,1280,3),22,np.uint8)
            canvas[105:681,15:783]=cv2.resize(bgr,(768,576),interpolation=cv2.INTER_NEAREST)
            text(canvas,title[:100],15,25,.65)
            text(canvas,f'RECORDED SIMULATION | average {speed:g}x simulated-time replay | one source frame per output',15,51)
            text(canvas,f'RGB source t={ns*1e-9:.3f}s | 320x240 original pixels | no interpolation / no stale boxes',15,80)
            text(canvas,'Latest actual inference frame (held separately)',805,107,.45)
            if inference is not None:
                canvas[145:385,850:1170]=inference
            else:text(canvas,'No saved inference image yet',830,235)
            age=(ns-det_ns)*1e-9 if det_ns is not None else None
            receipt_age=(receipt-sem['receipt_monotonic_ns'])*1e-9 if sem else None
            hz=unique_result_rate(inference_receipts,receipt)
            detail=f'source={det_ns*1e-9:.3f}s age={age:.3f} sim-s' if det_ns is not None else 'Waiting for detector'
            text(canvas,detail,805,128,.43)
            candidates=semdata.get('candidates',[])
            localized=sum(c.get('state')=='localized' for c in candidates)
            text(canvas,f'Latest inference: {len(candidates)} candidate(s), {localized} depth-localized',805,390,.39)
            text(canvas,f'Unique inference results: {hz:.2f} Hz wall (last 5s)',805,407,.40)
            text(canvas,'Estimated XY route + online track states',805,433,.43)
            pi=bisect.bisect_right(pt,ns)
            path=np.array([point(p['xyz']) for p in poses[:pi]],np.int32)
            if len(path)>1:cv2.polylines(canvas,[path],False,(220,140,60),2)
            if len(path):
                position=tuple(path[-1]);cv2.circle(canvas,position,5,(255,255,255),-1)
                q=poses[pi-1].get('quaternion')
                if q is not None:
                    x,y,z,w=q;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                    end=(position[0]+round(18*math.cos(yaw)),position[1]-round(18*math.sin(yaw)))
                    cv2.arrowedLine(canvas,position,end,(255,255,255),2,tipLength=.3)
            tr=tracks['data'].get('tracks',[]) if tracks else []
            for t in tr:
                if t.get('xyz') is None:continue
                p=point(t['xyz']);color=(50,220,70) if t['state']=='confirmed' else (0,170,255)
                if 805 <= p[0] <= 1260 and 445 <= p[1] <= 650:
                    cv2.circle(canvas,p,6,color,2);text(canvas,f"#{t['id']} {t['state']}",min(p[0]+8,1150),p[1],.35,color)
            cv2.line(canvas,(815,635),(815+round(200/span),635),(150,150,150),2)
            text(canvas,'1 m',815,655,.35)
            text(canvas,f"Tracks: {len(tr)} | algorithm-confirmed: {sum(t.get('state')=='confirmed' for t in tr)}",805,698,.39)
            review=current['active_confirmation']
            text(canvas,'Review: '+review_label(tracks,review)[:92],15,96,.42)
            reason=ex['data'].get('reason','unknown') if ex else 'waiting'
            flight=fl['data'].get('reason','unknown') if fl else 'waiting'
            text(canvas,f'Execution: {reason} | Flight: {flight}',805,675,.4)
            text(canvas,'No truth overlays. Track labels are algorithm outputs, not ground-truth confirmation.',15,707,.45)
            writer.write(canvas);written+=1
            index.write(json.dumps(dict(output_frame=n,source_frame=row['frame'],source_stamp_ns=ns,
                source_pixels_sha256=row.get('source_pixels_sha256'),source_receipt_monotonic_ns=receipt,
                unique_inference_receipts_hz_wall_last_5s=hz,
                detection_result_held_wall_s=receipt_age,
                detection_source_stamp_ns=det_ns,
                detection_receipt_monotonic_ns=sem['receipt_monotonic_ns'] if sem else None,
                tracks_receipt_monotonic_ns=tracks['receipt_monotonic_ns'] if tracks else None))+'\n')
    finally:
        writer.release();cap.release();index.close()
    check=cv2.VideoCapture(str(video));decoded=0;actual_fps=check.get(cv2.CAP_PROP_FPS)
    while check.read()[0]:decoded+=1
    check.release()
    report=dict(run=str(run),title=title,frames=written,decoded_frames=decoded,encoded_fps=actual_fps,
        duration_s=decoded/actual_fps,source_span_sim_s=source_span,
        requested_average_speed=speed,actual_average_speed=source_span/(decoded/actual_fps),
        timing_note='Each source frame encoded once. Irregular source gaps are compressed by CFR; source timestamp remains visible. Speed is average, not constant during gaps.',
        no_generated_or_held_main_rgb_frames=True,detector_view='Exact saved inference frame, separately held and labelled by source stamp and age.',
        overlay_causality='Only messages received by the recorder before each RGB receipt are shown.',
        inference_rate='First receipt for each distinct inference source timestamp, counted once in trailing 5 wall seconds; 2D/localized updates do not inflate rate.',
        map='Estimated XY route and online tracks only; fixed offline axes encompass all recorded positions. Not occupancy geometry. No truth files read.',
        map_center_xy=center.tolist(),map_span_m=span,
        source_timing=timing_report(records,expected_hz),source_video_sha256=sha256(run/'continuous-rgb.avi'),
        source_stamps_sha256=sha256(run/'continuous-rgb-stamps.jsonl'),
        source_overlays_sha256=sha256(run/'continuous-overlays.jsonl'),
        video_sha256=sha256(video),index_sha256=sha256(out/'frame-index.jsonl'))
    report['one_to_one_verified']=decoded==written==len(records)
    (out/'video-audit.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['one_to_one_verified']:raise RuntimeError('Encoded frame count mismatch')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--start',type=float);p.add_argument('--end',type=float);p.add_argument('--speed',type=float,default=1.)
    p.add_argument('--expected-hz',type=float);p.add_argument('--title',default='Recorded simulation')
    a=p.parse_args();print(json.dumps(compose(a.run,a.output,a.start,a.end,a.speed,a.title,a.expected_hz),indent=2))
