"""Audit recorded source cadence separately from container playback cadence."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def rows(path):
    path=Path(path)
    return [json.loads(s) for s in path.read_text().splitlines()] if path.exists() else []


def timing_report(records, expected_hz=None):
    ns = np.array([r['stamp_ns'] for r in records], dtype=np.int64)
    ds = np.diff(ns) * 1e-9
    duration = float((ns[-1]-ns[0])*1e-9) if len(ns)>1 else 0.
    wall = [r.get('receipt_monotonic_ns') for r in records]
    wall_duration = (wall[-1]-wall[0])*1e-9 if len(wall)>1 and all(v is not None for v in wall) else None
    hashes = [r.get('source_pixels_sha256') for r in records]
    hashes_known = bool(hashes) and all(h is not None for h in hashes)
    expected_missing = None
    if expected_hz and len(ds):
        expected_missing = int(sum(max(0, round(float(d)*expected_hz)-1) for d in ds))
    # Only complete elapsed one-second windows; no edge partial-window bias.
    one_second_counts = []
    if duration >= 1:
        relative = (ns-ns[0])*1e-9
        full_seconds = int(duration)
        bins = relative[(0 <= relative) & (relative < full_seconds)].astype(int)
        one_second_counts = np.bincount(bins, minlength=full_seconds).tolist()
    return dict(frames=len(records), distinct_source_stamps=len(set(ns.tolist())),
        first_source_stamp_ns=int(ns[0]) if len(ns) else None,
        last_source_stamp_ns=int(ns[-1]) if len(ns) else None,
        monotonic_source_stamps=bool(np.all(ds>0)), source_span_sim_s=duration,
        actual_capture_fps_sim=(len(ns)-1)/duration if duration else None,
        receipt_span_wall_s=wall_duration,
        actual_receipt_fps_wall=(len(ns)-1)/wall_duration if wall_duration and wall_duration>0 else None,
        source_gap_s=dict(median=float(np.median(ds)), p95=float(np.percentile(ds,95)), max=float(max(ds))) if len(ds) else None,
        expected_sensor_hz=expected_hz, estimated_missing_slots=expected_missing,
        missing_slot_note='Estimate from timestamp intervals and declared sensor rate; not a transport-loss diagnosis.',
        full_second_frame_counts=one_second_counts,
        unique_source_pixel_hashes=len(set(hashes)) if hashes_known else None,
        consecutive_identical_pixel_messages=sum(a==b for a,b in zip(hashes,hashes[1:])) if hashes_known else None,
        identical_pixel_note='Separate camera messages may legitimately have identical pixels while stationary; this is not interpolated footage.')


def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def audit(run, expected_hz=None, decode=True):
    import cv2
    run=Path(run)
    report=timing_report(rows(run/'continuous-rgb-stamps.jsonl'),expected_hz)
    metadata_path=run/'continuous-rgb-metadata.json'
    report['recorder_metadata']=json.loads(metadata_path.read_text()) if metadata_path.exists() else None
    path=run/'continuous-rgb.avi'
    cap=cv2.VideoCapture(str(path))
    if not cap.isOpened():raise ValueError('Cannot open recorded video')
    fps=cap.get(cv2.CAP_PROP_FPS); reported=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    count=0
    if decode:
        while cap.read()[0]:count+=1
    cap.release()
    report.update(encoded_fps=fps, container_reported_frames=reported,
        decoded_frames=count if decode else None,
        encoded_duration_s=(count if decode else reported)/fps if fps else None,
        source_video_sha256=sha256(path), source_stamps_sha256=sha256(run/'continuous-rgb-stamps.jsonl'))
    report['one_encoded_frame_per_source_record']=(count==report['frames']) if decode else None
    report['average_sim_playback_multiplier']=report['source_span_sim_s']/report['encoded_duration_s'] if report['encoded_duration_s'] else None
    report['high_cadence_capture_pass']=bool(report['monotonic_source_stamps'] and report['one_encoded_frame_per_source_record'] and
        report['actual_capture_fps_sim'] is not None and report['actual_capture_fps_sim']>=25 and
        report['source_gap_s'] and report['source_gap_s']['p95']<=.040001)
    report['high_cadence_scope']='At least 25 distinct source timestamps/sim second, P95 gap <=40ms; not a claim of realtime wall-clock rendering.'
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--expected-hz',type=float);p.add_argument('--output',type=Path)
    a=p.parse_args();j=audit(a.run,a.expected_hz);s=json.dumps(j,indent=2)
    if a.output:a.output.write_text(s+'\n')
    print(s)
