"""Replay stored RGB masks and capture transforms; never infer a target range."""
import argparse,bisect,collections,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
from person_search.bearing import BearingMemory,bearing_packet
from person_search.segmentation import PersonMask

def replay(run):
    run=Path(run);observations=[json.loads(v) for v in (run/'perception/observations.jsonl').read_text().splitlines()]
    semantics={v['stamp_ns']:v['candidates'] for v in observations if v['kind']=='semantic'}
    timings={v['stamp_ns']:v for v in observations if v['kind']=='timing' and v['status']=='accepted'}
    telemetry=[json.loads(v) for v in (run/'telemetry.jsonl').read_text().splitlines()]
    execution=sorted([r['data'] for r in telemetry if r['kind']=='execution'],key=lambda x:x['time'])
    times=[v['time'] for v in execution];memory=BearingMemory();counts=collections.Counter();events=[]
    masks=sorted((run/'perception').glob('*-rgb-masks.npz'),key=lambda p:int(p.name.split('-')[0]))
    for p in masks:
        ns=int(p.name.split('-')[0]);gpath=p.with_name(str(ns)+'-geometry.npz');counts['saved_rgb_captures']+=1
        if not gpath.exists() or ns not in timings:counts['no_saved_fresh_pose_geometry_skipped']+=1;continue
        with np.load(p,allow_pickle=False) as r:
            dets=[PersonMask(tuple(b),float(s),m.copy()) for b,s,m in zip(r['boxes'],r['scores'],r['masks'])];height,width=r['rgb'].shape[:2]
        now=timings[ns]['now_ns']*1e-9;index=bisect.bisect_right(times,now)-1;receipt=execution[index] if index>=0 else {}
        with np.load(gpath,allow_pickle=False) as g:
            memory.update(dets,g['k'],g['camera_to_map'],width,height,ns,now,receipt)
        rows=semantics.get(ns,[]);memory.mark_localization(ns,rows)
        reports=[r for r in memory.active(now) if r['capture_time_s']==ns*1e-9]
        persistent=[r for r in reports if r['persistent']]
        counts['processed_capture_transforms']+=1;counts['detector_candidate_frames']+=bool(dets)
        counts['localized_surface_frames']+=any(r['localization_state']=='localized_surface' for r in rows)
        counts['persistent_bearing_frames']+=bool(persistent)
        unlocalized=bool(rows) and not any(r['localization_state']=='localized_surface' for r in rows)
        counts['persistent_bearing_despite_unlocalized_frames']+=bool(persistent) and unlocalized
        for report in persistent:
            assert report['position'] is None and report['range_m'] is None
            events.append(dict(**report,native_packet=bearing_packet(report)))
    return dict(scope='Offline stored detector masks plus recorded capture transforms. No detector rerun, truth, new movement, or independent real-person count. Frames without a saved pose transform are skipped, never assigned latest pose.',
        run=str(run),counts=dict(counts),associated_angular_cue_ids=len(memory.tracks),count_scope='cue IDs are image/angular hypotheses, not unique people',
        reports=events,source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'src/person_search/bearing.py',ROOT/'tools/flight/bearing_bias.hpp']})

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if ROOT==a.output.resolve() or ROOT in a.output.resolve().parents:raise ValueError('Evidence must be outside code')
    d=replay(a.run);a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:json.dump(d,f,indent=2)
    print(json.dumps({k:v for k,v in d.items() if k!='reports'},indent=2))
