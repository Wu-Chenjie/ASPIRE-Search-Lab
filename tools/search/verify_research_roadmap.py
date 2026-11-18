"""Synthetic regression summary; no detector, ROS or flight performance claim."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
from person_search.presence import PresenceTracker
from person_search.segmentation import PersonMask
from person_search.surface import localize_surfaces

def run():
    mask=np.zeros((120,160),bool);mask[40:80,40:120]=True
    d=PersonMask((40,40,80,40),.95,mask)
    def observe(tracker,time,ds):return tracker.observe_rgb(ds,round(time*1e9),160,120,time)
    tracker=PresenceTracker();first=observe(tracker,1,[d])[0]['id']
    observe(tracker,1.2,[]);resumed=observe(tracker,1.4,[d])[0]['id']
    if first!=resumed:raise ValueError('Short miss did not resume the same identity')
    capacity=PresenceTracker()
    for i in range(140):latest=observe(capacity,1+i*5,[d])
    if latest[0]['id']!=140 or len(capacity.tracks)!=1:raise ValueError('Retirement episode accounting changed')
    a=mask.copy();a[:,80:]=False;b=mask.copy();b[:,:80]=False
    candidates=[PersonMask(d.box,.95,a),PersonMask(d.box,.93,b)]
    overlap=PresenceTracker();count=len(observe(overlap,1,candidates))
    if count!=2:raise ValueError('Disjoint masks were not retained independently')
    k=[[200,0,80],[0,200,60],[0,0,1]];t=np.diag([1.,-1.,-1.,1.]);t[2,3]=3
    rows,_=localize_surfaces([d],np.full(mask.shape,2.),k,t)
    if rows[0]['xyz'] is None:raise ValueError('Horizontal surface was not localized')
    rgb_only=PresenceTracker()
    for s in (1,1.2,1.4,1.6,1.8):
        observe(rgb_only,s,[d]);rgb_only.reject_localization(round(s*1e9),s,'no_depth_evidence')
    state=rgb_only.report(1.8)['tracks'][0]
    if state['semantic_state']!='confirmed' or state['current_surface'] is not None:
        raise ValueError('RGB-only localization changed the presence state')
    return dict(scope=__doc__,baseline='2a9a0d52fa5e7a045bfbd4cb94aecaf3ac6f3753',
        code_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        worktree_dirty=bool(subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip()),
        short_miss=dict(first_id=first,resumed_id=resumed),retirement=dict(episodes=140,latest_id=140,active_records=len(capacity.tracks)),
        disjoint_masks=dict(input=2,retained=count),horizontal_surface=rows[0],
        rgb_only=dict(semantic_state=state['semantic_state'],current_surface=state['current_surface']))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path);args=parser.parse_args()
    result=json.dumps(run(),indent=2,ensure_ascii=False,allow_nan=False)
    if args.output:
        with args.output.open('x') as stream:stream.write(result+'\n')
    print(result)
