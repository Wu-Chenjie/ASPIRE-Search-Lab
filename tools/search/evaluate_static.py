"""Independent evaluation of paired fixture frames. Never imported by policy."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
from person_search.perception import PeopleDetector,candidates
from person_search.tracking import Tracker

def main():
    import cv2
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();detector=PeopleDetector();tracker=Tracker();results={}
    # Only evaluator accesses view fixture / person truth.
    fixture=json.loads((a.run/'viewpoints.json').read_text());truth=np.array(fixture['person_pose'][:2])
    for i,name in enumerate(('visible','partial','hidden','revealed')):
        folder=a.run/name;rec=json.loads((folder/'frame.json').read_text());view=fixture['views'][name]
        t=np.eye(4);t[:3,3]=view['lens_xyz'];t[:3,:3]=Rotation.from_euler('z',view['lens_yaw']).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        rgb=cv2.imread(str(folder/'rgb.png'))[:,:,::-1];depth=np.load(folder/'depth.npy')
        cs=candidates(rgb,depth,rec['depth']['k'],t,detector);tracker.update(cs,t[:3,3],float(i))
        results[name]={'count':len(cs),'semantic_count':sum(c.semantic for c in cs),'positions':[c.xyz.tolist() for c in cs],'horizontal_surface_error_m':[float(np.linalg.norm(c.xyz[:2]-truth)) for c in cs],'depth_m':[c.depth_m for c in cs],'registered_pairs':rec['pairs_checked'],'tracks':[{'id':t.id,'state':t.state} for t in tracker.tracks]}
    checks={'visible_person':results['visible']['semantic_count']>=1,'partial_candidate':results['partial']['count']>=1,'hidden_no_candidate':results['hidden']['count']==0,'revealed_person':results['revealed']['semantic_count']>=1,'single_confirmed_track':len(tracker.tracks)==1 and tracker.tracks[0].state=='confirmed','partial_uses_person_depth':all(x>3.5 for x in results['partial']['depth_m'])}
    report={'scope':'Fixed manually positioned fixture evaluation, NOT autonomous search. Position is visible-surface median, not body center.','checks':checks,'views':results}
    (a.run/'perception-evaluation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if not all(checks.values()):raise SystemExit(1)
if __name__=='__main__':main()
