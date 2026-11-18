"""Actual fixed-detector static-frame regression; all truth use is offline here."""
import argparse,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from person_search.yolo import YoloPersonDetector,VERSION,WEIGHT_SHA256
from person_search.segmentation import localize_masks
from person_search.tracking import Tracker


def evaluate(run,weights):
    import cv2
    run=Path(run);fixture=json.loads((run/'viewpoints.json').read_text())
    detector=YoloPersonDetector(weights);tracker=Tracker();report={}
    for i,(name,view) in enumerate(fixture['views'].items()):
        folder=run/name;meta=json.loads((folder/'frame.json').read_text())
        rgb=cv2.imread(str(folder/'rgb.png'))[:,:,::-1];depth=np.load(folder/'depth.npy')
        transform=np.eye(4);transform[:3,3]=view['lens_xyz']
        transform[:3,:3]=Rotation.from_euler('z',view['lens_yaw']).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        start=time.perf_counter();ds=detector.detect(rgb);elapsed=time.perf_counter()-start
        rows,positions=localize_masks(ds,depth,meta['depth']['k'],transform)
        tracks=tracker.update(positions,transform[:3,3],float(i))
        image=rgb.copy();maskdata=[]
        for d in ds:
            image[d.mask]=(image[d.mask]*.6+np.array([30,220,50])*.4).astype(np.uint8)
            x,y,w,h=map(int,d.box);cv2.rectangle(image,(x,y),(x+w,y+h),(20,220,50),1)
            cv2.putText(image,f'person {d.confidence:.2f}',(x,max(10,y-4)),cv2.FONT_HERSHEY_SIMPLEX,.3,(20,220,50),1)
            maskdata.append(d.mask)
        cv2.imwrite(str(folder/'yolo.png'),image[:,:,::-1]);np.savez_compressed(folder/'masks.npz',masks=np.array(maskdata,dtype=bool))
        report[name]=dict(semantic_count=len(rows),localized_count=len(positions),candidates=rows,inference_wall_s=elapsed,
            tracks=[dict(id=t.id,state=t.state,xyz=t.xyz.tolist(),semantic_views=len(t.semantic_views)) for t in tracks])
    negative=fixture.get('negative',False)
    checks=dict(no_person_false_positive=all(r['semantic_count']==0 for r in report.values())) if negative else dict(
        visible_detected=report['visible']['semantic_count']>0,visible_localized=report['visible']['localized_count']>0,
        partial_detected=report['partial']['semantic_count']>0,
        partial_depth_safe=all((r['state']=='unlocalized' and r['xyz'] is None and bool(r['depth_reason'])) or
                              (r['state']=='localized' and r['xyz'] is not None) for r in report['partial']['candidates']),
        hidden_no_detection=report['hidden']['semantic_count']==0,revealed_detected=report['revealed']['semantic_count']>0,
        revealed_localized=report['revealed']['localized_count']>0,
        single_confirmed_track=len(tracker.tracks)==1 and tracker.tracks[0].state=='confirmed')
    result=dict(scope='Manually positioned static Gazebo camera, NOT autonomous search or flight',backend='yolo11n-seg',
        ultralytics=VERSION,weight_sha256=WEIGHT_SHA256,confidence=.35,imgsz=640,device='cpu',retina_masks=True,
        annotation_source='YOLO class-0 boxes and instance masks only; no truth projection drawn',negative=negative,views=report,checks=checks)
    result['partial_localization_required']=False
    result['depth_policy']='Severe occlusion may remain unlocalized; preserve 2-D evidence and reject ambiguous depth. Per-view localized_count remains reported.'
    (run/'segmentation-evaluation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    return all(checks.values())


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--weights',required=True,type=Path);a=p.parse_args()
    raise SystemExit(0 if evaluate(a.run,a.weights) else 1)
