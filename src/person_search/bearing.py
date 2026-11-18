"""RGB-only angular evidence. No range, human truth, or movement commands."""
from dataclasses import dataclass, field
import math
import numpy as np
from .confirmation_quality import best_box_match, box_iou
from .perception import validate_transform


def bearing_camera(k, camera_to_map):
    k = np.asarray(k, float).reshape(3, 3)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('invalid_rgb_intrinsics')
    return k, validate_transform(camera_to_map)


def bearing_geometry(box, k, camera_to_map, width, height):
    return calibrated_bearing(box, bearing_camera(k, camera_to_map), width, height)


def calibrated_bearing(box, camera, width, height, *, validated_box=False):
    k,t=camera
    x,y,w,h=map(float,box)
    if not validated_box and (not np.isfinite([x,y,w,h]).all() or w<=0 or h<=0):raise ValueError('invalid_rgb_box')
    left,right=np.clip([x,x+w],0,width-1);top,bottom=np.clip([y,y+h],0,height-1)
    if right<=left or bottom<=top:raise ValueError('empty_visible_box')
    pixels=np.array([((left+right)/2,(top+bottom)/2),(left,top),(left,bottom),(right,top),(right,bottom)])
    rays=np.ones((5,3))
    rays[:,:2]=(pixels-k[:2,2])/k.diagonal()[:2]
    rays=(rays/np.linalg.norm(rays,axis=1,keepdims=True))@t[:3,:3].T
    direction,corners=rays[0],rays[1:]
    half=math.acos(float(np.clip((corners@direction).min(),-1,1)))
    hfov=math.atan(max(k[0,2],width-1-k[0,2])/k[0,0])
    return dict(origin=t[:3,3].tolist(),direction=direction.tolist(),corner_rays=corners.tolist(),
                half_angle_rad=max(half,.01),camera_half_fov_rad=hfov,position=None,range_m=None)


def completed_observation(execution, capture_s):
    token = execution.get('token')
    proof = execution.get('observation_proof') or {}
    event = proof.get('completed_at', proof.get('time'))
    when = execution.get('time', -1e9)
    if (token and execution.get('arrived') and proof.get('token') == token
            and event is not None and event <= capture_s and when - .75 <= capture_s <= when + .75):
        return str(token), event
    return None


@dataclass
class BearingTrack:
    id:int
    first_capture_s:float
    box:tuple
    geometry:dict
    confidence:float
    stamps_ns:list=field(default_factory=list)
    coverage_tokens:set=field(default_factory=set)
    localization_state:str='unlocalized'
    localization_reason:str='not_yet_evaluated'
    localized_surface_xyz:object=None

    @property
    def last_capture_s(self):
        return self.stamps_ns[-1] * 1e-9


class BearingMemory:
    """Short image/angular associations are cue IDs, never distinct people.

    New RGB captures do not reset native selection budgets. Basic RGB gates
    are independent of depth, mask/box area ratio, and confirmation quality.
    """
    def __init__(self,min_confidence=.5,min_mask_pixels=40,ttl=20.):
        self.min_confidence=min_confidence;self.min_mask_pixels=min_mask_pixels;self.ttl=ttl
        self.tracks=[];self.last_ns=-1;self.next_id=1

    def update(self,detections,k,transform,width,height,stamp_ns,processing_s,execution=None):
        if stamp_ns<=self.last_ns:return []
        self.last_ns=stamp_ns;capture_s=stamp_ns*1e-9;used=set();reports=[]
        self.tracks=[t for t in self.tracks if stamp_ns-t.stamps_ns[-1]<=self.ttl*1e9]
        camera=None
        for detection in detections:
            if not detection.confidence>=self.min_confidence or np.count_nonzero(detection.mask)<self.min_mask_pixels:continue
            if camera is None:camera=bearing_camera(k,transform)
            geom=calibrated_bearing(detection.box,camera,width,height)
            best=None
            for t in self.tracks:
                if t.id in used or stamp_ns-t.stamps_ns[-1]>2_000_000_000:continue
                iou=box_iou(t.box,detection.box)
                angle=math.acos(float(np.clip(np.asarray(geom['direction'])@t.geometry['direction'],-1,1)))
                if iou>=.3 or (angle<=min(.35,geom['half_angle_rad']+t.geometry['half_angle_rad'])
                              and np.linalg.norm(np.asarray(geom['origin'])-t.geometry['origin'])<=1.):
                    rank=(iou,-angle,-t.id)
                    if best is None or rank>best[0]:best=(rank,t)
            if best is not None:t=best[1]
            else:
                if len(self.tracks)>=64:continue
                t=BearingTrack(self.next_id,capture_s,tuple(detection.box),geom,float(detection.confidence))
                self.next_id+=1;self.tracks.append(t)
            used.add(t.id);t.localization_state='unlocalized';t.localization_reason='not_yet_evaluated';t.localized_surface_xyz=None;t.box=tuple(detection.box);t.geometry=geom;t.confidence=float(detection.confidence)
            t.stamps_ns=[s for s in t.stamps_ns if stamp_ns-s<=2_000_000_000]+[stamp_ns]
            reports.append(self.report(t,processing_s))
        self.observe(transform,capture_s,execution or {})
        return reports

    def observe(self,transform,capture_s,execution):
        # One actually completed execution token contributes at most once to
        # angular observation coverage. This never proves a sector is empty.
        completed = completed_observation(execution, capture_s)
        if completed is None:return
        token, _ = completed
        forward=np.asarray(transform)[:3,2]
        for t in self.tracks:
            if token in t.coverage_tokens or not t.last_capture_s<=capture_s<=t.last_capture_s+self.ttl:continue
            if forward@np.asarray(t.geometry['direction'])>=math.cos(min(1.5,t.geometry['camera_half_fov_rad']+t.geometry['half_angle_rad'])):
                t.coverage_tokens.add(token)

    def mark_localization(self,stamp_ns,rows):
        for t in self.tracks:
            if t.stamps_ns[-1]!=stamp_ns:continue
            r=best_box_match(t.box,rows,lambda row: row['box'],.3)
            if r is not None:
                t.localization_state=r['localization_state'];t.localization_reason=r.get('depth_reason');t.localized_surface_xyz=r.get('xyz')

    def report(self,t,processing_s):
        persistent=len(t.stamps_ns)>=3 and t.stamps_ns[-1]-t.stamps_ns[0]>=400_000_000
        return dict(cue_id=t.id,source_uav=0,frame='map',capture_time_s=t.last_capture_s,first_capture_s=t.first_capture_s,
            processing_time_s=processing_s,confidence=t.confidence,support_captures=len(t.stamps_ns),support_first_s=t.stamps_ns[0]*1e-9,persistent=persistent,localized_surface_xyz=t.localized_surface_xyz,
            observation_coverage=len(t.coverage_tokens),localization_state=t.localization_state,localization_reason=t.localization_reason,
            semantic_state='rgb_person_candidate',count_scope='angular_cue_not_distinct_person',**t.geometry)

    def active(self,now):
        return [self.report(t,now) for t in self.tracks if t.last_capture_s<=now<=t.last_capture_s+self.ttl]


def bearing_packet(report):
    """Version1, fixed map frame; numeric layout is shared with native header."""
    return [1.,float(report['source_uav']),float(report['cue_id']),report['capture_time_s'],*report['origin'],*report['direction'],
        report['half_angle_rad'],report['confidence'],float(report['support_captures']),float(report['observation_coverage']),
        float(report['localization_state']=='localized_surface'),report['support_first_s'],report['camera_half_fov_rad']]
