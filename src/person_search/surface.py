"""Mask-supported depth components; no body centre, scene truth or guessed range."""
from dataclasses import dataclass, asdict
import numpy as np
from scipy.ndimage import binary_erosion,distance_transform_edt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from .perception import cloud,person_depth_pixels

@dataclass(frozen=True)
class SurfaceConfig:
    mask_border_px:int=3
    depth_jump_m:float=.12
    min_pixels:int=80
    min_valid_fraction:float=.60
    min_component_fraction:float=.55
    min_core_fraction:float=.60
    min_dominance_gap:float=.15
    max_span_m:float=.35
    min_major_extent_m:float=.12
    min_second_extent_m:float=.015
    max_patch_diameter_m:float=2.5
    pose_noise_floor_m:float=.05
    compact_surface_min_fraction:float=.80
    def __post_init__(self):
        if any(not np.isfinite(v) or v<=0 for v in asdict(self).values()):raise ValueError('Positive finite surface configuration required')
        if type(self.mask_border_px) is not int:raise ValueError('Integer surface mask border required')

DEFAULT_SURFACE_CONFIG=SurfaceConfig()


def depth_components(depth,valid,jump):
    """4-connected pixels linked only when their adjacent depths agree."""
    count=int(valid.sum())
    labels_by_pixel=np.full(valid.shape,-1,np.int32)
    if not count:return labels_by_pixel,[]
    labels_by_pixel[valid]=np.arange(count)
    edges=[]
    for a,b,da,db in ((labels_by_pixel[:,:-1],labels_by_pixel[:,1:],depth[:,:-1],depth[:,1:]),
                     (labels_by_pixel[:-1],labels_by_pixel[1:],depth[:-1],depth[1:])):
        adj=(a>=0)&(b>=0)
        agree=np.abs(da[adj]-db[adj])<=jump
        edges.append((a[adj][agree],b[adj][agree]))
    left=np.concatenate([x[0] for x in edges]);right=np.concatenate([x[1] for x in edges])
    graph=coo_matrix((np.ones(len(left),np.uint8),(left,right)),shape=(count,count)).tocsr()
    n,labels=connected_components(graph,directed=False)
    labels_by_pixel[valid]=labels
    return labels_by_pixel,np.bincount(labels,minlength=n)

def localize_surfaces(detections,depth,k,transform,config=None,*,registration_quality=None,common_covariance_m2=None):
    rows,supports,_=localize_surfaces_with_projection(detections,depth,k,transform,config,
        registration_quality=registration_quality,common_covariance_m2=common_covariance_m2)
    return rows,supports

def localize_surfaces_with_projection(detections,depth,k,transform,config=None,*,registration_quality=None,common_covariance_m2=None):
    """Return the surface rows, their support masks, and the shared lazy map projection."""
    cfg=config or DEFAULT_SURFACE_CONFIG;rows=[];supports=[]
    thresholds=asdict(cfg)
    if common_covariance_m2 is None:
        variance=cfg.pose_noise_floor_m**2
        if not np.isfinite(variance):raise ValueError('Invalid map-frame common covariance')
        common=np.eye(3)*variance
    else:
        common=np.asarray(common_covariance_m2,float)
        if common.shape!=(3,3) or not np.isfinite(common).all() or not np.allclose(common,common.T) or np.linalg.eigvalsh(common).min()<0:
            raise ValueError('Invalid map-frame common covariance')
    if depth is not None:depth=np.asarray(depth,float)
    world_points=None;valid_depth=None;invalid_projection=False
    for d in detections:
        mask=np.asarray(d.mask,bool);empty=np.zeros(mask.shape,bool);supports.append(empty)
        q=dict(method='mask_depth_component_v2_compact_surface',representative_point='median_retained_visible_surface_component_not_body_centre',thresholds=thresholds.copy(),mask_pixels=int(mask.sum()))
        q.update(covariance_calibrated=False,registration=dict(registration_quality or {}))
        row=dict(box=list(map(float,d.box)),confidence=float(d.confidence),localization_state='unlocalized',xyz=None,depth_reason=None,quality=q);rows.append(row)
        if depth is None or k is None or transform is None:row['depth_reason']='missing_registered_depth_or_capture_transform';continue
        if mask.ndim!=2 or mask.shape!=depth.shape:row['depth_reason']='mask_depth_shape_mismatch';continue
        if valid_depth is None:valid_depth=person_depth_pixels(depth)
        support=binary_erosion(mask,iterations=cfg.mask_border_px);valid=support&valid_depth
        valid_count=int(valid.sum())
        q['valid_depth_fraction']=float(valid_count/max(1,support.sum()))
        if valid_count<cfg.min_pixels or q['valid_depth_fraction']<cfg.min_valid_fraction:row['depth_reason']='insufficient_valid_component_support';continue
        labels,counts=depth_components(depth,valid,cfg.depth_jump_m)
        best=int(counts.argmax());fraction=float(counts[best]/valid_count)
        second=float(np.partition(counts,-2)[-2]/valid_count) if len(counts)>1 else 0.
        distance=distance_transform_edt(mask)
        valid_core=valid&(distance>=np.quantile(distance[support],.5))
        core_count=max(1,int(valid_core.sum()));selected=labels==best
        core_fraction=float((selected&valid_core).sum()/core_count)
        q.update(depth_component_count=len(counts),component_fraction=fraction,second_component_fraction=second,core_fraction=core_fraction,depth_pixels=int(counts[best]),raw_depth_span_m=float(np.ptp(np.quantile(depth[valid],[.05,.95]))))
        if counts[best]<cfg.min_pixels or fraction<cfg.min_component_fraction or fraction-second<cfg.min_dominance_gap or core_fraction<cfg.min_core_fraction:row['depth_reason']='ambiguous_depth_components';continue
        values=depth[selected];lo,hi=np.quantile(values,[.05,.95])
        q['component_input_depth_span_m']=float(hi-lo)
        # A continuous person surface can span much more depth than a single
        # representative patch. Retain a dense compact patch, never widen the
        # existing span gate or include unsupported/background pixels.
        if hi-lo>cfg.max_span_m:
            ordered=np.sort(values);ends=np.searchsorted(ordered,ordered+cfg.max_span_m,side='right')
            begin=int(np.argmax(ends-np.arange(len(ordered))));low=float(ordered[begin])
            compact=selected&(depth>=low)&(depth<=low+cfg.max_span_m)
            compact_count=int(compact.sum())
            compact_fraction=float(compact_count/counts[best])
            final_fraction=float(compact_count/valid_count)
            final_core=float((compact&valid_core).sum()/core_count)
            q.update(compact_surface_fraction=compact_fraction,compact_valid_fraction=final_fraction,
                     compact_core_fraction=final_core)
            if (compact_fraction<cfg.compact_surface_min_fraction or compact_count<cfg.min_pixels or
                final_fraction<cfg.min_component_fraction or final_core<cfg.min_core_fraction):
                row['depth_reason']='component_depth_span_rejected';continue
            selected=compact;lo,hi=np.quantile(depth[selected],[.05,.95])
            if hi-lo>cfg.max_span_m:
                q['depth_span_m']=float(hi-lo)
                row['depth_reason']='component_depth_span_rejected';continue
        q['depth_span_m']=float(hi-lo)
        retained=selected&(depth>=lo)&(depth<=hi)
        retained_count=int(retained.sum())
        if retained_count<cfg.min_pixels:row['depth_reason']='insufficient_retained_support';continue
        if world_points is None and not invalid_projection:
            try:world_points=cloud(depth,k,transform)
            except ValueError:invalid_projection=True
        if invalid_projection:
            row['depth_reason']='invalid_calibration_or_transform';continue
        points=world_points[retained]
        extent=np.ptp(np.quantile(points,[.05,.95],axis=0),axis=0);q['extent_m']=extent.tolist()
        centered=points-points.mean(axis=0)
        _,axes=np.linalg.eigh(centered.T@centered/max(1,len(points)-1))
        principal=np.sort(np.ptp(np.quantile(centered@axes,[.05,.95],axis=0),axis=0))
        diameter=float(2*np.linalg.norm(centered,axis=1).max())
        q.update(principal_extents_m=principal.tolist(),enclosing_diameter_m=diameter)
        if principal[-1]<cfg.min_major_extent_m or principal[-2]<cfg.min_second_extent_m or diameter>cfg.max_patch_diameter_m:row['depth_reason']='component_extent_rejected';continue
        xyz=np.median(points,axis=0);mad=1.4826*np.median(np.abs(points-xyz),axis=0)
        rounding=float(q['registration'].get('max_projection_quantization_m',0.))
        measurement=np.diag(mad**2/min(25,len(points))+rounding**2)
        covariance=measurement+common
        q.update(retained_pixels=retained_count,covariance_m2=covariance.tolist(),common_covariance_m2=common.tolist(),measurement_proxy_covariance_m2=measurement.tolist(),surface_spread_m=mad.tolist(),uncertainty_scope='modelled representative measurement and common pose error; surface spread reported separately; not calibrated body-centre error',valid=True)
        row.update(localization_state='localized_surface',xyz=xyz.tolist(),depth_reason=None);supports[-1]=retained
    return rows,np.array(supports,bool),world_points


def has_surface_measurement(row):
    return row is not None and row.get('localization_state') == 'localized_surface' and row.get('quality', {}).get('valid')


class SurfaceFusion:
    """Same representation only, bounded in time; rejection clears current XYZ."""
    def __init__(self):
        self.samples=[];self.current=None;self.last_reliable=None;self.state='unknown';self.reasons=['no_qualified_surface'];self.last_capture=-1.
    def observe(self,row,capture,processing,association_reliable=True):
        if capture<=self.last_capture:return
        self.last_capture=capture;self.current=None
        if not association_reliable:self.reject('identity_association_uncertain');return
        if not has_surface_measurement(row):
            reason=(row or {}).get('depth_reason') or 'missing_registered_surface'
            # Too broad to obtain a qualified patch is an absent measurement,
            # not a contradictory 3D point. Current XYZ still clears. A later
            # valid patch must satisfy all original count/span/window/dispersion gates.
            self.reject(reason,preserve_qualified=reason=='component_depth_span_rejected');return
        point=np.asarray(row['xyz'],float);cov=np.asarray(row['quality']['covariance_m2'],float)
        self.samples=[s for s in self.samples if capture-s[0]<=2.]
        self.samples.append((capture,point,cov))
        values=np.array([s[1] for s in self.samples]);median=np.median(values,axis=0);deviation=float(np.max(np.linalg.norm(values-median,axis=1)))
        if deviation>.25:self.reject('surface_sequence_dispersion');return
        if len(self.samples)<3 or capture-self.samples[0][0]<.8-1e-9:self.state='unreliable';self.reasons=['insufficient_qualified_surface_sequence'];return
        centered=values-median
        scatter=centered.T@centered/(len(values)-1);covariance=scatter+sum(s[2] for s in self.samples)/len(self.samples)
        quality=dict(row['quality'],qualified_frames=len(values),span_s=capture-self.samples[0][0],dispersion_m=deviation,covariance_calibrated=False,uncertainty_scope='visible-surface disagreement plus modelled pose floor; not body-centre bound')
        self.current=dict(xyz=median.tolist(),capture_time_s=capture,first_support_time_s=self.samples[0][0],processing_time_s=processing,frame='map',representative_point='median_retained_visible_surface_component_not_body_centre',covariance_m2=covariance.tolist(),covariance_calibrated=False,quality=quality,method='robust_surface_sequence_v1')
        self.last_reliable=dict(self.current,history_only=True);self.state='reliable';self.reasons=[]
    def reject(self,reason,preserve_qualified=False):
        # A stale processing result is not a contradictory surface measurement.
        # Never publish it; keep only previously qualified samples under the
        # unchanged capture-time window for a later fresh measurement.
        self.current=None
        if not preserve_qualified:self.samples=[]
        self.state='unknown' if any(k in reason for k in ('missing','no_','uncertain')) else 'unreliable';self.reasons=[reason]
    def expire(self,now):
        if self.current and not 0<=now-self.last_capture<=2.:self.current=None;self.samples=[];self.state='expired';self.reasons=['surface_capture_expired']
