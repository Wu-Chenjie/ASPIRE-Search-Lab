"""Optional trace segmentation → independent memory → search hints.

No hint confirms a person or bypasses native reachability/review authorization.
Only measured support is associated; ambiguous observations remain separate.
"""
from collections import Counter
import numpy as np
from .trace_contracts import SurfaceEstimate
from .trace_memory import TraceMemory,TraceObservation,TraceType,TraceDirection,DirectionState
from .surface import localize_surfaces_with_projection,SurfaceConfig

_SURFACE_CONFIG=SurfaceConfig(min_major_extent_m=.05)
_UNKNOWN_DIRECTION=TraceDirection()
_NS_PER_SECOND=1_000_000_000

class TracePipeline:
    def __init__(self,session,frame_version,clock_domain='mission',max_traces=256,ttl_s=2.):
        if not session or not frame_version or not clock_domain:raise ValueError('Trace provenance required')
        self.session=session;self.frame_version=frame_version;self.clock_domain=clock_domain
        self.max_traces=max_traces;self.ttl_s=ttl_s;self.memory=TraceMemory();self.next_id=1
        self.last_capture_ns=-1

    def observe(self,detections,depth,k,transform,stamp_ns,now=None,registration_quality=None,*,now_ns=None):
        """Gate by capture age, using exact input clock ticks when supplied."""
        if stamp_ns<=self.last_capture_ns:return []  # no duplicate inference/fusion
        capture=stamp_ns/_NS_PER_SECOND
        age=now-capture if now_ns is None else (now_ns-stamp_ns)/_NS_PER_SECOND
        if not 0<=age<=self.ttl_s:return []
        rows,supports,world_points=localize_surfaces_with_projection(detections,depth,k,transform,
            _SURFACE_CONFIG,registration_quality=registration_quality)
        self.last_capture_ns=stamp_ns
        try:
            trace_types=[TraceType(d.semantic_class) for d in detections]
        except ValueError as exc:
            raise ValueError('Unsupported trace label') from exc
        choices={};competitors=Counter()
        for i,(trace_type,row) in enumerate(zip(trace_types,rows)):
            if row['xyz'] is None:continue
            xyz=np.asarray(row['xyz'])
            for ident,record in self.memory._records.items():
                previous=record.last_reliable_support
                if previous and record.trace_type==trace_type and np.linalg.norm(xyz-previous.xyz)<.2:
                    choices.setdefault(i,[]).append(ident);competitors[ident]+=1
        hints=[]
        for i,(d,trace_type,row) in enumerate(zip(detections,trace_types,rows)):
            matches=choices.get(i,[])
            unique=len(matches)==1 and competitors[matches[0]]==1
            if row['xyz'] is None or (not unique and len(self.memory._records)>=self.max_traces):
                reason=row['depth_reason'] or 'trace_memory_capacity'
                hints.append(dict(entity_kind='search_trace',trace_type=d.semantic_class,semantic_confidence=d.confidence,
                                  current_support=None,reason=reason,contributes_to_person_count=False))
                continue
            if unique:ident=matches[0]
            else:
                ident=f'{self.session}/trace/{self.next_id}';self.next_id+=1
            xyz=row['xyz'];support_points=world_points[supports[i]]
            planar=support_points[:,:2]-support_points[:,:2].mean(axis=0)
            values,axes=np.linalg.eigh(planar.T@planar/max(1,len(planar)-1))
            direction=_UNKNOWN_DIRECTION
            if values[-1]>2*max(values[0],1e-9):
                direction=TraceDirection(DirectionState.AXIAL,tuple(axes[:,-1]),float(1-values[0]/values[-1]))
            q=row['quality']
            observation=TraceObservation(f'{self.session}/rgb/{stamp_ns}/{i}',ident,capture,trace_type,
                float(d.confidence),direction,SurfaceEstimate(tuple(xyz),capture,'map',q['component_fraction'],q['retained_pixels']))
            record=self.memory.observe(observation)
            hints.append(dict(entity_kind='search_trace',trace_id=ident,trace_type=d.semantic_class,
                evidence_version=record.observation_count,observation_id=observation.observation_id,
                frame='map',frame_version=self.frame_version,clock_domain=self.clock_domain,
                capture_time_s=capture,expires_at_s=capture+self.ttl_s,
                semantic_confidence=record.semantic_confidence,current_support=xyz,
                support_extent_m=np.ptp(support_points,axis=0).tolist(),quality=q,
                direction_state=direction.state.value,possible_directions=direction.possible_directions(),
                formation_time=None,contributes_to_person_count=False,review_purpose='verify_trace',
                association_state='unique_support' if unique else 'unresolved',
                requires_native_reachability_and_review_budget=True))
        return hints
