from dataclasses import replace
import json
import sys
from pathlib import Path
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from person_search.detector_spec import DetectorSpec
from person_search.trace_memory import TraceMemory,TraceObservation,TraceType,TraceDirection,DirectionState,FormationInterval
from person_search.training_data import occlude_visible_masks,audit_annotations

def test_manifest_fails_closed_on_hash_runtime_pixel_and_engine_contracts(tmp_path,monkeypatch):
    import hashlib
    p=tmp_path/'custom.pt';p.write_bytes(b'custom');digest=hashlib.sha256(p.read_bytes()).hexdigest()
    monkeypatch.setattr('person_search.detector_spec.importlib.metadata.version',lambda _: '8.3.228')
    spec=DetectorSpec(model_sha256=digest,class_map={'7':'person'})
    assert spec.verify(p)==p and spec.class_map=={7:'person'}
    alternate=DetectorSpec(model_sha256=digest.upper(),input_size=2080,runtime_versions={'ultralytics':'8.3.229'})
    with pytest.raises(ValueError,match='version mismatch'):alternate.verify(p)
    monkeypatch.setattr('person_search.detector_spec.importlib.metadata.version',lambda _: '8.3.229')
    assert alternate.verify(p)==p
    p.write_bytes(b'changed')
    with pytest.raises(ValueError,match='SHA256'):alternate.verify(p)
    for kw in ({'mask_output_contract':'amodal'},{'runtime_versions':{}},{'precision':'fp16'},{'backend':'tensorrt','device':'cuda:0'}):
        with pytest.raises(ValueError):DetectorSpec(model_sha256=digest,**kw)

def test_detector_normalizes_class_ids_once_and_requires_matching_engine_fields():
    sha='a'*64
    with pytest.raises(ValueError,match='Duplicate class IDs'):
        DetectorSpec(model_sha256=sha,class_map={7:'person','07':'person'})
    versions=dict(ultralytics='8.3.228',torch='2.0',tensorrt='10.0')
    engine=dict(cuda_version='12.3',compute_capability='8.7',precision='fp16',input_size=640,batch=1)
    fields=dict(model_sha256=sha,backend='tensorrt',device='cuda:0',precision='fp16',
                runtime_versions=versions)
    assert DetectorSpec(**fields,engine=engine).engine==engine
    for key in engine:
        with pytest.raises(ValueError):
            DetectorSpec(**fields,engine={name:value for name,value in engine.items() if name!=key})


def test_trace_duplicates_direction_and_formation_time_are_independent():
    memory=TraceMemory();o=TraceObservation('a','t',10,TraceType.FOOTPRINT,.8,
        TraceDirection(DirectionState.AXIAL,(-1,0),.7),formation_time=FormationInterval(1,5,'physical evidence'))
    first=memory.observe(o);assert memory.observe(o)==first and first.observation_count==1
    assert first.direction.possible_directions()==((1,0),(-1,0))
    later=memory.observe(replace(o,observation_id='b',observed_at=20,formation_time=None))
    assert later.formation_time==first.formation_time and later.physical_age_bounds_s(20)==(15,19)
    with pytest.raises(ValueError):memory.observe(replace(o,semantic_confidence=.9))
    with pytest.raises(ValueError):memory.observe(replace(o,observation_id='c',observed_at=21,formation_time=FormationInterval(11,15,'conflict')))
    assert not later.contributes_to_person_count

def test_occlusion_removes_visible_labels_and_capture_groups_cannot_leak():
    rgb=np.ones((10,10,3),np.uint8);mask=np.ones((1,10,10),bool);cover=np.zeros((10,10),bool);cover[:5]=True
    image,visible,kept=occlude_visible_masks(rgb,mask,cover)
    assert not visible[:,cover].any() and visible.sum()==50 and kept.tolist()==[0]
    row=dict(observation_id='a',capture_group='mission1',split='train',source='own',license='project',instances=[dict(**{'class':'person'},mask_type='visible',posture='lying')])
    assert audit_annotations([row])['records']==1
    with pytest.raises(ValueError,match='leaks'):audit_annotations([row,dict(row,observation_id='b',split='test')])


def test_annotation_audit_reports_each_bad_record():
    valid = dict(observation_id='a', capture_group='mission1', split='train',
                 source='own', license='project')
    with pytest.raises(ValueError) as invalid:
        audit_annotations([
            valid, dict(valid, observation_id='b', split='test'), None,
            dict(valid, observation_id='c', license=''),
            dict(valid, observation_id='d', instances=[{'class': 'person', 'mask_type': 'amodal'}]),
        ])
    assert 'records[1]: Capture group leaks' in str(invalid.value)
    assert 'records[2]:' in str(invalid.value)
    assert 'records[3]: Dataset provenance' in str(invalid.value)
    assert 'records[4]: Only visible masks' in str(invalid.value)


def test_trace_formation_clips_and_intersects_with_the_original_sources():
    memory = TraceMemory()
    first = TraceObservation('a', 't', 10, TraceType.FOOTPRINT, .8,
                             formation_time=FormationInterval(1, 9, 'prior'))
    memory.observe(first)
    newer = replace(first, observation_id='b', observed_at=20,
                    formation_time=FormationInterval(5, 15, 'fresh'))
    assert memory.observe(newer).formation_time == FormationInterval(5, 9, 'fresh | first sighting | prior')


def test_unrepresentable_trace_geometry_and_times_raise_contract_errors():
    from person_search.trace_contracts import SurfaceEstimate

    huge = 10**400
    with pytest.raises(ValueError, match='finite nonnegative'):
        SurfaceEstimate((1, 0, 0), huge, 'map', .5, 1)
    with pytest.raises(ValueError, match='finite coordinates'):
        SurfaceEstimate((huge, 0, 0), 1, 'map', .5, 1)
    with pytest.raises(ValueError, match='finite two-dimensional'):
        TraceDirection(DirectionState.AXIAL, (huge, 0), .5)

def test_trace_pipeline_emits_independent_axial_hint_without_invented_age():
    from person_search.trace_pipeline import TracePipeline
    from person_search.segmentation import PersonMask
    mask=np.zeros((120,160),bool);mask[40:80,40:120]=True
    d=PersonMask((40,40,80,40),.9,mask,semantic_class='footprint')
    pipe=TracePipeline('session','map1',max_traces=1);depth=np.full(mask.shape,2.);k=[[200,0,80],[0,200,60],[0,0,1]]
    hints=pipe.observe([d],depth,k,np.eye(4),1_000_000_000,1.)
    assert len(hints)==1 and not hints[0]['contributes_to_person_count'] and hints[0]['formation_time'] is None
    assert hints[0]['direction_state']=='axial' and len(hints[0]['possible_directions'])==2
    assert not pipe.observe([d],depth,k,np.eye(4),1_000_000_000,1.1)
    another_trace=replace(d,semantic_class='fallen_material')
    hints=pipe.observe([another_trace],depth,k,np.eye(4),1_200_000_000,1.21)
    assert len(hints)==1 and hints[0]['current_support'] is None
    assert hints[0]['reason']=='trace_memory_capacity' and 'trace_id' not in hints[0]


def test_trace_frame_shares_projection_and_preserves_unique_support(monkeypatch):
    from person_search import perception
    from person_search.segmentation import PersonMask
    from person_search.trace_pipeline import TracePipeline

    validated = []
    validate_transform = perception.validate_transform

    def count_projection(transform):
        validated.append(transform)
        return validate_transform(transform)

    monkeypatch.setattr(perception, 'validate_transform', count_projection)
    mask = np.zeros((120, 160), bool)
    mask[40:80, 40:120] = True
    trace = PersonMask((40, 40, 80, 40), .9, mask, semantic_class='footprint')
    another_type = replace(trace, semantic_class='fallen_material')
    pipe = TracePipeline('session', 'map1')
    depth = np.full(mask.shape, 2.)
    k = [[200, 0, 80], [0, 200, 60], [0, 0, 1]]
    transform = np.eye(4)

    first = pipe.observe([trace, another_type], depth, k, transform, 1_000_000_000, 1.)
    assert len(validated) == 1
    assert first[0]['trace_id'] != first[1]['trace_id']
    assert first[0]['current_support'] == first[1]['current_support']
    assert first[0]['support_extent_m'] == first[1]['support_extent_m']
    second = pipe.observe([trace], depth, k, transform, 2_000_000_000, 2.)
    assert len(validated) == 2
    assert second[0]['trace_id'] == first[0]['trace_id']
    assert second[0]['evidence_version'] == 2
    ambiguous = pipe.observe([trace, trace], depth, k, transform, 3_000_000_000, 3.)
    assert len(validated) == 3
    assert len({h['trace_id'] for h in ambiguous} | {first[0]['trace_id']}) == 3
    assert {h['association_state'] for h in ambiguous} == {'unresolved'}


@pytest.mark.parametrize('stamp_ns', [13_000_000_000, 1_700_000_000_000_000_000])
def test_trace_ttl_is_inclusive_at_exact_nanosecond_boundaries(stamp_ns):
    from person_search.segmentation import PersonMask
    from person_search.trace_pipeline import TracePipeline

    trace = PersonMask((0, 0, 1, 1), .9, np.ones((1, 1), bool), semantic_class='footprint')
    for elapsed_ns, accepted in ((-1, False), (0, True), (2_000_000_000, True), (2_000_000_001, False)):
        pipe = TracePipeline('session', 'map1')
        hints = pipe.observe([trace], None, None, None, stamp_ns, now_ns=stamp_ns + elapsed_ns)
        assert bool(hints) is accepted


def test_trace_seconds_api_accepts_an_equal_clock_and_rejects_nonfinite_values():
    from person_search.segmentation import PersonMask
    from person_search.trace_pipeline import TracePipeline

    trace = PersonMask((0, 0, 1, 1), .9, np.ones((1, 1), bool), semantic_class='footprint')
    pipe = TracePipeline('session', 'map1')
    for now in (float('nan'), float('inf'), float('-inf')):
        assert pipe.observe([trace], None, None, None, 15_000_000_000, now) == []
    assert pipe.observe([trace], None, None, None, 15_000_000_000, 15.)
