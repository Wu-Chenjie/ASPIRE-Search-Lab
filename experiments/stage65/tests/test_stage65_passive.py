"""No simulation. Current-state serialization, bounds and original-call identity."""
import json,time
from types import SimpleNamespace as N
import numpy as np
import pytest
from annual_runtime.trigger_evidence import EvidenceRing
from core.exploration.voxel_mapping import VoxelMap
from core.planning.execution_safety import NativeSafetyMap
import stage65_capture as c

def node():
    m=VoxelMap([[-3,-4,0],[7,4,4]],flight_limits=(.7,3.));n=NativeSafetyMap()
    return N(id=0,map=m,safety_replica=N(map=m),replica=N(map=m),native_safety=n,
        get_clock=lambda:N(now=lambda:N(nanoseconds=1000000000)),curve=None,token=None,
        positions={},pose=None,safety_stamp=-np.inf)
def trigger(ring,ident='test',**extra):
    return json.dumps(dict(schema=1,kind='map_evidence_trigger',id=ident,drone=0,
          session=ring.session,sim_s=1.,reason='bounded_startup_capture_probe',**extra))
def read(ring):
    out=[]
    for p in sorted(ring.folder.glob('*.npz')):
        with np.load(p,allow_pickle=False) as z:out.append((json.loads(bytes(z['metadata_json']).decode()),{k:z[k].copy() for k in z.files if k!='metadata_json'}))
    return out

@pytest.mark.parametrize('role',['mapper','executor','gateway'])
def test_empty_unknown_map_probe_needs_no_curve_and_gives_no_query_credit(tmp_path,role):
    x=node();r=EvidenceRing(tmp_path,0,role,post_records=4);before=x.map.state.copy()
    c.receive(x,r,role,trigger(r));c.receive(x,r,role,trigger(r));r.close()
    rows=read(r);assert len(rows)==1
    meta,arrays=rows[0];m=meta['record'];assert m['kind']=='probe_current_state' and not m['policy_query_performed']
    assert m['curve_present'] is False and m['native_state_available'] is False
    assert 'native_state' not in arrays and (arrays['mapper_state']==-1).all()
    np.testing.assert_array_equal(before,x.map.state)
    assert r.windows[0]['post_captured']==0 and meta['phase']=='probe'
    assert r.records==r.written==1 and r.write_errors==r.queue_dropped==0

def test_passive_probe_does_not_spend_real_post_check_credit(tmp_path):
    x=node();r=EvidenceRing(tmp_path,0,'executor',post_records=4)
    c.receive(x,r,'executor',trigger(r))
    for t in range(4):r.record('safety_query',t+2,{'policy_query_performed':True},{'a':np.array([t])})
    r.close();rows=read(r);assert len(rows)==5
    assert r.windows[0]['post_captured']==4
    assert sum(z[0]['record']['kind']=='safety_query' for z in rows)==4

def test_probe_rejects_wrong_session_and_preserves_original_exception(tmp_path):
    x=node();r=EvidenceRing(tmp_path,0,'executor');bad=json.loads(trigger(r));bad['session']='another'
    c.receive(x,r,'executor',json.dumps(bad));assert r.records==0 and not r.windows;r.close()
    calls=[]
    class Agent:
        def __init__(self):self.map_evidence=None
        def reject(self,row,reason):calls.append((row,reason));raise RuntimeError('original rejection exception')
        def destroy_node(self):return 17
    c.install_gateway(Agent);a=object.__new__(Agent);a.map_evidence=None
    with pytest.raises(RuntimeError,match='original rejection exception'):a.reject({'traj_id':1},'actual reason')
    assert calls==[({'traj_id':1},'actual reason')] and a.destroy_node()==17

def test_first_three_rejections_keep_exact_proposal_and_actual_maps_under_caps(tmp_path):
    x=node();r=EvidenceRing(tmp_path,0,'gateway',pre_records=1,post_records=0);x.map_evidence=r
    original=x.map.state.copy()
    for i in range(4):c.capture_rejection(x,{'traj_id':i,'points':[[1.,2.,3.]]},'no known free prefix')
    r.close();rows=read(r);assert len(rows)==3 and r.records==3
    assert [z[0]['record']['rejected_proposal']['traj_id'] for z in rows]==[0,1,2]
    assert all(not z[0]['record']['policy_query_performed'] and not z[0]['record']['native_state_available'] for z in rows)
    np.testing.assert_array_equal(original,x.map.state)
    assert r.write_errors==r.queue_dropped==r.disk_dropped==r.oversize==0
    assert r.memory_peak<=r.memory_limit and r.disk_bytes<=r.disk_limit

def test_oversize_probe_is_explicit_error_not_fabricated_or_unbounded(tmp_path):
    x=node();r=EvidenceRing(tmp_path,0,'executor',max_record_bytes=256)
    c.receive(x,r,'executor',trigger(r));r.close()
    assert r.oversize==1 and r.written==0 and not list(r.folder.glob('*.npz'))


def test_actual_native_state_codes_preserved_without_unknown_to_free_conversion(tmp_path):
    x=node();m=x.map;state=np.resize(np.array([0,1,2,4,5,6],np.uint8),m.shape)
    x.native_safety.ingest([1,1,1,m.resolution,*m.origin,*m.shape,*state.ravel()])
    r=EvidenceRing(tmp_path,0,'executor',post_records=4)
    c.receive(x,r,'executor',trigger(r));r.close();meta,arrays=read(r)[0]
    assert meta['record']['native_state_available'] is True and not meta['record']['policy_query_performed']
    np.testing.assert_array_equal(arrays['native_state'],state);assert (arrays['mapper_state']==-1).all()
    np.testing.assert_array_equal(x.native_safety.state,state)


def test_actual_reservation_geometry_is_copied_without_changing_policy_state(tmp_path):
    from scipy.spatial import cKDTree
    x=node();points=np.array([[1.,2.,3.],[2.,3.,4.]])
    x.map.reserved_points=cKDTree(points);x.map.reserved_radius=.8
    r=EvidenceRing(tmp_path,0,'executor');c.receive(x,r,'executor',trigger(r));r.close()
    meta,arrays=read(r)[0];a=meta['record']
    assert a['reserved_context_present'] and a['mapper_reserved_radius']==.8 and a['mapper_cache_matches_state']
    np.testing.assert_array_equal(arrays['mapper_reserved_points'],points)
    np.testing.assert_array_equal(x.map.reserved_points.data,points)
