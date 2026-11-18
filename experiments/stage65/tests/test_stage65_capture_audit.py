"""Recording schema fixtures only; never claimed as dynamic evidence."""
import gzip,json
from types import SimpleNamespace as N
import numpy as np
from core.exploration.voxel_mapping import VoxelMap
from core.planning.execution_safety import NativeSafetyMap
from annual_runtime.trigger_evidence import EvidenceRing
import stage65_capture as c
from stage65_capture_audit import validate
SESSION='explicit-unit-fixture'
def fixture(root,monkeypatch,exercised=False,queries=0,dropped=0):
    monkeypatch.setenv('RACER_OBSERVATION_SESSION',SESSION)
    for role in ['mapper','executor','gateway']:
        m=VoxelMap([[-3,-4,0],[7,4,4]],flight_limits=(.7,3.))
        node=N(id=0,map=m,replica=N(map=m),safety_replica=N(map=m),native_safety=NativeSafetyMap(),
               get_clock=lambda:N(now=lambda:N(nanoseconds=1000000000)),curve=None)
        ring=EvidenceRing(root/'map-evidence',0,role,post_records=2 if role=='mapper' else 4)
        data=json.dumps(dict(schema=1,kind='map_evidence_trigger',id='stage65-startup-probe',drone=0,
             session=SESSION,sim_s=1.,reason='bounded_startup_capture_probe'))
        c.receive(node,ring,role,data)
        if role=='mapper':
            for t in range(2):ring.record('mapper_consumed_input',2+t,{'fixture_only':True},{'a':np.zeros(1)})
        if role=='executor':
            for t in range(queries):ring.record('safety_query',2+t,{'fixture_only':True},{'a':np.zeros(1)})
        ring.close()
    (root/'startup-capture-probe.json').write_text(json.dumps(dict(subscriptions=3,trigger=dict(session=SESSION),diagnostic_only=True)))
    d=root/'timing';d.mkdir();ticks=[dict(session=SESSION,event='tick_boundary',event_seq=i+1,sim_ns=(i+1)*10**9,token='fixture' if exercised else None,native_id=1 if exercised else None,curve_duration=1. if exercised else None,safety_fault=None) for i in range(2)]
    with gzip.open(d/'drone-0-executor-observation.jsonl.gz','wt') as f:
        for t in ticks:f.write(json.dumps(t)+'\n')
    (d/'drone-0-executor-observation.status.json').write_text(json.dumps(dict(written=2,enqueued=2,dropped=dropped,write_errors=0,writer_alive=False,pending=0)))
    d=root/'flight/0';d.mkdir(parents=True);(d/'events.jsonl').write_text(json.dumps(dict(type='native_trajectory_activated'))+'\n' if exercised else '')

def test_no_execution_is_explicit_unexercised_not_a_safety_success(tmp_path,monkeypatch):
    fixture(tmp_path,monkeypatch);r=validate(tmp_path,SESSION)
    assert r['passed'] and r['execution_evidence_class']=='no_authorized_trajectory_policy_not_exercised'
    assert r['no_full_flight_safety_certification'] and not r['legacy_2mapper_4executor_counts_satisfied']

def test_actual_execution_still_requires_four_real_query_records(tmp_path,monkeypatch):
    fixture(tmp_path,monkeypatch,exercised=True,queries=0);r=validate(tmp_path,SESSION)
    assert not r['passed'] and r['policy_exercised'] and 'four actual post-trigger execution safety records when exercised' in r['failures']

def test_exercised_branch_with_required_typed_records(tmp_path,monkeypatch):
    fixture(tmp_path,monkeypatch,exercised=True,queries=4);r=validate(tmp_path,SESSION)
    assert r['passed'] and r['legacy_2mapper_4executor_counts_satisfied']

def test_missing_journal_cannot_be_used_as_no_motion_evidence(tmp_path,monkeypatch):
    fixture(tmp_path,monkeypatch,dropped=1);r=validate(tmp_path,SESSION)
    assert not r['passed'] and not r['journal_complete']
