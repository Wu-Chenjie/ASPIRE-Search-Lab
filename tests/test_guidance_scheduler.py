import json
import subprocess
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from concurrent.futures import ThreadPoolExecutor
import pytest
from person_search.guidance_scheduler import GuidanceScheduler,Reason


def test_long_canonical_id_and_disabled_audit_do_not_block_scheduler():
    scheduler = GuidanceScheduler(max_entities=1, audit_capacity=0)
    entity = 'registry/mission/' + 'long-component-' * 30
    assert scheduler.request(entity, 1, Reason.ELIGIBLE, 0.) == 'accepted'
    assert scheduler.request('other', 1, Reason.ELIGIBLE, 0.) == 'full'
    work = scheduler.start(0.)
    assert work.request.entity == entity and scheduler.finish(work, 0.)
    state = scheduler.snapshot()
    assert state['registered_ids'] == 1 and state['audit'] == ()
    assert state['audit_dropped'] == 4


def test_microbenchmark_audit_drops_are_per_workload(tmp_path):
    output = tmp_path/'report.json'
    benchmark = Path(__file__).resolve().parents[1]/'tools/search/benchmark_guidance.py'
    subprocess.run([sys.executable, str(benchmark), '--operations', '200', '--output', str(output)],
                   check=True)
    rows = json.loads(output.read_text())['rows']
    assert [row['audit_dropped'] for row in rows] == [0, 0, 0, 0, 8, 8]


def test_short_eligibility_survives_latest_snapshot_and_duplicates():
    s=GuidanceScheduler()
    assert s.request('A',1,Reason.ELIGIBLE,1.)=='accepted'
    assert s.request('A',1,Reason.ELIGIBLE,1.)=='duplicate'
    assert s.request('A',2,Reason.VISIBILITY,1.1)=='coalesced'
    pending=s.snapshot()['pending']
    assert len(pending)==1 and pending[0].eligible_generation is not None
    work=s.start(1.2)
    assert work.request.first_requested_s==1.
    assert work.request.reasons==Reason.ELIGIBLE|Reason.VISIBILITY
    assert s.finish(work,1.3)
    assert s.request('A',2,Reason.VISIBILITY,1.4)=='duplicate'
    assert s.start(1.4) is None


def test_concurrent_duplicate_burst_starts_only_one_computation():
    s=GuidanceScheduler(max_inflight=4,audit_capacity=8)
    with ThreadPoolExecutor(max_workers=8) as pool:
        results=list(pool.map(lambda _:s.request('A',1,Reason.ELIGIBLE,1.),range(400)))
        jobs=list(pool.map(lambda _:s.start(1.),range(40)))
    assert results.count('accepted')==1 and results.count('duplicate')==399
    assert sum(j is not None for j in jobs)==1
    assert len(s.snapshot()['active'])==1
    assert s.snapshot()['audit_dropped']>0
    assert len(s.snapshot()['audit'])==8


def test_global_limit_and_same_entity_limit_and_stale_result():
    s=GuidanceScheduler(max_inflight=2)
    s.request('A',1,Reason.ELIGIBLE,1.);a=s.start(1.)
    s.request('A',2,Reason.MAP,2.)
    assert s.start(2.) is None
    s.request('B',1,Reason.REACQUIRE,2.);b=s.start(2.)
    s.request('C',1,Reason.ELIGIBLE,2.)
    assert s.start(2.) is None
    assert not s.finish(a,3.)
    a2=s.start(3.)
    assert a2.request.entity=='A' and a2.request.reasons==Reason.MAP
    assert s.finish(b,3.)
    assert s.start(3.).request.entity=='C'
    assert not s.finish(a,3.)  # double completion cannot release a2's resource
    assert len(s.snapshot()['active'])==2


def test_alias_merge_preserves_eligibility_age_and_revision_watermarks():
    s=GuidanceScheduler()
    s.request('A',7,Reason.ELIGIBLE,1.)
    s.request('B',1,Reason.VISIBILITY,2.)
    s.alias('A','B',3.)
    pending=s.snapshot()['pending']
    assert len(pending)==1 and pending[0].first_requested_s==1.
    assert pending[0].eligible_generation is not None
    assert pending[0].reasons&Reason.ALIAS
    assert s.request('A',7,Reason.MAP,3.)=='duplicate'
    assert s.request('A',8,Reason.MAP,3.)=='coalesced'
    assert s.start(3.).request.entity=='B'


def test_alias_inflight_keeps_resources_until_completion():
    s=GuidanceScheduler(max_inflight=2)
    s.request('A',1,Reason.ELIGIBLE,1.);a=s.start(1.)
    s.request('B',1,Reason.ELIGIBLE,1.);b=s.start(1.)
    s.alias('A','B',2.)
    s.request('B',2,Reason.ALIAS,2.)
    assert s.start(2.) is None
    assert not s.finish(a,3.) and s.start(3.) is None
    assert not s.finish(b,3.)
    assert s.start(3.).request.entity=='B'


def test_alias_of_running_only_work_requeues_the_original_qualification():
    s=GuidanceScheduler()
    s.request('A',1,Reason.ELIGIBLE,1.);a=s.start(1.)
    s.request('B',1,Reason.MAP,1.);b=s.snapshot()['pending'][0]
    s.alias('A','B',2.)
    assert not s.finish(a,2.)
    work=s.start(2.)
    assert work.request.reasons&Reason.ELIGIBLE
    assert work.request.eligible_generation is not None


def test_queue_overflow_cannot_let_an_invalidated_computation_commit():
    s=GuidanceScheduler(max_pending=1)
    s.request('A',1,Reason.ELIGIBLE,1.);a=s.start(1.)
    s.request('B',1,Reason.ELIGIBLE,1.)
    assert s.request('A',2,Reason.CONTRADICTION,2.)=='full'
    assert not s.finish(a,2.)
    b=s.start(2.)
    assert s.request('A',2,Reason.CONTRADICTION,2.)=='accepted'
    assert s.finish(b,2.)


def test_confirmed_entity_never_spends_more_active_search_and_keeps_id():
    s=GuidanceScheduler()
    s.request('A',1,Reason.ELIGIBLE,1.);a=s.start(1.)
    s.request('A',2,Reason.MAP,2.)
    assert s.confirm('A',2.)
    assert s.snapshot()['pending']==()
    assert not s.finish(a,3.)
    assert s.request('A',3,Reason.REACQUIRE,4.)=='confirmed'
    assert s.start(4.) is None and s.snapshot()['entities']==1


def test_capacity_failure_is_visible_and_retry_is_possible_after_drain():
    s=GuidanceScheduler(max_pending=1,max_entities=3)
    s.request('A',1,Reason.ELIGIBLE,1.)
    assert s.request('B',1,Reason.ELIGIBLE,1.)=='full'
    a=s.start(1.);assert s.finish(a,1.)
    assert s.request('B',1,Reason.ELIGIBLE,1.)=='accepted'
    assert s.request('C',1,Reason.ELIGIBLE,1.)=='full'
    assert s.request('D',1,Reason.ELIGIBLE,1.)=='full'
    assert s.snapshot()['registered_ids']==3
    assert any(x[0]=='queue_capacity' for x in s.snapshot()['audit'])
    assert any(x[0]=='entity_capacity' for x in s.snapshot()['audit'])


def test_map_or_ledger_change_rejects_result_without_motion_authorization():
    s=GuidanceScheduler()
    s.request('A',1,Reason.ELIGIBLE,1.)
    assert not s.finish(s.start(1.),1.,map_valid=False)
    s.request('A',2,Reason.MAP,2.)
    assert not s.finish(s.start(2.),2.,ledger_valid=False)


@pytest.mark.parametrize('now',[float('nan'),float('inf'),-.1,.5])
def test_clock_reset_requires_new_run_scheduler_not_negative_age(now):
    s=GuidanceScheduler();s.request('A',1,Reason.ELIGIBLE,1.)
    with pytest.raises(ValueError):
        s.start(now)
    assert len(s.snapshot()['pending'])==1


@pytest.mark.parametrize('source,target', [('active', 'confirmed'), ('confirmed', 'active')])
def test_alias_with_confirmed_entity_invalidates_exact_active_work(source, target):
    scheduler = GuidanceScheduler()
    assert scheduler.confirm('confirmed', 0.)
    assert scheduler.request('active', 1, Reason.ELIGIBLE, 1.) == 'accepted'
    work = scheduler.start(1.)
    assert scheduler.request('active', 2, Reason.MAP, 2.) == 'accepted'
    scheduler.alias(source, target, 2.)
    assert scheduler.snapshot()['active'] == (work,)
    assert not scheduler.finish(work, 2.)
    assert scheduler.snapshot()['active'] == ()
    assert scheduler.snapshot()['pending'] == ()
    assert scheduler.request('active', 3, Reason.REACQUIRE, 3.) == 'confirmed'
