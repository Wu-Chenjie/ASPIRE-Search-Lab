"""Newest valid RGB wins; superseded backlog is retired and counted, not queued."""
import sys
import threading
from collections import OrderedDict
from types import SimpleNamespace as N
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
import pytest
from person_search.node import RgbReceipt, claim_rgb, enqueue_rgb
from person_search.presence_adapter import discard_superseded_rgb,select_presence_capture


def node(stamps,period_ns=0,observed=( )):
    tracker=N(last_rgb_ns=-1,frames=OrderedDict((ns,[]) for ns in observed),tracks=[])
    return N(tracker=tracker,registry_mode=True,rgbd_registration_mode='strict',rgbd_max_skew_s=0.,
        require_pose=False,max_age=1.5,last_ns=-1,min_rgb_period_ns=period_ns,processed=0,rejected=0,
        input_lock=threading.RLock(),rgb_receipts={ns:RgbReceipt(0.) for ns in stamps},input_queue_limit=40,
        rgb_skipped_total=0,rgb_ingest_dropped_total=0,
        queues={k:{ns:object() for ns in stamps} for k in ('rgb','depth','ri','di')})


def test_newest_capture_wins_and_older_backlog_is_retired():
    n=node((100,200,300))
    selected,geo,_=select_presence_capture(n,{k:q.copy() for k,q in n.queues.items()},None,350)
    assert selected==300 and geo==300
    assert discard_superseded_rgb(n,selected)==2
    assert list(n.queues['rgb'])==[300]
    assert n.rgb_skipped_total==2
    assert list(n.rgb_receipts)==[300]


def test_sampling_period_still_supersedes_frames_it_was_allowed_to_skip():
    n=node((100_000_000,200_000_000,300_000_000),period_ns=200_000_000)
    n.tracker.last_rgb_ns=0
    selected,_,_=select_presence_capture(n,{k:q.copy() for k,q in n.queues.items()},None,350_000_000)
    assert selected==300_000_000
    assert discard_superseded_rgb(n,selected)==2
    assert list(n.queues['rgb'])==[300_000_000]


def test_observed_captures_survive_because_their_depth_pass_may_be_pending():
    n=node((100,200,300),observed=(200,))
    assert discard_superseded_rgb(n,300)==1
    assert sorted(n.queues['rgb'])==[200,300]
    assert n.rgb_skipped_total==1


def test_backlog_is_bounded_by_the_newest_frame_not_by_the_queue_limit():
    n=node(range(1,41))
    assert discard_superseded_rgb(n,40)==39
    assert len(n.queues['rgb'])==1 and n.rgb_skipped_total==39


def test_queue_overflow_counts_only_frames_still_waiting_for_processing():
    n = node(())
    images = [object() for _ in range(60)]
    for ns, image in enumerate(images):
        enqueue_rgb(n, ns, image)
        claim_rgb(n, ns, image)
    assert len(n.queues['rgb']) == 40
    assert n.rgb_ingest_dropped_total == 0

    for ns in range(60, 101):
        enqueue_rgb(n, ns, object())
    assert n.rgb_ingest_dropped_total == 1
    with pytest.raises(ValueError):
        claim_rgb(n, 0, images[0])


def test_first_publication_keeps_receipt_across_eviction_and_republication(monkeypatch):
    n = node(())
    n.input_queue_limit = 2
    monkeypatch.setattr('person_search.node.time.monotonic', lambda: 10.)
    image = object()
    enqueue_rgb(n, 0, image)
    receipt = claim_rgb(n, 0, image)
    for ns in (1, 2):
        enqueue_rgb(n, ns, object())
    assert 0 not in n.rgb_receipts
    monkeypatch.setattr('person_search.node.time.monotonic', lambda: 10.25)
    first = receipt.publication_timing()
    monkeypatch.setattr('person_search.node.time.monotonic', lambda: 11.)
    assert receipt.publication_timing() == first == dict(received_wall_s=10., first_semantic_latency_wall_s=.25)
