from tools.flight.demo_video_audit import timing_report


def test_capture_audit_distinguishes_encoding_receipt_and_source_rates():
    records=[dict(stamp_ns=i*40_000_000,receipt_monotonic_ns=i*100_000_000,
                  source_pixels_sha256='same-static-scene') for i in range(51)]
    r=timing_report(records,25)
    assert r['actual_capture_fps_sim']==25
    assert r['actual_receipt_fps_wall']==10
    assert r['consecutive_identical_pixel_messages']==50
    assert r['distinct_source_stamps']==51
    assert r['estimated_missing_slots']==0


def test_capture_audit_detects_gaps_and_duplicate_timestamps():
    r=timing_report([dict(stamp_ns=i) for i in [0,40_000_000,120_000_000,120_000_000]],25)
    assert not r['monotonic_source_stamps']
    assert r['estimated_missing_slots']==1
    assert r['actual_receipt_fps_wall'] is None
    assert r['unique_source_pixel_hashes'] is None


def test_overlay_asof_never_exposes_a_future_result():
    from tools.flight.demo_video import asof
    events=[dict(receipt_monotonic_ns=20,data={'stamp_ns':5}),
            dict(receipt_monotonic_ns=40,data={'stamp_ns':10})]
    assert asof(events,19) is None
    assert asof(events,20)['data']['stamp_ns']==5
    assert asof(events,39)['data']['stamp_ns']==5
    assert asof(events,40)['data']['stamp_ns']==10


def test_demo_selection_rejects_duplicate_source_stamps_and_noncontiguous_frames():
    import pytest
    from tools.flight.demo_video import select_frames
    with pytest.raises(ValueError,match='Non-increasing'):
        select_frames([dict(frame=0,stamp_ns=5),dict(frame=1,stamp_ns=5)])
    with pytest.raises(ValueError,match='not contiguous'):
        select_frames([dict(frame=0,stamp_ns=5),dict(frame=2,stamp_ns=10)])
    records=[dict(frame=i,stamp_ns=i*1_000_000_000) for i in range(5)]
    assert [r['frame'] for r in select_frames(records,1,3)]==[1,2,3]


def test_actual_encode_decode_keeps_one_rgb_frame_and_causal_overlay(tmp_path):
    import json
    import numpy as np
    import pytest
    cv2=pytest.importorskip('cv2')
    from tools.flight.demo_video import compose
    run=tmp_path/'run';run.mkdir();out=tmp_path/'demo'
    writer=cv2.VideoWriter(str(run/'continuous-rgb.avi'),cv2.VideoWriter_fourcc(*'MJPG'),30.,(320,240))
    if not writer.isOpened():pytest.skip('MJPEG encoder not installed')
    records=[]
    for i in range(12):
        frame=np.full((240,320,3),i*15,np.uint8);writer.write(frame)
        records.append(dict(frame=i,stamp_ns=i*33_333_333,receipt_monotonic_ns=1_000_000_000+i*50_000_000))
    writer.release()
    (run/'continuous-rgb-stamps.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    overlay=dict(kind='semantic',receipt_monotonic_ns=1_200_000_000,
                 data=dict(stamp_ns=33_333_333,candidates=[]))
    future=dict(kind='semantic',receipt_monotonic_ns=1_300_000_000,
                data=dict(stamp_ns=99_000_000_000,candidates=[]))
    (run/'continuous-overlays.jsonl').write_text(json.dumps(overlay)+'\n'+json.dumps(future)+'\n')
    (run/'telemetry.jsonl').write_text('')
    # Encode support differs between host and native ROS images. Skip only the
    # codec absence, never a failed timing/causality assertion.
    try:r=compose(run,out,expected_hz=30)
    except RuntimeError as exc:
        if 'H264 encoder unavailable' in str(exc):pytest.skip(str(exc))
        raise
    assert r['one_to_one_verified']
    assert r['frames']==12
    idx=[json.loads(s) for s in (out/'frame-index.jsonl').read_text().splitlines()]
    assert all(v['detection_source_stamp_ns'] is None for v in idx[:4])
    assert all(v['detection_source_stamp_ns']==33_333_333 for v in idx[4:])
    assert [v['source_frame'] for v in idx]==list(range(12))


def test_inference_rate_deduplicates_2d_and_localized_status_messages():
    from tools.flight.demo_video import first_result_receipts,unique_result_rate
    events=[dict(receipt_monotonic_ns=t*1_000_000_000,data=dict(stamp_ns=stamp))
            for t,stamp in [(1,100),(1.1,100),(2,200),(2.1,200),(7,100),(8,300)]]
    receipts=first_result_receipts(events)
    assert receipts==[1_000_000_000,2_000_000_000,8_000_000_000]
    assert unique_result_rate(receipts,3_000_000_000)==.4
    # A late update for an old result must not become a new inference receipt.
    assert unique_result_rate(receipts,7_000_000_000)==0
    assert unique_result_rate(receipts,8_000_000_000)==.2


def test_offline_map_extent_includes_tracks_outside_flight_path():
    import numpy as np
    from tools.flight.demo_video import map_extent
    poses=[dict(xyz=[0,0,1]),dict(xyz=[1,1,1])]
    events=[dict(data=dict(tracks=[dict(xyz=[12,-8,2]),dict(xyz=None)]))]
    center,span=map_extent(poses,events)
    for xy in [[0,0],[1,1],[12,-8]]:
        pixel=np.array([1030,545])+(np.array(xy)-center)*[200/span,-200/span]
        assert 805<=pixel[0]<=1260 and 445<=pixel[1]<=650


def test_legacy_review_label_explicitly_disables_active_review():
    from tools.flight.demo_video import review_label
    event=dict(data=dict(mode='legacy',tracks=[dict(id=2,state='confirmed')]))
    assert review_label(event,None)=='disabled (legacy confirmation)'
    assert 'track #2' in review_label(dict(data=dict(mode='active_confirmation',tracks=[])),
        dict(data=dict(track=2,state='unresolved',reason='review_budget_expired')))
