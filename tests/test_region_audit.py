from tools.hm3d.region_audit import crossing_evidence

SPEC=dict(source_bounds=[[-2,-1,1],[-.1,1,3]],target_bounds=[[.1,-1,1],[2,1,3]],
          portal=dict(normal=[1,0,0],point=[0,0,0],bounds=[[0,-.5,1],[0,.5,3]],direction=1))


def test_crossing_needs_source_portal_and_target_in_order():
    samples=[(0,[-.2,0,2]),(.1,[-.05,0,2]),(.2,[.05,0,2]),(.3,[.2,0,2])]
    r=crossing_evidence(samples,SPEC)
    assert r['witness']['entered_target_at']==.3
    assert abs(r['witness']['crossing']['time']-.15)<1e-10
    assert not crossing_evidence([(i*.1,p) for i,(_,p) in enumerate(reversed(samples))],SPEC)['witness']


def test_missing_truth_and_crossing_outside_portal_do_not_pass():
    r=crossing_evidence([(0,[-.2,0,2]),(2,[.2,0,2])],SPEC)
    assert r['large_truth_gaps']==1 and r['witness'] is None
    assert crossing_evidence([(0,[-.2,.8,2]),(.1,[.2,.8,2])],SPEC)['witness'] is None
