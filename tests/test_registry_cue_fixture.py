from tools.flight.registry_cue_fixture import ChurningCueFixture

def test_negative_fixture_has_bounded_ids_no_positions_and_real_dropout():
    f=ChurningCueFixture();assert f.step(0,False) is None
    rows=[]
    for i in range(1801):
        t=20+i*.1;p=f.step(t,True)
        if p:
            r=p['observations'][0];rows.append(r)
            assert not r['has_position'] and r['localization_state']=='unknown'
            assert r['first_s']<=r['capture_s'] and r['session']=='explicit-empty-cue-fixture'
    assert not any(28<=r['capture_s']<35 for r in rows)
    assert rows and len(f.sequences)<=64 and f.report()['safety_mapping_actuators_changed'] is False
    for ident in f.sequences:
        same=[r for r in rows if int(r['counter'])==ident]
        assert len({r['first_s'] for r in same})==1
        assert len({r['sequence'] for r in same})==len(same)
