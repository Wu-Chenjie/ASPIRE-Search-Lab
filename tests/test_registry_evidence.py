"""Perception wire lifetime checks; synthetic surfaces are not accuracy evidence."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/"src"),str(ROOT/"src/annual_planner/src")]
import numpy as np
from person_search.presence import PresenceTracker
from person_search.registry_evidence import RegistryEvidence
from person_search.node import confirmation_options
from test_surface_components import sample,K,T
from person_search.surface import localize_surfaces

def test_registry_wire_visual_presence_sequence_and_current_geometry_are_separate():
    tracker=PresenceTracker();wire=RegistryEvidence(session='unit',frame_version='local-v1')
    d,depth=sample();depth[:]=2.;rows,_=localize_surfaces([d],depth,K,T)
    first=None
    for s in (10.,10.2,10.4,10.6,10.8):
        ns=round(s*1e9);tracker.observe_rgb([d],ns,320,240,s+.01)
        wire.capture(tracker,ns,rows,s+.01);packet=wire.batch(tracker,ns,s+.01)
        row=packet['observations'][0];first=row['first_s'] if first is None else first
        assert row['first_s']==first and wire.batch(tracker,ns,s+.01) is None
    assert row['has_position'] and row['visual_state']=='visually_confirmed'
    sequence=int(row['sequence']);support=tracker.tracks[0].captures
    wire.reject(tracker,ns,10.82,'missing_capture_tf')
    packet=wire.batch(tracker,ns,10.82);row=packet['observations'][0]
    assert not row['has_position'] and row['visual_state']=='visually_confirmed'
    assert 'px' not in row and int(row['sequence'])>sequence
    assert tracker.tracks[0].captures==support
    assert tracker.tracks[0].last_reliable_location['history_only']
    assert wire.robot=='uav0' and not row['shared_aligned']

def test_registry_mode_is_explicit_and_never_enables_legacy_xyz_review():
    assert confirmation_options('registry',False,False)==('registry',False,False)
    assert confirmation_options('registry',True,True)==('registry',False,False)



def test_expired_fusion_cannot_create_a_visibility_anchor():
    tracker = PresenceTracker()
    wire = RegistryEvidence(session='stale-fusion')
    detection, depth = sample()
    depth[:] = 2.
    rows, _ = localize_surfaces([detection], depth, K, T)
    for capture, processing in ((10., 10.01), (10.2, 10.21), (10.8, 13.)):
        ns = round(capture * 1e9)
        tracker.observe_rgb([detection], ns, 320, 240, capture + .01)
        wire.capture(tracker, ns, rows, processing)
    row = wire.batch(tracker, ns, processing)['observations'][0]
    assert not row['has_position'] and 'identity_surface_policy' not in row
    assert row['localization_state'] == 'expired'
    assert not wire.last_qualified


def test_old_frame_cannot_carry_the_latest_reliable_position_state():
    tracker = PresenceTracker()
    wire = RegistryEvidence(session='late-batch')
    detection, depth = sample()
    depth[:] = 2.
    rows, _ = localize_surfaces([detection], depth, K, T)
    old_ns, current_ns = 10_400_000_000, 10_800_000_000
    for ns in (10_000_000_000, old_ns, current_ns):
        processing = ns * 1e-9 + .01
        tracker.observe_rgb([detection], ns, 320, 240, processing)
        wire.capture(tracker, ns, rows, processing)

    current = wire.batch(tracker, current_ns, processing)['observations'][0]
    assert current['localization_state'] == 'reliable' and current['has_position']
    old = wire.batch(tracker, old_ns, processing)['observations'][0]
    assert old['localization_state'] == 'expired' and not old['has_position']
    assert not {'px', 'py', 'pz', 'radius_m', 'identity_surface_policy'}.intersection(old)
    assert tracker.at(1).position['capture_time_s'] == current['capture_s']

    for reason, state in (('invalid_calibration', 'unreliable'), ('missing_capture_tf', 'unknown')):
        wire.reject(tracker, current_ns, processing, reason)
        for ns in (old_ns, current_ns):
            row = wire.batch(tracker, ns, processing)['observations'][0]
            assert row['localization_state'] == state and not row['has_position']

def test_historical_bearing_keeps_its_capture_support_and_never_binds_to_adjacent_capture():
    tracker=PresenceTracker();wire=RegistryEvidence(session='exact-bearing')
    detection,_=sample()
    old_ns,new_ns=10_000_000_000,10_000_000_001
    tracker.observe_rgb([detection],old_ns,320,240,10.01)
    tracker.update_bearing([],K,T,320,240,old_ns,10.01,{})
    tracker.observe_rgb([detection],new_ns,320,240,10.02)
    current=wire.batch(tracker,new_ns,10.02)['observations'][0]
    historical=wire.batch(tracker,old_ns,10.02)['observations'][0]
    assert not current['has_bearing']
    assert historical['has_bearing'] and historical['support_captures']==1
