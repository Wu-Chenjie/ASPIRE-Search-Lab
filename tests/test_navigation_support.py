"""The real perception -> registry producer, without ROS or fabricated depth."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
import pytest
import shutil
import subprocess
from person_search.presence import PresenceTracker, NavigationConfig
from person_search.registry_evidence import RegistryEvidence
from person_search.bearing import bearing_packet
from test_bearing_evidence import detection,K,T


def feed(tracker,wire,score,stamp):
    ns=round(stamp*1e9)
    d=detection(score)
    tracker.observe_rgb([d],ns,320,240,stamp+.01)
    tracker.update_bearing([d],K,T,320,240,ns,stamp+.01,{})
    return wire.batch(tracker,ns,stamp+.01)


def test_weak_middle_frame_keeps_navigation_but_breaks_confirmation():
    tracker=PresenceTracker();wire=RegistryEvidence(session='contract')
    for score,stamp in zip((.71,.691,.72),(10.,10.2,10.4)):
        packet=feed(tracker,wire,score,stamp)
    row=packet['observations'][0]
    assert row['support_captures']==3
    assert row['support_first_s']==10.
    assert row['support_policy']=='navigation_rgb_v1'
    assert row['visual_state']=='person_candidate'
    assert not row['has_position']
    assert tracker.at(1).rgb_hits==[10.4]
    assert tracker.active_bearings(10.41)[0]['persistent']
    # Same score sequence never changes the established confirmation rule.
    for stamp in (10.6,10.8,11.,11.2):
        packet=feed(tracker,wire,.72,stamp)
    assert packet['observations'][0]['visual_state']=='visually_confirmed'


@pytest.mark.parametrize('score,nav,confirm',[
    (.5-1e-8,0,0),(.5,1,0),(.5+1e-8,1,0),
    (.7-1e-8,1,0),(.7,1,1),(.7+1e-8,1,1)])
def test_threshold_boundaries_are_inclusive_and_independent(score,nav,confirm):
    tracker=PresenceTracker();wire=RegistryEvidence(session='boundary')
    row=feed(tracker,wire,score,10.)['observations'][0]
    assert row['support_captures']==nav
    assert len(tracker.at(1).rgb_hits)==confirm
    assert row['visual_state']=='person_candidate'


def test_repeated_old_and_future_frames_do_not_contribute():
    tracker=PresenceTracker();wire=RegistryEvidence(session='duplicates')
    feed(tracker,wire,.6,10.)
    assert feed(tracker,wire,.6,10.) is None
    tracker.observe_rgb([detection(.6)],9_000_000_000,320,240,10.)
    tracker.observe_rgb([detection(.6)],11_000_000_000,320,240,10.)
    assert tracker.at(1).nav_hits==[10.]
    assert tracker.last_rgb_ns==10_000_000_000


def test_no_new_frame_does_not_delete_identity_and_tf_failure_is_not_rgb_miss():
    tracker=PresenceTracker();wire=RegistryEvidence(session='gaps')
    for stamp in (10.,10.2,10.4,10.6,10.8):
        feed(tracker,wire,.9,stamp)
    tracker.reject_bearing(10_800_000_000,10.81,'missing_capture_tf')
    tracker.reject_localization(10_800_000_000,10.81,'missing_depth')
    assert len(tracker.at(1).nav_hits)==5
    tracker.report(100.)
    assert tracker.at(1).visual_state=='visually_confirmed'
    assert len(tracker.at(1).nav_hits)==5
    # A real unmatched RGB capture breaks measurement continuity, not existence.
    tracker.observe_rgb([],100_000_000_000,320,240,100.)
    assert not tracker.tracks
    assert tracker.confirmed_history[1]['nav_support_captures']==0
    assert tracker.confirmed_history[1]['semantic_state']=='confirmed'


def test_navigation_configuration_does_not_modify_visual_confirmation():
    tracker=PresenceTracker(navigation=NavigationConfig(min_confidence=.55))
    assert tracker.visual.min_confidence==.70
    with pytest.raises(ValueError):
        NavigationConfig(min_confidence=float('nan'))
    with pytest.raises(ValueError):
        NavigationConfig(min_frames=2.5)


def test_navigation_reporting_retains_confirmation_support_for_audit():
    tracker=PresenceTracker();wire=RegistryEvidence(session='report')
    feed(tracker,wire,.6,10.)
    report=tracker.report(10.01)
    assert report['navigation_thresholds']['min_confidence']==.5
    assert report['visual_thresholds']['min_confidence']==.7
    assert report['tracks'][0]['nav_support_captures']==1
    assert report['tracks'][0]['rgb_support_captures']==0


def test_produced_packet_and_registry_share_compiled_native_support_contract(tmp_path):
    compiler=shutil.which('c++')
    if not compiler:
        pytest.skip('C++17 compiler unavailable')
    tracker=PresenceTracker();wire=RegistryEvidence(session='native')
    for score,stamp in zip((.71,.691,.72),(1.,1.2,1.4)):
        row=feed(tracker,wire,score,stamp)['observations'][0]
    packet=bearing_packet(tracker.active_bearings(1.41)[0])
    literal=','.join(format(x,'.17g') for x in packet)
    source=tmp_path/'contract.cpp'
    source.write_text('''#include "bearing_bias.hpp"
#include <cassert>
int main() {
  racer_task::BearingBias policy;
  std::vector<double> packet={'''+literal+'''};
  assert(policy.receive(packet,1.41));
  assert(racer_task::bearingNavigationSupported('''+
        ','.join(format(row[k],'.17g') for k in
                 ('confidence','support_captures','support_first_s','capture_s'))+'''));
  assert(!racer_task::bearingNavigationSupported(.5-1e-8,3,1.,1.4));
  assert(racer_task::bearingNavigationSupported(.5,3,1.,1.4));
  assert(!racer_task::bearingNavigationSupported(.7,2,1.,1.4));
  assert(!racer_task::bearingNavigationSupported(.7,3,1.,1.4-1e-7));
}
''')
    root=Path(__file__).resolve().parents[1]
    binary=tmp_path/'contract'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(root/'tools/flight'),
                    str(source),'-o',str(binary)],check=True,capture_output=True)
    subprocess.run([str(binary)],check=True,capture_output=True)
