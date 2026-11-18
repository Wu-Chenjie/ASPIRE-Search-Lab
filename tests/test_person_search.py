import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
import pytest
from person_search.perception import Candidate,cloud,validate_transform
from person_search.tracking import Tracker
from person_search.policy import ObservedVolume,choose_view


def cand(x=4.,semantic=True):
    return Candidate(np.array([x,0.,1.]),np.ones((2,2),bool),(0,0,2,2),semantic,1.,4.,.1)


def test_cross_view_confirmation_not_repeated_same_frame():
    tr=Tracker();tr.update([cand()],np.zeros(3),0)
    for i in range(1,10):tr.update([cand()],np.zeros(3),i)
    assert tr.tracks[0].state=='tentative'
    tr.update([cand(4.1)],np.array([0,1,0]),10)
    assert len(tr.tracks)==1 and tr.tracks[0].state=='confirmed'


def test_geometry_never_confirms_and_review_budget_expires():
    tr=Tracker(max_attempts=2)
    tr.update([cand(semantic=False)],np.zeros(3),0);t=tr.tracks[0]
    assert tr.review(t);assert tr.review(t);assert not tr.review(t)
    tr.update([],np.ones(3),1);assert t.state=='abandoned'
    assert choose_view(t,np.zeros(3),ObservedVolume()) is None


def test_time_and_rigid_transform_fail_closed():
    tr=Tracker();tr.update([],np.zeros(3),10)
    with pytest.raises(ValueError):tr.update([],np.zeros(3),9)
    for t in (np.eye(3),np.eye(4)*2,np.full((4,4),np.nan)):
        with pytest.raises(ValueError):validate_transform(t)


def test_projection_uses_optical_axes_and_translation():
    t=np.array([[0,0,1,1],[-1,0,0,2],[0,-1,0,3],[0,0,0,1.]])
    p=cloud(np.ones((3,3))*4,[2,0,1,0,2,1,0,0,1],t)
    np.testing.assert_allclose(p[1,1],[5,2,3]);np.testing.assert_allclose(p[1,2],[5,0,3])


def test_unknown_and_occlusion_not_free():
    vol=ObservedVolume(.1)
    assert not vol.free([0,0,2])
    vol.add(np.ones((100,100))*3,[50,0,50,0,50,50,0,0,1],np.eye(4))
    assert vol.free([0,0,2])
    assert not vol.free([0,0,3.2])
    assert not vol.free([0,0,-1])
    assert not vol.path_free([0,0,2],[0,0,4])


def test_two_people_in_same_frame_do_not_merge():
    tr=Tracker();tr.update([cand(4),cand(4.4)],np.zeros(3),0)
    assert len(tr.tracks)==2


def test_map_tf_quaternion_must_be_valid():
    from person_search.node import matrix
    from types import SimpleNamespace as S
    t=S(translation=S(x=1.,y=2.,z=3.),rotation=S(x=0.,y=0.,z=0.,w=1.))
    np.testing.assert_allclose(matrix(t)[:3,3],[1,2,3])
    t.rotation.w=0.
    with pytest.raises(ValueError):matrix(t)


def test_horizon_surface_cannot_be_free_when_range_is_missing():
    vol=ObservedVolume(.1)
    vol.add(np.full((100,100),np.inf),[50,0,50,0,50,50,0,0,1],np.eye(4))
    assert not vol.free([0,0,2])


def test_ideal_rig_reuses_the_pixel_aligned_projection(monkeypatch):
    from person_search import perception
    from aspire_planning import review_views
    from person_search.perception import candidates, candidates_from_cloud

    height, width = 88, 96
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    camera_to_map = np.array([[0, 0, 1, 1.], [-1, 0, 0, 2.],
                              [0, -1, 0, 1.4], [0, 0, 0, 1.]])
    k = [100., 0, 48., 0, 100., 44., 0, 0, 1.]
    depth = np.full((height, width), 8., dtype=np.float32)
    v, u = np.indices(depth.shape)
    body = (18 <= v) & (v < 70) & (42 <= u) & (u < 56)
    depth[body] = 2.35 + .035 * np.sin(v[body] / 6.)
    depth[0] = np.nan

    def detector(_):
        return [((40, 17, 18, 55), .85)]

    expected = candidates(rgb, depth, k, camera_to_map, detector)
    assert len(expected) == 1 and expected[0].semantic
    original_volume = ObservedVolume()
    original_volume.add(depth, k, camera_to_map)
    points = cloud(depth, k, camera_to_map)

    def unexpected_projection(*args):
        pytest.fail("a same-capture projection was already provided")

    monkeypatch.setattr(perception, "cloud", unexpected_projection)
    monkeypatch.setattr(review_views, "cloud", unexpected_projection)
    reused_volume = ObservedVolume()
    reused_volume.add(depth, k, camera_to_map, points=points)
    actual = candidates_from_cloud(rgb, depth, points, detector)
    assert len(actual) == len(expected)
    before, after = expected[0], actual[0]
    assert (before.box, before.semantic, before.confidence, before.depth_m, before.spread_m) == (
        after.box, after.semantic, after.confidence, after.depth_m, after.spread_m)
    np.testing.assert_array_equal(before.pixels, after.pixels)
    np.testing.assert_array_equal(before.xyz, after.xyz)
    np.testing.assert_array_equal(original_volume.surfaces[0], reused_volume.surfaces[0])
    for query in ([2., 2., 1.4], [4.5, 2., 1.4], [0., 0., 0.]):
        assert original_volume.free(query) == reused_volume.free(query)
