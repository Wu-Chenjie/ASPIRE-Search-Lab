import numpy as np
from scipy.spatial.transform import Rotation
from annual_runtime.pose_history import PoseHistory


def test_interpolation_crosses_yaw_wrap_and_uses_measured_bracket():
    history=PoseHistory()
    history.add(1., [0,0,0], Rotation.from_euler('z',179,degrees=True).as_quat())
    assert not history.brackets(1.01) and history.at(1.01) is None
    history.add(1.02, [2,0,0], Rotation.from_euler('z',-179,degrees=True).as_quat())
    assert history.brackets(1.01)
    p,r,bracket=history.at(1.01)
    assert np.allclose(p,[1,0,0]) and np.allclose(r.apply([1,0,0]),[-1,0,0])
    assert bracket==(1.,1.02)


def test_gaps_reordered_samples_and_invalid_poses_do_not_create_rays():
    history=PoseHistory()
    assert history.add(1.,[0,0,0],[0,0,0,1])
    assert not history.add(.99,[99,0,0],[0,0,0,1])
    assert not history.add(1.01,[0,0,0],[0,0,0,0])
    assert not history.add(1.01,[float('nan'),0,0],[0,0,0,1])
    assert history.add(1.1,[1,0,0],[0,0,0,1])
    assert not history.brackets(1.05) and history.at(1.05) is None
    assert history.at(.99) is None and history.at(1.11) is None
    assert history.brackets(1.) and history.at(1.)[2]==(1.,1.)


def test_ground_origin_averages_noise_and_restarts_after_motion_or_reset():
    from annual_runtime.pose_history import GroundOriginWindow
    window=GroundOriginWindow();result=None
    for i in range(151):
        result=window.observe(i*.02,[.03*np.sin(i*.4),0,.04*np.cos(i*.4)],
                              [0,0,0],(1,1,1),True)
        if i<150: assert result is None
    assert np.linalg.norm(result)<.003
    assert window.observe(3.02,[0,0,0],[.2,0,0],(1,1,1),True) is None
    assert not window.samples

    window.observe(4.,[0,0,0],[0,0,0],(1,1,1),True)
    window.observe(4.02,[0,0,0],[0,0,0],(2,1,1),True)
    assert len(window.samples)==1
    assert window.observe(4.04,[0,0,0],[0,0,0],(2,1,1),False) is None
    assert not window.samples


def test_ground_origin_window_cannot_be_starved_by_a_fast_publisher():
    from annual_runtime.pose_history import GroundOriginWindow
    window=GroundOriginWindow();result=None
    for i in range(6001):
        result=window.observe(i*.0005,[1,2,3],[0,0,0],(0,0,0),True)
    assert np.allclose(result,[1,2,3]) and len(window.samples)<=301


def test_ground_origin_can_collect_fifty_contiguous_samples_at_ten_to_twelve_hz():
    from annual_runtime.pose_history import GroundOriginWindow

    for period in (.08, .10):
        window = GroundOriginWindow()
        for i in range(50):
            center = window.observe(100. + i*period, [1, 2, 3], [0, 0, 0], (0, 0, 0), True)
            if i < 49:
                assert center is None
        np.testing.assert_allclose(center, [1, 2, 3])
        assert window.diagnostics()['source_span_s'] >= 3.
        assert window.observe(100. + 49*period + .11, [1, 2, 3], [0, 0, 0], (0, 0, 0), True) is None
        assert window.diagnostics()['samples'] == 1


def test_longer_ground_calibration_waits_for_full_measured_window():
    from annual_runtime.pose_history import GroundOriginWindow
    window=GroundOriginWindow(10.)
    for i in range(501):
        result=window.observe(i*.02,[1,2,3],[0,0,0],(0,0,0),True)
        if i<500: assert result is None
    assert np.allclose(result,[1,2,3])
