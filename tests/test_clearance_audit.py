import numpy as np
from tools.flight.clearance_audit import signed_box_distance


def test_signed_box_distance_faces_corners_and_inside():
    d = signed_box_distance(np.array([[2, 0, 0], [2, 2, 0], [0, 0, 0], [1, 0, 0]]),
                            np.zeros(3), np.array([2, 2, 2]))
    np.testing.assert_allclose(d, [1, np.sqrt(2), -1, 0])


def test_nonfinite_geometry_is_reported_missing(tmp_path):
    from tools.flight.stage23_clearance import Scene

    (tmp_path/'scene.sdf').write_text("""
      <sdf><world><model name="room"><static>true</static><link name="walls">
        <collision name="malformed_box"><geometry><box><size>2 nan 3</size></box></geometry></collision>
        <collision name="malformed_plane"><geometry><plane><normal>0 inf 1</normal></plane></geometry></collision>
        <collision name="valid_box"><geometry><box><size>2 2 3</size></box></geometry></collision>
      </link></model></world></sdf>
    """)
    scene = Scene(tmp_path, [])
    assert [g['name'] for g in scene.geometries] == ['/room/walls/valid_box']
    assert [row['model'] for row in scene.missing] == [
        '/room/walls/malformed_box', '/room/walls/malformed_plane',
    ]
