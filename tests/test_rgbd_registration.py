from types import SimpleNamespace as N
import numpy as np
import pytest
from annual_runtime.depth_projection import decode_depth as decode_raw_depth, project_depth, project_depth_message
from annual_runtime.rgbd_registration import RectifiedCamera,CaptureTransform,decode_depth,register_depth,camera_info_model

def test_depth_units_padding_endianness_and_invalids():
    for endian in ('>','<'):
        data=np.array([[1000,2000,999],[0,3000,999]],dtype=endian+'u2')
        z=decode_depth(data.tobytes(),2,2,6,'16UC1',endian=='>')
        np.testing.assert_allclose(z,[[1,2],[np.nan,3]],equal_nan=True)
    data=np.array([[0,np.inf,np.nan,-1,2]],'<f4')
    z=decode_depth(data.tobytes(),5,1,20,'32FC1')
    assert np.isnan(z[0,:4]).all() and z[0,4]==2
    with pytest.raises(ValueError):decode_depth(b'',2,2,3,'16UC1')


@pytest.mark.parametrize('encoding,dtype,pixels,expected', [
    ('16UC1', '>u2', [200, 4500, 19100, 0], [.2, 4.5, 19.1, 0]),
    ('32FC1', '<f4', [.2, 4.5, np.inf, -1], [np.float32(.2), 4.5, np.inf, -1]),
])
def test_mapper_samples_strided_depth_with_unpadded_last_row(encoding,dtype,pixels,expected):
    rows = np.array([[pixels[0], 0, pixels[1]], [0, 0, 0], [pixels[2], 0, pixels[3]]], dtype=dtype)
    payload = b'x'.join(row.tobytes() for row in rows)
    message = N(data=payload, width=3, height=3, step=rows[0].nbytes+1,
                encoding=encoding, is_bigendian=dtype.startswith('>'))
    metres = np.array([[expected[0],0,expected[1]], [0,0,0], [expected[2],0,expected[3]]])
    np.testing.assert_array_equal(decode_raw_depth(message), metres)
    k = np.diag([1e9, 1e9, 1.])
    points, hits = project_depth_message(message, k, stride=2)
    expected_points, expected_hits = project_depth(metres, k, stride=2)
    np.testing.assert_array_equal(points, expected_points)
    np.testing.assert_array_equal(hits, expected_hits)
    assert hits.tolist() == [True, False]
    with pytest.raises(ValueError, match='buffer/stride'):
        decode_depth(payload, message.width, message.height, message.step, encoding, message.is_bigendian)
    for truncated in (payload[:-1], b''):
        message.data = truncated
        with pytest.raises(ValueError):
            project_depth_message(message, k, stride=2)


def test_registration_uses_nonidentity_extrinsics_and_nearest_surface():
    dc=RectifiedCamera(2,1,np.eye(3),'depth');rc=RectifiedCamera(3,1,np.eye(3),'rgb')
    t=np.eye(4);t[0,3]=1
    transform=CaptureTransform(t,'depth','rgb',1,1)
    z,q=register_depth([[1,2]],dc,rc,transform,1,1)
    assert np.isnan(z[0,0]) and z[0,1]==1 and z[0,2]==2 and q['filled_or_interpolated_pixels']==0
    rc=RectifiedCamera(1,1,[[.1,0,0],[0,1,0],[0,0,1]],'rgb')
    z,q=register_depth([[2,1]],dc,rc,CaptureTransform(np.eye(4),'depth','rgb',1,1),1,1)
    assert z[0,0]==1 and q['collision_pixels']==1

def test_cross_time_transform_and_static_scene_must_be_explicit():
    c=RectifiedCamera(1,1,np.eye(3),'c');t=CaptureTransform(np.eye(4),'c','c',1,10_000_001)
    with pytest.raises(ValueError,match='skew'):register_depth([[1]],c,c,t,1,10_000_001)
    with pytest.raises(ValueError,match='static-scene'):register_depth([[1]],c,c,t,1,10_000_001,max_skew_s=.02)
    z,q=register_depth([[1]],c,c,t,1,10_000_001,max_skew_s=.02,static_scene=True)
    assert z[0,0]==1 and q['skew_s']==pytest.approx(.01)
    with pytest.raises(ValueError,match='timestamps'):register_depth([[1]],c,c,t,1,1)

def test_calibration_mode_and_version_are_bound_to_acquisition():
    header=N(frame_id='c',stamp=N(sec=1,nanosec=0));image=N(width=3,height=2,header=header)
    info=N(width=3,height=2,header=header,k=np.eye(3).ravel().tolist(),r=np.eye(3).ravel().tolist(),p=[2,0,0,0,0,2,0,0,0,0,1,0],d=[.1],distortion_model='plumb_bob',binning_x=0,binning_y=0,roi=N(x_offset=0,y_offset=0,width=0,height=0,do_rectify=False))
    camera,identity=camera_info_model(info,image);assert camera.k[0,0]==2
    with pytest.raises(ValueError,match='distortion'):camera_info_model(info,image,'raw_pinhole')
    info.p[0]=3;assert camera_info_model(info,image)[1]!=identity

def test_ros_adapter_queries_both_times_and_localizes_in_rgb_frame(monkeypatch):
    from person_search.presence_adapter import prepare_geometry
    def header(frame,stamp):return N(frame_id=frame,stamp=N(sec=stamp//10**9,nanosec=stamp%10**9))
    def info(image):
        return N(width=image.width,height=image.height,header=image.header,k=np.eye(3).ravel().tolist(),
            p=[1,0,0,0,0,1,0,0,0,0,1,0],r=np.eye(3).ravel().tolist(),d=[],distortion_model='plumb_bob',
            binning_x=0,binning_y=0,roi=N(x_offset=0,y_offset=0,width=0,height=0,do_rectify=False))
    rgb=N(width=3,height=1,header=header('rgb',1_010_000_000))
    depth=N(width=2,height=1,header=header('depth',1_000_000_000),encoding='16UC1',step=4,is_bigendian=False,data=np.array([1000,2000],'<u2').tobytes())
    transform=lambda x:N(translation=N(x=x,y=0.,z=0.),rotation=N(x=0.,y=0.,z=0.,w=1.))
    calls=[]
    def cross(*args):
        calls.append(args);return N(header=rgb.header,child_frame_id='depth',transform=transform(1.))
    node=N(rgbd_registration_mode='reproject',rgbd_image_mode='rectified',rgbd_max_skew_s=.02,rgbd_static_scene=True,
        require_pose=False,max_age=.75,fresh_capture=lambda _:True,get_clock=lambda:N(now=lambda:N(nanoseconds=1_020_000_000)),
        buffer=N(lookup_transform_full=cross,lookup_transform=lambda *args:N(header=header('map',1_010_000_000),child_frame_id='rgb',transform=transform(3.))))
    z,k,world,q=prepare_geometry(node,(rgb,depth,info(rgb),info(depth)),None,lambda **kw:kw['nanoseconds'])
    assert calls==[('rgb',1_010_000_000,'depth',1_000_000_000,'map')]
    assert np.isnan(z[0,0]) and z[0,1]==1 and z[0,2]==2 and world[0,3]==3
    assert q['skew_s']==pytest.approx(.01) and q['calibration_id']
    from aspire_camera import rgbd_registration
    from person_search.node import capture_age_valid
    now_ns=1_020_000_000
    def camera_info_with_delay(*args):
        nonlocal now_ns
        now_ns=2_000_000_000
        return camera_info_model(*args)
    monkeypatch.setattr(rgbd_registration,'camera_info_model',camera_info_with_delay)
    node.get_clock=lambda:N(now=lambda:N(nanoseconds=now_ns))
    node.fresh_capture=lambda stamp:capture_age_valid(stamp,now_ns,node.max_age)
    with pytest.raises(ValueError,match='stale_or_future_capture'):
        prepare_geometry(node,(rgb,depth,info(rgb),info(depth)),None,lambda **kw:kw['nanoseconds'])
    assert len(calls)==1
