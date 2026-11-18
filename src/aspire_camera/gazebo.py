"""Generated co-located pinhole RGB and depth cameras; unchanged depth optics."""
from .mount import DEPTH_TRANSLATION
from aspire_interfaces.topics import camera_topic
import copy
import math
import xml.etree.ElementTree as ET


def set_text(parent, path, value):
    node=parent
    for part in path.split('/'):
        child=node.find(part)
        if child is None:child=ET.SubElement(node,part)
        node=child
    node.text=str(value)


def configure_camera(tree, drone):
    link=tree.find('./model/link')
    depths=link.findall('sensor[@type="depth_camera"]')
    if len(depths)!=1:raise ValueError('Expected one locked depth camera')
    depth=depths[0]
    for sensor in list(link.findall('sensor')):
        if sensor is not depth:link.remove(sensor)
    set_text(depth,'topic',camera_topic(drone, 'depth', 'image'))
    set_text(depth,'update_rate',15)
    set_text(depth,'camera/image/width',320);set_text(depth,'camera/image/height',240)
    set_text(depth,'gz_frame_id',f'drone_{drone}/depth_optical')
    set_text(depth,'camera/optical_frame_id',f'drone_{drone}/depth_optical')
    set_text(depth,'camera/camera_info_topic',camera_topic(drone, 'depth', 'camera_info'))
    rgb=copy.deepcopy(depth);rgb.set('name','RGB');rgb.set('type','camera')
    set_text(rgb,'topic',camera_topic(drone, 'rgb', 'image'))
    set_text(rgb,'camera/image/format','R8G8B8')
    # RGB otherwise infers K from its float render matrix, unlike depth's double
    # HFOV-derived K. Explicit lens intrinsics remove the measured ~2.4e-5 px drift.
    hfov=float(depth.findtext('camera/horizontal_fov','1.274'))
    focal=160./math.tan(hfov/2.)
    for key,value in dict(fx=focal,fy=focal,cx=160.,cy=120.,s=0.).items():
        set_text(rgb,'camera/lens/intrinsics/'+key,value)
    for key,value in dict(p_fx=focal,p_fy=focal,p_cx=160.,p_cy=120.,tx=0.,ty=0.).items():
        set_text(rgb,'camera/lens/projection/'+key,value)
    set_text(rgb,'gz_frame_id',f'drone_{drone}/rgb_optical')
    set_text(rgb,'camera/optical_frame_id',f'drone_{drone}/rgb_optical')
    set_text(rgb,'camera/camera_info_topic',camera_topic(drone, 'rgb', 'camera_info'))
    link.append(rgb)


def transforms(drone):
    # Locked x500 base_link offset accounted for, independently tested against SDF.
    return [dict(parent=f'drone_{drone}/base_link',child=f'drone_{drone}/{kind}_optical',
                 xyz=list(DEPTH_TRANSLATION),xyzw=[-.5,.5,-.5,.5]) for kind in ('depth','rgb')]


def tf_arguments(transform):
    return [item for key,value in zip(('x','y','z','qx','qy','qz','qw'),transform['xyz']+transform['xyzw'])
            for item in ('--'+key,str(value))]+['--frame-id',transform['parent'],'--child-frame-id',transform['child']]


RGB_DEPTH_CONTRACT = dict(
    configuration='co-located equal-resolution pinhole cameras',
    direct_pixel_indexing='requires matching image stamps, dimensions, K/D/R/P and identity RGB-depth TF; reject mismatches',
    optical_axes='x right, y down, z forward',
    depth_units='metres, axial optical Z',
)


def bridge_topics(drone):
    for stream in ('depth', 'rgb'):
        for channel, kind in (('image', 'Image'), ('camera_info', 'CameraInfo')):
            name = camera_topic(drone, stream, channel)
            yield dict(ros_topic_name=name, gz_topic_name=name,
                       ros_type_name=f'sensor_msgs/msg/{kind}', gz_type_name=f'gz.msgs.{kind}',
                       direction='GZ_TO_ROS', qos_profile='SENSOR_DATA')
