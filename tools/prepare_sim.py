#!/usr/bin/env python3
"""Generate Gazebo scenes with paired RGB/depth topics; geometry stays in simulation."""
import argparse
import copy
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'src/annual_planner'
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(PACKAGE/'src'))
from tools.rgbd_scene.cameras import configure_camera, transforms
from core.planning.motion_limits import MotionLimits, RACER_EXAMPLE, AGGRESSIVE


def landing_support(models):
    """Pinned X500 skid bottom samples in base_link; scene generation only."""
    base = ET.parse(Path(models)/'x500_base/model.sdf')
    points = []
    for name in ('base_link_collision_3', 'base_link_collision_4'):
        part = base.find(f'./model/link[@name="base_link"]/collision[@name="{name}"]')
        if part is None: raise ValueError('Missing locked X500 landing skid')
        pose = np.fromstring(part.findtext('pose'), sep=' ')
        size = np.fromstring(part.findtext('geometry/box/size'), sep=' ')
        if len(pose)!=6 or len(size)!=3 or not np.allclose(pose[3:],0):
            raise ValueError('Review changed landing geometry before placing aircraft')
        points.extend(pose[:3]+[x,y,-size[2]/2]
                      for x in np.linspace(-size[0]/2,size[0]/2,5)
                      for y in np.linspace(-size[1]/2,size[1]/2,3))
    return np.asarray(points)


def model(world, name, center, geometry):
    m = ET.SubElement(world, 'model', name=name)
    ET.SubElement(m, 'static').text = 'true'
    ET.SubElement(m, 'pose').text = ' '.join(map(str, list(center)+[0,0,0]))
    link = ET.SubElement(m, 'link', name='body')
    for tag in ('collision', 'visual'):
        part = ET.SubElement(link, tag, name=tag)
        shape = ET.SubElement(ET.SubElement(part, 'geometry'), geometry[0])
        for key, value in geometry[1].items(): ET.SubElement(shape, key).text = value


def prepare(map_file, output, gui=False, models=None, *, depth_odometry=None,
            vision_height_reference=False, magnetometer_fusion='init',
            height_reference=None, heading_calibration_s=10, motion_profile='aggressive',
            rgbd_test_scene=False):
    if motion_profile not in ('legacy', 'racer-example', 'aggressive'):
        raise ValueError('Unsupported motion profile')
    envelope = {'legacy': MotionLimits(), 'racer-example': RACER_EXAMPLE, 'aggressive': AGGRESSIVE}[motion_profile]
    if depth_odometry not in (None, 'rtabmap_icp'):
        raise ValueError('Unsupported depth odometry frontend')
    if vision_height_reference and depth_odometry is None:
        raise ValueError('Vision height reference requires measured depth odometry')
    if magnetometer_fusion not in ('init', 'auto', 'ground_calibrated_init'):
        raise ValueError('Unsupported magnetometer fusion profile')
    if height_reference not in (None,'barometer') or (height_reference and vision_height_reference):
        raise ValueError('Unsupported or conflicting height reference')
    if magnetometer_fusion=='ground_calibrated_init' and (
            not isinstance(heading_calibration_s,int) or not 10<=heading_calibration_s<=120):
        raise ValueError('Heading calibration must be an integer in [10,120] seconds')
    data = json.loads(Path(map_file).read_text()); starts, bounds = data['search_starts'], data['bounds']
    if rgbd_test_scene and len(starts)!=1:raise ValueError('RGBD manual test requires one camera vehicle')
    if not starts or any(len(p) != 3 for p in starts): raise ValueError('Map requires 3D search_starts')
    source = Path(models or ROOT/'external/PX4-gazebo-models/models').resolve()
    initial = data.get('initial_positions')
    if initial is None:
        # The model starts with a small clearance, then settles on its skids.
        # The known initial pose is the ground contact datum, not the spawn pose.
        height = float(-landing_support(source)[:,2].min())
        initial = [[p[0],p[1],height] for p in starts]
    positions = np.asarray(initial)
    if positions.shape != (len(starts), 3) or not np.isfinite(positions).all():
        raise ValueError('One finite initial position is required per aircraft')
    camera_template = ET.parse(source/'OakD-Lite/model.sdf')
    vehicle_template = ET.parse(source/'x500_depth/model.sdf')
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=True)
    sdf = ET.Element('sdf', version='1.9'); world = ET.SubElement(sdf, 'world', name='annual')
    for key, system in [('physics','Physics'), ('user-commands','UserCommands'), ('scene-broadcaster','SceneBroadcaster'),
                        ('sensors','Sensors'), ('imu','Imu'), ('magnetometer','Magnetometer'),
                        ('navsat','NavSat'), ('air-pressure','AirPressure')]:
        plugin = ET.SubElement(world,'plugin',filename=f'gz-sim-{key}-system',name=f'gz::sim::systems::{system}')
        if system == 'Sensors': ET.SubElement(plugin,'render_engine').text = 'ogre2'
    ET.SubElement(world,'gravity').text = '0 0 -9.81'
    ET.SubElement(world,'magnetic_field').text = '6e-06 2.3e-05 -4.2e-05'
    ET.SubElement(world,'atmosphere',type='adiabatic')
    scene=ET.SubElement(world,'scene')
    ET.SubElement(scene,'ambient').text='0.4 0.4 0.4 1'
    ET.SubElement(scene,'shadows').text='false'
    physics = ET.SubElement(world,'physics',name='default',type='ignored')
    for key,value in [('max_step_size','0.004'),('real_time_factor','1')]: ET.SubElement(physics,key).text=value
    spherical=ET.SubElement(world,'spherical_coordinates')
    for key,value in [('surface_model','EARTH_WGS84'),('world_frame_orientation','ENU'),
                      ('latitude_deg','47.397742'),('longitude_deg','8.545594'),('elevation','488'),('heading_deg','0')]:
        ET.SubElement(spherical,key).text=value
    light=ET.SubElement(world,'light',name='sun',type='directional')
    for key,value in [('pose','0 0 10 0 0 0'),('diffuse','0.8 0.8 0.8 1'),('specular','0.2 0.2 0.2 1'),('direction','-0.5 0.1 -0.9')]:
        ET.SubElement(light,key).text=value
    model(world,'ground',[0,0,-.1],('box',{'size':'200 200 0.2'}))
    for i,o in enumerate(data.get('obstacles',[])):
        if o['type']=='aabb':
            center=[(a+b)/2 for a,b in zip(o['min'],o['max'])]
            shape=('box',{'size':' '.join(str(b-a) for a,b in zip(o['min'],o['max']))})
        elif o['type']=='cylinder':
            lo,hi=o['z_range']; center=list(o['center_xy'])+[(lo+hi)/2]
            shape=('cylinder',{'radius':str(o['radius']),'length':str(hi-lo)})
        else: raise ValueError('Unsupported geometry: '+o['type'])
        model(world,'obstacle_'+str(i),center,shape)
    bridge=[dict(ros_topic_name='/clock',gz_topic_name='/world/annual/clock',
                 ros_type_name='rosgraph_msgs/msg/Clock',gz_type_name='gz.msgs.Clock',direction='GZ_TO_ROS')]
    runtime=dict(bounds=bounds,fleet_starts=starts,gui=gui,models=str(source),vehicles=[],
                 magnetometer='px4_sensor_mag_sim', magnetometer_fusion=magnetometer_fusion,
                 estimator_timing='gz_sensor_timestamps', motion_limits=envelope.to_dict(), motion_profile=motion_profile)
    if depth_odometry:
        runtime['depth_odometry'] = depth_odometry
    if vision_height_reference:
        runtime['depth_odometry_height_reference'] = 'vision'
    if height_reference:
        runtime['height_reference'] = height_reference
    if magnetometer_fusion=='ground_calibrated_init':
        runtime['heading_calibration_s'] = heading_calibration_s
    for i,start in enumerate(starts):
        camera=copy.deepcopy(camera_template); camera_name=f'annual_camera_{i}'
        camera.getroot().find('model').set('name',camera_name)
        link=camera.getroot().find('./model/link')
        configure_camera(camera, i)
        runtime.setdefault('camera_transforms',[]).extend(transforms(i))
        vehicle=copy.deepcopy(vehicle_template); vehicle_name=f'annual_x500_{i}'
        # Harmonic's legacy magnetic earth frame and the locked GZBridge mag
        # conversion disagree. Use the locked PX4's noisy WMM sensor simulator
        # instead; its ground truth input stays inside the simulator boundary.
        # Remove the competing publisher (same device id) in a generated copy.
        base=ET.parse(source/'x500_base/model.sdf'); base_name=f'annual_base_{i}'
        rotor=ET.parse(source/'x500/model.sdf'); rotor_name=f'annual_rotors_{i}'
        rotor.getroot().find('model').set('name',rotor_name)
        for uri in rotor.getroot().findall('./model/include/uri'):
            if uri.text in ('x500_base','model://x500_base'): uri.text='model://'+base_name
        base.getroot().find('model').set('name',base_name)
        for link in base.getroot().findall('./model/link'):
            for sensor in list(link.findall('sensor')):
                if sensor.get('type')=='magnetometer': link.remove(sensor)
        vehicle.getroot().find('model').set('name',vehicle_name)
        for uri in vehicle.getroot().findall('./model/include/uri'):
            if uri.text=='model://OakD-Lite': uri.text='model://'+camera_name
            if uri.text in ('x500','model://x500'): uri.text='model://'+rotor_name
            if uri.text in ('x500_base','model://x500_base'): uri.text='model://'+base_name
        for name,tree in [(camera_name,camera),(vehicle_name,vehicle),(base_name,base),(rotor_name,rotor)]:
            directory=output/'models'/name; directory.mkdir(parents=True,exist_ok=True)
            tree.write(directory/'model.sdf',encoding='unicode')
            (directory/'model.config').write_text(f'<model><name>{name}</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
        include=ET.SubElement(world,'include')
        ET.SubElement(include,'uri').text='model://'+vehicle_name
        ET.SubElement(include,'name').text=vehicle_name
        ET.SubElement(include,'pose').text=f'{start[0]} {start[1]} 0 0 0 0'
        runtime['vehicles'].append(dict(drone_id=i,px4_instance=i+1,model=vehicle_name,
            spawn=[start[0],start[1],.24],initial_position=initial[i],start=start))
        for modality in ('depth','rgb'):
            for topic,kind,gz_kind in [('image','Image','Image'),('camera_info','CameraInfo','CameraInfo')]:
                bridge.append(dict(ros_topic_name=f'/drone_{i}/{modality}/{topic}',gz_topic_name=f'/drone_{i}/{modality}/{topic}',
                                   ros_type_name=f'sensor_msgs/msg/{kind}',gz_type_name=f'gz.msgs.{gz_kind}',direction='GZ_TO_ROS',qos_profile='SENSOR_DATA'))
    runtime['rgb_depth_contract']=dict(configuration='co-located equal-resolution pinhole cameras',
        direct_pixel_indexing='requires matching image stamps, dimensions, K/D/R/P and identity RGB-depth TF; reject mismatches',
        optical_axes='x right, y down, z forward',depth_units='metres, axial optical Z')
    if rgbd_test_scene:
        from tools.rgbd_scene.scene import add_test_scene
        add_test_scene(world,output,runtime)
    ET.ElementTree(sdf).write(output/'scene.sdf',encoding='unicode')
    (output/'bridge.yaml').write_text(yaml.safe_dump(bridge))
    (output/'runtime.json').write_text(json.dumps(runtime,indent=2)+'\n')
    return output/'runtime.json'


def main():
    p=argparse.ArgumentParser();p.add_argument('--map',default=str(PACKAGE/'maps/search_fusion_3d.json'))
    p.add_argument('--output',default=str(ROOT/'outputs/sim'));p.add_argument('--models')
    p.add_argument('--gui',action='store_true')
    p.add_argument('--rgbd-test-scene',action='store_true',help='Static person/occluder and manual camera viewpoints; no flight acceptance')
    p.add_argument('--depth-odometry',choices=['rtabmap_icp'],help='Optional actual-depth registration frontend')
    p.add_argument('--vision-height-reference',action='store_true')
    p.add_argument('--magnetometer-fusion',choices=['init','auto','ground_calibrated_init'],default='init')
    p.add_argument('--height-reference',choices=['barometer'])
    p.add_argument('--heading-calibration-seconds',default='10')
    p.add_argument('--motion-profile', choices=['legacy', 'racer-example', 'aggressive'], default='aggressive')
    a=p.parse_args()
    if a.magnetometer_fusion=='ground_calibrated_init':
        try:
            a.heading_calibration_seconds=int(a.heading_calibration_seconds)
        except ValueError:
            p.error('--heading-calibration-seconds requires an integer with ground calibration')
    print(prepare(a.map,a.output,a.gui,a.models,depth_odometry=a.depth_odometry,
                  vision_height_reference=a.vision_height_reference,magnetometer_fusion=a.magnetometer_fusion,
                  height_reference=a.height_reference,heading_calibration_s=a.heading_calibration_seconds,motion_profile=a.motion_profile,rgbd_test_scene=a.rgbd_test_scene))


if __name__=='__main__':main()
