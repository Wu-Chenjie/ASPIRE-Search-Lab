#!/usr/bin/env python3
"""Expand the pinned RACER template into native launch nodes."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

def partition_volume_counts(node,resolution):
    """Keep native Hgrid volume gates when changing the voxel lattice."""
    params={p.get('name'):p.get('value') for p in node.findall('param')}
    source=float(params['sdf_map/resolution'])
    if not math.isfinite(source) or source<=0:
        raise ValueError('Invalid native map resolution')
    result={}
    for name in ('partitioning/min_unknown','partitioning/min_free'):
        count=int(params[name])
        if count<0:raise ValueError('Invalid native partition volume gate')
        result[name]=math.ceil(count*(source/resolution)**3)
    return result


def launch_nodes(bounds,intrinsics,output,template_path,limits,fleet_size,
                 voxel_resolution=.3,body_clearance=.6):
    if type(fleet_size) is not int or fleet_size not in (1,2):
        raise ValueError('Supported fleet sizes: one or two')
    if not .05<=voxel_resolution<=.5 or body_clearance<=0 or not math.isfinite(body_clearance):
        raise ValueError('Invalid native voxel geometry')
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in limits.values()):
        raise ValueError('Invalid motion envelope')
    template = Path(template_path).read_text()
    for i in range(fleet_size):
        values=dict(drone_id=str(i+1),drone_num=str(fleet_size),max_vel=str(limits['speed']),max_acc=str(limits['acceleration']),simulation='true',single_expo='false',
            odometry_topic=f'/baseline/odom_{i}',sensor_pose_topic=f'/baseline/sensor_{i}',depth_topic=f'/baseline/unused_depth_{i}',cloud_topic=f'/baseline/cloud_{i}',
            cx=str(intrinsics[2]),cy=str(intrinsics[5]),fx=str(intrinsics[0]),fy=str(intrinsics[4]))
        for j,axis in enumerate('xyz'):
            values['map_size_'+axis]=str(bounds[1][j]-bounds[0][j]);values['map_size_'+axis+'_']=values['map_size_'+axis]
            values['box_min_'+axis]=str(bounds[0][j]);values['box_max_'+axis]=str(bounds[1][j])
        text=template
        for key,value in values.items():text=text.replace('$(arg '+key+')',value)
        tree=ET.fromstring(text)
        for node in tree.findall('node'):
            if node.get('type')=='exploration_node':
                for remap in list(node.findall('remap')):
                    if remap.get('unless')=='false' or remap.get('if')=='false':node.remove(remap);continue
                    remap.attrib.pop('if',None);remap.attrib.pop('unless',None)
                    if remap.get('from')=='/planning/bspline':remap.set('to',f'/baseline/bspline_{i}')
                overrides={'sdf_map/resolution':voxel_resolution,'sdf_map/origin_x':bounds[0][0],'sdf_map/origin_y':bounds[0][1],
                    'sdf_map/ground_height':bounds[0][2], 'sdf_map/obstacles_inflation':body_clearance,'sdf_map/optimistic':False,
                    'sdf_map/max_ray_length':4.5,'map_ros/show_all_map':False,
                    'frontier/cluster_min':5,'frontier/down_sample':round(.3/voxel_resolution),'frontier/min_visib_num':3,
                    'frontier/min_candidate_clearance':body_clearance,'frontier/candidate_rmin':.9,'frontier/candidate_rmax':1.8,
                    'frontier/cluster_size_z':2.,'exploration/yd':limits['yaw_rate'],'exploration/ydd':.8,
                    'perception_utils/top_angle':math.atan(.75*math.tan(1.274/2)),
                    'perception_utils/left_angle':1.274/2,'perception_utils/right_angle':1.274/2,'perception_utils/max_dist':4.5,
                    'manager/clearance_threshold':body_clearance,'optimization/dist0':body_clearance}
                overrides.update(partition_volume_counts(node,overrides['sdf_map/resolution']))
                for j,axis in enumerate('xyz'):
                    overrides['sdf_map/box_min_'+axis]=bounds[0][j]+(.7 if j==2 else 0)
                    overrides['sdf_map/box_max_'+axis]=min(3.1,bounds[1][j]-.3) if j==2 else bounds[1][j]
                for p in list(node.findall('param')):
                    if p.get('name') in overrides:node.remove(p)
                for key,value in overrides.items():ET.SubElement(node,'param',name=key,value=str(value).lower(),type='bool' if isinstance(value,bool) else 'int' if isinstance(value,int) else 'double')
            for p in node.findall('param'):
                if p.get('name') in ('exploration/tsp_dir','exploration/mtsp_dir'):
                    resource=output/f'resource_{i}_{p.get("name").split("/")[-1]}'
                    resource.mkdir(exist_ok=True);p.set('value',str(resource.resolve()))
            yield node
