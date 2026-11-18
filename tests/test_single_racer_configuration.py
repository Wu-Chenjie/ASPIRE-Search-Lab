from pathlib import Path

import pytest

from tools.racer_ros2.configuration import configuration


def test_single_fleet_keeps_authorization_and_only_own_native_nodes(tmp_path):
    root=Path(__file__).resolve().parents[1]
    hello=dict(starts=[[0,0,2.2]],bounds=[[-3,-4,0],[7,4,4]],intrinsics=[216,0,160,0,216,120,0,0,1],flight_limits=[.7,3.])
    template=root/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml'
    configs=configuration(hello,tmp_path,template)
    planners=[c for c in configs if c['executable']=='exploration_node']
    assert len(configs)==3 and len(planners)==1
    params=planners[0]['parameters']
    assert params['exploration.drone_num']==1
    assert params['fsm.authorized_execution'] is True
    assert params['sdf_map.obstacles_inflation']>=.6
    assert all(not c['name'].endswith('_2') for c in configs)


def test_explicit_nominal_limits_and_min_time_do_not_weaken_authorization(tmp_path):
    root=Path(__file__).resolve().parents[1]
    hello=dict(starts=[[0,0,2.2]],bounds=[[-3,-4,0],[7,4,4]],intrinsics=[216,0,160,0,216,120,0,0,1],flight_limits=[.7,3.],motion_limits=dict(speed=1.,acceleration=1.2,jerk=4.,yaw_rate=.9),planning_limits=dict(speed=.3,acceleration=.4,jerk=4.,yaw_rate=.325),min_time=False)
    configs=configuration(hello,tmp_path,root/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml')
    p=next(c['parameters'] for c in configs if c['executable']=='exploration_node')
    assert p['manager.min_time'] is False
    assert p['manager.max_vel']==.3 and p['manager.max_acc']==.4
    assert p['fsm.authorized_execution'] and p['sdf_map.obstacles_inflation']>=.6


@pytest.mark.parametrize('execution_limits', [{}, {'motion_limits': {'speed': .7}}])
def test_explicit_nominal_limits_use_implicit_execution_defaults(tmp_path, execution_limits):
    root=Path(__file__).resolve().parents[1]
    template=root/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml'
    hello=dict(starts=[[0,0,2.2]],bounds=[[-3,-4,0],[7,4,4]],
               intrinsics=[216,0,160,0,216,120,0,0,1],flight_limits=[.7,3.],
               planning_limits=dict(speed=.3,acceleration=.5,jerk=1.8,yaw_rate=.5),
               planning_scale='unused', **execution_limits)
    planner=next(c['parameters'] for c in configuration(hello,tmp_path,template)
                 if c['executable']=='exploration_node')
    assert planner['manager.max_vel']==.3 and planner['manager.max_yawdot']==.5
    assert planner['fsm.authorized_execution']
    hello['planning_limits']['yaw_rate']=.66
    with pytest.raises(ValueError, match='Invalid explicit planner envelope'):
        configuration(hello,tmp_path,template)
    hello.pop('planning_limits')
    with pytest.raises(ValueError, match='Native planning scale must be in'):
        configuration(hello,tmp_path,template)


def test_frontier_recovery_can_run_without_person_review(tmp_path):
    root=Path(__file__).resolve().parents[1]
    hello=dict(starts=[[4.3,2.5,1.5]],bounds=[[0,0,0],[15.4,12,3.7]],
               intrinsics=[216,0,160,0,216,120,0,0,1],flight_limits=[.7,3.],
               active_review=False,recover_frontiers=True,stop_and_plan=True)
    configs=configuration(hello,tmp_path,root/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml')
    p=next(c['parameters'] for c in configs if c['executable']=='exploration_node')
    assert p['search.recover_frontiers'] and p['search.stop_and_plan']
    assert not p['search.active_review']
    assert p['fsm.authorized_execution'] and not p['sdf_map.optimistic']
    assert p['sdf_map.obstacles_inflation']==.6


def test_vertical_sampling_is_opt_in_and_preserves_safety_limits(tmp_path):
    root=Path(__file__).resolve().parents[1]
    template=root/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml'
    hello=dict(starts=[[4.3,2.5,1.5]],bounds=[[0,0,0],[15.4,12,3.7]],
               intrinsics=[216,0,160,0,216,120,0,0,1],flight_limits=[.7,3.])
    a=next(c['parameters'] for c in configuration(hello,tmp_path,template) if c['executable']=='exploration_node')
    assert a['frontier.task_vertical_samples'] is False
    hello['vertical_samples']=True
    b=next(c['parameters'] for c in configuration(hello,tmp_path,template) if c['executable']=='exploration_node')
    assert b['frontier.task_vertical_samples'] is True
    assert {k:v for k,v in a.items() if k!='frontier.task_vertical_samples'}=={k:v for k,v in b.items() if k!='frontier.task_vertical_samples'}
