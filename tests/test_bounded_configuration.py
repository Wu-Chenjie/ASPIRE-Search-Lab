import json
from pathlib import Path
from tools.flight.run import scene
from tools.racer_ros2.configuration import configuration

def test_bounded_profile_keeps_safety_and_explicit_fallback(tmp_path):
    root=Path(__file__).resolve().parents[1]
    results=[]
    for mode in ['consistent','bounded-fast']:
        out=tmp_path/mode;out.mkdir();cfg=scene(out,False,'partial-left',mode);results.append(cfg)
        hello=dict(starts=cfg['fleet_starts'],bounds=cfg['bounds'],intrinsics=[216,0,160,0,216,120,0,0,1],flight_limits=cfg['racer_flight_limits'],motion_limits=cfg['motion_limits'],planning_limits=cfg['racer_planning_limits'],min_time=cfg['racer_min_time'],feasible_time=cfg.get('racer_feasible_time',False))
        configs=configuration(hello,out,root/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml');p=next(c['parameters'] for c in configs if c['executable']=='exploration_node')
        assert p['fsm.authorized_execution'] and p['sdf_map.obstacles_inflation']>=.6
        assert p['exploration.drone_num']==1
        assert len(configs)==3
        assert not list(out.glob('resource_1_*'))
        assert p['search.feasible_time']==(mode=='bounded-fast')
        assert p['search.hard_rest_boundary']==(mode=='bounded-fast')
        assert p['search.executor_speed']==cfg['motion_limits']['speed']
        assert p['manager.min_time'] is False
    assert results[0]['motion_limits']==dict(speed=1.,acceleration=1.2,jerk=4.,yaw_rate=.9)
    assert results[0]['fleet_starts']==results[1]['fleet_starts']
    assert results[0]['bounds']==results[1]['bounds']
