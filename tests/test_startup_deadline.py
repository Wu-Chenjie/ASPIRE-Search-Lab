"""Harness preflight never manufactures odometry or changes flight guards."""
import sys,os,subprocess,signal
from pathlib import Path
import pytest
from tools.flight.run import scene_real_time_factor
from tools.flight.run_safety import StartupDeadline

def test_missing_or_stale_odometry_stops_before_long_wall_watchdog():
 w=StartupDeadline()
 assert w.observe(2.,2.,{'stamp':2.},{'origin':None},{}) is None
 assert not w.has_odometry
 assert w.observe(30.,100.,None,{},{}).startswith('Startup odometry')
 w=StartupDeadline()
 assert w.observe(5.,120.,{'stamp':1.},{'origin':[0,0,0],'origin_calibration':'calibrated'},{}).startswith('Startup odometry')
 assert not w.has_odometry

def test_real_origin_fresh_odometry_starts_graph_then_requires_flight_ready():
 w=StartupDeadline()
 f=dict(origin=[0,0,0],origin_calibration='calibrated',armed=False,offboard=False)
 assert w.observe(4.,10.,{'stamp':3.9},f,{}) is None and w.has_odometry
 assert w.observe(59.,179.,None,f,{'ready':False}) is None
 assert w.observe(60.,179.,None,f,{'ready':False}).startswith('Startup flight-ready')
 w=StartupDeadline();f.update(armed=True,offboard=True)
 assert w.observe(10.,20.,{'stamp':9.9},f,{'ready':True}) is None and w.ready
 assert w.observe(180.,500.,None,f,{'ready':False}) is None

@pytest.mark.skipif(sys.platform!='linux',reason='Linux owned-PID cleanup')
def test_cleanup_reports_abort_that_occurs_after_main_loop(tmp_path):
 from tools.racer_ros2.light.processes import terminate
 code="import signal,os,time,resource;resource.setrlimit(resource.RLIMIT_CORE,(0,0));signal.signal(signal.SIGINT,lambda *_:os.abort());print('ready',flush=True);time.sleep(20)"
 child=subprocess.Popen([sys.executable,'-c',code],cwd=tmp_path,stdout=subprocess.PIPE,text=True,start_new_session=True)
 assert child.stdout.readline().strip()=='ready'
 r=terminate([child],timeout=3)
 assert not r['remaining_pids'] and not r['forced_kill_pids']
 assert r['direct_returncodes']==[dict(pid=child.pid,returncode=-signal.SIGABRT,was_running_before_cleanup=True)]



def test_measured_takeoff_ends_preflight_wait_without_claiming_bootstrap_complete():
 w=StartupDeadline();f=dict(origin=[0,0,0],origin_calibration='calibrated',armed=False,offboard=False)
 assert w.observe(4.,10.,{'stamp':4.,'xyz':[0,0,.24]},f,{'ready':False}) is None
 f.update(armed=True,offboard=True)
 assert w.observe(7.,30.,{'stamp':7.,'xyz':[0,0,.55]},f,{'ready':False}) is None
 assert w.ready and w.takeoff_observed and not w.executor_ready
 assert w.observe(61.,181.,None,f,{'ready':False}) is None


def test_scene_uses_prepared_pacing_until_overridden(tmp_path):
    path = tmp_path / 'scene.sdf'
    path.write_text('<sdf><world><physics><max_step_size>.004</max_step_size>'
                    '<real_time_factor>.2</real_time_factor></physics></world></sdf>')
    assert scene_real_time_factor(path, None, False) == (.2,.2)
    assert scene_real_time_factor(path, .1, True) == (.1,.1)
    assert scene_real_time_factor(path, None, True) == (.1,.1)
    assert '<max_step_size>.004</max_step_size>' in path.read_text()


@pytest.mark.parametrize('rtf', [0., 9e-10, .001, 2.])
def test_scene_reconfigures_rates_that_cannot_bound_guarded_timing(tmp_path, rtf):
    path=tmp_path/'scene.sdf'
    path.write_text('<sdf><world><physics><max_step_size>.004</max_step_size>'
                    f'<real_time_factor>{rtf}</real_time_factor></physics></world></sdf>')
    original=path.read_text()
    assert scene_real_time_factor(path,None,False)==(rtf,rtf)
    assert path.read_text()==original
    assert scene_real_time_factor(path,None,True)==(rtf,1.)
    assert '<max_step_size>.004</max_step_size>' in path.read_text()
    assert scene_real_time_factor(path,None,True)==(1.,1.)


@pytest.mark.parametrize('rate', [0., .002, .2, 2.])
def test_scene_can_override_with_normal_gazebo_pacing(tmp_path, rate):
    path=tmp_path/'scene.sdf'
    path.write_text('<sdf><world><physics><max_step_size>.004</max_step_size></physics></world></sdf>')
    assert scene_real_time_factor(path,rate,False)==(rate,rate)
    assert scene_real_time_factor(path,None,False)==(rate,rate)
    if 0.<rate<=1.:
        assert scene_real_time_factor(path,None,True)==(rate,rate)


@pytest.mark.parametrize('value', ['', 'nan', 'inf', 'invalid', '2x'])
def test_scene_rejects_unsupported_rates(tmp_path, value):
    path=tmp_path/'scene.sdf'
    path.write_text(f'<sdf><world><physics><real_time_factor>{value}</real_time_factor></physics></world></sdf>')
    with pytest.raises(ValueError,match='real_time_factor'):
        scene_real_time_factor(path,None,False)
    with pytest.raises(ValueError,match='real_time_factor'):
        scene_real_time_factor(path,None,True)


def test_scene_without_physics_creates_default_profile_for_override(tmp_path):
    path=tmp_path/'scene.sdf'
    path.write_text('<sdf><world/></sdf>')
    assert scene_real_time_factor(path,None,True)==(1.,1.)
    assert scene_real_time_factor(path,.1,False)==(.1,.1)
    assert scene_real_time_factor(path,None,True)==(.1,.1)
    assert '<max_step_size>0.001</max_step_size>' in path.read_text()


def test_scene_rejects_missing_world(tmp_path):
    path=tmp_path/'scene.sdf'
    path.write_text('<sdf/>')
    with pytest.raises(ValueError,match='Scene has no world'):
        scene_real_time_factor(path,.1,False)


@pytest.mark.parametrize('rtf,wall', [(1., (120., 180.)), (.1, (360., 660.)), (0., (120., 180.))])
def test_slow_startup_keeps_sim_deadlines_and_accounts_for_wall_pacing(rtf, wall):
    watchdog = StartupDeadline(real_time_factor=rtf)
    info = watchdog.report()
    assert (info['odom_wall_s'], info['ready_wall_s']) == wall
    assert (info['odom_sim_s'], info['ready_sim_s']) == (30., 60.)


def test_slow_startup_accepts_fresh_odometry_after_old_wall_deadline():
    watchdog = StartupDeadline(real_time_factor=.1)
    flight = dict(origin=[0, 0, 0], origin_calibration='calibrated', armed=False, offboard=False)
    assert watchdog.observe(13., 130., None, flight, {}) is None
    assert watchdog.observe(14., 140., {'stamp': 14., 'xyz': [0, 0, .24]}, flight, {}) is None
    assert watchdog.has_odometry
    assert watchdog.observe(19., 190., None, flight, {}) is None
    flight.update(armed=True, offboard=True)
    assert watchdog.observe(20., 200., {'stamp': 20., 'xyz': [0, 0, .55]}, flight, {}) is None
    assert watchdog.ready and watchdog.takeoff_observed


@pytest.mark.parametrize('odom_deadline,ready_deadline', [((0., 360.), (3., 660.)), ((30., 300.), (60., 600.))])
def test_slow_startup_still_stops_on_stall_or_sim_deadline(odom_deadline, ready_deadline):
    watchdog = StartupDeadline(real_time_factor=.1)
    assert watchdog.observe(*odom_deadline, None, {}, {}).startswith('Startup odometry')
    watchdog = StartupDeadline(real_time_factor=.1)
    flight = dict(origin=[0, 0, 0], origin_calibration='calibrated')
    assert watchdog.observe(3., 30., {'stamp': 3.}, flight, {}) is None
    assert watchdog.observe(*ready_deadline, None, flight, {}).startswith('Startup flight-ready')
