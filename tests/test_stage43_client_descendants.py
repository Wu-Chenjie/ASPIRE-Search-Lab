"""Actual ros2run descendant shutdown, ownership, quiescence and fixed total budget."""
from types import SimpleNamespace as NS
from pathlib import Path
import os,signal,subprocess,sys,time,shutil
import pytest

def test_pre_stop_keeps_known_owned_orphans_and_excludes_foreign_pid(monkeypatch):
    import tools.flight.gazebo_shutdown as m
    table={101:(1,'root'),102:(101,'child'),103:(102,'grandchild'),999:(1,'foreign')};events=[]
    monkeypatch.setattr(m,'snapshot',lambda:dict(table))
    monkeypatch.setattr(m,'same_process',lambda pid,birth:table.get(pid,(None,None))[1]==birth)
    def kill(pid,sig):
        events.append((pid,sig));table.pop(pid,None)
        if pid==101:table[102]=(1,'child')
    monkeypatch.setattr(m.os,'kill',kill)
    root=NS(pid=101,poll=lambda:None if 101 in table else 0)
    r=m.signal_owned_clients([root],{101:'sensor_bridge'},wait_s=0)
    assert {p for p,s in events}=={101,102,103} and all(s==signal.SIGINT for p,s in events)
    assert 999 in table and r['quiesced'] and r['remaining_alive']==[]

def test_unquiesced_client_skips_public_stop_and_preserves_total_eight_seconds(monkeypatch,tmp_path):
    import tools.flight.gazebo_shutdown as m
    monkeypatch.setattr(m,'signal_owned_clients',lambda *a,**k:dict(quiesced=False,remaining_alive=[2]))
    def forbidden(*a,**k):raise AssertionError('Cannot start server destruction with live clients')
    monkeypatch.setattr(m,'stop_owned_server',forbidden)
    def terminate(children,timeout,defer_pids,before_signals):
        assert timeout==8;before_signals();return dict(forced_kill_pids=[3])
    monkeypatch.setattr(m,'terminate',terminate)
    r=m.terminate_flight([NS(pid=3)],{3:'gazebo'},tmp_path,timeout=8)
    assert r['gazebo_stop_requests'][0]['reason']=='owned_clients_not_quiescent'
    assert r['forced_kill_pids']==[3] and r['non_display_budget_s']==8

def test_independent_gazebo_evaluation_subscriber_is_an_owned_client(monkeypatch):
    import tools.flight.gazebo_shutdown as m
    table={11:(1,'evaluation'),99:(1,'foreign')};signals=[]
    monkeypatch.setattr(m,'snapshot',lambda:dict(table))
    monkeypatch.setattr(m,'same_process',lambda pid,birth:table.get(pid,(None,None))[1]==birth)
    def kill(pid,sig):signals.append(pid);table.pop(pid)
    monkeypatch.setattr(m.os,'kill',kill)
    r=m.signal_owned_clients([NS(pid=11,poll=lambda:None if 11 in table else 0)],{11:'evaluation'},wait_s=0)
    assert signals==[11] and r['quiesced'] and 99 in table

def test_actual_ros2run_wrapper_and_cpp_bridge_both_exit_without_touching_sentinel(tmp_path,monkeypatch):
    if not Path('/proc').is_dir() or not shutil.which('ros2'):pytest.skip('Native ROS2 Linux environment required')
    import yaml
    from tools.flight.gazebo_shutdown import signal_owned_clients
    from tools.racer_ros2.light.processes import snapshot,terminate
    monkeypatch.setenv('ROS_DOMAIN_ID','196');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1');monkeypatch.setenv('GZ_PARTITION','stage43_test_client')
    config=tmp_path/'bridge.yaml';config.write_text(yaml.safe_dump([dict(ros_topic_name='/stage43/test',gz_topic_name='/stage43/test',ros_type_name='std_msgs/msg/String',gz_type_name='gz.msgs.StringMsg',direction='GZ_TO_ROS')]))
    log=(tmp_path/'bridge.log').open('w');p=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(config)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    sentinel=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],start_new_session=True)
    try:
        end=time.monotonic()+5;child=None
        while time.monotonic()<end:
            table=snapshot();child=next((pid for pid,(parent,start) in table.items() if parent==p.pid),None)
            if child and 'Creating GZ->ROS Bridge' in (tmp_path/'bridge.log').read_text():break
            assert p.poll() is None;time.sleep(.05)
        assert child is not None and 'Creating GZ->ROS Bridge' in (tmp_path/'bridge.log').read_text()
        r=signal_owned_clients([p],{p.pid:'sensor_bridge'},wait_s=2)
        assert r['quiesced'] and {v['pid'] for v in r['signals']}=={p.pid,child}
        assert p.wait(timeout=2)==0 and sentinel.poll() is None
    finally:
        terminate([p],timeout=8)
        if sentinel.poll() is None:os.kill(sentinel.pid,signal.SIGTERM)
        sentinel.wait(timeout=2);log.close()
