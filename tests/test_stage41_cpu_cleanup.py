"""Existing-CPU isolation, fixed cleanup budget and owned-orphan preservation."""
from types import SimpleNamespace as NS
from pathlib import Path
import json,os,signal,subprocess,sys,time,shutil
import pytest

def test_common_cpu_roles_remain_inside_existing_allowed_set(monkeypatch):
    from tools.flight.cpu_profile import CPUProfile
    monkeypatch.setattr(shutil,'which',lambda _: '/usr/bin/taskset')
    p=CPUProfile('render3-perception2',allowed=[2,3,4,6,7])
    assert p.command('gazebo',['gz','sim'])==['/usr/bin/taskset','-c','2,3,4','gz','sim']
    assert p.command('person_search',['python'])==['/usr/bin/taskset','-c','6,7','python']
    assert p.command('px4',['px4'])==['px4']
    assert p.report()['other_roles_inherit_parent']
    with pytest.raises(ValueError,match='5 existing'):CPUProfile('render3-perception2',allowed=range(4))
    assert CPUProfile().command('gazebo',['gz'])==['gz']

def test_real_child_affinity_is_installed_before_cpu_work():
    from tools.flight.cpu_profile import CPUProfile
    if not hasattr(os,'sched_getaffinity') or not shutil.which('taskset'):pytest.skip('Linux affinity required')
    if len(os.sched_getaffinity(0))<5:pytest.skip('Five existing CPUs required')
    profile=CPUProfile('render3-perception2')
    code="import os,json,sys;print(json.dumps(sorted(os.sched_getaffinity(0))),flush=True);sys.stdin.read()"
    p=subprocess.Popen(profile.command('person_search',[sys.executable,'-c',code]),stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    try:
        profile.verify('person_search',p)
        assert json.loads(p.stdout.readline())==profile.roles['person_search']
        assert profile.report()['verified']['person_search']==profile.roles['person_search']
    finally:p.communicate(timeout=5)

def test_graceful_hook_preserves_orphan_ownership_without_extending_eight_seconds(monkeypatch):
    from tools.racer_ros2.light import processes as m
    clock=[0.];table={100:(1,'parent'),101:(100,'child'),999:(1,'foreign')};signals=[];exited=[False]
    parent=NS(pid=100,returncode=0,poll=lambda:0 if exited[0] else None,wait=lambda timeout:0)
    monkeypatch.setattr(m,'snapshot',lambda:dict(table))
    monkeypatch.setattr(m,'same_process',lambda pid,birth:table.get(pid,(None,None))[1]==birth)
    monkeypatch.setattr(m.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(m.time,'sleep',lambda seconds:clock.__setitem__(0,clock[0]+seconds))
    def kill(pid,sig):
        signals.append((pid,sig,clock[0]))
        if sig==signal.SIGKILL:table.pop(pid,None)
    monkeypatch.setattr(m.os,'kill',kill)
    def stop_parent():
        clock[0]+=3.;table.pop(100);table[101]=(1,'child');exited[0]=True
    r=m.terminate([parent],timeout=8,before_signals=stop_parent)
    assert r['owned_processes']==2 and r['forced_kill_pids']==[101] and r['remaining_pids']==[]
    assert all(pid!=999 for pid,_,_ in signals)
    when=next(t for pid,sig,t in signals if sig==signal.SIGKILL)
    assert 8.<=when<8.1

def test_foreign_gazebo_partition_is_never_contacted(monkeypatch,tmp_path):
    from tools.flight.gazebo_shutdown import stop_owned_server
    calls=[];monkeypatch.setattr(subprocess,'run',lambda *a,**k:calls.append(a))
    with pytest.raises(ValueError,match='foreign Gazebo partition'):
        stop_owned_server(NS(pid=1,poll=lambda:None),tmp_path,environ={'GZ_PARTITION':'someone_else'})
    assert calls==[]

def test_public_paused_gazebo_stop_exits_zero_with_original_cleanup_budget(tmp_path,monkeypatch):
    from tools.flight.gazebo_shutdown import terminate_flight
    if not Path('/proc').is_dir() or not shutil.which('gz'):pytest.skip('Native Gazebo environment required')
    out=tmp_path/'paused-server';out.mkdir();partition='person_flight_'+out.name
    monkeypatch.setenv('GZ_PARTITION',partition)
    sdf=out/'world.sdf';sdf.write_text('<sdf version="1.9"><world name="cleanup_probe"/></sdf>')
    log=(out/'server.log').open('w');p=subprocess.Popen(['gz','sim','-s',str(sdf)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    result=None
    try:
        from gz.transport13 import Node,NodeOptions
        options=NodeOptions();options.partition=partition;discovery=Node(options)
        end=time.monotonic()+6
        while time.monotonic()+2.1<end:
            services=discovery.service_list()
            if '/server_control' in services:break
            assert p.poll() is None,(out/'server.log').read_text()
            time.sleep(.1)
        else:pytest.fail('Public server stop service absent within original6s discovery bound')
        del discovery
        result=terminate_flight([p],{p.pid:'gazebo'},out,timeout=8)
        assert p.returncode==0 and not result['forced_kill_pids'] and not result['remaining_pids']
        assert result['gazebo_stop_requests'][0]['accepted']
        assert result['non_display_budget_s']==8
    finally:
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT);p.wait(timeout=5)
        log.close()
