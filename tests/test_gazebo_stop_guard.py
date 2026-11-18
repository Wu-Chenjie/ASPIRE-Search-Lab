"""Guard integrity, unmodified parent environment and fail-closed real mapping."""
from pathlib import Path
from types import SimpleNamespace as NS
import json,os,sys,subprocess
import pytest

@pytest.fixture
def assets(tmp_path,monkeypatch):
    import tools.flight.gazebo_guard as g
    monkeypatch.setattr(g.sys,'platform','linux')
    plugin=tmp_path/'sensors.so';plugin.write_bytes(b'locked native plugin')
    monkeypatch.setattr(g,'PLUGIN',plugin);monkeypatch.setattr(g,'PLUGIN_SHA',g.sha(plugin))
    binary=tmp_path/'guard.so';binary.write_bytes(b'\x7fELFtest')
    meta=dict(schema=1,source_sha256=g.sha(g.SOURCE),binary_sha256=g.sha(binary),sensors_plugin_sha256=g.sha(plugin))
    manifest=Path(str(binary)+'.json');manifest.write_text(json.dumps(meta))
    scene=tmp_path/'scene.sdf';scene.write_text('<sdf><world name="one"><plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system"/></world></sdf>')
    return g,binary,manifest,meta,scene

def test_no_requested_guard_keeps_historical_environment(tmp_path):
    from tools.flight.gazebo_guard import configuration
    assert configuration(tmp_path/'absent.sdf',environ={})==({},None)

def test_verified_guard_applies_to_child_without_mutating_parent(assets):
    g,binary,manifest,meta,scene=assets;parent={'RACER_GZ_SHUTDOWN_GUARD':str(binary)}
    env,report=g.configuration(scene,parent)
    assert env=={'LD_PRELOAD':str(binary)} and report['enabled'] and report['binary_sha256']==meta['binary_sha256']
    assert parent=={'RACER_GZ_SHUTDOWN_GUARD':str(binary)}

@pytest.mark.parametrize('field',['source_sha256','binary_sha256','sensors_plugin_sha256'])
def test_incompatible_source_binary_or_plugin_cannot_launch(assets,field):
    g,binary,manifest,meta,scene=assets;meta[field]='0'*64;manifest.write_text(json.dumps(meta))
    with pytest.raises(ValueError,match='hash|version'):
        g.configuration(scene,{'RACER_GZ_SHUTDOWN_GUARD':str(binary)})

@pytest.mark.parametrize('flag',['LD_PRELOAD','RACER_GZ_STOP_FORCE_WINDOW','RACER_GZ_STOP_SERIALIZE','RACER_GZ_STOP_TRACE','RACER_GZ_BT_FILE'])
def test_diagnostic_injection_environment_cannot_enter_comparison(assets,flag):
    g,binary,manifest,meta,scene=assets
    with pytest.raises(ValueError,match='diagnostic|preload'):
        g.configuration(scene,{'RACER_GZ_SHUTDOWN_GUARD':str(binary),flag:'unreviewed'})

@pytest.mark.parametrize('xml',['<sdf><world/><world/></sdf>','<sdf><world/></sdf>','<sdf><world><plugin name="gz::sim::systems::Sensors"/><plugin name="gz::sim::systems::Sensors"/></world></sdf>'])
def test_unvalidated_world_sensor_scope_cannot_launch(assets,xml):
    g,binary,manifest,meta,scene=assets;scene.write_text(xml)
    with pytest.raises(ValueError,match='one world'):
        g.configuration(scene,{'RACER_GZ_SHUTDOWN_GUARD':str(binary)})

def test_native_mapping_verifies_both_actual_files(tmp_path):
    from tools.flight.gazebo_guard import verify_loaded
    p=tmp_path/'55';p.mkdir();(p/'cmdline').write_bytes(b'gz sim -r scene.sdf')
    (p/'maps').write_text('100-200 r-xp 0 00:01 1 /tmp/guard.so\n200-300 r-xp 0 00:01 2 /native/sensors.so\n')
    r=dict(binary_path='/tmp/guard.so',sensors_plugin_path='/native/sensors.so')
    verify_loaded(NS(pid=55,poll=lambda:None),r,timeout=0,proc_root=tmp_path)
    assert r['loaded_verified'] and r['loaded_pid']==55

@pytest.mark.parametrize('paths',['/tmp/guard.so.bad\n/native/sensors.so','/tmp/guard.so (deleted)\n/native/sensors.so','/tmp/guard.so\n/native/wrong.so'])
def test_missing_deleted_or_prefix_only_map_fails_closed(tmp_path,paths):
    from tools.flight.gazebo_guard import verify_loaded
    p=tmp_path/'55';p.mkdir();(p/'cmdline').write_bytes(b'gz sim -r scene.sdf');(p/'maps').write_text(''.join('100-200 r-xp 0 00:01 1 '+s+'\n' for s in paths.splitlines()))
    with pytest.raises(RuntimeError,match='load not verified'):
        verify_loaded(NS(pid=55,poll=lambda:None),dict(binary_path='/tmp/guard.so',sensors_plugin_path='/native/sensors.so'),timeout=0,proc_root=tmp_path)

def test_actual_production_elf_contains_no_injection_or_join_override():
    path=os.environ.get('RACER_GZ_TEST_GUARD')
    if not path:pytest.skip('Opt-in actual Linux production guard artifact')
    data=Path(path).read_bytes();assert data.startswith(b'\x7fELF')
    for marker in [b'RACER_GZ_STOP_FORCE_WINDOW',b'RACER_GZ_STOP_SERIALIZE',b'RACER_GZ_STOP_TRACE',b'RACER_GZ_BT_FILE',b'JOIN_BEGIN',b'STOP_ENTER']:
        assert marker not in data
    symbols=subprocess.check_output(['nm','-D','--defined-only',path],text=True)
    assert '_ZN2gz3sim2v87systems14SensorsPrivate4StopEv' in symbols
    assert not any(s.split()[-1]=='pthread_join' for s in symbols.splitlines() if s.split())
