"""Pinned teardown-only native guard; no scheduling injection or simulator edits."""
from pathlib import Path
import os,sys,json,hashlib,time,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'tools/flight/native/gz_sensor_stop_guard.cpp'
PLUGIN=Path('/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim-sensors-system.so')
DIAGNOSTIC_FLAGS=('RACER_GZ_STOP_FORCE_WINDOW','RACER_GZ_STOP_SERIALIZE','RACER_GZ_STOP_TRACE','RACER_GZ_BT_FILE')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def configuration(scene,environ=None):
    env=os.environ if environ is None else environ;requested=env.get('RACER_GZ_SHUTDOWN_GUARD')
    if not requested:return {},None
    if sys.platform!='linux':raise ValueError('Guard requires verified Linux runtime')
    if env.get('LD_PRELOAD') or any(k in env for k in DIAGNOSTIC_FLAGS):raise ValueError('Unreviewed preload or diagnostic injection environment')
    binary=Path(requested).resolve(strict=True);meta=json.loads(Path(str(binary)+'.json').read_text())
    source_sha=sha(SOURCE);binary_sha=sha(binary)
    if meta.get('schema')!=1 or meta.get('source_sha256')!=source_sha or meta.get('binary_sha256')!=binary_sha:raise ValueError('Guard source/binary hash mismatch')
    if not binary.read_bytes().startswith(b'\x7fELF'):raise ValueError('Expected Linux guard ELF')
    plugin_sha=meta.get('sensors_plugin_sha256')
    if not isinstance(plugin_sha,str) or not plugin_sha or not PLUGIN.is_file() or sha(PLUGIN)!=plugin_sha:raise ValueError('Unverified native sensors plugin version')
    worlds=ET.parse(scene).getroot().findall('world')
    plugins=[p for w in worlds for p in w.findall('plugin') if p.get('name')=='gz::sim::systems::Sensors']
    if len(worlds)!=1 or len(plugins)!=1:raise ValueError('Guard verified only for one world and one Sensors system')
    if plugins[0].get('filename') not in ('gz-sim-sensors-system','libgz-sim-sensors-system.so'):raise ValueError('Unverified Sensors plugin filename')
    return {'LD_PRELOAD':str(binary)},dict(sensors_plugin_path=str(PLUGIN.resolve()),enabled=True,binary_path=str(binary),binary_sha256=binary_sha,source_sha256=source_sha,sensors_plugin_sha256=plugin_sha,scope='Teardown SensorsPrivate.Stop serialization only; one world/one Sensors; no diagnostic injection')
def verify_loaded(process,report,timeout=2.,proc_root=Path('/proc')):
    if report is None:return
    deadline=time.monotonic()+timeout
    while True:
        p=proc_root/str(process.pid)
        try:
            command=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace');maps=(p/'maps').read_text()
            paths={s.split(None,5)[5] for s in maps.splitlines() if len(s.split(None,5))==6}
            if 'gz sim ' in command and report['binary_path'] in paths and report['sensors_plugin_path'] in paths:
                report.update(loaded_verified=True,loaded_pid=process.pid);return
        except (OSError,StopIteration):pass
        if process.poll() is not None or time.monotonic()>=deadline:raise RuntimeError('Native Gazebo guard load not verified')
        time.sleep(.01)
