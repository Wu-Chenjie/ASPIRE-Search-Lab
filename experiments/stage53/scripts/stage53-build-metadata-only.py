from pathlib import Path
import json,subprocess,shlex,shutil,hashlib,difflib
R=Path('/tmp/racer-stage13-px4/source');B=R/'build/px4_sitl_default';O=Path('/tmp/stage53-metadata-only-px4');O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert sha(B/'bin/px4')=='c359b7f3fde4678476d984a75b60d0d258146a90b0d1a3ad66e6a5b142c7968c'
original=(R/'src/modules/simulation/gz_bridge/GZBridge.cpp').read_text();proposed=Path('/tmp/stage50-publisher-readiness-candidate/proposed-GZBridge.cpp').read_text();assert hashlib.sha256(original.encode()).hexdigest()=='7757bd8b77803ac491ead893eee1156bb551d47ef45fedaff57f7dae85db07b7';assert hashlib.sha256(proposed.encode()).hexdigest()=='877ba32747b5c870804241153e7a48b8138b11236cd866cb80f32965dfffa05c'
(O/'GZBridge.cpp').write_text(proposed);(O/'metadata-only.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),proposed.splitlines(True),fromfile='original-GZBridge.cpp',tofile='metadata-only-GZBridge.cpp')))
commands=[]
def run(cmd,tag):
 commands.append(cmd);p=subprocess.run(cmd,cwd=B,capture_output=True,text=True);(O/(tag+'.log')).write_text(p.stdout+p.stderr);p.check_returncode();return p
lib=B/'src/modules/simulation/gz_bridge/libmodules__simulation__gz_bridge.a';imu=B/'src/modules/sensors/vehicle_imu/libvehicle_imu.a';baseline={'binary':sha(B/'bin/px4'),'gz_archive':sha(lib),'vehicle_imu_archive':sha(imu),'source':hashlib.sha256(original.encode()).hexdigest()}
db=json.loads((B/'compile_commands.json').read_text());e=next(x for x in db if x['file'].endswith('/GZBridge.cpp'));cmd=shlex.split(e['command']);cmd[cmd.index('-o')+1]=str(O/'GZBridge.cpp.o');cmd[cmd.index('-c')+1]=str(O/'GZBridge.cpp');cmd.append('-I'+str(Path(e['file']).parent));run(cmd,'compile')
newlib=O/lib.name;shutil.copy2(lib,newlib);run(['ar','r',str(newlib),str(O/'GZBridge.cpp.o')],'archive')
link_source=Path('/tmp/stage48-original-px4-link.txt').read_text();cmd=shlex.split(link_source.split('&&')[1].strip());cmd[cmd.index('-o')+1]=str(O/'px4');old=str(lib.relative_to(B));assert cmd.count(old)==1;cmd[cmd.index(old)]=str(newlib);run(cmd,'link')
(O/'bin').mkdir();(O/'bin/px4').symlink_to(O/'px4')
for p in (B/'bin').iterdir():
 if p.name=='px4':continue
 if p.is_symlink():(O/'bin'/p.name).symlink_to('px4')
 else:shutil.copy2(p,O/'bin'/p.name)
(O/'etc').symlink_to(B/'etc',target_is_directory=True)
after={'binary':sha(B/'bin/px4'),'gz_archive':sha(lib),'vehicle_imu_archive':sha(imu),'source':sha(R/'src/modules/simulation/gz_bridge/GZBridge.cpp')};assert baseline==after
symbols=subprocess.check_output(['nm','-C',str(O/'px4')],text=True);assert 'stage48_trace' not in symbols;data=(O/'px4').read_bytes();assert b'S48TRC1' not in data and b'STAGE48_TRACE_DIRECTORY' not in data
report={'commands':commands,'binary':str(O/'px4'),'binary_sha256':sha(O/'px4'),'only_recompiled_source':'GZBridge.cpp metadata advertise only','source_sha256':sha(O/'GZBridge.cpp'),'original_artifacts_before':baseline,'original_artifacts_after':after,'originals_unchanged':True,'VehicleIMU_and_all_other_archives_original':True,'trace_helper_linked':False,'stage48_trace_symbols_present':False,'not_installed_over_production':True,'formal_acceptance':False,'scope':'Private metadata-only engineering build for further validation under post-recovery continuation approval; no estimator math, IMU values/timestamps/integration/queues or safety limits changed'};(O/'metadata.json').write_text(json.dumps(report,indent=2));print(json.dumps(report|{'commands':None,'commands_count':len(commands)},indent=2))
