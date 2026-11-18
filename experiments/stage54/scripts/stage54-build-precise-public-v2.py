from pathlib import Path
import json,subprocess,shlex,shutil,hashlib,difflib,re
R=Path('/tmp/racer-stage13-px4/source');B=R/'build/px4_sitl_default';O=Path('/tmp/stage54-precise-public-px4-v2');O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert sha(B/'bin/px4')=='c359b7f3fde4678476d984a75b60d0d258146a90b0d1a3ad66e6a5b142c7968c'
source=R/'platforms/common/uORB/uORB.cpp';original=source.read_text();needle='PX4_INFO_RAW("%.5f", (double) * (float *)(data_ptr + data_offset));';replacement='PX4_INFO_RAW("%.9g", (double) * (float *)(data_ptr + data_offset));';assert original.count(needle)==1;new=original.replace(needle,replacement);assert new.replace(replacement,needle)==original;(O/'uORB.cpp').write_text(new);(O/'public-precision-only.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),new.splitlines(True),fromfile='original-uORB.cpp',tofile='private-uORB.cpp')))
probe=Path('/tmp/stage54-float-roundtrip.cpp').read_text();(O/'float-roundtrip.cpp').write_text(probe)
commands=[]
def run(cmd,tag):
 commands.append(cmd);p=subprocess.run(cmd,cwd=B,capture_output=True,text=True);(O/(tag+'.log')).write_text(p.stdout+p.stderr);p.check_returncode();return p
run(['c++','-std=c++14','-O2',str(O/'float-roundtrip.cpp'),'-o',str(O/'float-roundtrip')],'roundtrip-build');p=run([str(O/'float-roundtrip')],'roundtrip-run');roundtrip=dict(zip(['finite_float32_cases','new_roundtrip_mismatches','original_5decimal_mismatches'],map(int,p.stdout.split())));assert roundtrip['new_roundtrip_mismatches']==0 and roundtrip['original_5decimal_mismatches']>0
entries=json.loads((B/'compile_commands.json').read_text());e=next(x for x in entries if x['file']==str(source));cmd=shlex.split(e['command']);obj=cmd[cmd.index('-o')+1];match=re.match('(.*)CMakeFiles/([^/]+)\\.dir/',obj);assert match,obj;common=B/(match.group(1)+'lib'+match.group(2)+'.a');assert common.exists(),common
gz=B/'src/modules/simulation/gz_bridge/libmodules__simulation__gz_bridge.a';imu=B/'src/modules/sensors/vehicle_imu/libvehicle_imu.a';baseline={'binary':sha(B/'bin/px4'),'uorb_source':sha(source),'common_archive':sha(common),'gz_archive':sha(gz),'vehicle_imu_archive':sha(imu)}
cmd[cmd.index('-o')+1]=str(O/'uORB.cpp.o');cmd[cmd.index('-c')+1]=str(O/'uORB.cpp');cmd.append('-I'+str(source.parent));run(cmd,'compile-uorb-debug-format')
newcommon=O/common.name;shutil.copy2(common,newcommon);members=subprocess.check_output(['ar','t',str(newcommon)],text=True).splitlines();assert members.count('uORB.cpp.o')==1;run(['ar','r',str(newcommon),str(O/'uORB.cpp.o')],'archive-common')
newgz=O/gz.name;shutil.copy2('/tmp/stage53-metadata-only-px4/'+gz.name,newgz)
cmd=shlex.split(Path('/tmp/stage48-original-px4-link.txt').read_text().split('&&')[1].strip());cmd[cmd.index('-o')+1]=str(O/'px4')
link_replacements={}
for old,newpath in [(str(common.relative_to(B)),str(newcommon)),(str(gz.relative_to(B)),str(newgz))]:
 count=cmd.count(old);assert count>=1,old;link_replacements[old]=count;cmd=[newpath if x==old else x for x in cmd]
run(cmd,'link');(O/'bin').mkdir();(O/'bin/px4').symlink_to(O/'px4')
for p in (B/'bin').iterdir():
 if p.name=='px4':continue
 if p.is_symlink():(O/'bin'/p.name).symlink_to('px4')
 else:shutil.copy2(p,O/'bin'/p.name)
(O/'etc').symlink_to(B/'etc',target_is_directory=True)
after={'binary':sha(B/'bin/px4'),'uorb_source':sha(source),'common_archive':sha(common),'gz_archive':sha(gz),'vehicle_imu_archive':sha(imu)};assert baseline==after
symbols=subprocess.check_output(['nm','-C',str(O/'px4')],text=True);assert 'stage48_trace' not in symbols and b'STAGE48_TRACE_DIRECTORY' not in (O/'px4').read_bytes()
report={'binary':str(O/'px4'),'binary_sha256':sha(O/'px4'),'original_artifacts_before':baseline,'original_artifacts_after':after,'originals_unchanged':True,'metadata_source_sha256':'877ba32747b5c870804241153e7a48b8138b11236cd866cb80f32965dfffa05c','private_uorb_source_sha256':sha(O/'uORB.cpp'),'float_roundtrip':roundtrip,'debug_only_change':'one Float32 printf format string from5decimal to9significant digits; all uORB operations/message/queue/estimator/controller code identical','trace_instrumentation':False,'VehicleIMU_archive_original':True,'not_installed_over_original':True,'formal_acceptance':False,'commands':commands,'link_archive_multiplicity_preserved':link_replacements};(O/'metadata.json').write_text(json.dumps(report,indent=2));print(json.dumps(report|{'commands':None,'commands_count':len(commands)},indent=2))
