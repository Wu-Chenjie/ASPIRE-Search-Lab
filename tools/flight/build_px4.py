"""Offline isolated SITL build: publish real EKF flags at 10 Hz, guards unchanged."""
import argparse,hashlib,json,os,shutil,subprocess
from pathlib import Path

EKF_SHA='c032e72d2ff4582a319baa7cc44016ea5e7869fc8bd0ad5630a46fe6e5cf6bd5'

def local_microcdr(src, copy, out):
    local=src/'build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr'
    dest=out/'microcdr-source';shutil.copytree(local,dest,symlinks=True)
    cm=copy/'src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client/cmake/SuperBuild.cmake'
    text=cm.read_text();needle='GIT_REPOSITORY\n            https://github.com/eProsima/Micro-CDR.git\n        GIT_TAG\n            ${_microcdr_tag}'
    if text.count(needle)!=1:raise ValueError('Unexpected PX4 SuperBuild CMake source')
    cm.write_text(text.replace(needle,'SOURCE_DIR "'+str(dest)+'"\n        DOWNLOAD_COMMAND ""\n        UPDATE_COMMAND ""'))
    return {str(f.relative_to(dest)):hashlib.sha256(f.read_bytes()).hexdigest() for f in dest.rglob('*') if f.is_file() and '.git' not in f.parts}

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--python',default='/planner/external/px4_venv/bin/python3');p.add_argument('--jobs',type=int,default=3);p.add_argument('--ros-setup',default='/opt/ros/jazzy/setup.bash');a=p.parse_args()
    src=a.source.resolve();out=a.output.resolve();root=Path(__file__).resolve().parents[2]
    if out==src or src in out.parents or out==root or root in out.parents:raise ValueError('Build outside original and task source trees')
    ekf=src/'src/modules/ekf2/EKF2.cpp';raw=ekf.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=EKF_SHA:raise ValueError('Unexpected PX4 EKF2 source; review before applying overlay')
    out.mkdir(parents=True,exist_ok=True);copy=out/'source'
    if any(out.iterdir()):raise FileExistsError(f'Build directory is not empty: {out}')
    # Preserve the existing migrated/dirty source and submodules, never checkout/reset.
    # copytree creates source exclusively, including concurrent calls on a precreated output.
    shutil.copytree(src,copy,symlinks=True,ignore=lambda path,names:['build'] if Path(path)==src else [])
    target=copy/ekf.relative_to(src)
    if not target.resolve().is_relative_to(copy):raise ValueError('EKF2 overlay target escapes isolated source')
    # The locked input above includes this exact publish condition.
    patched=raw.replace(b'bool update = (timestamp >= _last_status_flags_publish + 1_s);',
                        b'bool update = (timestamp >= _last_status_flags_publish + 100_ms);')
    patched=patched.replace(b'// publish at ~ 1 Hz (or immediately if filter control status or fault status changes)',
                            b'// Task SITL: real state snapshots at 10 Hz; state/fault changes still publish immediately.')
    target.write_bytes(patched)
    dependency_hashes=local_microcdr(src,copy,out)
    build=copy/'build/px4_sitl_default'
    raw_env=subprocess.check_output(['bash','-c','source "$1" >/dev/null && env -0','bash',a.ros_setup])
    ros_env=dict(item.decode().split('=',1) for item in raw_env.split(b'\0') if b'=' in item)
    env=dict(ros_env,GZ_DISTRO='harmonic',GIT_SUBMODULES_ARE_EVIL='1',GIT_CONFIG_COUNT='1',GIT_CONFIG_KEY_0='protocol.allow',GIT_CONFIG_VALUE_0='never')
    report=dict(local_microcdr_source_hashes=dependency_hashes,source=str(src),build=str(build),source_ekf_sha256=EKF_SHA,patched_ekf_sha256=hashlib.sha256(patched).hexdigest(),flags_period_sim_s=.1,guards_changed=False,submodules='Existing snapshot copied; no update or download',source_git_head=subprocess.check_output(['git','-C',str(src),'rev-parse','HEAD'],text=True).strip(),source_git_status=subprocess.check_output(['git','-C',str(src),'status','--porcelain'],text=True))
    (out/'build-manifest.json').write_text(json.dumps(report,indent=2))
    subprocess.run(['cmake','-S',str(copy),'-B',str(build),'-G','Ninja','-DCONFIG=px4_sitl_default','-DCMAKE_BUILD_TYPE=RelWithDebInfo','-DPYTHON_EXECUTABLE='+a.python],env=env,check=True)
    subprocess.run(['cmake','--build',str(build),'--target','px4','-j',str(a.jobs)],env=env,check=True)
    bridge=build/'bin/px4-gz_bridge'
    if not bridge.is_file() or not os.access(bridge,os.X_OK):raise RuntimeError('Gazebo bridge missing or non-executable: check sourced ROS vendor paths')
    if ekf.read_bytes()!=raw:raise RuntimeError('Original source changed during isolated build')
    report['binary_sha256']=hashlib.sha256((build/'bin/px4').read_bytes()).hexdigest()
    (out/'build-manifest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
