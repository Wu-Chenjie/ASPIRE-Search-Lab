from pathlib import Path
import os,subprocess,shlex,json,hashlib
O=Path('/tmp/stage49-request-build');O.mkdir(exist_ok=False)
env=dict(os.environ);env['PKG_CONFIG_PATH']=os.pathsep.join([str(p) for p in Path('/opt/ros/jazzy/opt').glob('*/lib/pkgconfig')])+os.pathsep+env.get('PKG_CONFIG_PATH','')
flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','gz-transport13','gz-msgs10'],env=env,text=True));cmd=['c++','-std=c++17','-O2','/tmp/stage49-native-request.cpp','-o','/tmp/stage49-native-request',*flags];p=subprocess.run(cmd,text=True,capture_output=True);(O/'build.log').write_text(p.stdout+p.stderr);p.check_returncode();(O/'metadata.json').write_text(json.dumps(dict(command=cmd,source_sha256=hashlib.sha256(Path('/tmp/stage49-native-request.cpp').read_bytes()).hexdigest(),binary_sha256=hashlib.sha256(Path('/tmp/stage49-native-request').read_bytes()).hexdigest()),indent=2)+'\n');print('native public requester compiled')
