from pathlib import Path
import hashlib,json,shutil,subprocess
O=Path('/tmp/stage49-feedback-msg');O.mkdir(exist_ok=False);P=O/'src/px4_msgs';(P/'msg').mkdir(parents=True)
ros=Path('/planner/external/ros_ws/src/px4_msgs');sdk=Path('/tmp/racer-stage13-px4/source/msg/SensorCombined.msg')
def fields(p):return [' '.join(x.split('#')[0].split()) for x in p.read_text().splitlines() if x.split('#')[0].strip() and '=' not in x.split('#')[0]]
assert fields(sdk)==fields(ros/'msg/SensorCombined.msg'),(fields(sdk),fields(ros/'msg/SensorCombined.msg'))
shutil.copy2(ros/'msg/SensorCombined.msg',P/'msg/SensorCombined.msg');shutil.copy2(ros/'package.xml',P/'package.xml')
for p in ros.glob('LICENSE*'):shutil.copy2(p,P/p.name)
(P/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.8)
project(px4_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/SensorCombined.msg")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
''')
(O/'interface-proof.json').write_text(json.dumps(dict(source=str(ros/'msg/SensorCombined.msg'),source_sha256=hashlib.sha256((ros/'msg/SensorCombined.msg').read_bytes()).hexdigest(),sdk_source=str(sdk),sdk_sha256=hashlib.sha256(sdk.read_bytes()).hexdigest(),ordered_wire_fields_equal=True,ordered_wire_fields=fields(sdk),scope='Separate diagnostic read-only subscriber binding only; production ROS installs and PX4 unchanged'),indent=2)+'\n')
print('exact existing wire fields verified, independent binding source prepared')
