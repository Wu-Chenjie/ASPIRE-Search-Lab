"""Build outside code tree; preserve the verified migrated source snapshot."""
import argparse,shutil,sys
from importlib import import_module
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from tools.racer_ros2.verify import verified_migration
from tools.flight.optimizer_overlay import replace_once
p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--flight-fixture',action='store_true');p.add_argument('--map-probe',action='store_true');p.add_argument('--provenance-probe',action='store_true');p.add_argument('--execution-safety',action='store_true');p.add_argument('--consistent-inflation',action='store_true');a=p.parse_args()
if a.output.resolve().is_relative_to(ROOT):raise ValueError('Build must be outside the source tree')
base = ROOT/'external/racer_ros2_ws'
_, migration_sha256 = verified_migration(base)
a.output.mkdir(parents=True, exist_ok=True)
if any(a.output.iterdir()):
    raise FileExistsError(f'Build directory is not empty: {a.output}')
shutil.copytree(base/'src', a.output/'src')
core=a.output/'src/racer_core'
shutil.copyfile(ROOT/'tools/search/native_driver.cpp',core/'native_driver.cpp')
shutil.copyfile(ROOT/'tools/flight/native_recovery_probe.cpp',core/'task_recovery_probe.cpp')
# The migration manifest already verifies the CMake input before copying.
cm=(core/'CMakeLists.txt').read_text()
for anchor, replacement in (
    ('add_executable(exploration_node', 'add_library(racer_algorithms OBJECT'),
    ('  src/exploration_manager/exploration_node.cpp\n', ''),
    ('  src/exploration_manager/fast_exploration_fsm.cpp\n', ''),
    ('ament_target_dependencies(exploration_node', 'ament_target_dependencies(racer_algorithms'),
    ('target_link_libraries(exploration_node', 'target_link_libraries(racer_algorithms'),
):
    cm = cm.replace(anchor, replacement)
extra="""
add_executable(exploration_node src/exploration_manager/exploration_node.cpp src/exploration_manager/fast_exploration_fsm.cpp)
add_executable(search_racer native_driver.cpp)
add_executable(task_recovery_probe task_recovery_probe.cpp)
foreach(target exploration_node search_racer task_recovery_probe)
  target_link_libraries(${target} racer_algorithms)
  ament_target_dependencies(${target} rclcpp racer_interfaces std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs cv_bridge message_filters pcl_conversions ament_index_cpp)
endforeach()
"""
cm=cm.replace('install(TARGETS exploration_node',extra+'\ninstall(TARGETS task_recovery_probe search_racer exploration_node')
cm=cm.replace('ament_target_dependencies(runtime_probe rclcpp std_msgs)','ament_target_dependencies(runtime_probe rclcpp std_msgs racer_interfaces nav_msgs sensor_msgs geometry_msgs visualization_msgs)')
fsm = ' src/exploration_manager/fast_exploration_fsm.cpp'
for enabled, folder, source, target, companion in (
    (a.map_probe, 'hm3d', 'native_map_probe', 'hm3d_map_probe', ''),
    (a.provenance_probe, 'hm3d', 'native_provenance_probe', 'hm3d_provenance_probe', ''),
    (a.execution_safety, 'flight', 'native_authorization_recovery_probe', 'task_authorization_recovery_probe', fsm),
    (a.execution_safety, 'flight', 'native_review_continuation_probe', 'task_review_continuation_probe', fsm),
):
    if not enabled:
        continue
    filename = target + '.cpp' if folder == 'flight' else source + '.cpp'
    shutil.copyfile(ROOT/'tools'/folder/(source+'.cpp'), core/filename)
    cm = cm.replace('ament_package()', f"""add_executable({target} {filename}{companion})
target_link_libraries({target} racer_algorithms)
ament_target_dependencies({target} rclcpp racer_interfaces std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs cv_bridge message_filters pcl_conversions ament_index_cpp)
install(TARGETS {target} DESTINATION lib/${{PROJECT_NAME}})
ament_package()""")
(core/'CMakeLists.txt').write_text(cm)
for name,target in [('runtime.hpp','include/racer_ros2/runtime.hpp'),('runtime_probe.cpp','runtime_probe.cpp')]:
    shutil.copyfile(ROOT/'tools/search/compat'/name,core/target)
frontier=core/'src/active_perception/frontier_finder.cpp'
text=frontier.read_text();anchor='auto vectors = es.eigenvectors().real();'
frontier.write_text(replace_once(text,anchor,'Eigen::Matrix2d vectors = es.eigenvectors().real();'))
manager=core/'src/exploration_manager/fast_exploration_manager.cpp'
text=manager.read_text();anchor='  shortenPath(ed_->path_next_goal_);'
manager.write_text(replace_once(text,anchor,anchor+'\n  if (ed_->path_next_goal_.size() < 3) { ROS_WARN("Degenerate viewpoint path"); return FAIL; }'))
# The overlays modify the same pinned sources in this order.
for module, enabled in (
    ('flight.fsm_overlay', True),
    ('flight.recovery_overlay', True),
    ('flight.return_overlay', True),
    ('flight.dynamics_overlay', True),
    ('hm3d.frontier_diagnostics', True),
    ('hm3d.vertical_sampling', True),
    ('hm3d.inflation_overlay', a.consistent_inflation),
    ('hm3d.provenance_overlay', a.provenance_probe),
    ('flight.safety_map_overlay', a.execution_safety),
    ('flight.optimizer_overlay', True),
    ('flight.flight_fixture_overlay', a.flight_fixture),
    ('flight.authorization_recovery_overlay', a.execution_safety),
    ('flight.review_continuation_overlay', True),
    ('flight.bearing_overlay', True),
    ('flight.target_registry_overlay', True),
    ('flight.static_replan_overlay', a.execution_safety),
    ('flight.static_cancellation_overlay', a.execution_safety),
    ('flight.static_c2_overlay', a.execution_safety),
    ('flight.static_c2_boundary_overlay', a.execution_safety),
    ('flight.known_transit_overlay', a.execution_safety),
    ('flight.frontier_steps_overlay', a.execution_safety),
    ('flight.return_spline_overlay', a.execution_safety),
    ('flight.collision_stop_overlay', a.execution_safety),
    ('flight.fresh_review_overlay', a.execution_safety),
):
    if enabled:
        import_module('tools.' + module).apply(core)
import hashlib,json
(a.output/'task-overlay.json').write_text(json.dumps({'execution_safety':a.execution_safety,'diagnostic_map_probe':a.map_probe,'diagnostic_provenance_probe':a.provenance_probe,'consistent_inflation':a.consistent_inflation,'base_migration_sha256':migration_sha256,'changes':['opt-in static-event preview/replan/publication checks in execution-safety builds (default off; original stop-and-plan retained)', 'opt-in bounded same-request review continuation with correlated actual arrival and deferred cancellation (default off)', 'native viewpoint rejection diagnostics and opt-in bounded altitude sampling (default off)', 'single-machine stop-and-plan driver', 'constant-yaw normalization and fail-closed optimizer proposal transaction', 'opt-in bounded native no-path recovery and Astar diagnostics', 'opt-in native hard-rest boundaries and bounded common time feasibility', 'opt-in guarded flight review/frontier recovery and stop-and-plan FSM','reuse latest original-project ROS2 runtime shutdown fixes and probe','own Eigen frontier eigenvectors temporary','reject degenerate viewpoint path'],'files':{str(f.relative_to(a.output)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (a.output/'src').rglob('*') if f.is_file()}},indent=2)+'\n')
print(a.output)
