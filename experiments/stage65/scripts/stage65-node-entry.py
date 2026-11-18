"""Explicit measurement hook launcher; original node main/control code unchanged."""
import importlib,sys
import stage65_capture
module_name=sys.argv.pop(1)
allowed={'annual_runtime.pointcloud_mapping_node','annual_runtime.view_executor_node',
         'tools.research.frontier_baselines.live_ros2'}
if module_name not in allowed:raise ValueError('Unexpected diagnostic target')
if module_name.startswith('annual_runtime.'):
    import annual_runtime.map_evidence as wiring
    wiring.bind_capture=stage65_capture.bind_capture
module=importlib.import_module(module_name)
if module_name.endswith('live_ros2'):stage65_capture.install_gateway(module.NativeLeaseAgent)
module.main()
