"""Isolated real-node DDS fixture; no physics, PX4, bridge or aircraft."""
import importlib,sys,os,json,time
from pathlib import Path
from types import SimpleNamespace as N
sys.path.insert(0,'/tmp/stage65-measurement')
import stage65_capture as c
original=c.receive;counts={};first={}
def traced(node,sink,role,data):
    counts[role]=counts.get(role,0)+1
    if counts[role]<=10:print(json.dumps(dict(component_receipt=role,count=counts[role],drone=node.id,session=sink.session)),flush=True)
    if role=='executor':
        first.setdefault(role,time.monotonic())
        if os.environ.get('STAGE66_SILENT_EXECUTOR')=='1':return
        if time.monotonic()-first[role]<float(os.environ.get('STAGE66_DELAY_EXECUTOR','0')):return
    return original(node,sink,role,data)
c.receive=traced
role=sys.argv.pop(1)
if role=='gateway':
    import rclpy
    from tools.research.frontier_baselines.live_ros2 import NativeLeaseAgent
    rclpy.init();c.install_gateway(NativeLeaseAgent)
    config=json.loads(Path(os.environ['STAGE66_COMPONENT_CONFIG']).read_text());node=NativeLeaseAgent(0,config,N(send=lambda data:None),Path(os.environ['STAGE66_COMPONENT_OUTPUT'])/'flight')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node();node.log.close()
        if rclpy.ok():rclpy.shutdown()
else:
    import annual_runtime.map_evidence as wiring
    wiring.bind_capture=c.bind_capture
    module=importlib.import_module('annual_runtime.'+role);module.main()
