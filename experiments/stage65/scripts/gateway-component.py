"""Actual native-gateway node in isolated ROS domain. No physics or aircraft."""
from pathlib import Path
import json,os
from types import SimpleNamespace as N
import numpy as np
import rclpy
import stage65_capture as c
from tools.research.frontier_baselines.live_ros2 import NativeLeaseAgent
out=Path('/tmp/stage65-gateway-component');out.mkdir(exist_ok=False)
os.environ.update(RACER_MAP_EVIDENCE_DIR=str(out/'map-evidence'),RACER_OBSERVATION_SESSION='stage65-gateway-no-flight',RACER_EXECUTION_SAFETY='1')
rclpy.init(domain_id=215);c.install_gateway(NativeLeaseAgent)
config=dict(fleet_starts=[[0.,-.9,1.5]],bounds=[[-3.,-4.,0.],[7.,4.,4.]],racer_flight_limits=[.7,3.],stop_and_plan=True,native_frame_source='mapper_observation')
node=NativeLeaseAgent(0,config,N(send=lambda data:None),out/'flight');ring=node.map_evidence;original=node.replica.map.state.copy()
try:
    for i in range(4):assert node.reject(dict(traj_id=i,points=[[1.,2.,3.]]),'component unknown map rejection') is None
    assert node.last_rejection==3
    np.testing.assert_array_equal(node.replica.map.state,original)
finally:
    node.destroy_node();node.log.close();rclpy.shutdown()
lines=[json.loads(x) for x in (out/'flight/0/events.jsonl').read_text().splitlines()]
assert len(lines)==4 and [x['traj_id'] for x in lines]==list(range(4))
files=list(ring.folder.glob('*.npz'));assert len(files)==3
for f in files:
    with np.load(f) as z:
        record=json.loads(bytes(z['metadata_json']).decode())['record'];assert record['kind']=='gateway_rejected_proposal'
        assert record['policy_query_performed'] is False and record['native_state_available'] is False
        np.testing.assert_array_equal(z['mapper_state'],original)
assert ring.write_errors==ring.oversize==ring.queue_dropped==ring.disk_dropped==0
assert ring.hook_max_wall_s<.2
report=dict(no_flight=True,ROS_domain=215,real_NativeLeaseAgent=True,original_reject_calls=4,original_rejection_log_rows=4,last_rejection=3,bounded_full_snapshots=3,mapper_unchanged=True,native_state_absent_not_invented=True,hook_max_wall_s=ring.hook_max_wall_s)
(out/'RESULTS.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
