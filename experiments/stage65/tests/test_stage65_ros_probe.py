"""Actual ROS/DDS component fixtures only: no Gazebo, PX4 or flight."""
import json,time,os,sys,subprocess
from types import SimpleNamespace as N
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from rclpy.parameter import Parameter
from core.exploration.voxel_mapping import VoxelMap
from core.planning.execution_safety import NativeSafetyMap
from annual_runtime.trigger_evidence import EvidenceRing
import stage65_capture as c

def test_real_DDS_probe_records_empty_state_for_all_three_roles(tmp_path,monkeypatch):
    monkeypatch.setenv('RACER_MAP_EVIDENCE_DIR',str(tmp_path/'probe'))
    monkeypatch.setenv('RACER_OBSERVATION_SESSION','stage65-no-flight-dds')
    rclpy.init(domain_id=214);nodes=[];rings=[]
    try:
        for role in ['mapper','executor','gateway']:
            x=rclpy.create_node('stage65_empty_'+role);x.id=0;m=VoxelMap([[-3,-4,0],[7,4,4]],flight_limits=(.7,3.))
            x.map=m;x.safety_replica=N(map=m);x.replica=N(map=m);x.native_safety=NativeSafetyMap();x.curve=None;x.token=None
            c.bind_capture(x,role);nodes.append(x);rings.append(x.map_evidence)
        sender=rclpy.create_node('stage65_component_sender');nodes.append(sender)
        clockpub=sender.create_publisher(Clock,'/clock',10)
        for n in nodes:n.set_parameters([Parameter('use_sim_time',value=True)])
        pub=sender.create_publisher(String,'/drone_0/diagnostics/map_evidence_trigger',10)
        def spin():
            for node in nodes:rclpy.spin_once(node,timeout_sec=.01)
        end=time.monotonic()+8
        while pub.get_subscription_count()<3:spin();assert time.monotonic()<end
        destination=tmp_path/'actual-probe-result.json'
        env=dict(os.environ,ROS_DOMAIN_ID='214',ROS_LOCALHOST_ONLY='1',PYTHONDONTWRITEBYTECODE='1')
        with (tmp_path/'actual-probe.log').open('w') as log:
            proc=subprocess.Popen([sys.executable,'/tmp/stage65-measurement/stage65-startup-probe.py',str(destination),'stage65-no-flight-dds'],env=env,stdout=log,stderr=subprocess.STDOUT)
            try:
                while proc.poll() is None or not all(r.records==1 for r in rings):
                    clock=Clock();clock.clock.sec=1;clockpub.publish(clock);spin();assert time.monotonic()<end
            finally:
                if proc.poll() is None:proc.terminate()
                proc.wait(timeout=3)
        assert proc.returncode==0 and json.loads(destination.read_text())['subscriptions']==3
    finally:
        for r in rings:r.close()
        for n in nodes:n.destroy_node()
        rclpy.shutdown()
    for ring in rings:
        files=list(ring.folder.glob('*.npz'));assert len(files)==1 and ring.windows[0]['post_captured']==0
        with np.load(files[0]) as z:
            meta=json.loads(bytes(z['metadata_json']).decode())['record'];assert meta['kind']=='probe_current_state'
            assert meta['callback_sim_ns']>0 and meta['policy_query_performed'] is False
            assert meta['curve_present'] is False and meta['native_state_available'] is False
            assert (z['mapper_state']==-1).all() and 'native_state' not in z
        assert ring.write_errors==ring.oversize==ring.queue_dropped==ring.disk_dropped==0
        assert ring.hook_max_wall_s<.2

from test_execution_safety_ros import executor

def test_actual_executor_no_curve_still_returns_without_a_safety_query(executor,tmp_path):
    x=executor.node;assert x.curve is None
    r=EvidenceRing(tmp_path,0,'executor',post_records=4);x.map_evidence=r
    before=x.safety_replica.map.state.copy();reference=x.ref.copy();native=x.native_safety.state.copy()
    data=json.dumps(dict(schema=1,kind='map_evidence_trigger',id='actual-executor-no-curve',drone=0,
        session=r.session,sim_s=executor.now(),reason='bounded_startup_capture_probe'))
    try:
        assert x.check_execution_safety(executor.now()) is None
        c.receive(x,r,'executor',data)
        assert x.check_execution_safety(executor.now()) is None
    finally:r.close();x.map_evidence=None
    assert x.curve is None and not x.safety_fault and r.records==1 and r.windows[0]['post_captured']==0
    np.testing.assert_array_equal(x.safety_replica.map.state,before);np.testing.assert_array_equal(x.ref,reference)
    np.testing.assert_array_equal(x.native_safety.state,native)
