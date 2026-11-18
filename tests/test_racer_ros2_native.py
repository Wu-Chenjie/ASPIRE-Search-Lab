"""Native Jazzy checks: paused simulation clock, DDS observations and LKH service."""
import os
from pathlib import Path
import subprocess
import time
import pytest

rclpy=pytest.importorskip('rclpy')
pytest.importorskip('racer_interfaces')
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from racer_interfaces.srv import SolveMTSP


def spin_until(node, condition, seconds=5):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        if condition():return
    raise AssertionError('Native ROS2 condition timed out')


@pytest.fixture
def node():
    rclpy.init()
    n=Node('racer_port_pytest')
    yield n
    n.destroy_node();rclpy.shutdown()


def stop(child):
    child.terminate()
    try:child.wait(timeout=5)
    except subprocess.TimeoutExpired:child.kill();child.wait()


def test_cpp_timer_uses_simulation_clock_and_native_parameters(node):
    binary=Path(get_package_prefix('racer_core'))/'lib/racer_core/runtime_probe'
    child=subprocess.Popen([str(binary),'--ros-args','-p','use_sim_time:=true','-p','probe.value:=7.5'],stdout=subprocess.DEVNULL)
    rows=[]
    sub=node.create_subscription(Float64MultiArray,'/racer_port_probe',lambda m:rows.append(list(m.data)),10)
    clock=node.create_publisher(Clock,'/clock',10)
    try:
        spin_until(node,lambda:clock.get_subscription_count()>0)
        for _ in range(10):rclpy.spin_once(node,timeout_sec=.02)
        assert not rows # No simulation time means no timer progress.
        for ns in range(100_000_000,700_000_000,100_000_000):
            clock.publish(Clock(clock=Time(sec=0,nanosec=ns)))
            for _ in range(5):rclpy.spin_once(node,timeout_sec=.02)
        spin_until(node,lambda:len(rows)>1)
        for _ in range(10):rclpy.spin_once(node,timeout_sec=.02)
        count=len(rows)
        for _ in range(10):rclpy.spin_once(node,timeout_sec=.02)
        assert len(rows)==count
        assert all(row[2]==7.5 and 0.<row[1]<=.6+1e-9 for row in rows), rows
    finally:stop(child)


def test_dds_preserves_measured_cloud_origin_and_exact_timestamp(node,tmp_path,monkeypatch):
    monkeypatch.setenv('RACER_RECORD_INPUT','1')
    from tools.racer_ros2.transport import NativeDDSTransport
    transport=NativeDDSTransport(tmp_path/'native')
    clouds=[];poses=[]
    sub=node.create_subscription(PointCloud2,'/baseline/cloud_0',clouds.append,20)
    pose_sub=node.create_subscription(PoseStamped,'/baseline/sensor_0',poses.append,20)
    try:
        spin_until(node,lambda:transport.publishers_by_kind['cloud',0].get_subscription_count()>0 and transport.publishers_by_kind['sensor',0].get_subscription_count()>0)
        transport.publish_row({'type':'frame','drone':0,'time':12.123456789,'origin':[1.,2.,3.], 'points':[[4.,5.,6.]]})
        spin_until(node,lambda:bool(clouds and poses))
        assert clouds[0].header.stamp==poses[0].header.stamp==Time(sec=12,nanosec=123456789)
        assert (poses[0].pose.position.x,poses[0].pose.position.y,poses[0].pose.position.z)==(1.,2.,3.)
        assert clouds[0].width==1 and clouds[0].point_step==12
        with pytest.raises(ValueError,match='Nonfinite'):
            transport.publish_row({'type':'frame','drone':0,'time':13.,'origin':[1.,2.,3.],'points':[[float('nan'),0.,0.]]})
        assert transport.counts['frame']==1
    finally:transport.close();transport.destroy_node()
    import gzip,json
    with gzip.open(tmp_path/'native/measured-input.jsonl.gz','rt') as stream:recorded=[json.loads(line) for line in stream]
    assert len(recorded)==1 and recorded[0]['time']==12.123456789
    assert recorded[0]['origin']==[1.,2.,3.] and recorded[0]['points']==[[4.,5.,6.]]


def test_stop_feedback_keeps_latest_stamp_outside_full_transport_queue(node,tmp_path,monkeypatch):
    monkeypatch.setenv('ASPIRE_FAST_STOP_HANDOFF','1')
    from tools.racer_ros2.transport import NativeDDSTransport
    from std_msgs.msg import Float64MultiArray
    transport=NativeDDSTransport(tmp_path/'handoff',fleet_size=1)
    received=[]
    subscription=node.create_subscription(Float64MultiArray,'/baseline/feedback_0',received.append,20)
    try:
        spin_until(node,lambda:transport.publishers_by_kind['feedback',0].get_subscription_count()>0)
        transport.started=True
        for _ in range(transport.outgoing.maxsize):
            transport.send(dict(type='execution_grid',drone=0,time=1.))
        for t in (1.,1.2):
            transport.send(dict(type='feedback',drone=0,time=t,ready=True,data=[-1,0,0,-1,-1,t,0,7,.7]))
        assert transport.outgoing.full() and transport.drops==0
        transport.flush_feedback()
        spin_until(node,lambda:bool(received))
        assert len(received)==1 and list(received[0].data)==[-1,0,0,-1,-1,1.2,0,7,.7]
        assert transport.counts['feedback']==1
    finally:
        node.destroy_subscription(subscription)
        transport.close()
        transport.destroy_node()


def test_native_solver_succeeds_and_rejects_missing_problem(node,tmp_path):
    (tmp_path/'amtsp_1.par').write_text(f'PROBLEM_FILE = {tmp_path}/small.tsp\nOUTPUT_TOUR_FILE = {tmp_path}/small.tour\nRUNS = 1\nTRACE_LEVEL = 0\n')
    (tmp_path/'small.tsp').write_text('NAME : small\nTYPE : ATSP\nDIMENSION : 4\nEDGE_WEIGHT_TYPE : EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n0 1 4 3\n1 0 2 4\n4 2 0 1\n3 4 1 0\nEOF\n')
    binary=Path(get_package_prefix('racer_core'))/'lib/racer_core/mtsp_node'
    child=subprocess.Popen([str(binary),'--ros-args','-p',f'exploration.mtsp_dir:={tmp_path}'],stdout=subprocess.DEVNULL)
    client=node.create_client(SolveMTSP,'/solve_tsp_1')
    try:
        assert client.wait_for_service(timeout_sec=5)
        future=client.call_async(SolveMTSP.Request(prob=1))
        spin_until(node,future.done)
        assert future.result().empty==0
        assert 'TOUR_SECTION' in (tmp_path/'small.tour').read_text()
        future=client.call_async(SolveMTSP.Request(prob=3))
        spin_until(node,future.done)
        assert future.result().empty!=0
    finally:stop(child)


def test_all_native_planner_and_solver_nodes_start_from_calibration(node,tmp_path):
    from tools.racer_ros2.transport import NativeDDSTransport
    transport=NativeDDSTransport(tmp_path/'startup')
    hello={'bounds':[[0,0,0],[8,8,4]],'starts':[[2,2,1.5],[2,6,1.5]],
           'intrinsics':[200.,0.,160.,0.,200.,120.,0.,0.,1.], 'flight_limits':[.7,3.1]}
    try:
        transport.start(hello)
        end=time.monotonic()+3.
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            assert all(p.poll() is None for p in transport.children), '\n'.join(p.read_text() for p in transport.output.glob('*.log'))
        assert len(transport.children)==6
    finally:transport.close();transport.destroy_node()
    assert all(p.poll() is not None for p in transport.children)
    transport.close() # Shutdown is idempotent under repeated launch signals.


def test_shutdown_during_pending_native_service_is_clean(node,tmp_path):
    import signal
    # rclpy has discovered this service but is intentionally not spinning to
    # answer it. SIGINT must interrupt an in-flight C++ wait without aborting.
    import threading
    entered=threading.Event();release=threading.Event()
    def pending(request,response):
        entered.set();release.wait(5);return response
    service=node.create_service(SolveMTSP,'/racer_probe_solver',pending)
    worker=threading.Thread(target=lambda:spin_until(node,entered.is_set,seconds=8))
    worker.start()
    binary=Path(get_package_prefix('racer_core'))/'lib/racer_core/runtime_probe'
    log=tmp_path/'pending.log'
    with log.open('w') as output:
        child=subprocess.Popen([str(binary),'--ros-args','-p','probe.solver:=true'],stdout=output,stderr=output)
        try:
            assert entered.wait(5),'C++ request did not reach the blocked server'
            child.send_signal(signal.SIGINT);child.wait(timeout=3)
            assert child.returncode==0,log.read_text()
        finally:
            if child.poll() is None:stop(child)
            release.set();worker.join(timeout=6)
            assert not worker.is_alive()
    assert 'terminate called' not in log.read_text()
