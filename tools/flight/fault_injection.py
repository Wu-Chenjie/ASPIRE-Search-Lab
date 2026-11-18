"""Explicit local SITL fault fixture; never changes message stamps or guards."""
import argparse,json,os,signal,time
from pathlib import Path


class FaultSchedule:
    """Wall-bounded injection after actual armed Offboard exploration readiness."""
    def __init__(self,mode,folder,agent,after_sim=60.,hold_wall=12.,recovery_wall=30.):
        if mode not in ('flags','position','dds'):raise ValueError(mode)
        self.mode=mode;self.folder=Path(folder);self.agent=agent
        self.after_sim=after_sim;self.hold_wall=hold_wall;self.recovery_wall=recovery_wall
        self.state='waiting';self.start_wall=None;self.restore_wall=None
        self.file=(self.folder/'fault-schedule.jsonl').open('x',buffering=1)
        self.stopped=False;self.observed_stop=False;self.observed_resume=False

    def event(self,kind,sim,**data):
        row=dict(event=kind,mode=self.mode,sim_s=sim,wall_ns=time.monotonic_ns(),**data)
        self.file.write(json.dumps(row)+'\n')

    def apply(self,enabled,sim):
        if self.mode=='dds':
            if self.agent.poll() is not None:raise RuntimeError('Owned agent exited before injection')
            self.agent.send_signal(signal.SIGSTOP if enabled else signal.SIGCONT)
            self.stopped=enabled
            self.event('signal_sent',sim,enabled=enabled,pid=self.agent.pid,signal='SIGSTOP' if enabled else 'SIGCONT')
        else:
            p=self.folder/'fault-control.json';tmp=p.with_suffix('.tmp')
            tmp.write_text(json.dumps(dict(drop=enabled)));os.replace(tmp,p)
            self.event('relay_change_requested',sim,enabled=enabled)

    def step(self,sim,flight,execution):
        wall=time.monotonic()
        if self.state=='waiting' and sim>=self.after_sim and execution.get('ready') and flight.get('armed') and flight.get('offboard') and not flight.get('flight_session_fault'):
            self.apply(True,sim);self.start_wall=wall;self.state='active'
            self.event('injection_started',sim)
        if self.state=='active' and self.mode=='dds' and not self.observed_stop:
            p=Path('/proc')/str(self.agent.pid)/'stat'
            s=p.read_text();state=s[s.rfind(')')+2:].split()[0]
            if state in ('T','t'):
                self.observed_stop=True;self.event('agent_stop_observed',sim,pid=self.agent.pid,process_state=state)
        if self.state=='active' and wall-self.start_wall>=self.hold_wall:
            self.apply(False,sim);self.restore_wall=wall;self.state='restored';self.event('injection_removed',sim)
        if self.state=='restored' and self.mode=='dds' and not self.observed_resume:
            value=(Path('/proc')/str(self.agent.pid)/'stat').read_text()
            process_state=value[value.rfind(')')+2:].split()[0]
            if process_state not in ('T','t'):
                self.observed_resume=True;self.event('agent_resume_observed',sim,pid=self.agent.pid,process_state=process_state)
        if self.state=='restored' and wall-self.restore_wall>=self.recovery_wall:
            self.state='complete';self.event('recovery_observation_complete',sim)
        return self.state=='complete'

    def allows_observation(self,fault):
        return self.state!='waiting' and fault in ('px4_failsafe_requires_new_session','px4_disarmed_requires_new_session')

    def close(self,sim):
        # A stopped owned child must be resumed before normal SIGINT cleanup.
        if self.stopped and self.agent.poll() is None:
            self.apply(False,sim);self.event('cleanup_resumed_agent',sim)
        self.file.close()


def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['flags','position','dds'],required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import rclpy
    from rclpy.clock import Clock,ClockType
    from rclpy.serialization import serialize_message
    from px4_msgs.msg import EstimatorStatusFlags,VehicleLocalPosition,VehicleAttitude,VehicleStatus,OffboardControlMode,VehicleCommand
    from annual_runtime.px4_bridge_node import px4_topic,PX4_QOS
    import hashlib
    rclpy.init();node=rclpy.create_node('sitl_fault_observer');node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    out=a.output;log=(out/'fault-messages.jsonl').open('x',buffering=1);counts={};drop=False
    selected={'flags':'flags','position':'raw_position'}.get(a.mode)
    def event(kind,**data):
        log.write(json.dumps(dict(event=kind,wall_ns=time.monotonic_ns(),sim_s=node.get_clock().now().nanoseconds*1e-9,**data))+'\n')
    def control():
        nonlocal drop
        path=out/'fault-control.json';wanted=json.loads(path.read_text())['drop'] if path.exists() else False
        if wanted!=drop:
            drop=wanted;event('relay_gate_changed',drop=drop,counts={k:dict(v) for k,v in counts.items()})
    node.create_timer(.01,control,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def receive(kind,msg,publisher):
        counter=counts.setdefault(kind,dict(received=0,forwarded=0,dropped=0));counter['received']+=1
        action='observed'
        if publisher is not None:
            if drop:counter['dropped']+=1;action='dropped'
            else:publisher.publish(msg);counter['forwarded']+=1;action='forwarded'
        data=dict(input=kind,stamp_us=int(getattr(msg,'timestamp_sample',0) or msg.timestamp),action=action,count=counter['received'])
        if publisher is not None:data['unaltered_cdr_sha256']=hashlib.sha256(serialize_message(msg)).hexdigest()
        if kind=='command':data.update(command=int(msg.command),param1=float(msg.param1),param2=float(msg.param2))
        if kind=='status':data.update(armed=int(msg.arming_state),nav_state=int(msg.nav_state),failsafe=bool(msg.failsafe))
        event('message',**data)
    for kind,name,cls,direction in [('flags','estimator_status_flags',EstimatorStatusFlags,'out'),('raw_position','vehicle_local_position',VehicleLocalPosition,'out'),('attitude','vehicle_attitude',VehicleAttitude,'out'),('status','vehicle_status',VehicleStatus,'out'),('heartbeat','offboard_control_mode',OffboardControlMode,'in'),('command','vehicle_command',VehicleCommand,'in')]:
        topic=px4_topic('/px4_1/fmu',direction,name,cls)
        pub=node.create_publisher(cls,'/fault_fixture/'+kind,PX4_QOS) if kind==selected else None
        node.create_subscription(cls,topic,lambda m,k=kind,p=pub:receive(k,m,p),PX4_QOS)
    event('observer_started',mode=a.mode,selected=selected)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        event('observer_finished',counts=counts);log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
