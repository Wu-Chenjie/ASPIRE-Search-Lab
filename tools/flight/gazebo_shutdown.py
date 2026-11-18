"""Public stop request for this run's owned Gazebo, within original cleanup budget."""
import os,subprocess,time,signal
from pathlib import Path
from tools.racer_ros2.light.processes import same_process,terminate,snapshot
def stop_owned_server(process,out,environ=None,budget_s=2.):
    env=dict(os.environ if environ is None else environ)
    expected='person_flight_'+Path(out).name
    if env.get('GZ_PARTITION')!=expected:raise ValueError('Refuse foreign Gazebo partition')
    result=dict(owned_pid=process.pid,partition=expected,endpoint='/server_control',
        requested=False,budget_s=budget_s)
    if process.poll() is not None:result['already_exited']=process.returncode;return result
    started=time.monotonic()
    try:
        command=['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl',
            '--reptype','gz.msgs.Boolean','--timeout','1000','--req','stop: true']
        reply=subprocess.run(command,env=env,capture_output=True,text=True,
            timeout=min(1.25,budget_s))
        result.update(requested=True,returncode=reply.returncode,stdout=reply.stdout,stderr=reply.stderr)
        accepted=reply.returncode==0 and 'data: true' in reply.stdout
        result['accepted']=accepted
        if accepted:process.wait(timeout=max(.001,budget_s-(time.monotonic()-started)))
    except Exception as error:result['error']=repr(error)
    result.update(wall_s=time.monotonic()-started,exit_code=process.poll())
    return result
def signal_owned_clients(children,names,wait_s=2.):
    """Quiesce this run's client roots AND their PID-birth-verified descendants."""
    table=snapshot();roots={};owned={};events=[]
    for process in children:
        name=names[process.pid]
        if name not in {'px4','agent','sensor_bridge','evaluation'}:continue
        if process.poll() is not None or process.pid not in table:continue
        birth=table[process.pid][1]
        roots[process.pid]=(process,name);owned[process.pid]=(birth,process.pid)
    while True:
        extra={pid:(birth,owned[parent][1]) for pid,(parent,birth) in table.items()
            if parent in owned and table.get(parent,(None,None))[1]==owned[parent][0] and pid not in owned}
        if not extra:break
        owned.update(extra)
    for pid,(birth,root) in owned.items():
        if not same_process(pid,birth):continue
        try:os.kill(pid,signal.SIGINT)
        except ProcessLookupError:continue
        events.append(dict(name=roots[root][1],pid=pid,root_pid=root,
            descendant=pid!=root,birth=birth,signal='SIGINT'))
    def remaining():
        for process,_ in roots.values():process.poll()
        return [pid for pid,(birth,_) in owned.items() if same_process(pid,birth)]
    deadline=time.monotonic()+wait_s
    alive=remaining()
    while alive and time.monotonic()<deadline:
        time.sleep(min(.01,max(0.,deadline-time.monotonic())));alive=remaining()
    return dict(signals=events,wait_budget_s=wait_s,owned_client_processes=len(owned),
        remaining_alive=alive,quiesced=not alive,
        returncodes={name:process.poll() for process,name in roots.values()})

def terminate_flight(children,names,out,timeout=8.):
    attempts=[];client_stops=[]
    def before_signals():
        client_stops.append(signal_owned_clients(children,names,wait_s=min(2.,timeout)))
        if not client_stops[-1]['quiesced']:
            attempts.append(dict(skipped=True,reason='owned_clients_not_quiescent',remaining_pids=client_stops[-1]['remaining_alive']))
            return
        servers=[c for c in children if names[c.pid]=='gazebo']
        if len(servers)==1:
            try:attempts.append(stop_owned_server(servers[0],out,budget_s=min(2.,timeout)))
            except Exception as error:attempts.append(dict(error=repr(error)))
    # terminate snapshots all owned descendants first, including possible orphans.
    result=terminate(children,timeout=timeout,
        defer_pids=[c.pid for c in children if names[c.pid]=='xvfb'],
        before_signals=before_signals)
    result['gazebo_stop_requests']=attempts;result['client_pre_stop']=client_stops;result['non_display_budget_s']=timeout
    return result
