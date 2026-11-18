from pathlib import Path
import json,sys,time,os
sys.path[:0]=['/tmp/racer-stage45-src','/tmp/racer-stage45-src/src/annual_planner/src']
from tools.flight.capture_health import health_fault
from stage68_monitor_inventory import read_closed_status
root=Path(sys.argv[1]);session=sys.argv[2];now=time.monotonic_ns();errors=[];health={};status={};partial_status=[]
for role,logname in [('mapper','pointcloud_mapping_node'),('executor','view_executor_node'),('gateway','racer_adapter')]:
 folder=root/'map-evidence'/('drone-0-'+role);p=folder/'health.json';closed=folder/'status.json'
 if closed.exists():
  value=read_closed_status(closed)
  if value is not None:status[role]=value;continue
  partial_status.append(role)
 if p.exists():
  x=json.loads(p.read_text());health[role]=x;fault=health_fault(x,now,session)
  if fault:errors.append(role+':'+fault)
 elif (root/(logname+'.log')).exists() and time.time()-(root/(logname+'.log')).stat().st_ctime>10:errors.append(role+':capture_heartbeat_missing_after_node_launch')
from stage68_monitor_inventory import measure
inventory=measure(root);capture=inventory['capture_bytes'];other=inventory['other_bytes']
v=os.statvfs('/tmp');free=v.f_bavail*v.f_frsize
remaining=max(0,1618033370-other)+max(0,102760448-capture)+64000000+200000000
if free<remaining or free<200000000:errors.append('remaining_storage_budget_or_reserve')
if other>1618033370 or capture>102760448:errors.append('declared_output_budget_exceeded')
# Read only algorithm telemetry, never scene/person truth during the run.
last_sim=0.;algorithm_fault=None
p=root/'telemetry.jsonl'
if p.exists():
 with p.open('rb') as f:f.seek(max(0,p.stat().st_size-150000));text=f.read().decode(errors='replace')
 for line in text.splitlines():
  try:x=json.loads(line)
  except ValueError:continue
  a=x.get('data',{})
  if x.get('kind')=='execution':last_sim=max(last_sim,a.get('time',0));algorithm_fault=a.get('safety_fault') or algorithm_fault
  if x.get('kind')=='person_records' and a.get('identity_count',{}).get('status')=='inconsistent_constraints':errors.append('identity_constraints_inconsistent')
# Passive resource observations only; never influence policy or replace original gates.
resources={}
try:
 from tools.racer_ros2.light.processes import snapshot
 owner=int(Path('/tmp/'+session+'.runner.pid').read_text());table=snapshot();owned={owner}
 for _ in range(len(table)):
  extra={pid for pid,(parent,birth) in table.items() if parent in owned}-owned
  if not extra:break
  owned.update(extra)
 details=[]
 for pid in sorted(owned):
  q=Path('/proc')/str(pid)
  try:
   stat=(q/'stat').read_text();fields=stat[stat.rfind(')')+2:].split();status_text=(q/'status').read_text().splitlines()
   wanted={s.split(':',1)[0]:s.split(':',1)[1].strip() for s in status_text if s.split(':',1)[0] in ['Name','Threads','Cpus_allowed_list','VmRSS']}
   sched=(q/'schedstat').read_text().split();details.append(dict(pid=pid,start=fields[19],utime_ticks=int(fields[11]),stime_ticks=int(fields[12]),sched_runtime_ns=int(sched[0]),sched_wait_ns=int(sched[1]),sched_slices=int(sched[2]),**wanted))
  except (OSError,ValueError,IndexError):continue
 resources['owned_processes']=details
 for key,path in [('cpu_pressure','/proc/pressure/cpu'),('io_pressure','/proc/pressure/io'),('memory_pressure','/proc/pressure/memory'),('cgroup_cpu','/sys/fs/cgroup/cpu.stat'),('cgroup_memory_events','/sys/fs/cgroup/memory.events')]:
  q=Path(path);resources[key]=q.read_text() if q.exists() else None
 resources['guest_loadavg']=Path('/proc/loadavg').read_text()
except Exception as error:resources['capture_error']=repr(error)
print(json.dumps(dict(partial_closed_status_publication=partial_status,inventory=inventory,resources=resources,free_bytes=free,remaining_required_bytes=remaining,other_bytes=other,capture_bytes=capture,health=health,closed_status=status,errors=errors,last_sim_s=last_sim,safety_fault=algorithm_fault)))
