from pathlib import Path
s=Path('/tmp/stage49-pacer-v4.py').read_text().replace('import json,sys,threading,time,signal,queue,gzip,subprocess,select,hashlib','import json,sys,threading,time,signal,queue,gzip,subprocess,select,hashlib,re')
a=s.index('import rclpy\n');b=s.index('from gz.transport13');s=s[:a]+s[b:]
a=s.index('def ack_cb(m):');b=s.index('for s in [signal.SIGINT',a)
s=s[:a]+'''def read_native_feedback():
 global ack_us
 current={}
 try:
  with (out/'native-vehicle-imu-listener.txt').open('w') as raw:
   for line in listener.stdout:
    raw.write(line)
    m=re.search(r'\\b(timestamp_sample|delta_angle_dt|delta_velocity_dt):\\s*(\\d+)',line)
    if not m:continue
    name,value=m.group(1),int(m.group(2))
    if name=='timestamp_sample':current={'timestamp_sample':value}
    else:current[name]=value
    if all(k in current for k in ['timestamp_sample','delta_angle_dt','delta_velocity_dt']):
     t=current['timestamp_sample']
     with cv:
      if ack_us is not None and t<ack_us:err.append('Native consumed feedback timestamp regressed');stop.set()
      if current['delta_angle_dt']>20000 or current['delta_velocity_dt']>20000:err.append('Actual native IMU integral exceeds 20ms');stop.set()
      if t>0:ack_us=t
      counts['ack']+=1;emit(dict(kind='ack',source='original_native_listener_vehicle_imu',**current));cv.notify_all()
     current={}
 except Exception as e:err.append(repr(e));stop.set()
''' + s[b:]
a=s.index('rclpy.init();');b=s.index('gz=GZNode()',a)
s=s[:a]+'''listener=subprocess.Popen(['/tmp/racer-stage13-px4/source/build/px4_sitl_default/bin/px4-listener','--instance','1','vehicle_imu','-n','100000'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(out/'native-listener-errors.log').open('w'),text=True,bufsize=1)
reader=threading.Thread(target=read_native_feedback,daemon=True);reader.start();''' +s[b:]
a=s.index(' # DDS discovery is required');b=s.index(' time.sleep(.2)',a)
s=s[:a]+''' # Original public native topic exists before DDS startup; no synthetic feedback.
 if listener.poll() is not None:raise RuntimeError('Native listener failed before physical steps')
''' + s[b:]
s=s.replace("gz.unsubscribe('/world/annual/clock');rclpy.shutdown();spin.join(1);writer_stop.set();w.join(1)", """gz.unsubscribe('/world/annual/clock')
 try:listener.stdin.write('q\\n');listener.stdin.flush();listener.stdin.close();listener.wait(timeout=.5)
 except (BrokenPipeError,subprocess.TimeoutExpired):err.append('Native listener did not finish quit within .5s; owned PX4 cleanup required')
 reader.join(.5);result['native_listener_exit_before_owner_cleanup']=listener.poll();writer_stop.set();w.join(1)""")
s=s.replace('Existing consumption feedback publisher unavailable','Native feedback unavailable').replace('existing SensorCombined timestamp','original public vehicle_imu listener timestamp')
Path('/tmp/stage49-pacer-v5.py').write_text(s);compile(s,'pacer-v5','exec')
s=Path('/tmp/stage49-controller-v5.py').read_text().replace('P09-bootstrap','P11-bootstrap').replace('P10-load','P12-load').replace('/tmp/stage49-pacer-v4.py','/tmp/stage49-pacer-v5.py')
a=s.index(' deadline=time.monotonic()+10\n while \'Startup script returned successfully\'');b=s.index(' if loaded:\n',a)
s=s[:a]+''' startup_deadline=time.monotonic()+10
 deadline=time.monotonic()+1
 while True:
  state=subprocess.run([str(build/'bin/px4-sensors'),'--instance','1','status'],env=env,capture_output=True,text=True,timeout=2);(out/'initialized-sensors-status.txt').write_text(state.stdout+state.stderr)
  if '[vehicle_imu]' in state.stdout:break
  if time.monotonic()>deadline:raise RuntimeError('Original IMU instance not created after real first sample')
  time.sleep(.02)
 r['native_imu_created_before_more_physics']=True
''' + s[b:]
s=s.replace("   if q.poll() is not None:raise RuntimeError('Owned '+names[q.pid]+' exited during component')", "   if q.poll() is not None:raise RuntimeError('Owned '+names[q.pid]+' exited during component')\n  if time.monotonic()>startup_deadline and 'Startup script returned successfully' not in (out/'px4.log').read_text(errors='replace'):raise RuntimeError('Original PX4 startup not completed within10wall while real feedback advances')")
s=s.replace("r['pacer']=json.loads((out/'pacer-result.json').read_text())", "r['original_startup_finished_with_native_feedback']='Startup script returned successfully' in (out/'px4.log').read_text(errors='replace');assert r['original_startup_finished_with_native_feedback'],'Original startup never completed'\n r['pacer']=json.loads((out/'pacer-result.json').read_text())")
# No SensorCombined ROS overlay is needed by this diagnostic reader.
s=s.replace("['bash','-c','source /tmp/stage49-feedback-msg/install/local_setup.bash; exec \"$@\"','stage49-feedback',sys.executable,'/tmp/stage49-pacer-v5.py',str(out),str(target),str(wallcap)]", "[sys.executable,'/tmp/stage49-pacer-v5.py',str(out),str(target),str(wallcap)]")
Path('/tmp/stage49-controller-v6.py').write_text(s);compile(s,'controller-v6','exec');print('public native feedback candidate prepared')
