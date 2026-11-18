"""Bounded public diagnostic request + actual immutable response acknowledgments."""
import json,time,sys,zipfile
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
root=Path(sys.argv[1]).parent;session=sys.argv[2];roles=['mapper','executor','gateway']
rclpy.init();node=rclpy.create_node('stage66_startup_capture_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
pub=node.create_publisher(String,'/drone_0/diagnostics/map_evidence_trigger',10)
started=time.monotonic();deadline=started+8.;trigger=None;attempts=0;ack={};next_publish=started;publish_offsets=[]
try:
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=min(.05,max(0.,deadline-time.monotonic())))
        if trigger is None and pub.get_subscription_count()>=3 and node.get_clock().now().nanoseconds>0:
            trigger=dict(schema=1,kind='map_evidence_trigger',id='stage65-startup-probe',drone=0,session=session,
                sim_s=node.get_clock().now().nanoseconds*1e-9,reason='bounded_startup_capture_probe')
        if trigger is None:continue
        for role in roles:
            if role in ack:continue
            for f in (root/'map-evidence'/('drone-0-'+role)).glob('*-probe.npz'):
                try:
                    with np.load(f,allow_pickle=False) as z:
                        x=json.loads(bytes(z['metadata_json']).decode());a=x['record'];t=x['trigger']
                        if a.get('kind')=='probe_current_state' and a.get('role')==role and a.get('policy_query_performed') is False and t.get('id')==trigger['id'] and t.get('session')==session:
                            ack[role]=dict(file=str(f),callback_sim_ns=a['callback_sim_ns']);break
                except (OSError,ValueError,EOFError,KeyError,zipfile.BadZipFile):continue # incomplete writer artifact is not an acknowledgment
        if len(ack)==3:break
        # At most ten repeated requests per second and one original8s total budget.
        if attempts<80 and time.monotonic()>=next_publish:
            now=time.monotonic();pub.publish(String(data=json.dumps(trigger)));attempts+=1
            publish_offsets.append(now-started);next_publish=now+.1
    if len(ack)!=3 or time.monotonic()>deadline:raise RuntimeError('actual_probe_ack_timeout_or_late_response missing='+','.join(r for r in roles if r not in ack))
    Path(sys.argv[1]).write_text(json.dumps(dict(trigger=trigger,subscriptions=pub.get_subscription_count(),diagnostic_only=True,
        acknowledged_roles=ack,publish_attempts=attempts,publish_wall_offsets=publish_offsets,elapsed_wall_s=time.monotonic()-started,total_budget_wall_s=8.))+'\n')
finally:
    node.destroy_node();rclpy.shutdown()
