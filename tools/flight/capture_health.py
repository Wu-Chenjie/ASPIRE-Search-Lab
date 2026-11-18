"""Read-only online diagnostic gate; never a flight/planner policy input."""
ERRORS=('oversize','queue_dropped','disk_dropped','write_errors','trigger_dropped')
def health_fault(x,now_ns,session):
    if x.get('schema')!=1 or x.get('session')!=session:return 'capture_session_or_schema'
    if not 0<=(now_ns-x.get('wall_ns',0))*1e-9<=3.:return 'capture_heartbeat_stale'
    if x.get('closed'):return 'capture_closed_while_running'
    if any(x.get(k,0) for k in ERRORS):return 'capture_error_or_drop'
    if x.get('disk_bytes',0)>x.get('disk_limit',0):return 'capture_disk_limit'
    if x.get('hook_max_wall_s',0)>.2:return 'capture_sync_hook_over_unchanged_execution_budget'
    return None
