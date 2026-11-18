"""No-flight original Python authorization method on saved actual static contexts.

Never a replacement decision: original runtime judgments are immutable. Only
rejections before peer arbitration are reproduced; unsupported contexts stay
explicitly unverified, not relabelled as successful or free.
"""
from pathlib import Path
from types import SimpleNamespace as N
import json,sys,os
import numpy as np
sys.path[:0]=['/tmp/racer-stage45-src','/tmp/racer-stage45-src/src','/tmp/racer-stage45-src/src/annual_planner/src']
from core.exploration.voxel_mapping import VoxelMap
from core.planning.execution_safety import NativeSafetyMap,ExecutionCorridor
from core.planning.motion_limits import motion_environment
from core.exploration.voxel_config import voxel_environment
from tools.research.frontier_baselines.live_ros2 import NativeLeaseAgent
class Rejected(RuntimeError):pass
class ReachedPeer(RuntimeError):pass

def _replay(meta,arrays,config):
    expected=meta['rejection_reason'];packet=meta['rejected_proposal']
    result=dict(native_id=packet.get('traj_id'),expected=expected,scope='Unmodified NativeLeaseAgent.propose through original geometry/static-root gates; diagnostic file output suppressed only')
    if expected not in ['No known-free native prefix with C2 stop','native_static_root_stale','native_static_reference_discontinuity'] or packet.get('boundary_progress',0)>=0:
        return result|dict(status='outside_static_geometry_replay_scope',verified=None)
    if not meta.get('mapper_state_available') or not meta.get('native_state_available') or meta.get('mapper_cache_matches_state') is not True:
        return result|dict(status='actual_context_incomplete',verified=False)
    origin=np.asarray(meta['mapper_origin']);state=arrays['mapper_state'];res=meta['mapper_resolution']
    m=VoxelMap([origin,origin+np.asarray(state.shape)*res],resolution=res,clearance=meta['mapper_clearance'],flight_limits=meta['flight_limits']);m.shape=state.shape;m.state=state.copy();m.rebuild()
    if m.collision_profile!=meta['mapper_collision_profile'] or not np.array_equal(m.metric,meta['mapper_metric']):return result|dict(status='voxel_environment_not_exact',verified=False)
    if meta.get('mapper_recovery_yaw') is not None:m.recovery_yaw=float(meta['mapper_recovery_yaw'])
    if meta.get('reserved_context_present'):
        from scipy.spatial import cKDTree
        m.reserved_points=cKDTree(arrays['mapper_reserved_points']);m.reserved_radius=float(meta['mapper_reserved_radius'])
    n=NativeSafetyMap();n.ingest([1,meta['native_stamp'],meta['native_sequence'],meta['native_resolution'],*meta['native_origin'],*arrays['native_state'].shape,*arrays['native_state'].ravel()])
    now=meta['callback_sim_ns']*1e-9
    pose=arrays.get('actual_estimated_position');events=[]
    def reject(row,reason):raise Rejected(reason)
    def peer(*args):raise ReachedPeer('All reproduced earlier guards passed')
    x=N(pose=pose,flight_fault=N(reason=meta.get('flight_fault_reason')),pending={'present':True} if meta['pending_intent_present'] else None,dirty=False,
        safety_enabled=meta['safety_enabled'],replica=N(map=m),native_safety=n,now=lambda:now,last_sensor=float(meta['last_sensor_stamp']),
        intent={'present':True} if meta['current_intent_present'] else None,execution=meta['execution_feedback'],diagnostics=3,
        event=lambda kind,**kw:events.append(dict(type=kind,**kw)),reject=reject,ledger=N(can_propose=peer),id=0)
    def corridor():
        if not n.fresh(now):raise ValueError('native_safety_map_stale')
        return ExecutionCorridor(m,n)
    x.corridor=corridor
    try:NativeLeaseAgent.propose(x,packet)
    except Rejected as exc:actual=str(exc)
    except ReachedPeer:actual='reached_peer_after_geometry'
    except Exception as exc:return result|dict(status='replay_exception',verified=False,error=repr(exc))
    else:actual='returned_without_rejection'
    return result|dict(status='matched_original_rejection' if actual==expected else 'contradiction_requires_isolation',verified=actual==expected,actual=actual)

def replay(meta,arrays,config):
    updates={**motion_environment(config),**voxel_environment(config)};before={k:os.environ.get(k) for k in updates};os.environ.update(updates)
    try:return _replay(meta,arrays,config)
    finally:
        for k,v in before.items():
            if v is None:os.environ.pop(k,None)
            else:os.environ[k]=v

if __name__=='__main__':
    root=Path(sys.argv[1]);config=json.loads((root/'runtime.json').read_text());rows=[]
    for f in sorted((root/'map-evidence/drone-0-gateway').glob('*.npz')):
        with np.load(f,allow_pickle=False) as z:
            meta=json.loads(bytes(z['metadata_json']).decode())['record']
            if meta['kind']!='gateway_rejected_proposal':continue
            arrays={k:z[k].copy() for k in z.files if k!='metadata_json'};rows.append(replay(meta,arrays,config))
    report=dict(scope='Offline original method static rejection context replay; no truth or new flight',new_simulation_runs=0,results=rows,matched=sum(r['verified'] is True for r in rows),unsupported=sum(r['verified'] is None for r in rows),failed=sum(r['verified'] is False for r in rows),passed=not any(r['verified'] is False for r in rows))
    print(json.dumps(report,indent=2))
