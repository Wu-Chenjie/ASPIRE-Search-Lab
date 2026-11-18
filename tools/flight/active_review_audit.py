"""Strict offline review evidence audit; no truth, commands, or inferred success."""
import argparse
import json
import math
import re
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if not __package__:
    sys.path.insert(0, str(ROOT))
from tools.flight.confirmation_audit import rows as json_rows, fresh_timings, log_packets, reconstruct


def rows(path):
    return json_rows(path.read_text()) if path.exists() else []


def close(a,b,tolerance):
    try:return len(a)==len(b)==3 and all(math.isfinite(float(x)) for x in (*a,*b)) and math.dist(a,b)<=tolerance
    except (TypeError,ValueError):return False


def native_records(text):
    planned=[];authorized=[]
    for line_number,line in enumerate(text.splitlines()):
        match=re.search(r'Task review planned .* id=(\d+) status=(\d+) request_stamp=([\d.e+-]+) goal=([\d.e+,-]+)',line)
        if match:
            ident,status,stamp,goal=match.groups()
            planned.append(dict(traj_id=int(ident),status=int(status),request_stamp=float(stamp),
                                goal=list(map(float,goal.split(','))),line=line_number))
        match=re.search(r'TASK_REVIEW authorized id=(\d+)',line)
        if match:authorized.append(dict(traj_id=int(match[1]),line=line_number))
    return planned,authorized


def capture_evidence(run, stamp, confirmed, request, completion, observations, track_packets):
    """Reproduce mask-depth location and optical origin from stored sensor data."""
    ns=round(stamp*1e9);errors=[]
    semantic=[r for r in observations if r.get('kind')=='semantic' and r.get('stamp_ns')==ns]
    packet=next((p for p in track_packets if p.get('stamp_ns')==ns),{})
    track=next((t for t in packet.get('tracks',[]) if t.get('id')==confirmed.get('track')),None)
    if not (completion.get('time',math.inf)<stamp<=confirmed.get('time',-math.inf)):errors.append('capture_not_between_completion_and_confirmation')
    valid_timing=fresh_timings(observations,ns)
    if not valid_timing:errors.append('missing_accepted_fresh_capture_timing')
    if track is None:errors.append('missing_same_capture_track_packet')
    frame,_=reconstruct(run,ns,hash_sources=False)
    origin,positions=frame['origin'],frame['positions']
    try:
        if frame['errors']:
            reason=frame['errors'][0].partition(':')[2]
            raise ValueError('stored_capture_stamp_mismatch' if reason.endswith('_capture_stamp_mismatch') else reason)
        last_timing=max((t['now_ns'] for t in valid_timing),default=-1)
        supported=[c for s in semantic if s.get('position_frame')=='map' and
                   ns<=s.get('published_sim_ns',0)<=last_timing
                   for c in s.get('candidates',[]) if c.get('state')=='localized' and c.get('xyz') is not None
                   and any(close(c['xyz'],p,1e-6) for p in positions)]
        if not (track and any(math.dist(c['xyz'][:2],track['xyz'][:2])<.65 for c in supported)):
            errors.append('no_reproduced_localized_semantics_associated_with_track')
        if not close(origin,request.get('goal'),.4):errors.append('capture_pose_not_at_requested_lens')
        if abs(stamp-confirmed['time'])<1e-6 and not close(origin,confirmed.get('capture_origin'),1e-6):
            errors.append('confirmation_origin_disagrees_with_capture_transform')
    except (ValueError,KeyError,TypeError) as exc:
        errors.append('missing_or_invalid_sensor_position_evidence:'+str(exc))
    return dict(stamp_ns=ns,origin=origin,reproduced_positions=positions,errors=errors,passed=not errors)


def audit(run, output=None):
    run=Path(run);result=json.loads((run/'result.json').read_text())
    observations=rows(run/'perception/observations.jsonl')
    reviews=[e for e in observations if e.get('kind')=='active_confirmation']
    flight=rows(run/'flight/0/events.jsonl');telemetry=rows(run/'telemetry.jsonl')
    native=(run/'native-racer/exploration_node_1.log').read_text()
    plans,authorized=native_records(native)
    log=run/'person_search.log'
    wire=list(log_packets(log.read_text() if log.exists() else '', 'Review request ', allow_malformed=True))

    activations=[e for e in flight if e['type']=='native_trajectory_activated']
    proposals=[e for e in flight if e['type']=='path_proposed']
    proofs=[e for e in flight if e['type']=='terminal_observation_completed']
    execution=[r['data'] for r in telemetry if r['kind']=='execution']
    track_packets=[r['data'] for r in telemetry if r['kind']=='tracks']
    chains=[]
    for confirmed in [e for e in reviews if e.get('reason')=='active_multiview_confirmed']:
        prior=[e for e in reviews if e.get('track')==confirmed.get('track') and e.get('attempt')==confirmed.get('attempt') and e['time']<=confirmed['time']]
        requests=[e for e in prior if e['reason']=='visual_candidate_requested_safe_view']
        request=requests[-1] if requests else {};token=confirmed.get('token')
        completed=[e for e in prior if e['reason']=='execution_completed_waiting_for_fresh_visual_evidence' and e.get('token')==token and e['time']>request.get('time',math.inf)]
        completion=completed[-1] if completed else {}
        sent=[w for w in wire if w.get('track')==confirmed.get('track') and w.get('attempt')==confirmed.get('attempt') and abs(w.get('stamp_ns',-1)*1e-9-request.get('time',math.inf))<1e-6]
        wire_goal=sent[0].get('goal') if len(sent)==1 else None
        wire_offset_valid=False
        if close(request.get('goal'),wire_goal,float('inf')):
            delta=np.asarray(request['goal'])-np.asarray(wire_goal)
            wire_offset_valid=bool(np.isfinite(delta).all() and abs(np.linalg.norm(delta[:2])-.13233)<2e-6 and abs(delta[2]-.02078)<2e-6)
        matching_plans=[p for p in plans if p['status']==2 and abs(p['request_stamp']-request.get('time',math.inf))<1e-6 and close(p['goal'],wire_goal,2e-6)]
        plan=matching_plans[0] if len(matching_plans)==1 else None
        # IDs may restart; ambiguity fails closed instead of joining global ids.
        authorization=bool(plan and len([p for p in plans if p['status']==2 and p['traj_id']==plan['traj_id']])==1
                           and any(a['traj_id']==plan['traj_id'] and a['line']>plan['line'] for a in authorized))
        acts=[a for a in activations if plan and a.get('token')==token and a.get('traj_id')==plan['traj_id'] and request['time']<=a.get('time',-1)<=completion.get('time',-1)]
        act=acts[0] if len(acts)==1 else None
        proposed=bool(act and any(p.get('token')==token and p.get('native_id')==act['traj_id'] and request['time']<=p.get('time',-1)<=act['time'] for p in proposals))
        terminal=[p for p in proofs if p.get('token')==token and act and act['time']<=p.get('time',-1)<=confirmed['time']
                  and p.get('proof',{}).get('token')==token
                  and act['time']<=p['proof'].get('arrived_at',-1)<=p['proof'].get('time',-1)<=p['proof'].get('completed_at',-1)<=completion.get('time',-1)
                  and p['proof'].get('completion_reason')=='post_arrival_integrated_sensor_frame']
        reports=[e for e in execution if e.get('token')==token and e.get('ready') and e.get('arrived') and not e.get('flight_session_fault')
                 and act and act['time']<=e.get('time',-1)<=confirmed['time'] and close(e.get('reference'),wire_goal,.25)
                 and any(e.get('observation_proof')==p['proof'] for p in terminal)]
        capture_times=confirmed.get('post_capture_times',[])
        valid_times=bool(len(set(capture_times))>=2 and completion and all(math.isfinite(t) and completion['time']<t<=confirmed['time'] for t in capture_times)
                         and max(capture_times)-min(capture_times)>=.4 and abs(max(capture_times)-confirmed['time'])<1e-6)
        captures=[capture_evidence(run,t,confirmed,request,completion,observations,track_packets) for t in capture_times] if valid_times else []
        checks=dict(identified_attempt=isinstance(confirmed.get('attempt'),int) and confirmed['attempt']>0,
                    request_lens_matches_wire_base=wire_offset_valid,
                    same_request_native_plan_and_authorization=authorization,
                    same_native_id_token_activation=bool(act and proposed),
                    actual_executor_completion=bool(terminal and reports),
                    changed_execution_token=token is not None and token!=request.get('previous_token'),
                    request_before_completion=bool(request and completion and request['time']<completion['time']),
                    fresh_post_completion_frames=valid_times,
                    recorded_rgbd_position_evidence=bool(captures) and all(c['passed'] for c in captures),
                    capture_at_requested_view=bool(request and close(confirmed.get('capture_origin'),request.get('goal'),.4)))
        chains.append(dict(track=confirmed.get('track'),attempt=confirmed.get('attempt'),token=token,confirmed_time=confirmed['time'],
                           request=request,completion=completion,native_plan=plan,activation=act,captures=captures,checks=checks,passed=all(checks.values())))
    final={}
    for packet in track_packets:
        for track in packet['tracks']:final[str(track['id'])]=track
    report=dict(active_confirmation=result.get('active_confirmation',False),
        requests=sum(e.get('reason')=='visual_candidate_requested_safe_view' for e in reviews),
        native_authorized_review_ids=sorted({a['traj_id'] for a in authorized}),
        completed_reviews=sum(e.get('reason')=='execution_completed_waiting_for_fresh_visual_evidence' for e in reviews),
        confirmations=len(chains),confirmation_chains=chains,final_tracks=final,
        all_confirmation_chains_pass=bool(chains) and all(c['passed'] for c in chains),events=reviews,error=result['error'],
        scope='Offline native request/goal/id/token + executor proof + reproduced saved RGB-D evidence. Missing or ambiguous evidence fails closed. No semantic truth claim.',
        limitations=['Authorization logs have only id; unique successful native plan id must supply matching request_stamp and base goal.',
                     'Capture freshness is checked at recorded accepted timing (0.75 s); absent exact check-time evidence is not inferred.',
                     'No positive chains means no verified active confirmation; it is not evidence of no person.'])
    (Path(output) if output is not None else run/'active-review-audit.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();print(json.dumps(audit(args.run,args.output),indent=2))
