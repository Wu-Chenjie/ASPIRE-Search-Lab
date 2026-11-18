"""Adversarial offline chain fixtures; not flight or detector accuracy tests."""
import json
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from person_search.segmentation import PersonMask,localize_masks
from tools.flight.active_review_audit import audit


def write_rows(path,values):path.write_text(''.join(json.dumps(v)+'\n' for v in values))


def fixture(tmp_path):
    for d in ('perception','flight/0','native-racer'):(tmp_path/d).mkdir(parents=True)
    (tmp_path/'result.json').write_text(json.dumps(dict(active_confirmation=True,error=None)))
    goal=[1.,1.,1.2];base=[.86767,1.,1.17922];token='racer:0:fixture:7'
    request=dict(kind='active_confirmation',track=1,attempt=1,time=10.,reason='visual_candidate_requested_safe_view',goal=goal,previous_token='previous')
    completion=dict(kind='active_confirmation',track=1,attempt=1,time=12.,reason='execution_completed_waiting_for_fresh_visual_evidence',token=token)
    confirmation=dict(kind='active_confirmation',track=1,attempt=1,time=14.,reason='active_multiview_confirmed',token=token,post_capture_times=[13.,14.],capture_origin=goal)
    obs=[request,completion,confirmation];packets=[]
    tf=np.eye(4);tf[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];tf[:3,3]=goal
    depth=np.full((100,100),4.,np.float32);k=[100.,0,50.,0,100.,50.,0,0,1]
    mask=np.zeros((100,100),bool);mask[30:70,45:55]=True;box=(45,30,10,40)
    semantic,_=localize_masks([PersonMask(box,.4,mask)],depth,k,tf);xyz=semantic[0]['xyz'];assert xyz is not None
    for stamp in (13.,14.):
        ns=round(stamp*1e9)
        np.savez(tmp_path/'perception'/f'{ns}-geometry.npz',stamp_ns=ns,depth=depth,k=k,camera_to_map=tf)
        np.savez(tmp_path/'perception'/f'{ns}-rgb-masks.npz',stamp_ns=ns,masks=[mask],boxes=[box],scores=[.4])
        obs.extend([dict(kind='semantic',stamp_ns=ns,position_frame='map',published_sim_ns=ns+50_000_000,candidates=semantic),
                    dict(kind='timing',stamp_ns=ns,now_ns=ns+100_000_000,status='accepted',received_age_wall_s=.1)])
        packets.append(dict(kind='tracks',data=dict(stamp_ns=ns,tracks=[dict(id=1,xyz=xyz,state='confirmed' if stamp==14 else 'observing')])) )
    proof=dict(token=token,arrived_at=11.4,time=11.6,completed_at=11.8,completion_reason='post_arrival_integrated_sensor_frame',position=base)
    flight=[dict(type='path_proposed',time=10.5,token=token,native_id=7),dict(type='native_trajectory_activated',time=11.,token=token,traj_id=7),
            dict(type='terminal_observation_completed',time=12.1,token=token,proof=proof)]
    execution=dict(kind='execution',data=dict(time=12.1,token=token,ready=True,arrived=True,reference=base,observation_proof=proof,flight_session_fault=None))
    write_rows(tmp_path/'perception/observations.jsonl',obs);write_rows(tmp_path/'flight/0/events.jsonl',flight);write_rows(tmp_path/'telemetry.jsonl',[execution,*packets])
    (tmp_path/'native-racer/exploration_node_1.log').write_text('Task review planned using native RACER; external authorization still required id=7 status=2 request_stamp=10.000000000 goal=0.867670,1.000000,1.179220\nTASK_REVIEW authorized id=7\n')
    (tmp_path/'person_search.log').write_text('Review request '+json.dumps(dict(track=1,attempt=1,stamp_ns=10_000_000_000,goal=base))+'\n')
    return obs,flight,[execution,*packets]


def test_complete_same_request_rgbd_chain_passes(tmp_path):
    fixture(tmp_path);report=audit(tmp_path)
    assert report['all_confirmation_chains_pass']
    assert len(report['confirmation_chains'][0]['captures'])==2


def test_unrelated_earlier_review_cannot_authorize_current_request(tmp_path):
    fixture(tmp_path);log=tmp_path/'native-racer/exploration_node_1.log'
    log.write_text(log.read_text().replace('request_stamp=10.000000000','request_stamp=5.000000000'))
    report=audit(tmp_path)
    assert not report['all_confirmation_chains_pass']
    assert not report['confirmation_chains'][0]['checks']['same_request_native_plan_and_authorization']


def test_arrival_only_after_confirmation_does_not_pass(tmp_path):
    obs,flight,tel=fixture(tmp_path);tel[0]['data']['time']=100.
    write_rows(tmp_path/'telemetry.jsonl',tel)
    report=audit(tmp_path)
    assert not report['all_confirmation_chains_pass']
    assert not report['confirmation_chains'][0]['checks']['actual_executor_completion']


def test_future_claimed_captures_without_real_frames_do_not_pass(tmp_path):
    obs,_,_=fixture(tmp_path);obs[2]['post_capture_times']=[1000.,1001.]
    write_rows(tmp_path/'perception/observations.jsonl',obs)
    report=audit(tmp_path)
    assert not report['all_confirmation_chains_pass']
    assert not report['confirmation_chains'][0]['checks']['fresh_post_completion_frames']


def test_baseline_token_and_ambiguous_native_id_fail_closed(tmp_path):
    fixture(tmp_path);log=tmp_path/'native-racer/exploration_node_1.log';original=log.read_text()
    log.write_text(original.replace('TASK_REVIEW authorized id=7','ordinary exploration'))
    assert not audit(tmp_path)['all_confirmation_chains_pass']
    log.write_text(original+original) # possible concatenated process restart
    assert not audit(tmp_path)['all_confirmation_chains_pass']


def test_missing_sensor_position_or_wrong_proof_token_fails_closed(tmp_path):
    obs,flight,tel=fixture(tmp_path)
    (tmp_path/'perception/13000000000-geometry.npz').unlink()
    report=audit(tmp_path)
    assert not report['all_confirmation_chains_pass']
    assert not report['confirmation_chains'][0]['checks']['recorded_rgbd_position_evidence']
    flight[-1]['proof']['token']='other-token';write_rows(tmp_path/'flight/0/events.jsonl',flight)
    assert not audit(tmp_path)['confirmation_chains'][0]['checks']['actual_executor_completion']


def test_claimed_localization_needs_actual_accepted_and_reproduced_position(tmp_path):
    obs,flight,tel=fixture(tmp_path)
    semantic=next(r for r in obs if r.get('kind')=='semantic')
    semantic['candidates'][0]['xyz']=[100.,100.,100.]
    write_rows(tmp_path/'perception/observations.jsonl',obs)
    report=audit(tmp_path)
    assert not report['all_confirmation_chains_pass']
    assert not report['confirmation_chains'][0]['checks']['recorded_rgbd_position_evidence']


def test_explicit_output_preserves_previous_audit(tmp_path):
    fixture(tmp_path);old=tmp_path/'active-review-audit.json';old.write_text('previous archived result\n')
    report=audit(tmp_path,output=tmp_path/'active-review-audit-strict.json')
    assert report['all_confirmation_chains_pass']
    assert old.read_text()=='previous archived result\n'
    assert json.loads((tmp_path/'active-review-audit-strict.json').read_text())['all_confirmation_chains_pass']
