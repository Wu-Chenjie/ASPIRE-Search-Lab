"""Inputs delivered during inference are consumed without waiting for a new callback."""
from copy import copy
from types import SimpleNamespace as N,ModuleType
import json,sys,threading
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/"src"),str(ROOT/"src/annual_planner/src")]
import numpy as np
import pytest
from annual_runtime.pose_history import PoseHistory
from person_search.presence import PresenceTracker,association_view
from person_search.registry_evidence import RegistryEvidence
from person_search.segmentation import PersonMask
from person_search import surface
from person_search import presence_adapter as adapter
from test_surface_components import sample,K,T

@pytest.mark.parametrize('delivery',['same_capture','confirmation','slow_trace','absent','future_capture','no_pose','evicted_capture'])
def test_process_uses_only_already_delivered_registered_capture_without_waiting(monkeypatch,delivery):
    fake=ModuleType('rclpy.time');fake.Time=lambda **kw:kw
    monkeypatch.setitem(sys.modules,'rclpy.time',fake)
    class TransformException(Exception):pass
    monkeypatch.setitem(sys.modules,'tf2_ros',N(TransformException=TransformException))
    d,depth=sample();depth[:]=2.;ns=1_000_000_000
    duplicate=PersonMask(d.box,d.confidence-.1,d.mask)
    real_localize=surface.localize_surfaces
    def localize(detections,*args,**kwargs):
        assert len(detections)==1 and detections[0] is d
        return real_localize(detections,*args,**kwargs)
    monkeypatch.setattr(surface,'localize_surfaces',localize)
    header=N(frame_id='optical',stamp=N(sec=1,nanosec=0))
    rgb=N(width=320,height=240,header=header)
    info=N(width=320,height=240,header=header,k=np.asarray(K).reshape(-1).tolist(),
        p=[0.]*12,r=[0.]*9,d=[])
    dm=N(width=320,height=240,header=header,encoding='32FC1',is_bigendian=False,step=320*4,data=depth.astype('<f4').tobytes())
    history=PoseHistory();history.add(.99,[0,0,0],[0,0,0,1])
    events=[];registered=[];calls=[];views=[];published=[]
    node=N(tracker=PresenceTracker(),registry_mode=True,registry_evidence=RegistryEvidence(session='test'),
        rgbd_registration_mode='strict',trace_detector=None,static_visibility_config=None,
        review_depth_continuity=False,
        queues={k:{} for k in ['rgb','depth','ri','di']},pose_history=history,input_lock=threading.RLock(),
        max_age=.75,require_pose=True,last_ns=-1,last_semantic_ns=-1,min_rgb_period_ns=0,evidence_dir=None,
        processed=0,rejected=0,rgb_consumed_total=0,last_rgb_consumed_ns=-1)
    if delivery=='confirmation':
        for capture in (.2,.4,.6,.8):
            node.tracker.observe_rgb([d],round(capture*1e9),320,240,capture+.01,view=association_view(K,T,320,240))
        monkeypatch.setitem(sys.modules,'std_msgs.msg',N(String=N))
        node.presence_events=N(publish=lambda m:events.append(json.loads(m.data)['event']))
        node.registry_pub=N(publish=lambda m:published.append(json.loads(m.data)))
    node.queues['rgb'][ns]=rgb
    if delivery=='evicted_capture':
        for key,value in [('depth',dm),('ri',info),('di',info)]:node.queues[key][ns]=value
        history.add(1.01,[0,0,0],[0,0,0,1])
    node.get_clock=lambda:N(now=lambda:N(nanoseconds=1_020_000_000))
    node.fresh_capture=lambda stamp:stamp==ns
    node.timing=lambda stamp,status,reason=None:events.append(status)
    node.bearing_capture=lambda *args:None
    node.publish_bearings=lambda *args:None
    node.buffer=N(lookup_transform=lambda *args:N(header=header))
    def detect(stamp,image):
        calls.append(stamp);assert image is rgb
        if delivery=='evicted_capture':
            for q in node.queues.values():q.clear()
        elif delivery!='absent':
            stamp2=ns+100_000_000 if delivery=='future_capture' else ns
            node.queues['depth'][stamp2]=dm;node.queues['ri'][stamp2]=info;node.queues['di'][stamp2]=info
            if delivery!='no_pose':node.pose_history.add(1.01,[0,0,0],[0,0,0,1])
        return [duplicate,d]
    masks=[]
    node.publish_person_masks=lambda stamp,image,dets,associations:masks.append((stamp,list(associations)))
    node.detect_semantics=detect
    def capture_world(node,frame,stamp,hist,Time):
        if hist.at(stamp*1e-9) is None:raise ValueError('missing_capture_pose_bracket')
        return T.copy()
    monkeypatch.setattr(adapter,'capture_world',capture_world)
    monkeypatch.setattr(adapter,'require_registered_pair',lambda r,z,*args:registered.append((r,z)))
    monkeypatch.setattr(adapter,'publish_presence',lambda n:published.append(n.registry_evidence.batch(n.tracker,ns,1.02)))
    trace_calls=[]
    if delivery=='slow_trace':
        def slow_trace(_):
            assert published and node.processed==1
            trace_calls.append('detect')
            node.fresh_capture=lambda stamp:False
            return []
        node.trace_detector=N(detect=slow_trace)
        node.trace_pipeline=N(observe=lambda *args:trace_calls.append('observe'))
        node.trace_pub=N(publish=lambda msg:trace_calls.append('publish'))
        monkeypatch.setitem(sys.modules,'std_msgs.msg',N(String=N))
        monkeypatch.setattr(adapter,'rgb_pixels',lambda image:None)
    original=node.tracker.observe_rgb
    def observe(*args,**kwargs):views.append(kwargs.get('view'));return original(*args,**kwargs)
    node.tracker.observe_rgb=observe
    old_queues={k:q.copy() for k,q in node.queues.items()};old_history=copy(history);old_history.samples=history.samples.copy()
    assert (old_history.at(1.) is None)==(delivery!='evicted_capture')
    adapter.process_presence(node,old_queues,old_history)
    assert node.rgb_consumed_total==1 and node.tracker.last_rgb_ns==ns
    assert node.tracker.frames[ns][0]['raw_index']==1
    if delivery=='confirmation':
        assert events.index('visual_confirmed') < events.index('surface_start')
        first=published.pop(0)['observations'][0]
        assert first['visual_state']=='visually_confirmed' and first['identity_input_state']=='pending'
        assert not first['has_position'] and published[0]['observations'][0]['counter']==first['counter']
    else:
        assert node.tracker.tracks[0].captures==1 and node.tracker.tracks[0].visual_state=='person_candidate'
    assert len(published)==1 and not published[0]['observations'][0]['has_position']
    if delivery in ('same_capture','confirmation','slow_trace'):
        if delivery=='slow_trace':assert trace_calls==['detect']
        assert node.processed==1 and node.last_ns==ns
        assert registered==[(rgb,dm)] and views[0] is not None
        assert 'identity_surface_policy' in published[0]['observations'][0]
        assert 'rgb_association_image_only' not in events
    else:
        assert node.processed==0 and node.last_ns==-1 and registered==[]
        assert (views[0] is None)==(delivery!='evicted_capture')
        assert 'identity_surface_policy' not in published[0]['observations'][0]
    assert bool(old_queues['depth'])==(delivery=='evicted_capture')
    assert (old_history.at(1.) is None)==(delivery!='evicted_capture')
    assert calls == [ns]
    assert masks == [(ns, [(1, node.tracker.frames[ns][0]['track_id'])])]


def test_depth_review_uses_extra_paired_frame_only_until_first_fix():
    tracker=PresenceTracker()
    d,depth=sample()
    wire=RegistryEvidence(session='sampling')
    for capture in (1.,1.2,1.4,1.6,1.8):
        ns=round(capture*1e9)
        tracker.observe_rgb([d],ns,320,240,capture+.01)
    node=N(tracker=tracker,registry_mode=True,min_rgb_period_ns=200_000_000,
           rgbd_registration_mode='strict',last_ns=ns,max_age=.75,require_pose=False)
    stamp=ns+125_000_000
    queues={key:{stamp:True} for key in ('rgb','ri','depth','di')}
    def selected():
        return adapter.select_presence_capture(node,queues,None,stamp+10_000_000)[0]
    assert selected()==stamp
    queues['depth'].clear();assert selected() is None
    queues['depth'][stamp]=True
    depth[:]=2.;rows,_=surface.localize_surfaces([d],depth,K,T)
    for capture in (1.,1.4,1.8):
        wire.capture(tracker,round(capture*1e9),rows,capture+.01)
    assert tracker.at(1).position and selected() is None


def test_perception_worker_recovers_after_a_work_item_exception():
    import ast

    path = ROOT/'src/person_search/node.py'
    tree = ast.parse(path.read_text())
    search = next(node for node in ast.walk(tree) if isinstance(node, ast.ClassDef) and node.name == 'Search')
    methods = [node for node in search.body if isinstance(node, ast.FunctionDef)
               and node.name in ('start_worker', 'stop_worker', 'wake_process')]
    env = dict(threading=threading, rclpy=N(ok=lambda: True))
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), env)
    failed = threading.Event()
    succeeded = threading.Event()
    errors = []

    def report_error(message, **kwargs):
        errors.append(message)
        failed.set()

    def process():
        if not failed.is_set():
            raise RuntimeError('fixture work item failure')
        succeeded.set()

    node = N(input_event=threading.Event(), worker_stop=threading.Event(),
             expiry_requested=False, pending_capture=lambda: False, process=process,
             get_logger=lambda: N(error=report_error))
    env['start_worker'](node)
    try:
        env['wake_process'](node)
        assert failed.wait(2.)
        env['wake_process'](node)
        assert succeeded.wait(2.) and len(errors) == 1
    finally:
        assert env['stop_worker'](node)
