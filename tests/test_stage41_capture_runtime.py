"""Regress actual lazy backend reset, capture context and unchanged age refusal."""
from types import SimpleNamespace as NS
import hashlib,sys
import numpy as np
import pytest

@pytest.mark.parametrize('preserve_mask_candidates,end2end',[(False,False),(True,False),(True,True)])
def test_backend_thread_changes_are_reported_without_rejecting_inference(tmp_path,monkeypatch,preserve_mask_candidates,end2end):
    from person_search import yolo
    state = {'threads': 1, 'requests': []}
    torch = NS(set_num_threads=state['requests'].append, get_num_threads=lambda: state['threads'])
    class Model:
        names={0:'person'}
        predictor=NS(model=NS(end2end=end2end))
        def __init__(self,*a,**k):self.ready=False;self.sources=[]
        def predict(self,source,**kwargs):
            self.sources.append((source.copy(),kwargs))
            if not self.ready:
                state['threads'] = 5
                self.ready = True
            else:
                state['threads'] += 1
            return [NS(masks=None)]
    monkeypatch.setitem(sys.modules,'torch',torch)
    monkeypatch.setitem(sys.modules,'ultralytics',NS(YOLO=Model,settings={}))
    monkeypatch.setattr(yolo.importlib.metadata,'version',lambda _:yolo.VERSION)
    f=tmp_path/'weights.pt';f.write_bytes(b'fixture');monkeypatch.setattr(yolo,'WEIGHT_SHA256',hashlib.sha256(f.read_bytes()).hexdigest())
    if end2end:
        with pytest.raises(ValueError,match='without embedded NMS'):
            yolo.YoloPersonDetector(f,preserve_mask_candidates=preserve_mask_candidates)
        return
    d=yolo.YoloPersonDetector(f,preserve_mask_candidates=preserve_mask_candidates)
    assert d.runtime_diagnostics()['backend_threads_after_warmup'] == 5
    assert d.runtime_diagnostics()['actual_threads'] == 5
    assert state['requests'] == [2, 2]
    assert len(d.model.sources)==1 and not d.model.sources[0][0].any()
    assert d.detect(np.ones((240,320,3),np.uint8))==[]
    assert d.runtime_diagnostics()['actual_threads'] == 6
    assert d.model.sources[1][1]['imgsz']==640 and d.model.sources[1][1]['conf']==.35
    for _, options in d.model.sources:
        if preserve_mask_candidates:
            assert options['iou']==1.0 and options['max_det']==yolo.MASK_CANDIDATE_CAP
        else:
            assert 'iou' not in options and 'max_det' not in options
    state['threads'] = 3
    assert d.detect(np.ones((240,320,3), np.uint8)) == []
    assert d.runtime_diagnostics()['actual_threads'] == 4

def test_lazy_detector_reads_loaded_trace_labels_after_warmup_once(tmp_path,monkeypatch):
    from person_search.detector_spec import DetectorSpec
    from person_search import yolo
    torch=NS(set_num_threads=lambda _:None,get_num_threads=lambda:2)

    class Model:
        def __init__(self,*args,**kwargs):
            self.warmed=False
            self.reads=0
        def predict(self,**kwargs):
            self.warmed=True
        @property
        def names(self):
            assert self.warmed
            self.reads+=1
            return {0:'footprint',1:'fallen_material'}

    monkeypatch.setitem(sys.modules,'torch',torch)
    monkeypatch.setitem(sys.modules,'ultralytics',NS(YOLO=Model,settings={}))
    monkeypatch.setattr(yolo.importlib.metadata,'version',lambda _:yolo.VERSION)
    f=tmp_path/'weights.pt'
    f.write_bytes(b'fixture')
    spec=DetectorSpec(model_sha256=hashlib.sha256(f.read_bytes()).hexdigest(),
                      purpose='trace',class_map={0:'footprint',1:'fallen_material'})
    assert yolo.YoloPersonDetector(f,spec=spec).model.reads==1


def test_bound_identity_context_survives_inference_but_current_geometry_still_expires(monkeypatch):
    from person_search.presence_adapter import bind_identity_context,capture_world
    class TransformException(Exception):pass
    monkeypatch.setitem(sys.modules,'tf2_ros',NS(TransformException=TransformException))
    ns=10000000000;clock={'sim':ns+100000000,'wall':.1}
    header=NS(frame_id='optical')
    rgb=NS(header=header,width=320,height=240)
    info=NS(header=header,width=320,height=240,k=[200.,0,160.,0,200.,120.,0,0,1.])
    transform=NS(header=NS(frame_id='map',stamp=NS(sec=10,nanosec=0)),child_frame_id='optical',
        transform=NS(translation=NS(x=1.,y=2.,z=3.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    node=NS(require_pose=True,max_age=.75,rgbd_registration_mode='strict',fresh_capture=lambda _:clock['wall']<=.75,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock['sim'])),
        buffer=NS(lookup_transform=lambda *a:transform))
    history=NS(brackets=lambda _:True);queues={'ri':{ns:info}}
    bound,error=bind_identity_context(node,ns,rgb,queues,history,lambda **k:None)
    assert error is None and bound['bound_sim_ns']==clock['sim']
    clock.update(sim=ns+400000000,wall=.9)
    assert bound['transform'][:3,3].tolist()==[1.,2.,3.]
    with pytest.raises(ValueError,match='stale_or_future_capture'):capture_world(node,'optical',ns,history,lambda **k:None)
    missing,reason=bind_identity_context(node,ns,rgb,queues,history,lambda **k:None)
    assert missing is None and reason=='stale_or_future_capture'
    clock['wall']=.1
    def unavailable(*args):raise TransformException('missing capture TF')
    node.buffer.lookup_transform=unavailable
    assert bind_identity_context(node,ns,rgb,queues,history,lambda **k:None)==(None,'missing capture TF')
