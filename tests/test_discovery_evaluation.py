import copy
import json
import subprocess
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import pytest
from person_search.discovery_evaluation import evaluate_discoveries


def event(i,ident,t,state='visually_confirmed'):
    return dict(event_id=i,identity_id=ident,capture_s=t,visual_state=state)


def annotation(ident,target=None,status='matched'):
    return dict(identity_id=ident,truth_id=target,status=status,evidence_sha256='a'*64)


def test_counts_use_independent_identity_and_time_not_localization():
    events=[event('a','A',1),event('b','B',2),event('c','C',3),event('d','D',4),
            event('e','E',101),event('f','F',5,'person_candidate'),event('g','A',6)]
    annotations=[annotation('A','person1'),annotation('B','person1'),
                 annotation('C',status='false_positive'),annotation('E','person2')]
    result=evaluate_discoveries(events,annotations,['person1','person2'],100)
    assert [result[k] for k in ('unique_true_discoveries','false_positives','duplicates','identity_uncertain')]==[1,1,1,1]
    assert result['repeat_identity_events']==1 and result['outside_budget_events']==1
    assert result['undiscovered_truth_ids']==['person2']
    noisy=copy.deepcopy(events)
    for e in noisy:e.update(localization_state='unreliable',xyz=[10000,10000,10000])
    assert evaluate_discoveries(noisy,annotations,['person1','person2'],100)==result


def test_transport_repeats_are_idempotent_but_conflicting_ids_rejected():
    e=event('a','A',100);a=annotation('A','person1')
    result=evaluate_discoveries([e,e],[a],['person1'],100)
    assert result['unique_true_discoveries']==1 and result['duplicates']==0 and result['transport_duplicate_events']==1
    with pytest.raises(ValueError):evaluate_discoveries([e,dict(e,identity_id='B')],[a],['person1'],100)
    with pytest.raises(ValueError):evaluate_discoveries([e],[a,a],['person1'],100)
    with pytest.raises(ValueError):evaluate_discoveries([e],[dict(a,truth_id='absent')],['person1'],100)
    result=evaluate_discoveries([e],[],['person1'],100)
    assert result['unique_true_discoveries']==0 and result['identity_uncertain']==1


def test_oversized_discovery_seconds_are_reported_as_invalid():
    with pytest.raises(ValueError, match='Positive finite time budget'):
        evaluate_discoveries([], [], [], 10**400)
    with pytest.raises(ValueError, match='Capture must be finite'):
        evaluate_discoveries([event('a', 'A', 10**400)], [], [], 100)


def test_bad_annotations_and_events_are_reported_without_exporting_a_score(tmp_path):
    root = Path(__file__).resolve().parents[1]
    output = tmp_path/'score.json'
    cmd = [sys.executable, '-B', str(root/'tools/search/evaluate_discoveries.py'),
           '--time-budget-s', '100', '--output', str(output)]
    first = event('a', 'A', 10)
    for name, rows in (
        ('annotations', [annotation('A', 'absent'), annotation('B', status='bad')]),
        ('events', [first, dict(first, identity_id='conflict'), event('b', 'B', -1)]),
        ('truth-ids', ['person1']),
    ):
        path = tmp_path/(name + '.json')
        path.write_text(json.dumps(rows))
        cmd.extend(['--' + name, str(path)])
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 2 and not output.exists()
    assert 'annotations[0]: Unknown truth' in result.stderr
    assert 'annotations[1]: Annotation status' in result.stderr
    assert 'events[1]: Conflicting duplicate' in result.stderr
    assert 'events[2]: Capture must be finite' in result.stderr
