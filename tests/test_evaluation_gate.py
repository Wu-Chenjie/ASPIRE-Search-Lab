import copy
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from person_search.evaluation_gate import evaluate_claim,protocol_digest,SHARED_HASHES


def fixture():
    p=dict(status='frozen',false_positive_cap=1,validation_ids=['dev'],test_ids=['test'],time_budget_s=100,
           frozen_at='2026-01-01T00:00:00Z',detector_sha256='a'*64,confirmation_sha256='b'*64,
           identity_sha256='c'*64,validation_evidence_sha256='d'*64,
           common_stack_sha256='a'*64,resource_budget_sha256='b'*64,
           case_specs={'test':dict(seed=7,scene_sha256='c'*64,common_runtime_sha256='d'*64)})
    rows=[]
    for arm,n in [('baseline',2),('guided',3)]:
        rows.append(dict(test_id='test',arm=arm,protocol_sha256=protocol_digest(p),time_budget_s=100,
            started_at='2026-01-02T00:00:00Z',outcome='complete',execution_mode='closed_loop',evidence_sha256='f'*64,
            confirmation_latency_s=2.,reacquisitions=0,extra_path_m=0.,compute_s=.2,identity_uncertain=0,unique_true_discoveries=n,false_positives=1,duplicates=0,
            guidance_enabled=arm=='guided',original_safety_gates_passed=True,
            **p['case_specs']['test'],**{k:p[k] for k in SHARED_HASHES}))
    return p,rows


def test_unknown_cap_blocks_formal_claim_and_has_no_default_threshold():
    p,rows=fixture();p['false_positive_cap']=None
    result=evaluate_claim(p,rows)
    assert not result['claim_allowed'] and 'false_positive_cap_not_frozen' in result['errors']


def test_paired_gain_requires_frozen_shared_rules_and_full_results():
    p,rows=fixture();assert evaluate_claim(p,rows)['claim_allowed']
    for field,value,error in [('false_positives',2,'false_positive_cap_exceeded'),
                             ('outcome','aborted','incomplete_or_failed_run_requires_reporting'),
                             ('confirmation_sha256','e'*64,'shared_confirmation_sha256_mismatch'),
                             ('started_at','2025-01-01T00:00:00Z','test_not_after_freeze')]:
        changed=copy.deepcopy(rows);changed[1][field]=value
        result=evaluate_claim(p,changed);assert not result['claim_allowed'] and error in result['errors']
    assert not evaluate_claim(p,rows[:1])['claim_allowed']
    rows[1]['unique_true_discoveries']=2
    assert not evaluate_claim(p,rows)['claim_allowed']


def test_malformed_or_replayed_results_cannot_pass():
    p,rows=fixture()
    for field,value in [('test_id',[]),('execution_mode','recorded_replay'),
                        ('evidence_sha256',''),('compute_s',None),('started_at',None)]:
        changed=copy.deepcopy(rows);changed[1][field]=value
        assert not evaluate_claim(p,changed)['claim_allowed']
    p['validation_ids']=['test']
    assert not evaluate_claim(p,rows)['claim_allowed']


def test_arm_scene_resource_and_safety_mismatches_cannot_claim_gain():
    p,rows=fixture()
    for field,value in [('guidance_enabled',False),('seed',8),('scene_sha256','e'*64),
                        ('common_runtime_sha256','e'*64),('common_stack_sha256','e'*64),
                        ('resource_budget_sha256','e'*64),('original_safety_gates_passed',False),
                        ('reacquisitions',.5)]:
        changed=copy.deepcopy(rows);changed[1][field]=value
        assert not evaluate_claim(p,changed)['claim_allowed'],field
    rows[1]['extra_path_m']=-2.
    assert evaluate_claim(p,rows)['claim_allowed']


def test_no_confirmation_has_no_latency_and_must_report_it_explicitly():
    protocol, rows = fixture()
    baseline = rows[0]
    baseline.update(unique_true_discoveries=0, false_positives=0, confirmation_latency_s=None)
    result = evaluate_claim(protocol, rows)
    assert result['claim_allowed'] and result['paired_true_discovery_delta'] == 3
    for field in ('unique_true_discoveries', 'false_positives', 'duplicates', 'identity_uncertain'):
        for value in (1, False):
            changed = copy.deepcopy(rows)
            changed[0][field] = value
            assert 'missing_or_invalid_confirmation_latency_s' in evaluate_claim(protocol, changed)['errors']
    del baseline['confirmation_latency_s']
    assert 'missing_or_invalid_confirmation_latency_s' in evaluate_claim(protocol, rows)['errors']


def test_oversized_evidence_numbers_return_gate_errors():
    protocol, rows = fixture()
    rows[1]['compute_s'] = 10**400
    assert 'missing_or_invalid_compute_s' in evaluate_claim(protocol, rows)['errors']
    protocol['time_budget_s'] = 10**400
    assert 'time_budget_invalid' in evaluate_claim(protocol, rows)['errors']


def test_disqualified_run_rows_still_contribute_independent_diagnostics():
    protocol, rows = fixture()
    duplicate = dict(rows[1], outcome='aborted', original_safety_gates_passed=False)
    unexpected = dict(rows[0], arm='other', false_positives=5, duplicates=-1)
    result = evaluate_claim(protocol, [*rows, duplicate, unexpected])
    assert not result['claim_allowed']
    assert {
        'duplicate_run', 'unexpected_run', 'original_safety_gates_not_passed',
        'incomplete_or_failed_run_requires_reporting', 'false_positive_cap_exceeded',
        'invalid_duplicates',
    } <= set(result['errors'])
