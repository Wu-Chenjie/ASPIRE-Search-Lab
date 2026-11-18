"""Offline claim gate. No truth data may enter perception/navigation.

Checks declared freeze/provenance contracts; does not authenticate dataset
independence, annotation correctness or caller-provided timestamps.
"""
import datetime as dt
from aspire_interfaces.provenance import protocol_digest
import re
from sys import float_info

SHARED_HASHES=('detector_sha256','confirmation_sha256','identity_sha256',
               'common_stack_sha256','resource_budget_sha256')
CASE_HASHES = ('scene_sha256', 'common_runtime_sha256')
DISCOVERY_COUNTS = ('unique_true_discoveries', 'false_positives', 'duplicates')


def is_sha256(value):
    return isinstance(value,str) and re.fullmatch('[0-9a-f]{64}',value) is not None


def is_finite_number(value):
    return type(value) in (int, float) and -float_info.max <= value <= float_info.max


def unique_ids(values):
    if not isinstance(values, list) or any(not isinstance(v, str) or not v for v in values):
        return None
    ids = set(values)
    return ids if len(ids) == len(values) else None


def _count(value):
    return type(value) is int and value >= 0


def _timestamp(value):
    if not isinstance(value, str):
        return None
    try:
        stamp = dt.datetime.fromisoformat(value[:-1] + '+00:00' if value.endswith('Z') else value)
    except ValueError:
        return None
    return stamp if stamp.tzinfo is not None else None


def evaluate_claim(protocol, runs):
    errors=set()
    if not isinstance(protocol,dict) or not isinstance(runs,list):
        return dict(claim_allowed=False,errors=['invalid_protocol_or_runs'],reason='blocked')
    cap=protocol.get('false_positive_cap')
    if not _count(cap):errors.add('false_positive_cap_not_frozen')
    if protocol.get('status')!='frozen':errors.add('protocol_not_frozen')

    validation = unique_ids(protocol.get('validation_ids', []))
    test = unique_ids(protocol.get('test_ids', []))
    if validation is None:errors.add('validation_ids_invalid')
    if test is None:errors.add('test_ids_invalid')
    if not validation or not test or validation&test:errors.add('validation_test_split_invalid')
    test = test or set()
    for field in (*SHARED_HASHES,'validation_evidence_sha256'):
        if not is_sha256(protocol.get(field)):errors.add(field+'_missing')
    cases=protocol.get('case_specs')
    if not isinstance(cases,dict):
        errors.add('frozen_case_specs_missing')
        cases={}
    elif cases.keys()!=test:
        errors.add('frozen_case_specs_missing')
    for case in cases.values():
        if (not isinstance(case,dict) or type(case.get('seed')) is not int
                or not all(is_sha256(case.get(k)) for k in CASE_HASHES)):
            errors.add('frozen_case_spec_invalid')
    budget=protocol.get('time_budget_s')
    if not is_finite_number(budget) or budget<=0:errors.add('time_budget_invalid')
    frozen = _timestamp(protocol.get('frozen_at'))
    if frozen is None:errors.add('freeze_time_invalid')
    try:digest=protocol_digest(protocol)
    except (ValueError,TypeError):digest=None;errors.add('protocol_not_serializable')
    groups={}
    for row in runs:
        if not isinstance(row,dict):errors.add('invalid_run');continue
        if row.get('execution_mode')!='closed_loop':errors.add('not_closed_loop_evidence')
        if not is_sha256(row.get('evidence_sha256')):errors.add('run_evidence_missing')
        for metric in ('compute_s', 'extra_path_m'):
            value = row.get(metric)
            if not is_finite_number(value) or (metric == 'compute_s' and value < 0):
                errors.add('missing_or_invalid_'+metric)
        latency = row.get('confirmation_latency_s')
        if latency is None and 'confirmation_latency_s' in row:
            valid_latency = all(type(row.get(field)) is int and row[field] == 0
                                for field in (*DISCOVERY_COUNTS, 'identity_uncertain'))
        else:
            valid_latency = is_finite_number(latency) and latency >= 0
        if not valid_latency:
            errors.add('missing_or_invalid_confirmation_latency_s')
        for metric in ('reacquisitions','identity_uncertain'):
            if not _count(row.get(metric)):errors.add('missing_or_invalid_'+metric)
        if row.get('original_safety_gates_passed') is not True:errors.add('original_safety_gates_not_passed')
        if type(row.get('seed')) is not int:errors.add('invalid_seed')
        if row.get('protocol_sha256')!=digest:errors.add('protocol_changed_after_freeze')
        row_budget=row.get('time_budget_s')
        if type(row_budget) not in (int,float) or row_budget!=budget:errors.add('unequal_time_budget')
        for field in SHARED_HASHES:
            if row.get(field)!=protocol.get(field):errors.add('shared_'+field+'_mismatch')
        started = _timestamp(row.get('started_at'))
        if frozen is None or started is None or started <= frozen:errors.add('test_not_after_freeze')
        if row.get('outcome')!='complete':errors.add('incomplete_or_failed_run_requires_reporting')
        for field in DISCOVERY_COUNTS:
            if not _count(row.get(field)):errors.add('invalid_'+field)
        false_positives = row.get('false_positives')
        if type(cap) is int and type(false_positives) is int and false_positives>cap:
            errors.add('false_positive_cap_exceeded')
        ident=row.get('test_id');arm=row.get('arm')
        if not isinstance(ident,str) or ident not in test or arm not in ('baseline','guided'):
            errors.add('unexpected_run');continue
        if row.get('guidance_enabled') is not (arm=='guided'):
            errors.add('guidance_arm_mismatch')
        case=cases.get(ident)
        if not isinstance(case,dict) or any(row.get(k)!=case.get(k) for k in (*CASE_HASHES, 'seed')):
            errors.add('frozen_case_mismatch')
        pair = groups.setdefault(ident, {})
        if arm in pair:
            errors.add('duplicate_run')
        else:
            discoveries = row.get('unique_true_discoveries')
            pair[arm] = discoveries if _count(discoveries) else None
    if len(groups) != len(test) or any(len(pair) != 2 for pair in groups.values()):
        errors.add('missing_paired_run')
    total_delta = sum(pair['guided'] - pair['baseline'] for pair in groups.values()
                      if len(pair) == 2 and None not in pair.values())
    if not runs:errors.add('no_test_results')
    accepted=not errors and total_delta>0
    return dict(claim_allowed=accepted,errors=sorted(errors),paired_true_discovery_delta=total_delta,
                false_positive_cap=cap,scope='offline_declared_protocol_checks_not_independence_certification',
                reason='positive_paired_gain_within_frozen_cap' if accepted else 'blocked_or_no_positive_gain')
