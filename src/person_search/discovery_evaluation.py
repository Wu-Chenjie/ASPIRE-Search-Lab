"""Offline scoring from independently reviewed identity matches, never online truth.

Annotations are supplied evidence, not inferred from 3-D position or model labels.
This module does not certify annotation independence or correctness.
"""
from .evaluation_gate import is_finite_number, is_sha256, unique_ids


def evaluate_discoveries(events, annotations, truth_ids, time_budget_s):
    if not is_finite_number(time_budget_s) or time_budget_s<=0:
        raise ValueError('Positive finite time budget required')
    truth = unique_ids(truth_ids)
    if truth is None:
        raise ValueError('Explicit unique offline truth IDs required')
    if not isinstance(events,list) or not isinstance(annotations,list):
        raise ValueError('Events and annotations must be arrays')
    errors=[];review={}
    for index,row in enumerate(annotations):
        try:
            if not isinstance(row,dict):raise ValueError('Invalid annotation')
            ident=row.get('identity_id');status=row.get('status')
            if not isinstance(ident,str) or not ident or ident in review:raise ValueError('Unique annotation identity required')
            if status not in ('matched','false_positive','uncertain') or not is_sha256(row.get('evidence_sha256')):
                raise ValueError('Annotation status and evidence hash required')
            target=row.get('truth_id')
            if status=='matched':
                if not isinstance(target,str) or target not in truth:raise ValueError('Unknown truth identity')
            elif target is not None:raise ValueError('Ambiguous annotation must not assign truth')
            review[ident]=row
        except ValueError as error:
            errors.append(f'annotations[{index}]: {error}')
    seen={};first={};transport_duplicates=0;outside=0;unconfirmed=0;repeat_identity_events=0
    for index,row in enumerate(events):
        try:
            if not isinstance(row,dict):raise ValueError('Invalid discovery event')
            event=row.get('event_id');ident=row.get('identity_id');stamp=row.get('capture_s')
            if not isinstance(event,str) or not event or not isinstance(ident,str) or not ident:
                raise ValueError('Stable event and algorithm identity IDs required')
            if not is_finite_number(stamp) or stamp<0:
                raise ValueError('Capture must be finite mission-relative simulation seconds')
            state=row.get('visual_state')
            if state not in ('person_candidate','visually_confirmed'):raise ValueError('Explicit visual state required')
            signature=(ident,stamp,state)
            prior=seen.get(event)
            if prior is not None:
                if prior!=signature:raise ValueError('Conflicting duplicate event ID')
                transport_duplicates+=1
                continue
        except ValueError as error:
            errors.append(f'events[{index}]: {error}')
            continue
        seen[event]=signature
        if stamp>time_budget_s:outside+=1;continue
        if state!='visually_confirmed':unconfirmed+=1;continue
        previous=first.get(ident)
        if previous is not None:
            repeat_identity_events+=1
            stamp=min(stamp,previous)
        first[ident]=stamp
    if errors:
        raise ValueError('\n'.join(errors))
    matched={};false=[];uncertain=[]
    for ident,stamp in sorted(first.items(),key=lambda pair:(pair[1],pair[0])):
        row=review.get(ident)
        if row is None or row['status']=='uncertain':uncertain.append(ident)
        elif row['status']=='false_positive':false.append(ident)
        else:matched.setdefault(row['truth_id'],[]).append(dict(identity_id=ident,first_confirmation_s=stamp))
    return dict(unique_true_discoveries=len(matched),false_positives=len(false),
        duplicates=sum(len(items)-1 for items in matched.values()),identity_uncertain=len(uncertain),
        matched_targets=matched,false_positive_identities=false,uncertain_identities=uncertain,
        transport_duplicate_events=transport_duplicates,repeat_identity_events=repeat_identity_events,
        outside_budget_events=outside,unconfirmed_events=unconfirmed,time_budget_s=time_budget_s,
        undiscovered_truth_ids=sorted(truth-set(matched)),localization_is_discovery_gate=False,
        scope='offline_independent_annotation_input_not_annotation_certification_or_closed_loop_gain')
