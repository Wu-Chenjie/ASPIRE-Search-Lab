"""Independent reporting only; never imported by an online decision node."""
from collections import Counter

def audit_presence(telemetry,truth):
    reports=[r['data'] for r in telemetry if r['kind']=='tracks' and r['data'].get('schema_version')==2]
    events=[r['data'] for r in telemetry if r['kind']=='presence_event']
    confirmed={t['id'] for r in reports for t in r['tracks'] if t['visual_state']=='visually_confirmed'}
    state_counts=Counter(t['localization_state'] for r in reports for t in r['tracks'])
    total=0 if truth.get('person') is None else 1
    return dict(confirmation_policy='decoupled',confirmed_people=None,matched_person_tracks=None,duplicate_confirmed_tracks=None,
        false_confirmed_tracks=len(confirmed) if total==0 else None,first_true_confirmation_wall_s=None,confirmed_tracks=[],
        evaluator_ground_truth_people_total=total,visually_confirmed_local_track_records=len(confirmed),
        visually_confirmed_local_track_ids=sorted(confirmed),visual_confirmation_event_count=sum(e.get('event')=='visual_confirmed' for e in events),
        local_visual_track_records_max=max((r['counts']['local_visual_track_records'] for r in reports),default=0),
        reliable_localizations_max=max((r['counts']['reliable_localizations_current'] for r in reports),default=0),
        reliable_localizations_at_end=reports[-1]['counts']['reliable_localizations_current'] if reports else 0,
        same_frame_duplicates_suppressed_total=max((r['counts']['same_frame_duplicates_suppressed_total'] for r in reports),default=0),
        localization_state_report_samples=dict(state_counts),
        person_count_scope='Visual IDs are short-time local image tracks, not verified cross-view independent people. Ground truth total is evaluator-only. Positive visual identity requires independent image adjudication; no XYZ proximity shortcut.',
        actual_true_independent_confirmations='not_established_by_automatic_track_count')
