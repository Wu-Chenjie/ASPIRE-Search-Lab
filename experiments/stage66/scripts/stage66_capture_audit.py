"""Post-run actual recording audit. Absence of motion is not a safety success."""
from pathlib import Path
import collections,gzip,json
import numpy as np

ERRORS=('oversize','queue_dropped','disk_dropped','write_errors','trigger_dropped')
def validate(root,session):
    root=Path(root);fail=[];roles={};records={}
    def check(ok,why):
        if not ok:fail.append(why)
    for role in ['mapper','executor','gateway']:
        folder=root/'map-evidence'/('drone-0-'+role);sf=folder/'status.json'
        status=json.loads(sf.read_text()) if sf.exists() else {};roles[role]=dict(status=status)
        check(status.get('session')==session and status.get('closed') is True,role+':closed session')
        check(not any(status.get(k,0) for k in ERRORS) and not status.get('writer_alive') and status.get('pending',0)==0,role+':no capture loss')
        rows=[]
        for f in sorted(folder.glob('*.npz')):
            with np.load(f,allow_pickle=False) as z:
                meta=json.loads(bytes(z['metadata_json']).decode());a=meta['record'];rows.append(meta)
                if a.get('kind') not in ['probe_current_state','gateway_rejected_proposal']:continue
                check(a.get('diagnostic_only') is True and a.get('policy_query_performed') is False,role+':passive never query credit')
                for flag,key,shape,codes in [('mapper_state_available','mapper_state','mapper_shape',[-1,0,1]),('native_state_available','native_state','native_shape',[0,1,2,4,5,6])]:
                    if a.get(flag):check(key in z and list(z[key].shape)==a.get(shape) and bool(np.isin(z[key],codes).all()),role+':actual '+key+' shape/codes')
                    else:check(key not in z,role+':absent '+key+' not invented')
        check(status.get('written')==len(rows),role+':all written files present')
        check(status.get('hook_max_wall_s',1e9)<=.2 and status.get('disk_bytes',1e9)<=status.get('disk_limit',0),role+':original hook and disk budgets')
        probes=[a for a in rows if a['record']['kind']=='probe_current_state' and a['trigger']['id']=='stage65-startup-probe']
        check(len(probes)==1,role+':one actual startup state response')
        check(all(a['phase']=='probe' and a['record'].get('callback_sim_ns',-1)>=0 and a['record'].get('probe_id')=='stage65-startup-probe' for a in probes),role+':typed actual probe response')
        records[role]=rows;roles[role].update(probe_responses=len(probes))
    mapper_post=[a for a in records['mapper'] if a['phase']=='post' and a['trigger']['id']=='stage65-startup-probe' and a['record']['kind']=='mapper_consumed_input']
    executor_post=[a for a in records['executor'] if a['phase']=='post' and a['trigger']['id']=='stage65-startup-probe' and a['record']['kind']=='safety_query']
    check(len(mapper_post)>=2,'two real post-trigger mapper inputs')
    producer=root/'startup-capture-probe.json';published=json.loads(producer.read_text()) if producer.exists() else {}
    check(published.get('subscriptions',0)>=3 and published.get('trigger',{}).get('session')==session and published.get('diagnostic_only') is True,'actual three-subscription public trigger')
    ack=published.get('acknowledged_roles',{})
    sends=published.get('publish_wall_offsets',[])
    check(len(sends)==published.get('publish_attempts') and all(b-a>=.1-1e-6 for a,b in zip(sends,sends[1:])),'actual request rate <=10Hz without discovery catch-up bursts')
    check(set(ack)=={'mapper','executor','gateway'} and published.get('elapsed_wall_s',1e9)<=8. and published.get('publish_attempts',1000)<=80,'bounded actual three-role acknowledgment')
    for role in ['mapper','executor','gateway']:
        probes=[a for a in records[role] if a['record']['kind']=='probe_current_state' and a['trigger']['id']=='stage65-startup-probe']
        check(len(probes)==1 and ack.get(role,{}).get('callback_sim_ns')==probes[0]['record'].get('callback_sim_ns'),role+':ack bound to actual state record')
    startup_failures=list(fail)
    journal=root/'timing/drone-0-executor-observation.jsonl.gz';statusfile=root/'timing/drone-0-executor-observation.status.json';ticks=[];journal_ok=False;journal_rows=0
    if journal.exists() and statusfile.exists():
        s=json.loads(statusfile.read_text());seq=collections.Counter();ok=True
        with gzip.open(journal,'rt') as f:
            for line in f:
                x=json.loads(line);journal_rows+=1;seq[x['event']]+=1
                ok=ok and x.get('session')==session and x.get('event_seq')==seq[x['event']]
                if x['event']=='tick_boundary':
                    ok=ok and {'token','native_id','curve_duration','sim_ns','safety_fault'}<=x.keys()
                    ticks.append(x)
        journal_ok=bool(ticks) and ok and s.get('written')==s.get('enqueued')==journal_rows and s.get('dropped')==s.get('write_errors')==0 and not s.get('writer_alive') and s.get('pending',0)==0
    check(journal_ok,'complete actual high-rate execution journal')
    eventfile=root/'flight/0/events.jsonl';events=[json.loads(x) for x in eventfile.read_text().splitlines()] if eventfile.exists() else []
    activations=[x for x in events if x['type']=='native_trajectory_activated'];rejections=[x for x in events if x['type']=='native_proposal_rejected']
    token_ticks=[x for x in ticks if x.get('token') is not None or x.get('native_id') is not None or x.get('curve_duration') is not None]
    any_queries=any(x['record']['kind']=='safety_query' for x in records['executor'])
    exercised=bool(activations or token_ticks or any_queries)
    if exercised:check(len(executor_post)>=4,'four actual post-trigger execution safety records when exercised')
    else:check(journal_ok and not activations and not token_ticks and not any_queries,'no-authorized-execution must be evidenced, not assumed')
    captured=[a['record'] for a in records['gateway'] if a['record']['kind']=='gateway_rejected_proposal']
    check(len(captured)==min(3,len(rejections)),'first-three actual rejection contexts complete')
    for i,a in enumerate(captured):
        check(a.get('rejection_ordinal')==i+1 and a.get('rejected_proposal',{}).get('traj_id')==rejections[i].get('traj_id') and a.get('rejection_reason')==rejections[i].get('reason'),'rejection exact original binding')
    return dict(version='Stage66 actual acknowledgment + immutable Stage65 typed capture contract',startup_state_verified=not startup_failures,execution_context_verified=len(fail)==len(startup_failures),passed=not fail,failures=fail,roles=roles,
        mapper_actual_post_inputs=len(mapper_post),executor_actual_post_queries=len(executor_post),
        legacy_2mapper_4executor_counts_satisfied=len(mapper_post)>=2 and len(executor_post)>=4,
        policy_exercised=exercised,execution_evidence_class='exercised_and_recorded' if exercised else 'no_authorized_trajectory_policy_not_exercised',
        no_full_flight_safety_certification=True,journal_complete=journal_ok,actual_journal_rows=journal_rows,actual_tick_boundaries=len(ticks),actual_native_activations=len(activations),actual_native_rejections=len(rejections),bounded_rejection_contexts=len(captured))

if __name__=='__main__':
    import sys
    print(json.dumps(validate(Path(sys.argv[1]),sys.argv[2]),indent=2))
