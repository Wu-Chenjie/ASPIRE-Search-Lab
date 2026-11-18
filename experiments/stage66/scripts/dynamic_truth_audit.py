"""Post-run GT audit. Frozen data only; no new inference or simulation."""
from pathlib import Path
import sys,json,math,collections,hashlib,xml.etree.ElementTree as X
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw
B=Path('/Users/wuchenjie/Documents/Codex/2026-10-13');S=B/'task-3-stage45'
sys.path[:0]=[str(S),str(S/'src'),str(S/'src/annual_planner/src')]
from tools.flight.flight_localization import interpolate,camera
from person_search.perception import cloud
E=Path('/Volumes/WuChenjie/Mac-runtime/runtime-stage52-20261018T1315Z/evidence/stage66');O=(Path(sys.argv[2]) if len(sys.argv)>2 else E/'audits');O.mkdir(parents=True,exist_ok=True)
from raster_fast import raster

def stats(v):
 return dict(n=len(v),median_m=float(np.median(v)),p95_m=float(np.percentile(v,95)),max_m=max(v)) if v else dict(n=0)
def iou(a,b):
 a=np.asarray(a);b=np.asarray(b);lo=np.maximum(a[:2],b[:2]);hi=np.minimum(a[:2]+a[2:],b[:2]+b[2:]);inter=np.maximum(0,hi-lo).prod()
 return inter/(a[2]*a[3]+b[2]*b[3]-inter+1e-9)
D=np.load(B/'task-3-evidence/stage-31-hypotheses-localization/audits/actual-person-mesh.npz');reports=[];panels=[]
for p in ([Path('/Volumes/ColimaRuntime/runtime/research-stage66/runs')/sys.argv[1]] if len(sys.argv)>1 else sorted(Path('/Volumes/ColimaRuntime/runtime/research-stage66/runs').iterdir())):
 world=X.parse(p/'scene.sdf').getroot().find('world');actors=[];boxes=[]
 for model in world.findall('model'):
  v=list(map(float,(model.findtext('pose') or '0 0 0 0 0 0').split()));R=Rotation.from_euler('xyz',v[3:]).as_matrix()
  if model.get('name').startswith('stationary_person'):
   vertices=D['vertices']@R.T+v[:3];actors.append((model.get('name'),vertices[D['faces']]))
  else:
   for link in model.findall('link'):
    for col in link.findall('collision'):
     size=col.findtext('geometry/box/size')
     if size:
      assert np.max(np.abs(v[3:]))<1e-9 and link.find('pose') is None and col.find('pose') is None
      half=np.array(list(map(float,size.split())))/2;boxes.append((np.array(v[:3])-half,np.array(v[:3])+half))
 truth=sorted({x['stamp']:x for x in map(json.loads,(p/'truth-poses.jsonl').read_text().splitlines())}.values(),key=lambda x:x['stamp'])
 rows=[json.loads(s) for s in (p/'perception/observations.jsonl').read_text().splitlines()]
 sem={int(x['stamp_ns']):x for x in rows if x['kind']=='semantic'}
 semrows=collections.defaultdict(list)
 for x in rows:
  if x['kind']=='semantic':semrows[int(x['stamp_ns'])].extend(x['candidates'])
 sourcekeys={}
 for x in rows:
  if x['kind']=='registry_observations':
   for o in x['batch']['observations']:sourcekeys[o['counter']]='/'.join([o['robot'],o['session'],o['counter']])

 confirms=[x for x in rows if x['kind']=='presence_event' and x['event']=='visual_confirmed']
 k=np.array([[320/(2*math.tan(1.274/2)),0,160],[0,320/(2*math.tan(1.274/2)),120],[0,0,1.]])
 def getgt(ns,k=k,shape=(240,320)):
  pose=interpolate(truth,ns*1e-9)
  if pose is None:return None
  tc=camera(*pose);depths=[raster(mesh,k,tc,shape) for _,mesh in actors]
  yy,xx=np.mgrid[:shape[0],:shape[1]];dirs=np.stack(((xx-k[0,2])/k[0,0],(yy-k[1,2])/k[1,1],np.ones(shape)),axis=-1)@tc[:3,:3].T
  occluder=np.full(shape,np.inf)
  with np.errstate(divide='ignore',invalid='ignore'):
   for lo,hi in boxes:
    a=(lo-tc[:3,3])/dirs;b=(hi-tc[:3,3])/dirs
    near=np.minimum(a,b).max(axis=-1);far=np.maximum(a,b).min(axis=-1)
    hits=(far>=near)&(near>.05);occluder=np.minimum(occluder,np.where(hits,near,np.inf))
  minimum=np.minimum.reduce([occluder,*depths]);visible=[np.isfinite(d)&(d<=minimum+.005) for d in depths]
  return tc,depths,visible
 sources={};confirmation_rows=[]
 for event in confirms:
  ns=int(round(event['capture_time_s']*1e9));rec=dict(source_counter=str(event['track_id']),capture_ns=ns,processing_s=event['processing_time_s'])
  zpath=p/'perception'/f'{ns}-rgb-masks.npz';row=next((v for v in sem.get(ns,{}).get('candidates',[]) if v['id']==event['track_id']),None)
  gt=getgt(ns)
  if not zpath.exists() or row is None or gt is None:
   rec.update(status='unscorable',reason='missing_capture_mask_semantic_or_truth_bracket');confirmation_rows.append(rec);continue
  z=np.load(zpath);j=max(range(len(z['boxes'])),key=lambda j:iou(row['box'],z['boxes'][j]),default=None)
  if j is None or iou(row['box'],z['boxes'][j])<.95:
   rec.update(status='unscorable',reason='no_exact_detection_box');confirmation_rows.append(rec);continue
  mask=z['masks'][j];tc,depths,visible=gt
  scores=[dict(actor=name,predicted_mask_overlap=float((mask&v).sum()/max(1,mask.sum())),intersection_pixels=int((mask&v).sum()),visible_actor_pixels=int(v.sum())) for (name,_),v in zip(actors,visible)]
  ordered=sorted(scores,key=lambda s:s['predicted_mask_overlap'],reverse=True);first=ordered[0] if ordered else dict(actor=None,predicted_mask_overlap=0.,intersection_pixels=0);second=ordered[1]['predicted_mask_overlap'] if len(ordered)>1 else 0
  matched=first['predicted_mask_overlap']>=.5 and first['predicted_mask_overlap']-second>=.1 and first['intersection_pixels']>=20
  rec.update(status='matched_true_actor' if matched else 'unscorable_or_unmatched',scores=scores,box=row['box'],actor=first['actor'] if matched else None)
  sources[sourcekeys[str(event['track_id'])]]=rec['actor'];confirmation_rows.append(rec)
  im=Image.fromarray(z['rgb']).copy();draw=ImageDraw.Draw(im);a=row['box'];draw.rectangle((a[0],a[1],a[0]+a[2],a[1]+a[3]),outline='yellow',width=2)
  for index,v in enumerate(visible):
   ys,xs=np.where(v)
   if len(xs):draw.rectangle((xs.min(),ys.min(),xs.max(),ys.max()),outline=('cyan','magenta')[index%2])
  panel=Image.new('RGB',(400,280),'white');panel.paste(im,(0,0));pd=ImageDraw.Draw(panel)
  pd.text((4,244),f'{p.name} source {event["track_id"]} {ns/1e9:.3f}s',fill='black')
  pd.text((4,260),str(rec['actor'])+' '+str(round(first['predicted_mask_overlap'],3)),fill='black');panels.append(panel)
 telemetry=[json.loads(s) for s in (p/'telemetry.jsonl').read_text().splitlines()]
 last=next(x['data'] for x in reversed(telemetry) if x['kind']=='person_records')
 hyp=[]
 for h in last['hypotheses']:
  if h['visual_state']!='visually_confirmed':continue
  labels={sources.get(i) for i in h['source_ids']};labels.discard(None)
  hyp.append(dict(id=h['id'],source_ids=h['source_ids'],matched_actors=sorted(labels),wrong_merge=len(labels)>1))
 matched_hyp=[h for h in hyp if len(h['matched_actors'])==1];unique={h['matched_actors'][0] for h in matched_hyp}
 confreport=dict(run=p.name,scope='Offline post-run matching at first visual confirmation only; scoring rule frozen before Stage34 runs; finite diagnostic batch, not benchmark.',
  rule='Saved predicted mask overlaps one visible true actor >=0.5, >=0.1 margin over next and >=20 pixels; exact saved detection box IoU>=0.95. Mesh visibility includes other actors and scene box collisions.',
  true_scene_actors=[n for n,_ in actors],events=confirmation_rows,matched_confirmation_events=sum(x['status']=='matched_true_actor' for x in confirmation_rows),
  unique_matched_true_actors=len(unique),source_confirmation_duplicates=sum(x['status']=='matched_true_actor' for x in confirmation_rows)-len({x['actor'] for x in confirmation_rows if x.get('actor')}),
  confirmed_hypotheses=hyp,confirmed_hypothesis_duplicate_excess=len(matched_hyp)-len(unique),known_wrong_merges=sum(h['wrong_merge'] for h in hyp),
  limit='First-confirmation image audit does not prove stable identity through every later frame or no false positives at other times. Later selected reappearance/identity checks are separate; ground and post-takeoff confirmations remain distinguishable.')
 (O/(p.name+'-confirmation-truth.json')).write_text(json.dumps(confreport,indent=2)+'\n')
 # Current reliable publications, preserving the earliest qualified sample for each source/capture.
 qualified={}
 for x in rows:
  if x['kind']!='registry_observations':continue
  for a in x['batch']['observations']:
   if a['has_position']:qualified.setdefault((a['counter'],int(round(a['capture_s']*1e9))),a)
 q=sorted(qualified.items(),key=lambda z:(z[0][1],z[0][0]))
 chosen=list(q) # All unique qualified saved source/captures, no sampling
 for h in hyp:
  item=next((item for item in q if sourcekeys.get(item[0][0]) in h['source_ids']),None)
  if item is not None and item[0] not in {entry[0] for entry in chosen}:chosen.append(item)
 chosen.sort(key=lambda item:(item[0][1],item[0][0]))
 samples=[]
 for (counter,ns),a in chosen:
  rec=dict(source_counter=counter,source_capture_ns=ns)
  row=next((v for v in semrows[ns] if str(v['id'])==counter and v.get('position') is not None and
            np.linalg.norm(np.array(v['position']['xyz'])-np.array([a[c] for c in ('px','py','pz')]))<1e-8),None)
  pos=row.get('position') if row else None
  if pos is None:
   rec.update(status='failure',reason='no_matching_position_semantic');samples.append(rec);continue
  geo_ns=int(round(pos['capture_time_s']*1e9));rec['position_capture_ns']=geo_ns
  gpath=p/'perception'/f'{geo_ns}-geometry.npz';sp=p/'perception'/f'{geo_ns}-supports.npz';zp=p/'perception'/f'{geo_ns}-rgb-masks.npz'
  grow=next((v for v in sem.get(geo_ns,{}).get('candidates',[]) if str(v['id'])==counter),None)
  if not all(f.exists() for f in (gpath,sp,zp)) or grow is None:
   rec.update(status='failure',reason='missing_matching_saved_support');samples.append(rec);continue
  geo=np.load(gpath);z=np.load(zp);j=max(range(len(z['boxes'])),key=lambda j:iou(grow['box'],z['boxes'][j]),default=None)
  if j is None or iou(grow['box'],z['boxes'][j])<.95:
   rec.update(status='failure',reason='detection_box_unmatched');samples.append(rec);continue
  support=np.load(sp)['supports'][j];gt=getgt(geo_ns,geo['k'].reshape(3,3),geo['depth'].shape)
  if gt is None:
   rec.update(status='failure',reason='no_truth_bracket');samples.append(rec);continue
  tc,depths,visible=gt;matches=[support&v&(np.abs(geo['depth']-d)<.04) for d,v in zip(depths,visible)]
  fractions=[float(v.sum()/max(1,support.sum())) for v in matches]
  if not fractions:
   rec.update(status='failure',reason='no_scene_person_but_algorithm_position');samples.append(rec);continue
  best=int(np.argmax(fractions))
  rec.update(support_pixels=int(support.sum()),actor=actors[best][0],person_model_match_fraction=fractions[best])
  if fractions[best]<.95:
   rec.update(status='failure',reason='retained_support_contamination');samples.append(rec);continue
  surface=np.median(cloud(depths[best],geo['k'],tc)[matches[best]],axis=0);fused=np.array([a[c] for c in ('px','py','pz')])
  rec.update(status='passed_surface_match',surface_truth_xyz=surface.tolist(),published_fused_surface_xyz=fused.tolist(),fused_to_current_surface_error_m=float(np.linalg.norm(fused-surface)),radius_m=a['radius_m'])
  samples.append(rec)
 good=[x for x in samples if x['status']=='passed_surface_match']
 surf=dict(run=p.name,selection='All unique qualified saved source/captures; every retained-support mismatch and missing original is retained. Surface point, not body centre.',
  qualified_publications=len(q),sampled=len(samples),matched=len(good),failures=len(samples)-len(good),
  fused_to_current_surface_error=stats([x['fused_to_current_surface_error_m'] for x in good]),samples=samples,
  reference='Median GT visible actor mesh intersections at retained support pixels, true capture camera pose. Fused temporal surface estimate compared with current support; not body centre, feet, general calibrated uncertainty or independent trials.')
 (O/(p.name+'-dynamic-surface.json')).write_text(json.dumps(surf,indent=2)+'\n')
 # Audit natural short-ID / reappearance stability using all first/last and
 # first-after->.75s-gap samples of each source, plus first confirmations.
 # This is bounded by the recorded batch; it is not full-frame identity proof.
 by_source=collections.defaultdict(dict)
 for ns,report in sem.items():
  for candidate in report.get('candidates',[]):
   if 'id' in candidate:by_source[str(candidate['id'])][ns]=candidate
 temporal=[];source_labels=collections.defaultdict(set)
 for counter,frames in by_source.items():
  stamps=sorted(frames);chosen_ns=set(stamps)
  chosen_ns.update(b for a,b in zip(stamps,stamps[1:]) if b-a>750000000)
  chosen_ns.update(x['capture_ns'] for x in confirmation_rows if x['source_counter']==counter)
  for ns in sorted(chosen_ns):
   candidate=frames.get(ns);zp=p/'perception'/f'{ns}-rgb-masks.npz';rec=dict(counter=counter,capture_ns=ns)
   gt=getgt(ns)
   if candidate is None or not zp.exists() or gt is None:
    rec.update(status='unscorable',reason='missing_capture_or_truth');temporal.append(rec);continue
   z=np.load(zp);j=max(range(len(z['boxes'])),key=lambda j:iou(candidate['box'],z['boxes'][j]),default=None)
   if j is None or iou(candidate['box'],z['boxes'][j])<.95:
    rec.update(status='unscorable',reason='missing_exact_box');temporal.append(rec);continue
   mask=z['masks'][j];tc,depths,visible=gt
   scores=sorted([(float((mask&v).sum()/max(1,mask.sum())),int((mask&v).sum()),name) for (name,_),v in zip(actors,visible)],reverse=True)
   best=scores[0] if scores else (0.,0,None);second=scores[1][0] if len(scores)>1 else 0.
   matched=best[0]>=.5 and best[0]-second>=.1 and best[1]>=20
   actor=best[2] if matched else None
   rec.update(status='matched_true_actor' if matched else 'unmatched_or_ambiguous',actor=actor,overlap=best[0],scores=scores)
   if actor:source_labels[counter].add(actor)
   temporal.append(rec)
 transitions=[dict(counter=key,matched_actors=sorted(value)) for key,value in source_labels.items() if len(value)>1]
 switch_details=[];rotations=[]
 for counter in by_source:
  seq=sorted([row for row in temporal if row['counter']==counter and row.get('actor')],key=lambda row:row['capture_ns'])
  for left,right in zip(seq,seq[1:]):
   if left['actor']!=right['actor']:switch_details.append(dict(counter=counter,previous_capture_ns=left['capture_ns'],capture_ns=right['capture_ns'],previous_actor=left['actor'],actor=right['actor']))
  first=None;angles=[]
  for row in seq:
   ns=row['capture_ns'];path=p/'perception'/f'{ns}-rgb-association.npz'
   if not path.exists():path=p/'perception'/f'{ns}-geometry.npz'
   if not path.exists():continue
   rotation=np.load(path)['camera_to_map'][:3,:3]
   if first is None:first=rotation
   angles.append(float(Rotation.from_matrix(first.T@rotation).magnitude()))
  rotations.append(dict(counter=counter,matched_captures=len(seq),max_estimated_capture_rotation_from_first_rad=max(angles,default=0.)))

 for h in hyp:
  observed=set(h['matched_actors'])
  for source in h['source_ids']:observed.update(source_labels.get(source.rsplit('/',1)[-1],set()))
  h['temporal_matched_actors']=sorted(observed);h['known_temporal_wrong_merge_or_source_switch']=len(observed)>1
 confreport.update(total_scene_actors=len(actors),verified_confirmed_actor_names=sorted(unique),
  not_verified_confirmed_actor_names=sorted({a for a,_ in actors}-unique),
  verified_recall_under_rule=len(unique)/len(actors) if actors else None,unscorable_confirmation_events=sum(x['status']!='matched_true_actor' for x in confirmation_rows),
  known_temporal_wrong_merge_or_source_switch=sum(h['known_temporal_wrong_merge_or_source_switch'] for h in hyp))
 (O/(p.name+'-confirmation-truth.json')).write_text(json.dumps(confreport,indent=2)+'\n')
 identity=dict(run=p.name,selection='Every distinct saved semantic source/capture with corresponding original detector mask; full saved inference output coverage, not every sensor frame.',
  rule=confreport['rule'],selected_samples=len(temporal),matched_samples=sum(x['status']=='matched_true_actor' for x in temporal),
  source_switches=transitions,source_switch_details=switch_details,source_rotation_coverage=rotations,confirmed_hypotheses=hyp,samples=temporal,
  limit='All saved processed source/captures are audited. Missing and ambiguous labels remain explicit; unsaved/unprocessed camera frames, other maps, motion and physical flights are outside scope.')
 (O/(p.name+'-temporal-identity.json')).write_text(json.dumps(identity,indent=2)+'\n')
 reports.append(dict(run=p.name,confirmation=confreport,surface=surf,identity=identity))
 print(p.name,'confirm matches',confreport['matched_confirmation_events'],'unique',len(unique),'surface',surf['fused_to_current_surface_error'],'failures',surf['failures'],flush=True)
if panels:
 sheet=Image.new('RGB',(800,280*math.ceil(len(panels)/2)),'#eeeeee')
 for i,im in enumerate(panels):sheet.paste(im,((i%2)*400,(i//2)*280))
 sheet.save(O/((Path(sys.argv[1]).name+'-confirmation-contactsheet.png') if len(sys.argv)>1 else 'confirmation-contactsheet.png'))
previous=json.loads((O/'TRUTH-AUDIT.json').read_text()) if (O/'TRUTH-AUDIT.json').exists() else []
merged=[r for r in previous if r['run'] not in {x['run'] for x in reports}]+reports
(O/'TRUTH-AUDIT.json').write_text(json.dumps(sorted(merged,key=lambda r:r['run']),indent=2)+'\n')

