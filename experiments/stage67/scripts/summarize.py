"""Post-run aggregation only. No new thresholds, inference, simulator, or source edits."""
from pathlib import Path
import ast,csv,datetime,json,statistics
E=Path(__file__).resolve().parents[1]
batch=json.loads((E/'BATCH.json').read_text());state=json.loads((E/'STATUS.json').read_text())
queue=[('G01','C1','B')]+[(p['id']+'-'+arm,p['condition'],arm) for p in batch['pairs'] for arm in p['order']]
hard=[]
for node in ast.walk(ast.parse((E/'scripts/audit_one.py').read_text())):
 if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='hard_keys' for t in node.targets):hard=ast.literal_eval(node.value)
assert hard
rows=[];details={}
for sequence,(tag,condition,arm) in enumerate(queue):
 run=batch['run_bindings'][tag];f=E/'audits'/(run+'-fair-metrics.json')
 if not f.exists():continue
 m=json.loads(f.read_text());details[run]=m;sfile=E/'supplemental-audits'/(run+'-metrics.json');s=json.loads(sfile.read_text()) if sfile.exists() else {}
 technical=all(m.get('checks',{}).values()) if tag=='G01' else all(m.get('checks',{}).get(k,False) for k in hard)
 changed=s.get('native_changed',[])
 af=Path('/Volumes/ColimaRuntime/runtime/research-stage63/runs')/run/'audit.json' if tag=='G01' else Path('/Volumes/ColimaRuntime/runtime/'+('research-stage66' if run.startswith('racer-stage66-') else 'research-stage67')+'/runs')/run/'audit.json'
 ia=json.loads(af.read_text()) if af.exists() else {}
 ul=ia.get('actual_ULog',{})
 prefix_would_pass=True if tag=='G01' else bool(ul) and all(x.get('metadata',{}).get('valid',False) and not x.get('missing_interior_times',[]) for x in ul.values())
 orig=E.parent/'stage63/audits'/(run+'-fair-metrics.json')
 original_class=json.loads(orig.read_text()).get('strict_gate_pass') if orig.exists() else None
 row=dict(sequence=sequence,run=run,kind='historical_control_gate' if tag=='G01' else 'paired',condition=condition,arm=arm,
  strict_gate_pass=m.get('strict_gate_pass'),technical_checks_passed=technical,check_scope='historical original27, measurement version differs' if tag=='G01' else 'Stage67 prospective core plus typed recording contract',execution_evidence_class=(m.get('capture_contract') or {}).get('execution_evidence_class'),actual_native_activations=(m.get('capture_contract') or {}).get('actual_native_activations'),legacy_startup_post_counts=m.get('legacy_startup_2mapper_4executor_counts'),original_stage63_actual_strict_gate_pass=original_class,original_ULog_prefix_rule_would_pass=prefix_would_pass,original_prefix_technical_sensitivity=technical and prefix_would_pass,ULog_missing_interior_records=sum(len(x.get('missing_interior_times',[])) for x in ul.values()),ULog_missing_tail_records=sum(len(x.get('missing_tail_times',x.get('missing_actual_public_tail',[]))) for x in ul.values()),error=m.get('error'),sim_s=m.get('sim_s'),wall_s=m.get('wall_s'),true_actors=m.get('true_actor_count'),verified_unique_confirmed=m.get('verified_unique_confirmed'),recall=m.get('verified_recall'),
  final_algorithm_hypotheses=m.get('algorithm_confirmed_final'),unknown_or_false_confirmation_events=m.get('unknown_confirmation_events'),final_duplicate_excess=(m.get('final') or {}).get('duplicate_excess'),peak_duplicate_excess=m.get('peak_duplicate_excess'),duplicate_snapshots=m.get('snapshots_with_duplicate_excess'),duplicate_observed_processing_interval_s=m.get('duplicate_duration_s'),known_source_switches=m.get('known_source_switches'),
  qualified_surface_points=m.get('surface_qualified'),surface_failures=m.get('surface_failure'),surface_median_m=m.get('surface_error_m',{}).get('median_m'),surface_p95_m=m.get('surface_error_m',{}).get('p95_m'),surface_max_m=m.get('surface_error_m',{}).get('max_m'),
  saved_odom_path_m=m.get('saved_odom_path_m'),odom_gap_intervals=len(m.get('odom_gap_intervals',[])),runner_high_rate_estimated_path_m=m.get('runner_high_rate_path_m'),native_decisions=s.get('native_decisions'),guidance_changed_selections=len(changed),guidance_changes_with_actual_activation=sum(bool(x.get('activation')) for x in changed),guidance_changes_with_completed_observation=sum(bool(x.get('completion')) for x in changed),numeric_discount_pivotal=s.get('numeric_discount_pivotal_count'),baseline_decisions_checked=m.get('baseline_decisions_checked'),baseline_choice_violations=len(m.get('baseline_violations',[])))
 rows.append(row)
paired=[];actor_times=[]
for pair in batch['pairs']:
 ids={arm:batch['run_bindings'][pair['id']+'-'+arm] for arm in ['A','B']}
 present={arm:details.get(run) for arm,run in ids.items()}
 out=dict(pair=pair['id'],condition=pair['condition'],predeclared_order=pair['order'],runs=ids,complete=all(present.values()))
 if out['complete']:
  a,b=present['A'],present['B'];out.update(B_minus_A_verified_people=b['verified_unique_confirmed']-a['verified_unique_confirmed'],A_error=a['error'],B_error=b['error'],A_sim_s=a['sim_s'],B_sim_s=b['sim_s'],B_minus_A_estimated_saved_path_m=b['saved_odom_path_m']-a['saved_odom_path_m'],B_minus_A_final_hypotheses=b['algorithm_confirmed_final']-a['algorithm_confirmed_final'])
  for actor in sorted(set(a['times'])|set(b['times'])):
   for metric in ['first_detected','first_visual_confirmed','first_independently_valid_surface']:
    av=a['times'].get(actor,{}).get(metric,{});bv=b['times'].get(actor,{}).get(metric,{})
    both=bool(av.get('event') and bv.get('event'))
    actor_times.append(dict(pair=pair['id'],condition=pair['condition'],actor=actor,metric=metric,A=av,B=bv,both_observed=both,B_minus_A_processing_sim_s=(bv['processing_sim_s']-av['processing_sim_s']) if both else None))
 paired.append(out)
complete=(state.get('phase')=='all12_uniform_measurement_arms_audited' and len(rows)==13 and all(p['complete'] for p in paired))
report=dict(updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),overall_comparison_complete=complete,phase=state.get('phase'),scored_gate_and_complete_pair_arms=rows,interrupted_attempts=json.loads((E/'INTERRUPTED-ATTEMPTS.json').read_text()),predeclared_pairs=paired,actor_event_times=actor_times,hard_checks_unchanged_from_frozen_auditor=hard,gate_excluded_from_pairs=True,protocol_revision={'kind':'Stage67 monitor-only continuation; all8 completed Stage66 prefix inherited plus4new suffix, interruptedP05A retained; earlier ULog revision remains posthoc','original_Stage63_batch_remains_closed':True,'original_failure_preserved':True,'same_outcome_math_old6_not_in_main':True,'remaining4_monitor_fix_frozen_before_execution':True,'all8_complete_prefix_inherited_regardless_of_performance':True,'Stage66_interrupted_attempt_excluded_from_180sim_pairs_but_retained':True,'original_prefix_sensitivity_reported_per_run':True,'not_original_preregistered_batch_success':True},
  limits=['Single room, three fixed conditions, standing mesh, seed123, two repetitions per condition, single UAV, GNSS-aided SITL','C0/C1 development-used; C1 saturated start; C2 predeclared starting variant, not unseen map','Visible surface point is not human centre; repeated surface captures are not independent persons or independent trials','Transient duplication and ambiguous/unmatched samples retained; no failure-only exclusion','Application UTF8 communication counters do not measure network bytes; coverage/stagnation not newly fabricated','No statistical-significance or general superiority claim; both arms share changed complete engineering stack','First trulyDisarmed/noOffboard4ms source generation permits5wall; original first500ms failures retained; subsequent/query/flight gates unchanged'])
with (E/'RUNS.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else ['run']);w.writeheader();w.writerows(rows)
(E/'COMPARISON.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
md=['# Stage67 有界实际响应取证版本的RACER六对比较'+('最终结果' if complete else '进行中快照'),'','更新：'+report['updated_utc'],'',('原实际控制门有限复用，六对已在明确监督修复续采范围下完成并审计；Stage66中止仍保留。' if complete else '本文件只汇总当前已有结果；原六对尚未全部完成，不是最终结论。'),'','Stage63/64原批保持关闭，旧6个已开始arm及Stage66第九轮中止全作历史附录，不继承性能样本、不挑成功者。本主比较前8arm继承Stage66完整前缀，后4arm为Stage67新ID；实际运行与测量源同一Stage66冻结版本，仅外部只读文件清点修复。Stage66中止轮另列失败，不冒充原预注册完成。原实际G01只有限复用控制/身份门，新增测量启动层不同，不称其为新完整栈门；详见GATE-REUSE.json。它排除配对统计。A为RACER-ROS2＋共同执行/输入修复＋相同感知（引导关），B仅开启bearing_bias。','','| 运行 | 条件/组 | sim秒 | 验证独立人数/真值 | 算法末假设 | 末重复/峰值 | 表面P95 m | 路径m | 技术门 |','| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |']
fmt=lambda x:'NA' if x is None else ('%.3f'%x if isinstance(x,float) else str(x))
for row in rows:
 md.append('| '+row['run']+' | '+row['condition']+'/'+row['arm']+' | '+fmt(row['sim_s'])+' | '+fmt(row['verified_unique_confirmed'])+'/'+fmt(row['true_actors'])+' | '+fmt(row['final_algorithm_hypotheses'])+' | '+fmt(row['final_duplicate_excess'])+'/'+fmt(row['peak_duplicate_excess'])+' | '+fmt(row['surface_p95_m'])+' | '+fmt(row['saved_odom_path_m'])+' | '+('通过' if row['technical_checks_passed'] else '未过')+' |')
md+=['','Stage66第九轮P05-A在30.108sim/125.957057wall因只读监督清点竞态中止，另保留[全部中止指标](INTERRUPTED-ATTEMPTS.json)和原Stage66证据；本范围实际共13次开始，不能只说12次成功。前8个完整连续前缀包含所有软失败并全部继承，后4个新ID实际运行/测量同栈。','','## 预定配对（B−A）','','| 对 | 条件 | 独立人数差 | 估计路径差m | 完成 |','| --- | --- | ---: | ---: | --- |']
for pair in paired:md.append('| '+pair['pair']+' | '+pair['condition']+' | '+fmt(pair.get('B_minus_A_verified_people'))+' | '+fmt(pair.get('B_minus_A_estimated_saved_path_m'))+' | '+('是' if pair['complete'] else '否')+' |')
if complete:
 diffs=[p['B_minus_A_verified_people'] for p in paired];md+=['','六对独立人数：B更多%d对、相同%d对、较少%d对。'%(sum(x>0 for x in diffs),sum(x==0 for x in diffs),sum(x<0 for x in diffs))]
 if not any(diffs):md+=['本批固定预算人数没有显示B优于A；不能把末假设数、不同路径或短组件性能当人数增益。']
 else:md+=['仅按预定条件作描述性先导比较；不外推普遍优势或统计显著。']
md+=['','## 取证规则与历史敏感性','','所有arm运行及typed取证规则相同，前8arm完整继承，后4arm开始前冻结监督读取修复；原Stage66中止保留。有界实际响应握手及同一typed取证合同：真实三角色当前状态、两条真实后续mapper输入；有准入时仍须四条实际执行安全记录，无准入则完整高率日志证明未行使，不把无运动当安全认证。首三拒绝完整上下文有界保存。旧Stage64-P03-A缺新探针记录，仍未过；未伪造旧证据。旧6个已开始arm及Stage66第九轮中止不混入主分析。','','ULog次级子集规则继续使用；原Stage63事后修订和旧判定保持。新数据同时报告旧ULog前缀敏感性及旧2/4计数是否满足；后者只是新记录上的计数比较，非旧控制器反事实重跑或原预注册成功。','','## 时间、实际引导与证据','','[COMPARISON.json](COMPARISON.json)保留每位真演员首次检测/视觉确认/独立有效表面定位的处理与捕获时间、event标志、观察结束/失败信息。未发生保留null，不只平均成功者。CSV另列实际引导改选、激活和完成观察数量；开关开启不等于实际改变规划，也不等于数值折扣单项贡献。','','原始容器数据、外盘PAX与APFS分析视图regular内容逐SHA核验，见各run-stop.json和外层audit/*ARCHIVE-VERIFIED.json。主原件及所有历史失败保留。完整实际输入/RPC字节/ULog交叠/原生DownSampler审计在每run的audit.json。','','## 局限','','单房间三条件/站立资产/同seed123/每条件两重复/单机GNSS辅助；C0/C1已用于开发，C1是启动饱和控制，C2不是未见地图。可见表面不是人体中心，重复捕获不是独立试验。过程暂态重复完整保留，末无重复不等于全程身份完全正确。应用UTF8不是网络总字节，未可靠记录的覆盖/停滞指标不补造。本批不证明多地图、坐卧、多机、Orin、实物或全程安全。','','首次源初始化合同已明确变化：仅真正未解锁/无Offboard且未启动bridge的首4ms允许source生成5wall；旧500ms首次失败不改判。实际消费/后续反馈/飞行安全和外层预算遵循[协议](PROTOCOL.md)与[运行时断言](INITIALIZATION-RUNTIME-ASSERTIONS.md)。']
(E/('RESULTS.md' if complete else 'PROGRESS-SUMMARY.md')).write_text('\n'.join(md)+'\n')
print(json.dumps(dict(complete=complete,scored=len(rows),complete_pairs=sum(p['complete'] for p in paired),phase=state.get('phase'))))
