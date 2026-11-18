# ASPIRE static identity and cancellation continuation

The single prospective run demonstrates confirmed-person deduplication in the fixed one-person engineering scene. Active C2 cancellation terminal acceptance remains unobserved. Original-scene benefit, multi-person calibration, the historical200ms safety problem and independent energy calibration remain open. This is software dynamics simulation only.

## Changes and causal evidence

`71daf0d` adds local registered-surface sequence correspondence to an existing qualified static anchor. It does not lower detector/visual confirmation thresholds, set a new reliable XYZ, or infer range from a bearing. At least3distinct valid depth captures over.4s within2s must agree with one anchor; existing confidence/support and unmatched-cost contracts apply, with uncertainty-overlap admission and no extra surface margin. Competing components, co-observed distinct sources, frame incompatibility and contradictory range prevent or reverse a match. Missing/occluded measurements retain history. Native correspondence remains bounded and the compute slot remains1. Raw local surface evidence is not exported as peer localization authority.

Min03 root audit: source5 had valid individual registered surface samples but failed temporal fusion due surface dispersion/clipping; source10 had only4captures. Frozen pre-inference pose snapshots caused transient missing_capture_pose_bracket at196.024 and197.344, splitting8→9→10→11→12. Source10 never joined confirmed1, so12 confirmation did not invalidate its activity. No transport failure was proved. Actual saved online RGBD/estimated-transform replay (no truth input) reproduces old1–2 and gives1–1 after the new correspondence; source5 links146.172,10 links196.856. Single-frame9/11 remain unresolved.

`6667c6c` subsequently fixes an avoidable callback race found in min04: refresh inputs already delivered while inference ran, once and without waiting, preserve the exact detected RGB object, then apply original capture/registration/pose checks. This can bind missing RGB context and process ready same-capture depth before publishing its registry batch. RGB judgement remains independent when inputs are absent. `f867c64` adds a final guard that clears an old geometry selection if its queue entry was evicted during inference. Data arriving after this one refresh remain asynchronous; no universal elimination of duplicate work is claimed. This follow-up has regression evidence only, not a second live run.

## Single prospective run

Run `gpu-c2-min04-71daf0d` at `/opt/aspire/validation/static-c2-b70a582-20261021/gpu-diagnostic/runs/` uses exactly71daf0db825e8685f93d924d28fc4fd04587aa51,850source files and verified native ELF hashes. Scene, helper scripts, settings and safety/storage budgets equal min03. It must not be relabelled as running the later snapshot fix.

|Measure|Observed result|
|---|---|
|Budget|240.000sim seconds;623.5266main wall seconds|
|CUDA|871detector calls;162candidate captures;0CPU fallback|
|Perception|2838semantic reports,525with candidates;21RGB source records|
|Correct confirmation|8confirmed sources, all matched the same actual person offline; true unique1|
|Online final identity|confirmed interval1–1,0unresolved additional confirmed hypotheses|
|Remaining uncertainty|13components total;12unconfirmed unresolved components retained|
|New correspondence|7new surface-sequence edges;1existing reliable-position edge|
|Production protocol|151identity-patch callbacks,52without a reliable position;0interface errors|
|Scheduling|10previews,8planned static tasks,6submitted;compute active peak1|
|Execution|77native proposals,53activations,52completed observations;0static C2 handoffs|
|Safety|0latched safety faults;33protective brakes;unchanged200ms rejection still observed|
|Cleanup|0unexpected exits,0forced kills,0remaining owned processes|
|Storage|about967.8MB raw run;I disk about70.08GB after final tests|

The8confirmed sources are1,4,5,7,12,14,18,20. Their actual-mask/person-depth overlap fractions are.6866,.7105,.7284,.8692,.7826,.8744,.8795,.8935. This evaluates one scene actor, not false-positive/general multi-person performance. Online policy never receives that truth.

Three submitted proposals(41,73,75) were rejected as `static_task_cancelled_or_invalid`. Two planned tasks(70,72) were invalidated before publication. Proposal37 failed predecessor safety revision matching;44and69 failed the original200ms handoff safety admission budget. No static task was activated. No executor active cancellation terminal acknowledgement was recorded. The gateway can directly revoke before start without an executor acknowledgement, but raw cancellation/ack wires were not recorded; do not fabricate exact terminal receipts from the rejection reason.

Known residual race in71daf0d: first navigation qualification sometimes preceded the same-capture registered surface callback by a few milliseconds. For example cue4 preview138.408, alias138.436, published proposal41@138.460 then cancelled/invalidated. Cue5 preview145.528, alias145.548, proposal44@145.548 then safety-delay rejection. Hence the new association resolved final identity but did not prevent every redundant solve. The later snapshot refresh handles inputs already available after inference, without postponing correct confirmation or forcing a cancellation scenario. It is not live-validated here.

## Evidence and supervision limits

The outer disk supervisor crashed around350.5wall seconds because `du -sb` raced atomic replacement of `telemetry-codec-status.tmp`. The flight process continued under its own240sim/2100wall limits. Read-only checks found bounded data/ample space; recovery preserved the original supervisor traceback and stale status before reconciling actual result, closed telemetry and absent flight processes. No run was restarted and no limit was raised. The original child exit code is unknown(null); main result has `error=null`, clean owned-process shutdown and closed lossless telemetry. Do not claim uninterrupted outer supervision. Future supervisor copies must tolerate vanished atomic temporary files.

Primary IMU is complete:60000actual raw samples per accel/gyro channel,59997integrals,bit checks pass. Secondary ULog has59993raw records per channel (7tail omissions each),59993vehicle_imu records (2interior+2tail omissions). ULog overall is not complete. Synthetic energy parameters remain uncalibrated.

## Validation and provenance

-71daf0d: Mac447passed/37skipped; native584passed/7skipped after2min8sbuild. Strict native JSON protocol test passes. Targeted tests cover same-ray/different-range people, neighbour ambiguity, coexistence, missing geometry, stale/duplicate/late evidence, no new position/confirmation, and actual inbox→execution monitor cancellation with exact receipt/inactive feedback ordering.
-6667c6c snapshot refresh: regression red on preceding code(1failed/3passed),green4cases; Mac451passed/37skipped. Final queue-eviction guard makes5cases; latestMac452passed/37skipped. Native full suite at6667c6c:588passed/7skipped273.73s; final f867c64 queue-eviction/identity/cancellation targeted suite:66passed19.66s. Native C++ ELFs remain compiled from71daf0d;851current source files and the unchanged binary hashes were verified. No second dynamics run.
-Frozen original d75c0fa tree untouched; no GitHub push, no third-party upload, no raw evidence deletion or environment copy. No review subagents. Actual compaction count3 at closure.

Local evidence is in `/Users/wuchenjie/Documents/Codex/2026-10-21/task-2/evidence/`: `IDENTITY-V1-DECODED.json`, `IDENTITY-SURFACES-DECODED.json`, `IDENTITY-REPLAY-{BASELINE,PATCHED}.txt`, `IDENTITY-TIMING.json`, `IDENTITY-RUN-BINDING.json`, `MINIMAL-IDENTITY-COMPLETE-{0,1,2}.json`, `MINIMAL-IDENTITY-IMU-RESULT.json`, `IDENTITY-SUPERVISOR-CHECK.json`, `IDENTITY-SUPERVISOR-RECOVERY.json`, plus replay/audit scripts and red/green logs. Remote closed reports use the unique min04 run prefix. `PARENT-STATUS.md` records contemporaneous progress and interruptions.

Final read-only check at 2026-10-21 08:29:33 UTC: all 851 source and 20 ELF hashes verified; GPU log independently confirms 871 calls and 162 nonzero calls; no flight, IMU, audit or test processes; I free 70,084,722,688 bytes. See `evidence/IDENTITY-FINAL-STATE.json`.
