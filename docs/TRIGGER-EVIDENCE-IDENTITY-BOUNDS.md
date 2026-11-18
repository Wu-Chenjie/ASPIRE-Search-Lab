# Stage36：有界触发取证与身份计数范围

2026-10-17 UTC。仅在独立Stage36修改；[Stage35/B1](../../task-3-evidence/stage-35-motion-aware-identity/validation-b1/RESULTS.md)和原证据冻结。唯一持久记忆为[共享入口](../../task-3/PROJECT_MEMORY.md)，实际结论见[Stage36报告](../../task-3-evidence/stage-36-trigger-evidence-identity-bounds/RESULTS.md)。本轮0新仿真。

## 开关与数据契约

`RACER_MAP_EVIDENCE_DIR`默认不设，旁路关闭；启用时须代码树外新目录。同批mapper/executor共享`RACER_OBSERVATION_SESSION`。每role异步写独立子目录，拒绝已存在同名目录。诊断String话题`/drone_ID/diagnostics/map_evidence_trigger`含schema/kind/id/drone/session/sim_s/reason/details，schema=1；同ID去重，错机/session丢弃。它不接入规划或安全判据。

mapper在原project_depth成功之后、原integration之前保存原始depth编码/尺寸/step/大小端/字节、CameraInfo K/时间/坐标、消费的估计位姿/插值端点、接收回调序号/仿真及单调时刻/原队列深度、消费时刻、前图/belief/ego；之后保存实际射线/hits/origin、后图/belief、原发布packet的测量与全量/增量字段。接收元数据最多8项，和原8项pending同量级，不影响原入队/插值。未走接收回调的组件重放receipt明确null；缺端点明确false，不编造。

executor仍调用原ExecutionCorridor；旁路统计查询数、最多8个拒绝AABB、完整mapper/native状态及时间/版本、原查询曲线/进度、最终曲线、实际/参考/速度/加速度/航向、故障。安全检查中第一次产生的故障触发一次本地和同机mapper取证；后续最多4个回调状态继续保存但不重新执行已锁存的安全策略，也不重复触发。后续状态不等于重新作过安全查询，query_count可能为0。

前窗最多4sim秒、mapper12/executor48条；后窗2/4条，最多2触发。数组payload环形8MiB、待写8MiB、单条2MiB，NPZ磁盘各32MiB，满则计数丢弃而不阻塞控制。异步压缩、非阻塞入队；数组分离拷贝，不能被后续状态污染。关闭最多排队2秒/等待5秒，写status记录所有窗口、pending/writeralive、drop/error、内存payload峰值、record与hook最大墙钟开销。关闭无触发时写明确的`shutdown_tail_no_trigger`；后窗不足如实记录。status紧凑JSON，异常隔离，旁路不吞掉原策略异常。

环形/队列限额不等于进程RSS或实际运行时限已获保障。hook_max计量旁路同步部分，policy_check_wall_s保留原安全调用耗时；完整tick和Orin必须另测。原.2秒预算的策略实现/安全阈值未改。未来监督器须据错误/容量/开销停止诊断；本轮未新增自动飞行协调器。

## 身份范围接口

C++`targetCountBounds`只读已有组件和cannot-link，不新增关联。`identity_count`含visual_confirmed_hypotheses、distinct_lower_bound/upper_bound、conditional_distinct_count、unresolved_additional_hypotheses、status、见证ID及成对预算。下界为最多16384对检查的贪心排斥团见证，上界为确认组件数；矛盾输出不一致。所有“独立”结论以视觉、alias、排斥有效为前提，未见人员不受约束，`independent_true_people`永远null。2–4表示2个有区别证据的组及最多2个身份待定附加假设，不能标成已找到4真人或已证实仅2人。

无可靠XYZ源仍保留视觉确认和新鲜方位。方向接近、已知场景人数、真值都不能自动合并。B1新旧561批配对重放全关联状态保持，期末4假设/条件2–4，见[实消息审计](../../task-3-evidence/stage-36-trigger-evidence-identity-bounds/audits/ACTUAL-B1-COUNT-REPLAY.json)。

## 验证与部署边界

回归及原始日志见[最终报告](../../task-3-evidence/stage-36-trigger-evidence-identity-bounds/RESULTS.md)。组件重放保存RGB-D不是新自主飞行。完整Jazzy环境沿用历史Stage34 native安装；当前新C++注册器独立编译/真实ROS接口通过，但新完整RACER安装仍须未来动态前生成核验。建图、安全策略、视觉门和关联不变，未实现历史占用保留，也没有解决旧hybrid空包或补出动态handback覆盖。[后续最小条件与预算](../../task-3-evidence/stage-36-trigger-evidence-identity-bounds/NEXT-DYNAMIC.md)仅建议，未启动。
