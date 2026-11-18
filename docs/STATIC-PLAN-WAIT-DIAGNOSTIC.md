# 静态事件等待状态修复 — 2026-10-21

工程场景 `gpu-c2-min02-ec3af05` 完成 240 仿真秒/507.03 墙钟秒：871 CUDA 检测中118次有候选；两次视觉确认经独立深度/模型投影核对均为同一人物，正确确认去重数1；静态提案和C2换轨均0。无安全故障、强杀或残留。此结果不代表原场景搜索收益。

原生日志在约136.8至145.668仿真秒保持PLAN_TRAJ，旧的已接受轨迹39仍由生产执行器执行。导航支持在141.640满足，视觉确认在142.168发生。旧taskStaticEventTick仅接受EXEC_TRAJ/IDLE，故在此等待区间不能领取事件；确认随后应当撤销主动引导。日志未记录内部队列全过程，不将缺失队列日志当作队列从未入队。

最小修复允许PLAN_TRAJ中“stop-and-plan开启、有有效执行轨迹及ID、执行未完成”的状态评估静态事件。保留全局单任务、竞争任务、反馈/里程计时效、预算、确认失效、C2发布与独立执行安全门控；不允许PUB_TRAJ或已停止规划覆盖。

新增实际C++方法回归在旧代码失败（2.41s），修复后Mac全套445 passed/36 skipped（28.84s）。这验证状态编排，不是在线C2换轨通过。下一步增量原生构建、受控同场景动力学验证。

证据位于 `/Users/wuchenjie/Documents/Codex/2026-10-21/task-2/evidence/`：FSM-WINDOW-DECODED.json、EVENT-CAUSAL-DECODED.json、MINIMAL-COMPLETE-0/1/2.json，以及plan-wait-red-test.log、plan-wait-full-tests.log。原场景871帧的完整零候选诊断另见同任务ZERO-CANDIDATE-DIAGNOSIS.md。
