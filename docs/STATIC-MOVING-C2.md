# 静态事件移动 C2：源码与验收边界

2026-10-21 04:40 UTC：补齐实际原生优化器的固定 knot-span 边界约束，启用须显式 min_time=false。真实原生曲线经 DDS/执行器完成一次 C2 切换和取消；完整 FSM 在线事件至物理运动整链仍未验证。最新测试与 GPU 范围见 [原生验证](STATIC-NATIVE-VALIDATION.md)。
2026-10-21：新增[原生构建、回归和 DDS 有限验证](STATIC-NATIVE-VALIDATION.md)。下文的环境/禁止仿真描述为此前交接状态；本轮按最新用户授权分阶段执行，旧批次不重开。

本次续接补齐执行监视/计算槽分离和被替换任务终态回执；不是重新应用 S9 备份。
`static_moving_c2` 默认 false，原生验证前不得启用。沿用原 RACER 规划器、原执行器、
几何/动力学/停止预留和 200ms 安全预算。宿主测试不证明 ROS、DDS 或物理运动闭环。

## 计算与执行分离

一个全局 inbox 仍最多一项计算/待提交事务。生成或发布曲线不释放它；原 feedback
回调先核对时效并应用匹配的 active proposal，`taskStaticExecutionFeedback` 再把
work 快照和精确取消身份移入执行监视，并 finish inbox token。此后旧轨迹仍在
执行，下一实体的新线索即可取得唯一计算槽。无引导/关闭 moving C2 时保留原行为。

监视保留 entity/generation/members，用 inbox.current 检查确认、身份或 frame
变化；失效或已非活动任务限频请求精确取消。发布期间已经发出的取消状态一起移交。
每个请求仍为 session/task/proposal/map/revision，回执不能跨任务结清。旧监视不会
被新活动任务覆盖：正常完成，或精确终态回执与非活动反馈都齐全，才能收集旧监视。
两者顺序均可；仅收到 publish、pending、无关回执或非活动反馈均不代表取消成功。

活动任务提交端点深度观测后，监视和末端参考点继续保留 `static_guidance_budget.observation_s`，
让同一实体的新观测有时间通过检测和登记。重复完成反馈不延长窗口。若此时登记使原复查任务
失效，则发送精确取消，并等执行器终态回执和独立的非活动反馈；否则窗口结束后正常继续规划。
返航和恢复仍优先，实体关联规则与原执行安全门限不变。

监视最多 256 项，与会话取消账本相同；未结清时不淘汰，满额阻止新增引导计算。
墓碑仍保留到会话结束。普通有效续帧不新增计算；同实体实质版本变化仍合并 pending，
拒绝旧版本结果。该容量是保守上界，不是独立实测得出的最佳设置。

## 未来边界、接收和实际切换

1. 原生 PLAN 从活动轨迹的未来 trajectory time 采样 p/v/a、yaw/yaw rate/yaw
   acceleration。显式正 lead 不裁短；窗口不足则撤销未发布事务、保留旧轨迹。
2. 规划返回和发布前复查旧 proposal、map revision、反馈进度和实际规划墙钟耗时。
   只对这条静态事务开放 stop-and-plan 的移动例外，默认停止门、返航/恢复优先级仍在。
3. 授权代理绑定旧 executor token/epoch、实际曲线 SHA 和未消费边界；旧安全修订
   不可沿用旧原生样条。保留原 ledger、路径、地图、动力学及停止预留检查。
4. 执行器接收时重新检查实际活动 token/epoch、曲线指纹、里程计、地图、连续性、
   动力学、已知自由走廊和停止预留，保存 admission receipt。不存在曲线、旧会话、
   已取消或已占用 pending 均拒绝。
5. 实际 tick 穿越边界前再次检查：旧边界尚未消费，真实步长不超过原 .04 秒，
   新旧六项边界连续性、新曲线 SHA、实际新参考点跟踪管及原安全条件。
   当前旧曲线安全检查与新曲线切换检查共用计时起点；证据写入也在原 200ms 内。
6. 通过后才激活新任务，不授予旧任务观察/发现信用。普通过期/版本/几何拒绝继续
   原安全轨迹；安全预算超限沿原 safety fault 保守处理，不宣称旧轨迹仍安全。

此处的交接基于执行器实际 trajectory time 和 reference step，不是只更改消息时间戳。
原生 map revision 每次导出递增可能导致保守重算/饥饿，仍须真实频率与负载验证。

## 替换旧任务的终态回执

实际切换成功后旧 ledger 记录 `superseded_by_c2`，保存 transition 证明及 successor
executor token；不会伪装为原地静止或完成观察。若取消请求尚未到达，revision 保持 0，
不充当取消确认。晚到 revision 1 请求返回同一替换证明，重复请求保持幂等且不复活旧任务。

代理只接受新鲜 ready 且无 safety/session fault 的执行报告；核对原 task/session/map/
executor token、transition 证明以及自己实际发送过的 successor→predecessor 命令关系。
后续任务已再次切换时，历史终态仍可返回，不能错误要求历史 successor 仍为当前活动 token。
错 successor、旧会话、错版本、陈旧报告、缺证明均不结清。原生线上的终态状态 4 仅表示
经验证 C2 替换；仍需旧 proposal 非活动的独立反馈，且不生成观察完成信用。

## 配置与运行

两组共用 `static_cancel_enabled`、`static_moving_c2` 和 `static_c2_lead_s`；只有
bearing_bias/static_event_replan 是引导组差异。每运行仍生成新 session。开启 moving
要求单机、registry、原 execution_safety、stop_and_plan=true、取消协议以及显式正 lead。
模板 moving=false / lead=null；null 不提供部署值，开启时必须从独立验证填写。
不能修改参数后继续使用旧 native 二进制冒充本次实现。

当前仅可执行宿主检查，不启动仿真：

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -ra -p no:cacheprovider tests/test_guarded_handoff.py tests/test_static_cancellation.py tests/test_static_replan.py tests/test_static_pair.py
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -ra -p no:cacheprovider tests
```

具备已批准的现成 ROS/Boost 环境后，按 STATIC-REPLAN-ACCEPTANCE.md 在全新代码树外
目录生成/编译；再验证真实接收→实际边界→接受反馈→计算槽释放→新线索→替换终态，
包括反馈/回执乱序、丢失、连续两次换轨、确认后取消、会话容量满、原预算和失败回退。
源码/配置/依赖/时序证据必须关联同一提交。此步骤当前未执行，不放行 GPU 批次。

## 三类剩余工作

- **源码尚缺、此次不扩展**：主动提前刹停曲线、跨进程崩溃恢复；纯几何/动态预算
  事件唤醒和完整人体可见性仍为后续工作。本次 C2 计算/执行分离及终态接线已完成。
- **源码完成但原生未验证**：当前 C2 与取消全链、完整 ROS/Boost 构建、DDS 丢失/乱序、
  真实线程/磁盘时序、长期容量与地图版本饥饿、实际运动。缺环境不替代这些验收。
- **待独立数据校准**：lead、事件间隔、弱动作/预算/可见性参数及误报上限。
  false_positive_cap 仍 null；没有正式搜索收益结论。

旧 GPU 200ms 故障、当前 I 盘余量和连接仍未重新验证；保持不 SSH、不安装、不上传/
推送、不仿真。独立审查子代理没有授权，作者复核不称独立审查。
