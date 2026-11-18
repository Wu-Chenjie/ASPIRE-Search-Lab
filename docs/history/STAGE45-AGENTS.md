> 2026-10-18 Stage45仅保存IMU积分异常隔离与CPU资源缓解候选；原PX4/RACER/安全门不变，尚未新动态验收。见[方法](docs/IMU-RESOURCE-ISOLATION.md)及[状态](../task-3-evidence/stage-45-tracking-vertical-observation/STATUS.json)。唯一记忆仍共享task-3/PROJECT_MEMORY.md。

<!-- Stage44 2026-10-18: pinned native Sensors.Stop serialization compatibility guard; no search resume before stable minimal proof and regression. Evidence ../task-3-evidence/stage-44-gazebo-shutdown-stack/ -->
<!-- Stage43 2026-10-18: teardown-only owned client descendant fix; flight code identical to frozenStage42. Evidence ../task-3-evidence/stage-43-owned-client-descendants/ -->
<!-- Stage42 2026-10-18: new common safety runtime/cleanup engineering source; priorStage41 frozen. Actual state in ../task-3-evidence/stage-42-safety-runtime-cost/STATUS.json -->
> 2026-10-18 Stage41：离线证据支持的CPU线程/捕获上下文/有界公开停服修复，当前尚未动态；状态 ../task-3-evidence/stage-41-capture-context-cleanup/STATUS.json。旧Stage40末5假设/2真人及清理强杀未过，唯一记忆仍 ../task-3/PROJECT_MEMORY.md。

> 2026-10-18 Stage39：用户继续完成修复及RACER对照的独立版本；修复已保存反馈复现的PLAN状态旧执行ID卡死。原Stage38两批失败保留；状态 ../task-3-evidence/stage-39-stopped-feedback-state/STATUS.json，唯一记忆 ../task-3/PROJECT_MEMORY.md。原观察/安全期限不变。

> 2026-10-18 Stage38：用户授权的独立表面支持修复与RACER公平对照版本。旧Stage37/旧批次冻结不追加；当前状态见 ../task-3-evidence/stage-38-racer-comparison/STATUS.json，唯一项目记忆仍为 ../task-3/PROJECT_MEMORY.md。

> 2026-10-17 Stage37：仅离线候选竞争修复版本，未授权新仿真。独立旧版D1与历史证据冻结；唯一项目记忆见 ../task-3/PROJECT_MEMORY.md。方法见 docs/ASSOCIATION-ELIGIBILITY.md。

> **Stage35 B1已结束冻结（2026-10-17 UTC）**：仅两人114.8sim秒执行安全故障后止批，三人未跑。386保存源捕获0换人，但4确认假设对应2真人，重复2；未整体验收。源码248/285与412回归不变。见[最终报告](../task-3-evidence/stage-35-motion-aware-identity/validation-b1/RESULTS.md)与[隔离](../task-3-evidence/stage-35-motion-aware-identity/validation-b1/ROOT-CAUSE.md)。不重启或追加B1；下文为历史检查点。

> **2026-10-17 06:01 UTC更新**：最新授权的[B1独立验证](../task-3-evidence/stage-35-motion-aware-identity/validation-b1/)已启动相邻两人/相机旋转，过门才三人；不重跑单人。下文零新动态为此前工程冻结检查点；本批实时结果见[状态](../task-3-evidence/stage-35-motion-aware-identity/validation-b1/STATUS.json)。运行源码248文件不改。

> **Stage35工程版已冻结（2026-10-17 UTC）**：估计RGB旋转补偿、双向唯一对应及异步因果标定修复；宿主285/27、Jazzy412/6、针对性24通过。按恢复指示先报告现有结果，实际新仿真0轮，多人动态/观察失败交还仍未验收；不重跑旧批。见[最终报告](../task-3-evidence/stage-35-motion-aware-identity/RESULTS.md)与[方法](docs/MOTION-AWARE-IDENTITY.md)。下文为历史入口。

> **Stage34独立续作（2026-10-17 UTC）**：Stage33/A批冻结。先完成观测失败有界交还与组件锚点选择的工程修复/回归，再声明新单人先行多人批次。安全/到达几何门不改，未通过前不声称验收。见[阶段证据](../task-3-evidence/stage-34-observation-exit-association/)。

> **Stage33本轮已结束并冻结，整体未通过（2026-10-17 UTC）**：终端参考/观测分离已修复，宿主269/26、Jazzy387/6；实际单人132.2sim秒终端等待超时，双人/三人未启动，未声明B批，不追加。旧hybrid间歇空包唯一根因未定，单人仍2确认假设。见[最终报告](../task-3-evidence/stage-33-terminal-bearing-multiperson/RESULTS.md)、[方法](docs/TERMINAL-BEARING-MULTIPERSON.md)及[唯一记忆](../task-3/PROJECT_MEMORY.md)。下文为冻结历史。

> **Stage32本轮已结束，整体未通过（2026-10-17 UTC）**：恢复/表面空间对应和有界启动修复已落盘；宿主266/25、Jazzy374/6并保留1项旧hybrid RGB-only快钟失败。实际单轮127.012sim秒触发tracking_step_headroom: Unsafe terminal hold，余两轮未启动，不追加。单人4确认源最终1假设不是多人泛化；曾有定位、期末过期与历史锚点分开。见[最终报告](../task-3-evidence/stage-32-recovery-startup-association/RESULTS.md)、[Stage32方法](docs/RECOVERY-STARTUP-ASSOCIATION.md)及[唯一记忆](../task-3/PROJECT_MEMORY.md)。下文Stage31及更早内容为历史。

> **Stage31本轮已结束，整体未通过（2026-10-16 UTC）**：C++持久源记录/保守对应、独立表面融合与有限引导已接入；宿主259通过/23跳过，Jazzy362通过/6跳过/1项定位恢复时效失败。clear01完成180sim秒但一人仍有2确认假设；two02未起飞，160.416sim秒watchdog，第三轮按停止规则未跑。首轮评测记录器另有SIGABRT core。没有活跃仿真，不再追加。见[最终报告](../task-3-evidence/stage-31-hypotheses-localization/RESULTS.md)、[方法](docs/PERSISTENT-RECORDS.md)及唯一[项目记忆](../task-3/PROJECT_MEMORY.md)。双源ROS不是双机飞行。下文Stage30及更早内容为历史说明。

> 历史[暂停检查点](../task-3-evidence/stage-31-hypotheses-localization/PAUSED-CHECKPOINT.md)和[恢复授权](../task-3-evidence/stage-31-hypotheses-localization/RESUMED-MINIMAL.md)保留；研究线程维护docs/research，本轮未改该目录。

# 本任务边界
- 使用锁定的原生 ROS2 RACER；不引入替代探索规划器。
- 原 Annual-Planner 只读保留，源文件哈希见 docs/source-manifest.json。
- 只保留任务源码、必要模型与许可证、测试及简洁文档。
- 构建、日志、图像与运行记录写入 /tmp 或用户指定的代码树外目录。
- 规划及检测不得读取人体或障碍真值；真值仅供场景生成和独立评测。
- 静态相机、回放、理想运动不能称作自主飞行。未知空间不可当作自由空间。
- 不 push、上传、发布或运行真实无人机。测试用本机已有 Jazzy/Harmonic 容器。
- 修改后执行 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests；ROS 接口另做容器原生验证。

## Stage31 持久假设、定位与有界失败回退（2026-10-16）

- 本目录按后续用户授权开发；stage30及三轮证据冻结。原生RACER和物理安全门保留。显式registry模式，旧decoupled/hybrid保留。
- 协议与先验核查见[stage31证据](../task-3-evidence/stage-31-hypotheses-localization/)。预置最多三轮各180仿真秒，本轮已在第二轮watchdog后停止，第三轮未跑，不追加；禁止无限矩阵与结果后调参。可恢复实现bug须有根因与回归，不改跟踪/安全包络强过。
- 唯一中央记忆在[PROJECT_MEMORY](../task-3/PROJECT_MEMORY.md)。独立aster-search仓库是stage30快照，本轮不改不推送，记录后续同步需求。
- 仅单机静止人体，不训练ReID；歧义关联不强并、不靠真值人数凑数；定位独立，表面点不冒称中心误差。失败记忆与存在历史分开。

## Stage32实际停止边界（2026-10-17）

- 实际仅新1轮127.012sim秒，执行安全故障后关闭余批；后续先读唯一中央记忆和本轮报告，不重启或补齐未用轮数。
- 运行代码245文件已冻结，安全阈值/动力学不改强过；本轮停止后仅做离线审计/文档收尾。SITL只有1架；ROS双源不是双机。

- 2026-10-17最新授权的[本Stage36源码](./)仅离线诊断/有界取证与条件人数范围，本轮0新仿真；见[结果](../task-3-evidence/stage-36-trigger-evidence-identity-bounds/RESULTS.md)。B1原件中299帧射线已恢复目标格历史，旧“114.712输入缺失”缩小为原始像素/精确回调等缺口；未放松安全门，未硬合并无XYZ源。唯一项目记忆入口不变，后续动态仅建议、不得自动重启B1。

- 本副本唯一记忆读取[共享项目记忆](../task-3/PROJECT_MEMORY.md)，完整规划见[共享研究规划](../task-3/docs/RESEARCH-PLAN.md)，不在本副本另建冲突记忆。

- 本D1唯一双人取证已结束冻结，148.7sim秒原840wall看门狗止批，未追加。最终结果见[报告](../task-3-evidence/stage-36-trigger-evidence-identity-bounds/dynamic-d1/RESULTS.md)，唯一记忆在[共享入口](../task-3/PROJECT_MEMORY.md)，本副本不另建冲突记忆。
