> 2026-10-18 Stage45仅保存IMU积分异常隔离与CPU资源缓解候选；原PX4/RACER/安全门不变，尚未新动态验收。见[方法](docs/IMU-RESOURCE-ISOLATION.md)及[状态](../task-3-evidence/stage-45-tracking-vertical-observation/STATUS.json)。唯一记忆仍共享task-3/PROJECT_MEMORY.md。

> Stage44当前候选：原生Gazebo停止竞争的版本绑定兼容库，见[方法](docs/GAZEBO-STOP-GUARD.md)及[阶段证据](../task-3-evidence/stage-44-gazebo-shutdown-stack/PROTOCOL.md)。两组共同启用，原安全/退出门保持；实时实际结果见阶段STATUS，不以历史段落当当前验收。

<!-- Stage44 2026-10-18: pinned native Sensors.Stop serialization compatibility guard; no search resume before stable minimal proof and regression. Evidence ../task-3-evidence/stage-44-gazebo-shutdown-stack/ -->
<!-- Stage43 2026-10-18: teardown-only owned client descendant fix; flight code identical to frozenStage42. Evidence ../task-3-evidence/stage-43-owned-client-descendants/ -->

Stage43 当前仅退出阶段修复，接口及边界见 [OWNED-CLIENT-SHUTDOWN.md](docs/OWNED-CLIENT-SHUTDOWN.md)，实际回归和比较见 [Stage43证据](../task-3-evidence/stage-43-owned-client-descendants/STATUS.json)。以下保留旧阶段背景。
<!-- Stage42 2026-10-18: new common safety runtime/cleanup engineering source; priorStage41 frozen. Actual state in ../task-3-evidence/stage-42-safety-runtime-cost/STATUS.json -->

Stage42 当前工程改动及边界见 [SAFETY-RUNTIME-COST.md](docs/SAFETY-RUNTIME-COST.md)，完整验证与实际动态状态见 [阶段证据](../task-3-evidence/stage-42-safety-runtime-cost/README.md)。下文保留原阶段背景，不代表新动态已过。
> 2026-10-18 Stage41：离线证据支持的CPU线程/捕获上下文/有界公开停服修复，当前尚未动态；状态 ../task-3-evidence/stage-41-capture-context-cleanup/STATUS.json。旧Stage40末5假设/2真人及清理强杀未过，唯一记忆仍 ../task-3/PROJECT_MEMORY.md。

> 2026-10-18 Stage39：用户继续完成修复及RACER对照的独立版本；修复已保存反馈复现的PLAN状态旧执行ID卡死。原Stage38两批失败保留；状态 ../task-3-evidence/stage-39-stopped-feedback-state/STATUS.json，唯一记忆 ../task-3/PROJECT_MEMORY.md。原观察/安全期限不变。

> 2026-10-18 Stage38：用户授权的独立表面支持修复与RACER公平对照版本。旧Stage37/旧批次冻结不追加；当前状态见 ../task-3-evidence/stage-38-racer-comparison/STATUS.json，唯一项目记忆仍为 ../task-3/PROJECT_MEMORY.md。

> 2026-10-17 Stage37：仅离线候选竞争修复版本，未授权新仿真。独立旧版D1与历史证据冻结；唯一项目记忆见 ../task-3/PROJECT_MEMORY.md。方法见 docs/ASSOCIATION-ELIGIBILITY.md。

> **D1已结束冻结，整体未过**：唯一148.7sim秒由原840wall看门狗停止；14原始输入重放通过但旧故障未复现，末10确认假设对应2真人。完整结果见[最终报告](../task-3-evidence/stage-36-trigger-evidence-identity-bounds/dynamic-d1/RESULTS.md)，不重试或追加。

> **Stage36 D1独立动态取证（2026-10-17）**：仅一轮双人，实际状态见[动态D1](../task-3-evidence/stage-36-trigger-evidence-identity-bounds/dynamic-d1/STATUS.json)，冻结协议见[PROTOCOL](../task-3-evidence/stage-36-trigger-evidence-identity-bounds/dynamic-d1/PROTOCOL.json)。只新增异步取证健康状态及只读止批监督，C++与Stage36相同，原安全/动力学/确认/对应不变。下文为继承的历史说明。

> **Stage36离线取证续作（2026-10-17 UTC）**：当前仅完成冻结B1的射线/身份诊断、有界旁路取证和条件人数范围；0新仿真，未解决唯一物理根因或独立身份。实际回归/保全见[结果](../task-3-evidence/stage-36-trigger-evidence-identity-bounds/RESULTS.md)，新接口见[方法](docs/TRIGGER-EVIDENCE-IDENTITY-BOUNDS.md)，唯一记忆在[共享入口](../task-3/PROJECT_MEMORY.md)。后续动态未启动，新完整native安装尚未生成；下文均为继承的历史版本说明。

> **Stage35 B1已结束冻结（2026-10-17 UTC）**：仅两人114.8sim秒执行安全故障后止批，三人未跑。386保存源捕获0换人，但4确认假设对应2真人，重复2；未整体验收。源码248/285与412回归不变。见[最终报告](../task-3-evidence/stage-35-motion-aware-identity/validation-b1/RESULTS.md)与[隔离](../task-3-evidence/stage-35-motion-aware-identity/validation-b1/ROOT-CAUSE.md)。不重启或追加B1；下文为历史检查点。

> **2026-10-17 06:01 UTC更新**：最新授权的[B1独立验证](../task-3-evidence/stage-35-motion-aware-identity/validation-b1/)已启动相邻两人/相机旋转，过门才三人；不重跑单人。下文零新动态为此前工程冻结检查点；本批实时结果见[状态](../task-3-evidence/stage-35-motion-aware-identity/validation-b1/STATUS.json)。运行源码248文件不改。

> **Stage35工程版已冻结（2026-10-17 UTC）**：估计RGB旋转补偿、双向唯一对应及异步因果标定修复；宿主285/27、Jazzy412/6、针对性24通过。按恢复指示先报告现有结果，实际新仿真0轮，多人动态/观察失败交还仍未验收；不重跑旧批。见[最终报告](../task-3-evidence/stage-35-motion-aware-identity/RESULTS.md)与[方法](docs/MOTION-AWARE-IDENTITY.md)。下文为历史入口。

> **Stage34独立续作（2026-10-17 UTC）**：Stage33/A批冻结。先完成观测失败有界交还与组件锚点选择的工程修复/回归，再声明新单人先行多人批次。安全/到达几何门不改，未通过前不声称验收。见[阶段证据](../task-3-evidence/stage-34-observation-exit-association/)。

> **Stage33本轮已结束并冻结，整体未通过（2026-10-17 UTC）**：终端参考/观测分离已修复，宿主269/26、Jazzy387/6；实际单人132.2sim秒终端等待超时，双人/三人未启动，未声明B批，不追加。旧hybrid间歇空包唯一根因未定，单人仍2确认假设。见[最终报告](../task-3-evidence/stage-33-terminal-bearing-multiperson/RESULTS.md)、[方法](docs/TERMINAL-BEARING-MULTIPERSON.md)及[唯一记忆](../task-3/PROJECT_MEMORY.md)。下文为冻结历史。

> **Stage32本轮已结束，整体未通过（2026-10-17 UTC）**：恢复/表面空间对应和有界启动修复已落盘；宿主266/25、Jazzy374/6并保留1项旧hybrid RGB-only快钟失败。实际单轮127.012sim秒触发tracking_step_headroom: Unsafe terminal hold，余两轮未启动，不追加。单人4确认源最终1假设不是多人泛化；曾有定位、期末过期与历史锚点分开。见[最终报告](../task-3-evidence/stage-32-recovery-startup-association/RESULTS.md)、[Stage32方法](docs/RECOVERY-STARTUP-ASSOCIATION.md)及[唯一记忆](../task-3/PROJECT_MEMORY.md)。下文Stage31及更早内容为历史。

> **Stage31本轮已结束，整体未通过（2026-10-16 UTC）**：C++持久源记录/保守对应、独立表面融合与有限引导已接入；宿主259通过/23跳过，Jazzy362通过/6跳过/1项定位恢复时效失败。clear01完成180sim秒但一人仍有2确认假设；two02未起飞，160.416sim秒watchdog，第三轮按停止规则未跑。首轮评测记录器另有SIGABRT core。没有活跃仿真，不再追加。见[最终报告](../task-3-evidence/stage-31-hypotheses-localization/RESULTS.md)、[方法](docs/PERSISTENT-RECORDS.md)及唯一[项目记忆](../task-3/PROJECT_MEMORY.md)。双源ROS不是双机飞行。下文Stage30及更早内容为历史说明。

> 历史[暂停检查点](../task-3-evidence/stage-31-hypotheses-localization/PAUSED-CHECKPOINT.md)和[恢复授权](../task-3-evidence/stage-31-hypotheses-localization/RESUMED-MINIMAL.md)保留；研究线程维护docs/research，本轮未改该目录。

# ROS2 RACER 室内人员搜索 · 最小研究工作区

> **Stage30视觉确认与定位分离（2026-10-16）**：新模式`--confirmation-mode decoupled`按连续可信RGB确认本地视觉轨迹，定位状态另记为unknown/unreliable/reliable/expired；仅当前可靠可见表面点进入三维人员标记，无位置者只显示新鲜相机射线。视觉轨迹数不等于独立真实人数。旧hybrid/strict/legacy保留；原生方位引导及安全门不改。字段、状态和计数见[分离接口](docs/PRESENCE-LOCALIZATION.md)；以下继承历史说明不代表已完成跨视角独立身份去重。协议和结果见[本阶段证据](../task-3-evidence/stage-30-presence-localization/RESULTS.md)。

> **历史Stage29 v2观测诊断（2026-10-16）**：本目录只增加控制链旁路记录。v1运行因日志目录遮蔽PX4资源入口而启动失败、0有效秒，原件保留；修正后的v2已实际启动并完成原定唯一180仿真秒，未重现硬故障。仍有10个源参考和411个桥发送时间戳的覆盖缺口，不能宣称已修复或端到端逐包完整；结束本轮，不追加有效仿真。实际结果、v1源码和完整原件见[stage29报告](../task-3-evidence/stage-29-control-observation/RESULTS.md)。以下继承说明以各历史版本报告为准，不代表新增稳定性或搜索收益验收。

> **继承Stage27冻结说明（2026-10-16）**：修复stage26将观察位置限制在人体像素锥内的零效用问题，新增原生有界侧向观察候选；原净空、原轨迹/执行安全及确认门保留。见[本轮方法](docs/BEARING-REPAIR.md)、[实际结果](../task-3-evidence/stage-27-bearing-repair/RESULTS.md)及唯一[项目记忆](../task-3/PROJECT_MEMORY.md)。stage26源码和失败证据保持原样；以下正文保留历史说明。

本副本最新结果见[执行安全修复与原档验证](docs/EXECUTION-SAFETY.md)。严格主动人员搜索仍未通过：原档有人复测后期恢复预算耗尽、零确认，1.8m初始化仍受未知净空阻挡。保留冻结stage20；激进动力学档位未运行。

最新进展见[中文简报](docs/BRIEFING.md)和[主动确认验证](docs/ACTIVE-CONFIRMATION.md)：最终严格主动复查尚未通过，完整失败链和连续视频已保留。

上一阶段结果见[新地图实测](docs/NEW-MAP-TESTS.md)：10 个新场景、29 个楼层候选、5 次实际 300 秒飞行均已完成。两次正例均确认真实人体，但一次多确认背景目标；无人负例 2 帧二维误检、零误确认。所有运行无起飞就绪后接触或 PX4 保护，但预定远端区域门槛均未通过，存在探索停滞与部分遮挡漏检，整体新地图验收未通过。

本机 ROS2 Jazzy / Gazebo Harmonic。保留已迁移 RACER 的探索、A*、ESDF 和 B 样条；RGB-D 人体候选、可靠表面定位、跨视角去重与有限复查均无大模型依赖。演示使用 `consistent`；HOG 保留，YOLO 由显式参数启用。已有 NBg 单场景闭环见[人员搜索记录](docs/YOLO-SEARCH.md)，不代表多场景泛化。

历史证据按阶段保留：[静态与初始闭环](docs/RESULTS.md)、[定位口径](docs/PRECISION.md)、[故障与 HOG 诊断](docs/DIAGNOSIS.md)、[恢复](docs/RECOVERY.md)、[动力学对照](docs/DYNAMICS.md)、[原生失败分支](docs/FLIGHT-BUG-VALIDATION.md)、[心跳与故障注入](docs/FAULT-VALIDATION.md)、[返航状态](docs/RETURN-FIX.md)、[旧地图筛选](docs/NEW-MAP-SCREENING.md)、[下载访问历史](docs/NEW-MAP-ACCESS.md)。GPU、主动视角公平对照和多机扩展尚未完成。

**历史 NBg 单场景曾跑通最小自主探索＋检测闭环，不能代替本轮验收**：原生 RACER → 原授权执行器 → EKF2 估计位姿与实测 RGB-D → 跨视角确认；不是实机验收。原四视角静态测试与理想相机回归继续单独保留。当前使用仿真 GNSS 辅助定位，尚不等于完成无 GNSS 室内定位。

## 目录

- `src/person_search/`：HOG / 显式启用的 YOLO 分割、可靠深度定位、去重、复查策略与 ROS2 感知节点。
- `external/racer_ros2_ws/`：锁定迁移源码和 migration.json；不含 build/install。
- `external/*models/`、`external/rgbd_scene_assets/`：仅必要机体、相机和纹理人体资产。
- `tools/flight/`：单机 SITL 启动、可关闭的主动复查、飞行证据和事后评测。
- `src/annual_planner/src/`：从原工程精简复用的估计桥、实测建图、原生曲线执行和安全模块。
- `tools/search/`：单机 RACER 接口、独立构建、自动理想相机实验及事后评测。
- `tools/rgbd_scene/`、`tools/prepare_sim.py`：保留原四视角测试与 RGB/深度生成链路。
- `tests/`、`config/search.rviz`、`licenses/`：回归、可视化配置、来源与许可记录。
- `docs/source-manifest.json`：来源哈希。原 Annual-Planner 保留全部历史和未提交文件，可恢复。

无源码树内构建、日志、录制数据；没有其他探索规划器、旧实验输出或大型依赖环境。

## 运行

本轮实际动力学安全修复使用[stage23 命令](docs/EXECUTION-SAFETY.md)。下面保留静态四视角与理想相机回归入口，不能用其结果冒充实际飞行验收。

宿主单测（ROS 原生部分在 Mac 跳过）：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests
python3 tools/search/verify_workspace.py
```

复用本机现有容器，先复制到新目录，脚本不会覆盖已存在目录：

```bash
SEARCH_STAGE=/tmp/person-search-demo bash tools/search/local_container.sh
# 进入同一容器；以下均为容器内路径
# docker --context colima-racer-gvp exec -it annual-px4-research bash
cd /tmp/person-search-demo
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR=/tmp/person-search-logs CMAKE_BUILD_PARALLEL_LEVEL=2
bash tools/search/build.sh /tmp/person-search-build
source /tmp/person-search-build/install/setup.bash
python3 tools/rgbd_scene/run.py --output /tmp/person-search-static
python3 tools/search/evaluate_static.py /tmp/person-search-static
python3 tools/search/run.py --native /tmp/person-search-build \
  --output /tmp/person-search-active --strategy active --seconds 45
```

自动测试默认初始光心高度 **2.2 m**，允许升降。复现原四视角对应的低位部分遮挡场景加 `--height 1.05`。严格三策略对照都加 `--no-frontier-recovery`，用 `--strategy racer` 或 `--strategy lateral` 切换；无人负例加 `--negative`，全遮挡初始条件用 `--start hidden`。每个输出目录必须新建；同一固定 ROS domain/display 的入口不要并发运行。

原生完整回归：

```bash
ROS_DOMAIN_ID=196 SEARCH_CAPTURE=/tmp/person-search-static \
  python3 -m pytest -q -p no:cacheprovider tests
```

实际动力学入口和依赖环境见 [飞行命令](tools/flight/README.md)，用 `--active-review` 启用有限复查及等待实测到达后再规划。下列独立节点只做感知：

```bash
PYTHONPATH="src:src/annual_planner/src:$PYTHONPATH" python3 -m person_search.node
# 本机 Jazzy 容器已安装 RViz2；也可使用飞行入口 --rviz：
ROS_DOMAIN_ID=197 rviz2 -d config/search.rviz
```

感知节点要求采集时刻的 `map -> drone_0/depth_optical` TF；缺失、不同步或不配准直接拒绝，不使用最新位姿凑配。默认只发布 `/search/tracks` 与 `/search/targets`；可选复查模式另发 `/search/review_goal`，仍须 RACER 和原安全执行器批准，不直接发飞控命令。相机话题保持 `/drone_0/{rgb,depth}/{image,camera_info}`，320×240；保留 `/tf_static`。

## 判定边界

- HOG 是已有 OpenCV 分类器。细长非平面深度物体只是未确认线索；缺失深度始终未知。非人细长物也可能进入复查，只有不同视点的人体检测支持才确认。
- 定位使用连通表面内部像素的三维中位数，不使用检测框中心；输出是可见表面位置，不是人体中心。假设静止目标、重力对齐地图及已知地面 z=0。
- 复查仅在已观测自由空间筛选点，利用已测表面预测遮挡；所有路径由 RACER 生成，并额外检查曲线采样的已知自由空间与膨胀占据。没有可用线索时走 RACER 探索，不推定板后有人。
- 场景生成器可以设置人体/板；策略模块不读 `viewpoints.json`、网格或人体真值。事后 evaluate 脚本单独读取真值计算指标。
- Visual Jev 尚未接入；基础闭环不依赖它。多机、无 GNSS 室内定位、多房间、多种子性能和实机安全均未验收。持续 2–3 m 巡航偏好仍待加入；默认起航 2.2 m，观测路径允许降低高度。

必要证据持久保存在项目旁 `../task-3-evidence/`。代码树约 64 MB；构建、运行记录和 PX4 大日志全部在代码树外。
