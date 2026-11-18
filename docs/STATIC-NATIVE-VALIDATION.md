# 静态搜索原生验证 · 2026-10-21

最新阶段截至 06:08 UTC：617dfc0 完整原生 **580 passed / 7 skipped**；ec3af05 完整Mac **445 passed / 36 skipped**，另有WSL配置回归6项通过。
下文先保留早期阶段记录，最新 GPU 与移动边界修复见文末。

本轮用户已授权在现有 Windows/WSL 仿真机分阶段构建和仿真。原生验证基于
b70a582，随后只修正两处过时测试夹具；功能默认关闭，原 200ms 安全预算不变。
旧 GPU 批次与原始 d75c0fa 基线保持冻结，不把旧实验重新打开。

## 已执行

- `hit-win-lan` 实际连接到原服务器，现有主机密钥正常校验。
- 843 个源文件哈希一致；原生 `racer_interfaces`、`racer_core` 编译成功，
  209.516 秒。331 个相同 external 文件复用，不复制整个环境。
- 第一轮原生全量：575 passed / 7 skipped / 4 failed，246.41 秒。
  三个旧 SimpleNamespace 夹具缺取消桥方法；已改用真实默认关闭的取消桥。
  原生 registry 探针把已确认目标当成仍可主动引导，已改为候选目标触发，
  并新增真实 ROS 确认后停止引导、保留身份记录检查。未放宽生产条件。
- 相关 54 项复测通过；第二轮原生全量 **579 passed / 7 skipped / 0 failed**，
  245.95 秒。Mac 全量 **444 passed / 35 skipped**，32.05 秒。
- 真实 rclpy DDS 与生产 ViewExecutor / StaticCancelGateway 探针通过，
  8.229 秒内实际定时回调连续两次 C2 切换；验证历史任务晚到取消、待切换取消、
  重复命令和回执、旧会话拒绝、活动任务原认证静止退役且不授予观察信用。

DDS 探针使用明确标记的合成全空地图和跟随参考的合成里程计，没有 PX4/Gazebo，
也没有原生优化器在线生成这些后继曲线。因此不能据此宣称完整原生规划→授权→
执行闭环、真实飞行、200ms故障修复或搜索收益。完整事件负载、规划槽峰值、
地图版本饥饿和原生规划曲线的实际换轨仍须分别验证。

## 原生跳过项

共 7 项：指定的 73.756 秒重复 RGB-D 记录 1 项；显式生产 Gazebo 守卫产物
1 项；静态 HM3D 捕获 1 项；指定原生 RGB-D 记录 2 项；锁定模型集成 2 项。
这些不是通过。Mac 35 项另含 ROS、Boost 和 Linux 运行时限制，见完整宿主日志。

## 证据位置与运行边界

本地证据：`/Users/wuchenjie/Documents/Codex/2026-10-21/task/evidence/native-gpu-connected-20261021/`。
关键记录：`BUILD-PROGRESS-2.json`、`TEST-PROGRESS-7.json`（首轮失败）、
`TEST-FIX-PROGRESS-3.json`、`STAGE-PROGRESS-1.json`（完整复测）、
`DDS-RESULT-1.json`、`probe-c2-dds.py`、`HOST-REGRESSION-2.log`。

远端独立根：`/opt/aspire/validation/static-c2-b70a582-20261021`。
原生测试先 source 已有 runtime setup，再 source 本次 native/install/local_setup.bash，
将本次 source、source/src、source/src/annual_planner/src 前置 PYTHONPATH；
运行 `python3 -B -m pytest -q -ra -p no:cacheprovider tests`，使用独立临时目录和 ROS 域。

03:54 UTC I 盘可用 77,190,922,240 B，本次目录 216,142,458 B。继续限制日志，
保留失败证据，不删除旧实验。下一阶段是有界分段计时与最小 GPU 诊断；当前此文
不声明 GPU 已运行。旧运行栈还有独立的命令参考、时钟推进与测量层，不能仅替换
二进制便宣称与旧实验等价。误报上限和部署参数仍待独立数据校准。


## 04:40 UTC 阶段结果与新增修复

真实优化器检查暴露了此前未完成的边界生成约束，不是环境问题：旧 soft START/END
成本成功返回后，移动新轨迹的速度、加速度、yaw rate 边界误差分别约
0.00420、0.01875、0.08336，原 1e-5 连续性门正确拒绝。因此早期“C2 源码完成”
仅覆盖事务与执行接线，不能外推为实际优化器可生成可接受曲线。

新增 `static_c2_boundary_overlay.py`，仅在显式 `static_moving_c2=true` 时使用。
固定 cubic knot span 下，将请求的 p/v/a 与 yaw 的两阶导编码为首尾控制点等式约束，
内点继续使用原 RACER 成本；kinodynamic LS 拟合后恢复实际请求的起始 jet。
变量时间优化或边界控制点超出原地图界限直接拒绝，不裁剪或放松连续性。
配置现要求显式 `min_time=false`；原几何、动力学、停止预留、200ms 预算不变。
默认关闭路径不变，两组共用该设置。

- 增量原生构建成功：134.600 秒；原 Boost deprecation 警告保留。
- 新原生非零移动/偏航边界回归通过：1 passed / 6.66 秒；最大连续性误差
  1.031e-15，后续原几何与动力学检查通过。
- 修复后完整原生：**580 passed / 7 skipped / 0 failed**，253.43 秒。
- 修复后完整 Mac：**444 passed / 36 skipped**，31.93 秒；比此前增加的一个
  跳过是本机无 ROS 的真实优化器测试，该项已在 WSL 通过。
- 原生生成曲线→真实 DDS→生产执行器/取消桥：10.402 秒，通过实际定时 C2 切换、
  旧任务晚到取消的 superseded 回执、旧会话拒绝、后继按原认证静止退役且无观察信用。
  此探针的地图和跟随里程计仍是合成输入，曲线是实际原生优化器输出。

完整 native FSM 在线事件→授权代理→实际活动反馈释放计算槽→新线索再规划→
物理执行仍未完成整链验收；当前分开的原生组件、DDS 与合同测试不能相加冒充它。
地图版本饥饿、真实负载下连续多次 C2、长期容量仍待验证，功能默认关闭。

## 本轮唯一 GPU 闭环

编号 `gpu-g01-current-1d55803`，基于 1d55803（包括 b70a582 功能代码），另有
明确保留的原共同静态命令参考/kino 修复层和测量层。它在上述新边界修复之前运行，
**没有开启 static event/C2，也不验证修复后的移动 C2 物理飞行**。

L1-v3 / seed 123 / bearing_bias=true / registry / 原安全门，240.000 仿真秒，
579.401 墙钟秒，估计路径 27.050 m，50 个执行 token；无安全故障或异常退出，
22 个自有进程清理完毕，无强杀或残留。871 次实际图像推理全部 CUDA，平均
16.899ms、最大 50.031ms；没有 CPU/GPU 配对，不能据此宣称端到端加速。

11666 次安全回调，完整回调最大 wall 50.850ms。遥测捕获的 618 个独立停止
预留检查最大 45.157ms，不当作全部回调最大。新增测量层 marker 最大 wall
6.741ms、wrapper 最大 wall 5.337ms（包含抢占等待），不宣称零开销。
旧 200ms 故障本次未复现，**尚未解决**；无新故障窗口可用于归因。
旧记录最大 full-callback wall 268.853ms / thread CPU 229.337ms 是整回调最大值，
不能直接当作故障比较点或某个阶段耗时。未提高阈值，未恢复旧批次或启动新配对。

完整原始 IMU 审计通过：60000 源 IMU、两个 raw topic 各 60000，59997 公开积分，
29999 原 DownSampler 输出；无 bit mismatch、clips 或内部缺口。ULog raw 尾4个、
积分尾1个缺样，按原次级有序子集规则记录，未冒称完整 ULog。

原始运行约 1.69GB，未另造大归档；04:24 I 盘余量 75.379GB（后续构建/审计有增量，
最终快照见本地交接）。旧基线、旧批次与唯一失败证据保留。误报上限、lead、弱动作
预算和可见性仍待独立数据校准，尚无发现人数收益结论。

关键本地证据：`GPU-SINGLE-AUDIT.json`、`IMU-AUDIT-RESULT.json`、
`MOVING-COMPONENT-RESULT-2.json`（修复前真实拒绝）、`BOUNDARY-TARGETED-RESULT.json`、
`NATIVE-DDS-LAUNCH-2.json`（完整回归输出）、`NATIVE-DDS-RESULT.json`、
`HOST-BOUNDARY-REGRESSION.log`。均位于本文前述本地 evidence 根。

## 复核入口

源代码外生成原生树：`python3 -B tools/search/build_native.py <new-output> --execution-safety --consistent-inflation`。
在既有 Jazzy 环境以该树编译 `racer_interfaces racer_core`；source 其 install 后将本次
源码路径前置 PYTHONPATH，执行完整 `python3 -B -m pytest -q -ra -p no:cacheprovider tests`。
新增定向项为 `tests/test_native_moving_boundary.py`。编译源码不自动重启仿真。
GPU 需另行复用已冻结共同运行层及新 ID，不能直接运行旧批次脚本；当前不放行配对。

04:45 UTC 已核对无活跃本轮构建/测试/仿真进程；I 盘可用 75.204GB，独立根约 2.109GB。
另外从当前生成器全新生成源码并应用 `common-after-boundary-2.patch`，296 个实际
构建源文件全部哈希一致（排除 patch 的 `.orig/.rej` 备份）。该补丁仅含此前明示的
7 个共同静态命令参考/kino 运行层文件，SHA256 为
`c1b5901a627c53d961241ec821e8b5a3e2a17d9e16c52881ed6e591c7c5f2525`。
远端 `BOUNDARY-REPRODUCTION.json` 保存逐文件清单。首次核验因 patch 自动备份
`.orig` 不一致而失败，记录已保留；实际构建文件未变，未重复编译或运行仿真。

## 04:53 UTC 在线运行入口检查

真实 prepared runtime 使用 `fleet_starts` 和 `racer_min_time`，静态配置准备器此前仅
识别测试中的 `search_starts`/`starts` 和 `min_time`，会在真实启动前误拒绝。
现在按实际 NativeLeaseAgent 字段验证并规范化，冲突别名直接拒绝，不能通过另一字段
掩盖真实时间优化或多机配置。定向6项通过；完整Mac **445 passed / 36 skipped / 29.62秒**。
这是Python配置修复，不改变已构建原生二进制。后续若用合同合成能量包络开展隔离
物理接线诊断，须明确其非校准输入，不得称能量安全或正式完整验收通过。


## 06:08 UTC：C2 开启动力学仿真与完整后审计

用户新增明确授权后，唯一 `gpu-c2-g02-ec3af05` 已于05:03启动并完成，未重复运行。
源码ec3af05（原生边界修复617dfc0），原生二进制另含此前明示共同静态参考/kino层。
实际runtime、原生YAML/protocol均证实C2、事件重规划、取消开启，固定时间优化设置正确。
没有注入目标事件或预制换轨曲线。L1-v3场景、seed123、原检测器及registry确认规则保持。

240.000sim秒 /577.635wall秒，估计路径27.716m；47个实际执行token，49次提交、
47次激活、46次终态观察完成。最后native_plan_id57是ID，不等于成功执行数。
无安全故障、异常退出，22个自有进程全部清理，无强杀或残留。原始运行约1.70GB。

**实际换轨为0，完整C2动力学仿真验收尚未通过。直接原因是本轮没有检测候选：**

- 871次实际CUDA检测全部输出0候选，无CPU回退；2859条语义报告的候选、视觉轨迹、
  已确认轨迹、同帧重复抑制计数均为0。不能据此证明确认/身份去重功能通过。
- 57份原生提案全部没有static_task；STATIC_EVENT日志为空，无handoff授权或消耗事件。
  本次没有进入线索触发的移动C2提案链路，不能据此判断该链路成功或失败。
- 场景XML及独立truth记录包含2个静态人物。后验选择的3帧朝目标方向RGB画面主要为
  墙面/房间/走廊，未见人物；只说明抽检画面，不能证明全部帧的遮挡状态或检测召回率。
  下一最小场景须在运动中产生真实可检测线索；不得用注入事件冒充完整在线验收。

日志仍有16次nlopt提案失败，原保守回退继续；10次外部拒绝中2次为最新走廊拒绝，
8次为没有已知自由空间中的C2停止前缀。25次安全制动事件不等于安全故障，也不等于
移动C2换轨；此处的“C2停止前缀”是原安全停止连续性语义。没有调阈值绕过这些门。

11664次安全回调，完整回调max wall54.321ms /threadCPU47.360ms；遥测捕获637个
独立停止预留检查max44.983ms。测量marker max8.908ms、wrapper4.024ms，零错误/drop，
无故障窗口，不宣称零开销。旧200ms故障未复现、仍未解决。871次检测mean17.536ms、
max70.567ms，不构成CPU/GPU端到端收益对比。

独立IMU后审计14.955秒通过主记录组件：60000源样本、290133次protobuf往返、
两路各60000公开原始样本逐位一致、59997公开积分、29999原DownSampler，无clips。
ULog次级记录raw两路各缺尾7个；vehicle_imu缺内部4个，逐位匹配部分无误差。
既有Stage64次级有序子集规则允许该记录，但**ULog不完整**，不能外推正式飞行验收。

任务能量包络明确为synthetic_contract_wiring_only_not_energy_calibration，非独立校准。
误报上限、能量/预留和其他阈值仍待独立数据，未启动配对或宣布目标发现收益。

期间启动和后审计曾被自动审批按旧限制拒绝，均保存未执行证据；在分别收到新增明确
授权后才重试并获准。当前后审计已完成，无待执行审批或不确定在途操作。
关键本地记录：C2-ONLINE-AUDIT.json、C2-IMU-AUDIT-RESULT.json、C2-FINAL-AUDIT-STATE.json、
C2-WIRING-1.json、C2-FRAME-SELECTION.json及c2-selected-frames/。原始数据仍仅保留远端，
仅取回约430KB的3份图像证据，不复制整批日志或旧实验。
