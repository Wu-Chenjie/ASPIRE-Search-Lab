# 心跳修复与验证（阶段13）

修复选择：在独立PX4副本中把`EKF2::PublishStatusFlags()`的保底周期从1 s改为100 ms仿真时间，状态改变仍立即发布真实快照。不改变RACER 0.3 m体素/0.6 m净空、轨迹/动力学、桥接0.5 s输入保护和3 s flags墙钟保护，PX4 Offboard失联保护保持0.5 s。不安装GPU依赖、不上传。

## 代码与构建

- `tools/flight/build_px4.py`：锁定原EKF2源文件SHA，在代码树外复制既有PX4及其未提交成果；不checkout/reset。离线复用本机Micro-CDR源码，加载既有Jazzy Gazebo vendor路径，验证产物包含`gz_bridge`。原源目录保持只读。
- `px4_bridge_node.py`、`heartbeat_diagnostics.py`：每项拒绝门槛、flags/原始位置/姿态/参考/状态输入的双时钟年龄和接收间隔、每次心跳间隔。诊断明确区分“收到原始位置”与“通过门控发布位置”。已有拒绝条件逐项展开，阈值未放宽。
- 增加保守时间轴保护：ROS时间倒退超过1 μs，或前进量超过`max(0.5 s, 2×墙钟间隔+0.1 s)`，在已建立地图原点后锁存退出当前会话，不自动恢复旧地图。正常暂停仍由原0.5 s输入墙钟保护停发。
- `run.py --px4-build`显式选择修复产物。`--sim-real-time-factor`只改仿真节拍，4 ms物理步长不变；`--wall-watchdog-seconds`只用于慢速诊断的测试总预算，不改变飞控保护。
- `heartbeat_audit.py`：运行结束后独立读取PX4 ULog和桥接原始日志，区分解锁前启动等待与解锁后的停发，输出Offboard信号丢失、failsafe及最大心跳间隔。

构建：

```bash
python3 -m tools.flight.build_px4 \
  --source /planner/external/PX4-Autopilot \
  --output /tmp/racer-stage13-px4 --jobs 3
```

在[人员搜索](YOLO-SEARCH.md)的既有环境和命令中加：

```bash
--px4-build /tmp/racer-stage13-px4/source/build/px4_sitl_default
```

仍使用原权重、CPU推理、Gazebo/RViz、正常RACER探索。必须使用修复后的构建路径；原PX4保留用于恢复和对照。

## 验证口径

`tests/test_heartbeat_timing.py`使用真实桥接类及原生ROS/PX4消息、受控双时钟和替身发布器，无DDS、无飞机，不能称真实飞行。覆盖RTF 1/.2/.1/.05、旧1 Hz负对照、flags/位置/姿态/参考断流、暂停、正反时钟跳变、真实对齐失效与恢复、PX4已触发failsafe后不得自动恢复。54项桥接专项通过；完整原生首次143通过。第二次与编译并行时检测桩接口短窗口测试失败，142通过，失败日志保留；无负载重跑及实际飞行结果见最终证据。

初次隔离构建的Micro-CDR下载被离线限制阻止，改用已有同版本源码后完成。第一次新飞行在0.012 s因未编入`gz_bridge`退出，未起飞；重新加载ROS vendor路径后修正。两个问题均保留日志，不计作飞行成功。

## 实际运行记录

有效回归02完成240 s仿真，解锁后ULog无`offboard_control_signal_lost`、无failsafe，桥接无门控拒绝、无时钟跳变。18359个心跳间隔最大87.991 ms墙钟/36 ms仿真；flags接收墙钟间隔中位数0.133 s、最大0.661 s，ULog flags间隔中位数0.104 s。就绪后0接触，19次轨迹激活/18次观测完成，新增4281个观测体素，路径34.439 m。

本次二维首次检出108.968 s，有效定位201.568 s，但只累计到一个分离语义视角，240 s内没有确认，亦无错误确认；不能把它写成完整人员确认成功。CPU推理中位数0.450 s、P95为0.647 s；跟踪误差P95为0.0802 m。未改变检测、深度过滤或观察点来强行确认。

无编译负载完整回归143通过，Mac72通过/12跳过（ROS原生用例在Mac不执行）。低速回归03以RTF 0.2完成75 s仿真/389.422 s墙钟：解锁后无Offboard丢失、无failsafe、无门控拒绝、无时钟跳变，就绪后记录0接触。16638个心跳间隔最大67.645 ms墙钟/20 ms仿真；flags接收间隔中位数0.521 s墙钟、最大0.633 s。3次轨迹激活/2次观测完成，26进程全部正常清理。这是短时低速稳定性验证，不是完整搜索发现率试验。

最终正常回归04完成300 s仿真/698.093 s墙钟，解锁后仍无Offboard丢失、failsafe、门控拒绝或时钟跳变。33643个心跳间隔最大158.955 ms墙钟/36 ms仿真；flags接收墙钟间隔中位数0.164 s、最大0.968 s。24次激活/23次完成，新增4242个观测体素，路径37.667 m，最大完成间隔51.408 s。就绪后记录0接触，跟踪误差P95为0.0734 m；授权轨迹速度/加速度/jerk/偏航率峰值0.610/0.932/3.928/0.618，原执行限制保持。26个所属进程全部清理，无强杀或残留。

04首次二维检出231.728 s、三维定位231.928 s，300 s内仍未跨视角确认；真目标track 1只有一个分离语义视角，后续超时放弃。961次CPU推理中位数0.416 s、P95为0.649 s、最大1.211 s，过期结果仍被拒绝。本轮没有启用目标复查、手动观察点或真值引导，不能称稳定的人员确认成功。

独立几何抽样重算40次定位，在线位置全部可复现，人体深度支持比例中位数97.66%、表面距离中位数6.72 cm。但293.572 s的图像边缘窄框（约12×111像素、置信度0.439）产生了错误候选：128个抽样像素均不支持目标人体，定位距人体表面6.972 m，成为未确认track 2。最终正确确认0、错误确认0，**并非零错误候选**。异常原记录见`perception-anomaly.json`；中位数不能掩盖这个离群值。需另行解决语义误检及复查确认稳定性，本次未为消除此样本临时改检测阈值。

## 复现与保存

低RTF诊断在正常命令上增加`--sim-real-time-factor 0.2 --wall-watchdog-seconds 600 --seconds 75`。最终正常回归使用`--seconds 300`；均带修复后的`--px4-build`、CPU YOLO、`--stop-and-plan --recover-no-path --record-video --rviz`。详细基础环境见[人员搜索](YOLO-SEARCH.md)。运行后执行：

```bash
python3 -m tools.flight.heartbeat_audit NEW_RUN
python3 -m tools.hm3d.flight_audit NEW_RUN
python3 -m tools.flight.search_evidence NEW_RUN
python3 -m tools.flight.motion_audit NEW_RUN --output /tmp/motion-audit.json
```

证据保存在代码树外`../task-3-evidence/stage-13-heartbeat/`：全部四次运行（含未起飞失败01）的原始RGB、推理记录、ULog、双时钟诊断、遥测、输入记录、运行源码快照；全部测试及构建失败日志；修复PX4源码、许可证、离线依赖及bin/etc；本任务源码检查点和SHA清单。来源资产与模型仍复用阶段10/11归档。原Annual-Planner的353个初始来源和30个飞行来源哈希未变，141项既有未提交记录保留；原PX4仍为原三项未提交状态。

这三次有效运行支持本次修复的有限时长验证，不能代表无限负载或长时间稳定性；非常低的RTF或真实输入停滞仍应按原保护退出。单次300 s内未确认及一个错误候选是明确剩余问题。未开展GPU安装/迁移、无GNSS定位、同预算主动复查对照、多场景泛化或实机验收；没有上传、push或发布。
