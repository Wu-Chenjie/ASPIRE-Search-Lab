# 单机动力学验证

当前 stage23 的构建、启动和验收边界以[执行安全修复说明](../../docs/EXECUTION-SAFETY.md)为准；下文保留旧阶段接口与回放命令，不能作为最新安全构建的启动命令。

只在已授权本机 `annual-px4-research` 容器运行；使用现有 PX4 SITL 二进制、Micro XRCE Agent 和 px4_msgs，不连接飞行硬件。原工程仅提供只读依赖，不用其同名 Python 包。

```bash
source /opt/ros/jazzy/setup.bash
source /planner/external/ros_ws/install/setup.bash
source /tmp/racer-search-task3-native-stage2b/install/setup.bash
export PYTHONPATH=/tmp/racer-search-task3:/tmp/racer-search-task3/src:/tmp/racer-search-task3/src/annual_planner/src:$PYTHONPATH
cd /tmp/racer-search-task3
python3 tools/flight/run.py --output /tmp/racer-search-task3-results/flight-new --active-review --performance consistent --layout visible-left --rviz --seconds 180
# 无人负例：额外加 --negative；每次使用新的输出目录，顺序运行。
```

执行链：Gazebo 动态 x500 → PX4 EKF2 → 原有 PX4Bridge → 原有 ViewExecutor；深度时刻位姿插值建图 → ROS2 RACER 完整 FSM → 精确 B-spline、动态约束、已知自由空间制停和执行租约验证 → PX4 Offboard。没有 `set_pose`。GNSS 辅助 EKF2 仅验证仿真执行链，未完成真实室内定位。

输出包括运行配置、世界模型、独立真值、组件日志、估计里程计/执行状态、原生提案和拒绝地图快照。真值文件未传入运行决策节点。`result.json` 中 `error:null` 只代表进程完成，是否自主探索须检查 `flight/0/events.jsonl` 的轨迹激活及终端观测证明，不能把扫描和悬停视为探索成功。

启动柱上下扫描高度按机体已知自由空间检查需要设置为 1.75 / 2.65 / 2.2 m，不依据人体坐标。它沿用原执行器的已知安全起降区域假设；还需补充通用起降柱的实测安全验证。

`--active-review` 同时启用保守的逐段执行和原生前沿拒绝恢复。它不是单独只改变候选评分的消融开关；不能将它与旧运行的差异归因为主动视角收益。每个启动先记录运行源码快照，配对图像带采集时刻估计位姿；不修改已有运行目录。


可视化与受限性能验证：

```bash
python3 tools/flight/run.py --output /tmp/person-search-visible \
  --active-review --performance consistent --layout visible-left --rviz --seconds 180
# --layout partial-left / partial-right / visible-right；无人加 --negative。
python3 tools/flight/evaluate.py /tmp/person-search-visible --output /tmp/flight-summary.json
python3 -m tools.flight.flight_localization /tmp/person-search-visible --output /tmp/flight-position.json
```

`--rviz` 启动 RViz2 和仅绑定本机回环的 noVNC。RViz 同时显示实时原始 RGB、机体和轨迹。使用带安全地图导出的原生构建和 `RACER_EXECUTION_SAFETY=1` 时，三维窗口默认显示 `Accumulated occupancy`，也就是原生融合地图中占据体素的中心点，每秒最多更新两次。可在 RViz 中切换 `Measured depth` 查看单帧深度点云，或勾选 `YOLO masks and boxes` 查看分割。关闭 `--rviz` 可运行无显示的对照实验。任务结束会关闭本轮显示进程。

WSL 安装 `ros-jazzy-rviz2` 后，如使用 WSLg，在运行命令前加 `ASPIRE_RVIZ_DISPLAY=:0`，再从 Mac 的 UU 远程桌面看 Windows 上的 RViz 窗口。使用浏览器时则保留默认 noVNC 模式，另外需要 `x11vnc`、`novnc` 和 `websockify`。若 Windows 可以访问 WSL 的回环端口，在 Mac 运行以下 SSH 隧道后打开 `http://127.0.0.1:18681/vnc.html?autoconnect=true&resize=scale`。走已有 UU SSH 映射时，将 SSH 别名换为 `microgrid-275hx-uu`。

```bash
ssh -o ExitOnForwardFailure=yes -N -L 127.0.0.1:18681:127.0.0.1:18680 microgrid-275hx
```

`truth-poses.jsonl` 仅由独立 Gazebo 订阅器记录，不发布到 ROS 或决策链；接触记录没有样本时不能声称零碰撞。新配置与定位口径见 [精度说明](../../docs/PRECISION.md)。

若 Colima 自动转发在重启显示服务后失效，可在 Mac 使用现有本机 VM 配置建立临时回环转发（已实际验证）：

```bash
ssh -F "$HOME/.colima/_lima/colima-racer-gvp/ssh.config" \
  -o ExitOnForwardFailure=yes -N -L 127.0.0.1:18681:127.0.0.1:18680 lima-colima-racer-gvp
# 浏览器改用 http://127.0.0.1:18681/vnc.html?autoconnect=true&resize=scale
# 演示后只撤销此转发，不停止用户的 VM 或 SSH 主连接：
ssh -F "$HOME/.colima/_lima/colima-racer-gvp/ssh.config" \
  -O cancel -L 127.0.0.1:18681:127.0.0.1:18680 lima-colima-racer-gvp
```


确认标记的窄项回放（同一容器环境，无新飞行/检测）：

```bash
# Mac：从项目旁持久归档自动复制、恢复到新容器 /tmp 目录
bash tools/flight/replay_saved.sh
# 可选参数：其他持久 runs.tar.gz 的绝对路径
```

它读取已有 `telemetry.jsonl` 中首次真实 `confirmed`，复用在线节点的 `target_markers()`，经实际 DDS 订阅核对标记坐标、原时间戳和绿色 RGBA。`source-record.json` 留存原始行及源文件 SHA256；`received-marker.json` 是订阅所得 ROS Marker。RViz 明示 `RECORDED OUTPUT REPLAY`。RGB/点云仅为最近保存帧的上下文，当前证据早于确认2.44秒，不声称它们同步或产生了本次确认。截图为突出标记隐藏了点云/网格层，不改变目标数据。回放显示使用容器回环18682端口，可按上面的本机SSH方式转发到Mac18684端口。

实验性能选项 `--performance bounded-fast` 的受限配对结果与退回建议见 [动态约束核验](../../docs/DYNAMICS.md)。它不替换上面的 `consistent` 演示配置，也不关闭任何授权检查。

第五阶段新增 `--budget-clock sim`（默认仍为wall）与可关闭的 `--recover-no-path`。开关须配合 `--active-review`；使用第五阶段新构建。七次180秒仿真时间试验均未自然触发恢复，不能声称飞行脱困，见 [恢复验证](../../docs/RECOVERY.md)。
