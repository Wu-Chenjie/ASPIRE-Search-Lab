# 执行安全修复与原档验证（stage 23）

本目录为独立修复副本；冻结 stage20 与原 Annual-Planner 未修改。当前仍未通过完整自主人员搜索验收，激进动力学档位未运行。

## 改动和边界

保留锁定 ROS2 RACER 的探索、A*、ESDF、轨迹和碰撞检查。运行时只读实测 RGB-D、估计位姿及真实融合后的地图，不读人体、障碍或场景网格真值。网格只用于独立离线评测。

- 授权与激活检查整条连续曲线、原物理净空、原生立方膨胀、较早 mapper 占据和 10 cm 跟踪管；未知空间仍阻挡。原问题来自追加制动超出原生检查范围及后续地图更新，不是膨胀算法漏格。
- 保留原动力学限制，每隔最多 0.10 s 检查 1.2 s 延迟预算及其后的可行停稳段。地图过期或轨迹失效时选择有限 C2 制动；不能认证制动时明确失败。参考停止期限固定，观测回执独立，不无限延期。
- 5 cm 跟踪误差触发提前制动，10 cm 硬管不变，发布下一参考前再次检查。5 cm 是保守触发阈值，不是实际机体跟踪误差的证明。
- 三个不同提案的授权拒绝进入原有有限恢复，每轮最多八次，仅实际激活重置。冷却过期可重新检查候选，但不补充预算。
- 安全构建下，当前执行的有效完成回执可触发原生前沿更新，修复短制动完成仍等待原时长的问题。过期、错误编号和未完成回执不能放行；返回任务逻辑不变。
- 仅单机停止后再规划；未知净空、整路径检查和限速均未放宽。单线程回调耗时用单调墙钟，依赖配置 RTF≤1 的条件，不声称测到了回调内部仿真耗时。

默认非安全构建仍无执行安全补丁；共享的恢复冷却头文件有改动，不能声称全部默认生成文件字节未变。安全构建用 `--execution-safety --consistent-inflation`，运行用 `RACER_EXECUTION_SAFETY=1`。

## 验证

宿主：153 passed / 16 skipped；Jazzy 原生：247 passed / 6 skipped（native-tests-v13.log）。实际 C++ FSM 探针重现原时长 5.69 s、制动完成进度 0.564 s，验证正常无前沿 IDLE 分支与过期/错编号拒绝。接口测试按采集时间等待空画面结果，保留第一次异步等待失败记录。

最新原生二进制 SHA256：`173af2ac156e28589468ac56ea1d2471b5f5712590541f60d179b470d2b7498e`。逐次飞行结果、代码和二进制哈希均写入各自 result.json。

三轮均为同一最新构建、原动力学、YOLO11n-seg、180 s 仿真预算。下表是单次记录，不是策略优劣统计。

| 运行 | 激活/观测完成 | 制动 | 探索阶段估计路径 | 已知自由格净增 | 确认 | 判定 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 无人13 | 19/18 | 6 | 12.588 m | 1054 | 0 | 末段因预算截止未完成；候选误检未确认 |
| 有人14 | 9/9 | 6 | 6.995 m | 850 | 0 | 恢复8次耗尽，末尾约47秒未推进；搜索失败 |
| 有人+延迟15 | 16/15 | 10 | 8.469 m | 179 | 0 | 延迟后恢复；末段预算截止；搜索仍未确认 |

三轮保存的激活/制动证书均通过，采样遥测未发现安全故障或预算违规；各制动参考均在固定期限内结束。探索阶段均0接触、无PX4 failsafe和Offboard丢失，各22个自有进程正常退出。无人13最后参考尚余1.259s，延迟15尚余1.525s，未把预算截止记成观测完成。

无人13：探索阶段378帧不同采集时间的实际推理图像中3帧有误候选（0.794%），1帧得到深度定位，1条背景轨迹，零误确认；全程543帧中3帧有误候选。分母不是全部相机帧，不能据此推出其他场景的误报率。有人14及延迟15各4条候选，独立单人评测严格确认均为0/1、首确认延迟不可得、零误确认；这不是检测器召回率。

延迟15：86.416s暂停本轮mapper，87.404s收到制动报告，88.220s恢复；完整运行16激活/15观测。故障注入不改变场景或规划目标。其余9次制动也逐条核对，明细见lifecycle审计。

三轮均覆盖全部解析到的静态碰撞几何；探索阶段减去0.43m分析球后的最小采样/直线连接下界分别为：无人13 0.531192/0.529931m，有人14 0.392458/0.392238m，延迟15 0.452022/0.451739m。启动地面阶段有非正球界，未隐去；球界非正本身不证明真实机体碰撞。

历史失败均保留：positive01 日志字段冲突、positive02 起飞前 SIGKILL（OOM 未证实）、positive03 激活起点偏离、mapdelay05 两次报告误差超过10cm、mapdelay09 拒绝将越过10cm的下一参考；均不能用后来通过的片段覆盖。mapdelay10 诊断中止前已恢复，不能写成永久死锁。negative12 完成四次观测后因短制动状态机条件停滞。

审计只保存激活/制动/故障时地图，未覆盖每个中间更新，`full_flight_certified=false`。网格净空为采样及直线连接的下界，不是连续真实扫掠或封闭实体内部分类。制动参考速度为零不等于物理速度精确为零。欧拉角跨±π产生的模拟器原始角速度尖峰保留，另统计四元数区间平均旋转率，未改飞控。

## 运行和证据

只使用本机已有 `colima-racer-gvp` 的 `annual-px4-research` 容器；不安装新依赖、不连接其他机器。新源码、构建和运行目录必须分别放置，运行目录不能复用。

```bash
# 在源码目录执行；Mac 跳过 ROS 原生部分
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests
python3 tools/search/verify_workspace.py

# 容器内重建（路径须为新目录）
source /opt/ros/jazzy/setup.bash
source /planner/external/ros_ws/install/setup.bash
python3 tools/search/build_native.py /tmp/racer-rebuild --map-probe --consistent-inflation --execution-safety
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon --log-base /tmp/racer-rebuild/log build --base-paths /tmp/racer-rebuild/src --build-base /tmp/racer-rebuild/build --install-base /tmp/racer-rebuild/install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release

# 复用本轮已验证环境；source 脚本路径由归档恢复到 /tmp
source /tmp/racer-stage23/flight-v5.sh \
  --prepared /tmp/racer-stage20-k1cup-person-30hz-scene \
  --seconds 180 --wall-watchdog-seconds 1200 --active-confirmation --record-video \
  --output /tmp/racer-new-positive
# 无人：改用 negative 场景并加 --negative。
# 延迟故障注入：加 --map-delay-fixture；只暂停/恢复本轮自有 mapper。
```

持久源码为项目旁 `task-3-stage23/`；证据为 `task-3-evidence/stage-23-execution-safety/final-validation/`。录制视频和日志在源码树之外。`v5-positive-14-video/demo.mp4` 为最新有人复测，明确标注搜索未通过；`v2-positive-06-video/demo.mp4` 为较早运行，约2:25有检测框。两者均使用YOLO11n-seg，不能混标版本。原始连续 RGB、时间索引、PX4 日志、地图证书及失败记录随证据归档，场景资产另包。归档有逐文件 SHA256 校验，保留容器原件。ROS/PX4 与 Python 依赖复用本机现有路径，不宣称归档包含完整可移植操作系统环境。

## 尚未完成

1. **1.8 m 初始高度测试阻塞**：中心 FREE 但所需物理包络涉及上方 UNKNOWN，10个提案全部拒绝，恢复八次后保持；0激活不算探索成功。没有清空未知或使用真值逃逸。启动扫描仍依赖预设安全起降柱，尚无通用实测起降柱认证。
2. 严格主动跨视角确认、多房间搜索及同预算三策略公平对照未验收。候选表面定位不是人员确认，不能声称 RGB 看穿遮挡。
3. 当前实际动力学启动为1.5m，持续2–3m巡航偏好、无GNSS室内定位未实现；现有 EKF2 为仿真 GNSS 辅助。
4. 原档门槛未全部通过，1.25×（0.375m/s、0.5m/s²）和1.5×（0.45m/s、0.6m/s²）仅有配置，均未运行；原档为0.3m/s、0.4m/s²。不宣称提速收益。
