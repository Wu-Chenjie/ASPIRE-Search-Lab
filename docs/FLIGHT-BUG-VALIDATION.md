# 新增飞行：优化异常与有界恢复验收

本机 `annual-px4-research` / `colima-racer-gvp`，Gazebo + PX4 SITL + Jazzy，7轮新增动力学飞行。全部使用 `consistent`、`partial-left`、`--active-review`；感知、控制器、未知空间/C2/制停/执行租约检查均未改。GNSS辅助EKF2，不是实机或无GNSS室内定位验收。

## 结论

| 验收 | 结果 |
|---|---|
| 正常探索回归 | 180仿真秒，7次激活及7次末端实测观测；跟踪P95 0.108m、最大0.152m。后段选到非自由目标而停滞，不能称完整探索通过。恢复关闭。 |
| A*失败→恢复→实际执行 | 首次注入飞行成功；计数修复后再次成功：83.392秒注入，3次真实A*超时；选择此前已授权根点作退回目标。轨迹2于83.944秒获执行器激活，92.000秒完成观测，实际位移2.400m。 |
| 所有搜索失败→有界停止 | 旧计数器实际尝试24次。修复复验：3次触发失败+累计8次恢复尝试，77.100秒进入预算停止，直到160秒无新激活。停止段跟踪P95 0.160m、最大0.202m。 |
| 常量初值零间距→优化→执行 | 83.280秒以实测航向0.468610250rad构造15个相同偏航初始控制点；日志命中 `TASK_OPT constant_signal_normalization dim=1`。轨迹2于83.804秒激活，88.028秒完成观测，实际位移0.263m。 |
| 普通优化失败处理 | 飞行中仍有普通 `nlopt failure`，被转为规划失败，没有崩溃或激活半条曲线；不保证所有请求求解成功。 |

常量用例原生完整时长7.053秒，执行器只允许3.303秒已知自由前缀+0.600秒制停，共3.903秒；执行偏航率峰值约1.53e-16rad/s。完成的是安全截取段及其末端观测，不是到达原请求终点。没有放宽安全检查。

“所有搜索失败”是持续搜索预算故障注入，不是物理封死房间；可恢复用例也不是自然障碍脱困。真实障碍导致的飞行脱困仍未覆盖。第六阶段不连通几何原生测试继续通过，但没有飞控，不能与本轮合称物理障碍飞行验收。

## 修复与测试边界

生产原生代码仅改 `tools/flight/recovery_policy.hpp` 对应生成头文件：同一恢复事件累计最多8次；观测地图、前沿变化或悬停漂移不补充次数。仍至少间隔0.5秒，耗尽后每5秒检查并保持停止。只有真实外部轨迹激活或新会话重置；耗尽后地图变化也不自动恢复，属于保守停止取舍。

新增5000次持续变化签名/根点调度回归；同一测试对旧代码失败、对新代码通过。原生优化器沿用第六阶段修复，没有另调权重、速度或安全限值。

`tools/flight/flight_fixture_overlay.py`仅在 `build_native.py --flight-fixture` 构建中存在，普通构建不含钩子：

- `timeout-primary`：首条正常曲线完成后，把真实A*的1秒搜索预算临时设为0，由原超时分支返回NO_PATH。进入恢复后还原1秒，实际激活后停止注入。没有伪造返回值、地图或优化结果。
- `timeout-all`：普通与恢复搜索均使用0秒预算，验证资源持续故障时停止。
- `constant-yaw`：目标来自实测原生前沿，起终偏航相同，给原优化器合法常量初值。代价、NLopt和安全执行不变。早期仅设置相同起终偏航未产生零间距，明确记为未覆盖后重新验收。

## 全部运行

| 名称（均以stage7-开头） | 仿真秒 | 激活/完成 | 恢复尝试/激活 | 说明 |
|---|---:|---:|---:|---|
| normal | 180 | 7/7 | 0/0 | 恢复关闭、无注入 |
| recover | 180 | 11/11 | 1/1 | 旧计数器；首次真实执行恢复 |
| bounded-stop | 160 | 1/1 | 24/0 | 失败证据：地图变化补充预算 |
| fixed-timeout-all | 160 | 1/1 | 8/0 | 修复后停止通过 |
| fixed-constant-yaw | 160 | 8/7 | 1/1 | 起终航向相同但零间距未覆盖；最后1条因时间到未完成 |
| fixed-timeout-primary | 160 | 5/5 | 9/1 | 注入事件1次成功恢复；后续自然失败事件8次后停止 |
| yaw-zero-spacing | 160 | 4/4 | 12/2 | 零间距实际覆盖；不同事件分别计数，非单事件12次 |

全部正常收尾，无遗留/强杀进程，无锁存飞行故障。记录中无非地面接触事件，但缺少已验证的接触传感器心跳，不能据此证明零碰撞。非自由前缀、过期根点等拒绝完整保留。全套测试：Mac 58 passed / 10 skipped；Jazzy 112 passed。原工程381个来源文件哈希未变，未提交成果保留。

完整证据在 `../task-3-evidence/stage-7/`：`final-audit.json`逐轨迹关联原生ID、执行token、授权时长/限值、实际位移与末端观测；运行归档内 `result.json`保存真实开关、模式、运行源码及二进制SHA256。正常/旧计数器/新计数器/常量初值的二进制SHA前缀依次是 `38fc3194` / `562faef5` / `b719646f` / `e90cabb3`；完整值及精确生成源码、二进制保存在归档。

## 复现入口

既有容器内源码 `/tmp/racer-search-task3-stage7`。构建和输出目录每次新建，禁止在代码树内构建，无需安装依赖：

```bash
source /opt/ros/jazzy/setup.bash
source /planner/external/ros_ws/install/setup.bash
cd /tmp/racer-search-task3-stage7
export PYTHONDONTWRITEBYTECODE=1
python3 tools/search/build_native.py /tmp/racer-new-validation --flight-fixture
colcon --log-base /tmp/racer-new-validation/log build \
  --base-paths /tmp/racer-new-validation/src --build-base /tmp/racer-new-validation/build \
  --install-base /tmp/racer-new-validation/install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /tmp/racer-new-validation/install/setup.bash
export PYTHONPATH=$PWD:$PWD/src:$PWD/src/annual_planner/src:$PYTHONPATH
RACER_FLIGHT_FIXTURE=timeout-primary python3 tools/flight/run.py \
  --output /tmp/racer-new-recover --performance consistent --layout partial-left \
  --active-review --recover-no-path --budget-clock sim --seconds 160 --record-video
# 顺序运行：模式改 timeout-all / constant-yaw，输出目录每次新建。
# 普通构建去掉 --flight-fixture；正常回归 unset RACER_FLIGHT_FIXTURE，
# 去掉 --recover-no-path，运行180秒。恢复默认关闭，需显式开启。
python3 -m tools.flight.bug_validation /tmp/racer-new-recover --output /tmp/racer-new-audit.json
ROS_DOMAIN_ID=196 SEARCH_CAPTURE=/tmp/racer-search-task3-results/static-01 \
  python3 -m pytest -q -p no:cacheprovider tests
```

## 录像

实时订阅RGB写入 `continuous-rgb.avi`，约15Hz，每帧时间戳另存；少量丢帧以JSONL为准。不是Gazebo屏幕录制，也不是旧5秒一帧采样回放。

`media/racer-stage7-flight-validation.mp4`是74秒连续采集片段加注释，无补帧：0–19.2秒正常探索；19.2–33.87秒恢复执行；33.87–44.73秒常量初值；44.73–74秒8次后停止。无HOG框，不评价本轮感知。完整原始AVI在三份运行归档内；来源/帧索引/SHA在 `media/video-source.json`。

Library：`libfile_4e8aa72efed4819189dcfa4299d9c124`。
