# 联调环境与验收

日常开发使用 [Python 依赖](../requirements.txt)。涉及 C++ 协议与执行的主机回归还需要 C++ 编译器和 Boost 头文件。macOS 如通过 Homebrew 安装 Boost，可以把对应 include 目录添加到 CPATH。

```sh
export PYTHONDONTWRITEBYTECODE=1
python3 -m pytest -q -p no:cacheprovider tests
python3 tools/racer_ros2/verify.py external/racer_ros2_ws
```

A 在 Linux 联调机准备 ROS 2 Jazzy、Gazebo Harmonic、colcon、px4_msgs、PX4 SITL 与 Micro XRCE Agent。系统包与已有构建由运行机统一管理。B 负责提供匹配 [检测配置](../config/detectors/yolo11n-seg-locked.json) 的本地权重及检测环境。权重、场景生成物、构建和日志写到仓库外。

在载入 ROS 与 px4_msgs 后构建启用执行校验的原生 RACER。构建目录使用新目录或空目录。

```sh
bash tools/search/build.sh "$ASPIRE_NATIVE_WS" --execution-safety
source "$ASPIRE_NATIVE_WS/install/setup.bash"
```

如果暂时没有 ROS 2，可用下面的命令验证源码生成和覆盖流程。之后由 A 按 [README](../README.md) 的仿真命令执行整套联调。

```sh
python3 tools/search/build_native.py "$ASPIRE_NATIVE_SRC_CHECK" --execution-safety
```

A 与 D 先用同一个房间确认三组场景能加载。无人组用于检查误报，起点可见组用于检查感知链路，遮挡组用于检查复查决策。每次使用独立输出目录，并记录代码提交、场景、检测权重和运行配置。结束后看 `result.json`、`telemetry.jsonl` 和 `flight/0/events.jsonl`。需要复核执行证书时，使用 [离线证书审计](../tools/flight/stage23_safety_audit.py)。报告中注明使用了主机测试、静态采集还是 PX4 SITL 闭环，跳过项注明原因。

同一固定场景的引导开关对照见 [配对验收](STATIC-EVALUATION.md)。

## Depot 07 的 60 Hz 相机时钟

团队 Jazzy 运行机自带的 gz-sensors 8.2.2 按整毫秒推进相机截止时间，60 Hz 会被量化为 62.5 Hz。Depot 07 启动脚本优先加载 `/opt/aspire/build/gz-sensors8-clock/lib` 中的核心库。该库使用 8.2.2 源码和上游的高精度时钟修复，适配现有 Harmonic 渲染库。[Gazebo 上游改动](https://github.com/gazebosim/gz-sensors/commit/8d9fef11078368622150c98688a6eda5fc7163c1)

在相同版本的新运行机上，可以在仓库外构建该库。以下命令需要对团队的构建目录有写权限。

```sh
(
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/jazzy/setup.bash
work=$(mktemp -d)
git clone --branch gz-sensors8 https://github.com/gazebosim/gz-sensors.git "$work/src"
git -C "$work/src" checkout --detach 9348f9fe8a11b9d50381819f51e17e136aedab8a
git -C "$work/src" cherry-pick --no-commit 8d9fef11078368622150c98688a6eda5fc7163c1
cmake -S "$work/src" -B "$work/build" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "$work/build" --target gz-sensors8 -j4
install -Dm755 "$work/build/lib/libgz-sensors8.so.8.2.2" \
  /opt/aspire/build/gz-sensors8-clock/lib/libgz-sensors8.so.8
install -m644 "$work/src/LICENSE" /opt/aspire/build/gz-sensors8-clock/LICENSE
)
```
