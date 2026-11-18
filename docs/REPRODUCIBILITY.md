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
