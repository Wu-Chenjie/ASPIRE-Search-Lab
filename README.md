# ASPIRE Search Lab

灾后室内单机自主建图与人员搜索仿真。A 运行 ROS 2 / Gazebo / PX4 SITL，使用原生 RACER 探索和复查。B 提供人员观测，C 提供配准的 RGB-D 数据，D 提供房间几何、遮挡和起点。

| 成员 | 开发入口 | 交给谁 |
| --- | --- | --- |
| A · 规划与仿真 | [仿真入口](tools/flight/run.py)、[场景装配与飞行参数](tools/flight/scene_bundle.py)、[规划与执行](src/annual_planner/src)、[复查策略](src/aspire_planning)、[RACER 集成](tools/flight) | 消费 D 的场景、C 的深度和 B 的观测，发布执行反馈 |
| B · 视觉识别 | [视觉节点](src/person_search/node.py)、[人员观测编码](src/person_search/registry_evidence.py)、[检测器配置](config/detectors) | 给 A 发布候选、确认状态、方位和可选表面位置 |
| C · 深度相机集成 | [相机包](src/aspire_camera)、[仿真相机与 ROS 桥接](src/aspire_camera/gazebo.py)、[安装几何](src/aspire_camera/mount.py) | 给 A、B 提供图像、标定、TF 和捕获时刻的位姿 |
| D · 地图与场景 | [房间定义](tools/scenarios/room.json)、[世界生成器](tools/scenarios/room.py)、[已有 HM3D 工具](tools/hm3d) | 给 A 交付地图 JSON 或可直接加载的场景包 |

接口变更由提供方和使用方共同核对。[模块接口与验收](docs/TEAM-INTERFACES.md)

## D 准备场景

在仓库根目录使用已安装 [Python 依赖](requirements.txt) 的环境运行。复制房间 JSON 即可修改地图边界、命名起点、遮挡物、墙体和单个人物的初始姿态。下面三条命令只生成场景，各需一个新的输出目录。

```sh
python3 -m tools.flight.scene_bundle --output /tmp/aspire-lab-visible --layout visible-left
python3 -m tools.flight.scene_bundle --output /tmp/aspire-lab-occluded --layout original
python3 -m tools.flight.scene_bundle --output /tmp/aspire-lab-empty --negative
```

自定义地图加 `--spec path/to/room.json --layout 自定义起点名`。场景包内包含世界 SDF、无人机和相机模型、ROS 桥配置、运行配置、地图原稿和独立评测真值。提交地图原稿即可；生成物放到代码树外。

## A 加载并运行

飞行入口支持 D 提供的 `--prepared 场景包目录`，也支持 `--scene-spec 地图JSON --layout 起点名`。运行前加载团队已有的 ROS 2 Jazzy、px4_msgs 和对应 RACER 安装环境，准备 Gazebo Harmonic、PX4 SITL、Micro XRCE Agent、检测环境及匹配的本地权重。下列变量由运行机设置，输出路径须尚未使用。

```sh
export PYTHONPATH="$PWD:$PWD/src:$PWD/src/annual_planner/src${PYTHONPATH:+:$PYTHONPATH}"
RACER_EXECUTION_SAFETY=1 python3 tools/flight/run.py \
  --prepared /tmp/aspire-lab-occluded --output "$ASPIRE_RUN_DIR" \
  --external "$ASPIRE_SITL_EXTERNAL" --budget-clock sim --seconds 180 \
  --confirmation-mode registry --detector yolo11n-seg \
  --segmentation-weights "$ASPIRE_SEG_WEIGHTS" --bearing-bias
```

`--external` 指向运行机上已有的 PX4 和 Agent 构建依赖目录。仓库自带模型可供离线生成场景。上述组合接入人员观测注册表及方位引导；主动静态复查的成对实验还需经验证的 [运行设置](docs/STATIC-EVALUATION.md)。历史运行入口、固定批次与当前演示的适用范围见 [复现说明](docs/REPRODUCIBILITY.md)。

## 开发验证

```sh
python3 -m pytest -q -p no:cacheprovider tests
python3 tools/racer_ros2/verify.py external/racer_ros2_ws
```

原生协议测试使用 C++ 编译器和 Boost 头文件。若 macOS 的 Boost 安装在 Homebrew 的 include 目录，可先运行 `export CPATH="/opt/homebrew/include${CPATH:+:$CPATH}"`。缺少 ROS、指定录制或运行环境的测试会分别跳过。

代码基于 [Wu-Chenjie/ASPIRE 的 8892f464](https://github.com/Wu-Chenjie/ASPIRE/tree/8892f4643ce740129a5691f6f6514363d82e06cb)，取自第 28 轮使用的开发分支后续提交。本仓库从该提交的完整文件快照独立建库。历史结果位于 [状态记录](docs/STATUS.md)，第三方来源见 [许可证目录](licenses) 和 [依赖清单](dependencies.lock.json)。
