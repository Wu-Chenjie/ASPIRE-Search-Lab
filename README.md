# ASPIRE Search Lab

灾后室内单机自主建图与人员搜索仿真。A 运行 ROS 2 / Gazebo / PX4 SITL，使用原生 RACER 探索和复查。B 提供人员观测，C 提供配准的 RGB-D 数据，D 提供房间几何、遮挡和起点。

| 成员 | 开发入口 | 提交前先跑 |
| --- | --- | --- |
| A · 规划与仿真 | [仿真入口](tools/flight/run.py)、[规划与执行](src/annual_planner/src)、[复查策略](src/aspire_planning)、[原生构建](tools/search/build.sh) | `tests/test_flight_entry_supervision.py`、`tests/test_build_native_output.py` |
| B · 视觉识别 | [视觉节点](src/person_search/node.py)、[检测配置](config/detectors) | `tests/test_person_search.py`、`tests/test_registry_evidence.py` |
| C · 深度相机 | [相机包](src/aspire_camera)、[ROS 桥接](src/aspire_camera/gazebo.py) | `tests/test_rgbd_scene.py`、`tests/test_rgbd_registration.py` |
| D · 地图与场景 | [房间定义](tools/scenarios/room.json)、[世界生成器](tools/scenarios/room.py)、[HM3D 导入](tools/hm3d/prepare.py) | `tests/test_team_boundaries.py`、`tests/test_person_scene.py` |

日常改动从 `main` 开短分支，如 `b/vision-confidence` 或 `d/room-layout`。提交 PR 时附上验证命令和结果。改动话题、坐标、时间戳或场景格式时，同步更新接口说明，并请数据提供方和使用方共同复核。合并前跑一次全套主机测试。[接口与验收](docs/TEAM-INTERFACES.md)

## D 准备场景

在仓库根目录使用已安装 [Python 依赖](requirements.txt) 的环境运行。复制房间 JSON 即可修改地图边界、命名起点、遮挡物、墙体和单个人物的初始姿态。下面三条命令只生成场景，各需一个新的输出目录。

```sh
python3 -m tools.flight.scene_bundle --output /tmp/aspire-lab-visible --layout visible-left
python3 -m tools.flight.scene_bundle --output /tmp/aspire-lab-occluded --layout original
python3 -m tools.flight.scene_bundle --output /tmp/aspire-lab-empty --negative
```

自定义地图加 `--spec path/to/room.json --layout 自定义起点名`。场景包内包含世界 SDF、无人机和相机模型、ROS 桥配置、运行配置、地图原稿和独立评测真值。提交地图原稿即可；生成物放到代码树外。HM3D 场景导入另需本地数据集文件以及 `trimesh`、`rtree`。

## A 加载并运行

飞行入口支持 D 提供的 `--prepared 场景包目录`，也支持 `--scene-spec 地图JSON --layout 起点名`。运行前准备 ROS 2、PX4 SITL 和本地检测权重，并构建、载入原生 RACER。运行机设置下列变量及尚未使用的输出路径。[联调环境与构建](docs/REPRODUCIBILITY.md)

```sh
export PYTHONPATH="$PWD:$PWD/src:$PWD/src/annual_planner/src${PYTHONPATH:+:$PYTHONPATH}"
RACER_EXECUTION_SAFETY=1 python3 tools/flight/run.py \
  --prepared /tmp/aspire-lab-occluded --output "$ASPIRE_RUN_DIR" \
  --external "$ASPIRE_SITL_EXTERNAL" --budget-clock sim --seconds 180 \
  --confirmation-mode registry --detector yolo11n-seg \
  --segmentation-weights "$ASPIRE_SEG_WEIGHTS" --bearing-bias
```

`--external` 指向运行机上已有的 PX4 和 Agent 构建依赖目录。仓库自带模型可供离线生成场景。上述组合接入人员观测注册表及方位引导。主动静态复查的成对实验另有运行设置和评分协议。[配对验收](docs/STATIC-EVALUATION.md)

## 开发验证

```sh
python3 -m pytest -q -p no:cacheprovider tests
python3 tools/racer_ros2/verify.py external/racer_ros2_ws
```

原生协议测试使用 C++ 编译器和 Boost 头文件。若 macOS 的 Boost 安装在 Homebrew 的 include 目录，可先运行 `export CPATH="/opt/homebrew/include${CPATH:+:$CPATH}"`。缺少 ROS、指定录制或运行环境的测试会分别跳过。

代码来源和依赖见 [原 ASPIRE 提交](https://github.com/Wu-Chenjie/ASPIRE/tree/8892f4643ce740129a5691f6f6514363d82e06cb)、[依赖清单](dependencies.lock.json)和[第三方许可证](licenses)。早期实验批次、阶段报告和专用复盘脚本见 [精简前快照](https://github.com/Wu-Chenjie/ASPIRE-Search-Lab/tree/5c886f12039a18d6bafed0d6c93f9953454c30a1)。
