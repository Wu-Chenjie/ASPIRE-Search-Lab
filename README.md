# ASPIRE Search Lab

灾后室内单机自主建图与人员搜索仿真。A 运行 ROS 2 / Gazebo / PX4 SITL，使用原生 RACER 探索和复查。B 提供人员观测，C 提供配准的 RGB-D 数据，D 提供房间几何、遮挡和起点。

| 成员 | 开发入口 | 提交前先跑 |
| --- | --- | --- |
| A · 规划与仿真 | [仿真入口](tools/flight/run.py)、[规划与执行](src/annual_planner/src)、[复查策略](src/aspire_planning)、[原生构建](tools/search/build.sh) | `tests/test_flight_entry_supervision.py`、`tests/test_build_native_output.py` |
| B · 视觉识别 | [视觉节点](src/person_search/node.py)、[检测配置](config/detectors) | `tests/test_person_search.py`、`tests/test_registry_evidence.py` |
| C · 深度相机 | [相机包](src/aspire_camera)、[ROS 桥接](src/aspire_camera/gazebo.py) | `tests/test_rgbd_scene.py`、`tests/test_rgbd_registration.py` |
| D · 地图与场景 | [房间定义](tools/scenarios/room.json)、[世界生成器](tools/scenarios/room.py)、[HM3D 导入](tools/hm3d/prepare.py) | `tests/test_team_boundaries.py`、`tests/test_person_scene.py` |

日常改动从 `main` 开短分支，如 `b/vision-confidence` 或 `d/room-layout`。提交 PR 时附上验证命令和结果。改动话题、坐标、时间戳或场景格式时，同步更新接口说明，并请数据提供方和使用方共同复核。合并前跑一次全套主机测试。[接口与验收](docs/TEAM-INTERFACES.md)

## 复现 Depot 07 联调

在团队的 WSL 运行机上，从仓库根目录执行。资产使用 `/opt/aspire/assets/depot07-20261118`。该目录的 `start.sh` 用于单独查看 Gazebo 世界。联调入口读取同一套资产，保留仓库、人物、R-01 及原始摆位，把配准 RGB-D 和 PX4 所需的改动写入外部运行目录中的场景副本。原始资产仍由资产目录维护。

运行机应已配置 ROS 2 Jazzy、Gazebo Harmonic 8.15、colcon、RViz、Xvfb、x11vnc 和 noVNC。脚本使用现有的 `/opt/aspire/build/ros-ws`、`/opt/aspire/build/px4-sitl`、`/planner/external/agent_install`、分析及分割 Python 环境、DDS 配置、RF-DETR-Seg Nano 权重和已校验的 x86 Gazebo 退出保护。WSL 的 Windows I 盘启动时需有至少 6.4 GB 可用空间。[联调环境与验收](docs/REPRODUCIBILITY.md)

首次构建或原生代码变更后，在仓库外构建原生 RACER。再次运行可以复用同一提交的构建。

```sh
source /opt/ros/jazzy/setup.bash
source /opt/aspire/build/ros-ws/install/setup.bash
native="$(mktemp -d /opt/aspire/build/search-lab-native.XXXXXX)"
CMAKE_BUILD_PARALLEL_LEVEL=2 bash tools/search/build.sh "$native" --execution-safety --consistent-inflation
```

```sh
run="/opt/aspire/evidence/depot07-$(date +%Y%m%d-%H%M%S)"
bash tools/flight/run_depot07.sh /opt/aspire/assets/depot07-20261118 "$native" "$run"
```

常规启动以 60 Hz 生成配准的 RGB 和深度图像，识别节点默认按 60 Hz 采样新图，最小间隔约为 16.67 毫秒。两者均按仿真时钟计算。60 Hz 场景副本将五个物理步对齐到一个相机周期。默认仿真实时因子为 0.42，实际墙钟吞吐取决于仿真速度和识别负载。[运行机的 Gazebo 时钟修复](docs/REPRODUCIBILITY.md)

可分别用 `ASPIRE_CAMERA_FPS` 和 `ASPIRE_MIN_RGB_PERIOD_S` 覆盖相机频率和最小采样间隔。复现此前关闭采样限速的仿真配置时，可以使用以下启动参数。

```sh
ASPIRE_MIN_RGB_PERIOD_S=0 ASPIRE_SIM_REAL_TIME_FACTOR=1 \
  bash tools/flight/run_depot07.sh /opt/aspire/assets/depot07-20261118 "$native" "$run"
```

参数 `ASPIRE_MAP_PROJECTION_STRIDE` 可调整深度建图的像素步长（默认 8）。相机频率写入 `scene/scene-audit.json`。逐人掩膜话题以及计算实际吞吐与延迟的字段见[联调接口](docs/TEAM-INTERFACES.md)。

Depot 07 默认在 CUDA GPU 上运行 RF-DETR-Seg Nano，使用 CUDA Graph、FP32 和 batch 1。团队运行机的权重位于 `/opt/aspire/assets/rf-detr-seg-nano.pt`。已验证环境为 Python 3.12、`rfdetr==1.12.1`、`torch==2.11.0+cu128` 和 `torchvision==0.26.0+cu128`。其他安装位置可用 `ASPIRE_DETECTOR_PYTHON_PATH` 指定。人员候选阈值取自 Depot 07 的 625 张校准图（0.2918）。识别节点将逐人可见掩膜交给现有的跟踪、RGB-D 定位和注册表。相机帧率与采样间隔由上面的两个参数独立控制。

用 `ASPIRE_DETECTOR=yolo11n-seg` 可复跑 YOLO。已有显式指定 `ASPIRE_SEG_WEIGHTS` 或 `ASPIRE_DETECTOR_MANIFEST` 的 YOLO 命令仍可直接使用。若 RF-DETR 官方权重放在其他目录，同时传入 `ASPIRE_DETECTOR=rfdetr-seg-nano` 和 `ASPIRE_SEG_WEIGHTS`。

默认传入 `--rviz`，运行 180 仿真秒。运行期间在运行机上打开 [RViz 浏览器视图](http://127.0.0.1:18680/vnc.html?autoconnect=true&resize=scale)。远程观看时通过已有 SSH 连接将本机端口转发到运行机的 `127.0.0.1:18680`。

启动日志在 `$run/launch.log`，生成的场景在 `$run/scene/`，退出码在 `$run/exit-code.txt`。验收查看 `$run/run/result.json`、`telemetry.jsonl` 和 `flight/0/events.jsonl`，核对场景加载、RGB-D 配对、人员观测进入原生注册表，以及轨迹激活和到达后的新观测。

### Depot 07 人员掩膜评测场景

可从原始 Depot 07 资产生成独立的相机评测场景。南北两区各布置一名倒地人员和一名箱体后的站立人员。相机路径含 25 段扫拍，每段 125 个位姿，前五段用于校准，后 20 段用于测试。

```sh
PYTHONPATH=src python3 -m tools.scenarios.depot07_labelled \
  --assets /opt/aspire/assets/depot07-20261118 --output /tmp/depot07-labelled-scene
```

在配置好的 Gazebo 环境中设置 `GZ_SIM_RESOURCE_PATH` 为资产包的 `models` 目录。`scene.sdf` 内的相机提供 `/labelled/rgbd/image` 和 `/labelled/rgbd/depth_image`。评测器按相同时间戳读取 `/labelled/truth/labels_map`。该图的标签 11、21 对应倒地人员，12、22 对应站立人员，0 对应背景。`scene-truth.json` 提供模型摆位和用于采集的相机轨迹。使用遮挡比例分组时需要结合当帧可见面积及人物独立投影面积。

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
  --segmentation-weights "$ASPIRE_SEG_WEIGHTS" --bearing-bias --rviz
```

`--external` 指向运行机上已有的 PX4 和 Agent 构建依赖目录。仓库自带模型可供离线生成场景。上述组合接入人员观测注册表及方位引导。主动静态复查的成对实验另有运行设置和评分协议。[配对验收](docs/STATIC-EVALUATION.md)

直接运行飞行入口时，切换 YOLO 权重可同时传入 `--segmentation-weights` 和 `--detector-manifest`。可参考现有清单，按新模型填写权重 SHA-256、运行库版本、人员类别 ID 和输入尺寸。模型须返回与原始 RGB 图像对齐的人员分割掩膜。省略清单时使用原有的权重校验。运行记录会保存显式传入的清单 SHA-256。[清单示例](config/detectors/yolo11n-seg-locked.json)

## 开发验证

```sh
python3 -m pytest -q -p no:cacheprovider tests
python3 tools/racer_ros2/verify.py external/racer_ros2_ws
```

原生协议测试使用 C++ 编译器和 Boost 头文件。若 macOS 的 Boost 安装在 Homebrew 的 include 目录，可先运行 `export CPATH="/opt/homebrew/include${CPATH:+:$CPATH}"`。缺少 ROS、指定录制或运行环境的测试会分别跳过。

代码来源和依赖见 [原 ASPIRE 提交](https://github.com/Wu-Chenjie/ASPIRE/tree/8892f4643ce740129a5691f6f6514363d82e06cb)、[依赖清单](dependencies.lock.json)和[第三方许可证](licenses)。早期实验批次、阶段报告和专用复盘脚本见 [精简前快照](https://github.com/Wu-Chenjie/ASPIRE-Search-Lab/tree/5c886f12039a18d6bafed0d6c93f9953454c30a1)。
