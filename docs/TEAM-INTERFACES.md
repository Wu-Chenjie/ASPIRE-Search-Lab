# 四人联调接口

在线信息流为 C → A/B、B → A、A → B。D 的场景由 A 的启动程序交给 Gazebo。A 的规划输入来自实测建图、里程计和人员观测。人物真值供结束后的独立评测使用。

| 方向 | 数据与入口 | 关键约定 |
| --- | --- | --- |
| D → A | JSON 地图或由它生成的目录 | 地图包含 bounds、starts、person_pose、boxes。位置和尺寸用米，人物姿态为 x/y/z/roll/pitch/yaw，角度用弧度。--layout 选择起点，--negative 生成同地图无人对照。准备好的包由 --prepared 接收。 |
| C → B、A | /drone_0/{rgb,depth}/{image,camera_info} | sensor_msgs/Image 和 CameraInfo，传感器 QoS。光学坐标为右、下、前。默认 Gazebo 深度为 32FC1，单位米。直接取同像素深度需通过采集时刻、标定、分辨率和外参检查。缺少有效深度时仍可保留 RGB 线索。 |
| B → A | /uav0/search/target_observations，std_msgs/String 中的 JSON | registry 模式使用 schema 1。observations 行含 robot、session、counter、sequence、capture_s、visual_state、localization_state、has_bearing、has_position。位置或方位仅在相应标志有效时消费。位置指可见表面。一个来源 ID 可在多次观测中出现。 |
| B → A | /search/bearing_evidence，std_msgs/Float64MultiArray | 方位引导使用已有 v1 定长 17 项数值布局，包含采集时间、原点、方向、角范围及支持帧数。Python 编码位于 aspire_interfaces/bearing.py。 |
| A → B | /drone_0/view_execution，std_msgs/String 中的 JSON | 执行节点报告活动任务、实际到达及观测证据；B 以捕获时间判断是否为到达后的新证据。历史复查模式另用 /search/review_goal、/search/review_status 和 /search/review_cancel。 |

B 在 `/search/semantic_candidates` 中随每个采集时间戳发布 `received_wall_s`（节点接收该 RGB 帧时的单调墙钟秒数）和 `first_semantic_latency_wall_s`（接收到首次发布语义的墙钟耗时）。同一帧补发深度或状态时保留首发值。吞吐诊断中的 `rgb_ingest_dropped_total` 只计开始处理前因队列满而被挤出的 RGB 帧。

B 在 `/search/person_masks`（`std_msgs/msg/String`，reliable，深度 10）上为分割节点接受的每张新图发布一次 JSON。消息含 `schema_version=1`、原始 RGB 的 `stamp_ns`、光学坐标系 `frame_id`、`width`、`height` 和 `instances`。每个实例有帧内唯一的 `instance_id`、置信度 `confidence`、局部跟踪编号 `track_id`（仅 registry / decoupled 有值），以及 `mask_png_base64`。Base64 解码后是原 RGB 尺寸的单通道 8 位 PNG，前景为 255，背景为 0。不同实例可以重叠。无人员时发布空数组，深度补发和跟踪定时器均不重发掩膜。

在 `/search/perception_timing` 中按 `status=person_masks_published` 选择发布记录。`mask_publish_latency_wall_s` 从节点接收 RGB 入队计到掩膜 ROS `publish()` 返回，包含等待、推理、实例关联、编码和发布调用。`mask_published_wall_s` 与 `received_wall_s` 使用同一单调墙钟。按 RGB 采集时间戳去重，可用相邻首末发布之间的帧间隔数除以墙钟跨度计算吞吐。

Depot 07 的 `registry` + `--bearing-bias` 联调中，局部人体通过现有 RGB 多帧确认后，B 立即发布 `visual_confirmed` 事件和人员观测。缺少可靠坐标时，A 暂停派发新的探索轨迹，让已授权航段完成（有界等待上限 4.0 秒）；B 在观测位继续接收配准 RGB-D，并优先处理可用的配对帧。A 收到该静态人员或其已关联来源的可靠可见表面坐标后恢复探索；若超过 4.0 秒仍未定位，A 释放该源的阻塞并恢复探索，目标本身保持待定位直至后续有效观测到达或任务结束。

启用 `ASPIRE_FRONTIER_STEPS=1` 的单机停稳探索可在 `/baseline/feedback_0` 的普通授权反馈后收到一条 6 项参考位姿消息，顺序为原生航迹 ID、执行报告时间、x、y、z、yaw。首次起步用 ID `-1`。授权网关仅在执行器就绪、已到达、无待执行任务、参考导数为零且实际位姿处于跟踪管（0.25 米）内时转发。RACER 核对航迹 ID、报告和里程计时效及当前跟踪误差后使用参考位姿起步。新轨迹仍通过原有双图和停车授权。重型逐帧源码归档、timing/certificate/perception/RGB-D 调试落盘需通过 `--record-debug-evidence` 显式开启。旧接收方可继续使用普通授权反馈。

- **A** 负责现有原生规划集成、执行器和启动程序。准备好的 D 场景在本次运行目录内使用。对比方案时保留起点、预算和检测配置。
- **B** 在 registry 模式发布识别和定位证据。Python 语义处理可以独立导入；运行 ROS 节点需 ROS 环境和检测依赖。单独修改识别时使用 tests/test_person_search.py 和 tests/test_registry_evidence.py。
- **C** 修改相机模型、配准、光学坐标或安装位置后，用 tests/test_rgbd_scene.py、tests/test_rgbd_registration.py 检查。允许跨时刻配准的配置需满足注册模块的条件。
- **D** 在同一张图中准备起点可见、受遮挡及无人对照。地图 JSON 的 bounds 是搜索边界，boxes 构成可见且可碰撞的场景几何。检查生成的 scene.sdf 和 evaluation-truth.json，再把场景包交给 A。

验收建议采用一个遮挡实例。D 核验遮挡与可见区域，C 核验图像和深度配准，B 对同一来源输出带采集时刻的观测，A 按实际授权与到达反馈检查复查结果。主机交接回归见 tests/test_team_boundaries.py，仿真入口的进程替身测试见 tests/test_flight_entry_supervision.py。

共享话题见 [topics.py](../src/aspire_interfaces/topics.py)。人员观测的生产者见 [registry_evidence.py](../src/person_search/registry_evidence.py)，原生解码器见 [target_protocol.hpp](../tools/flight/target_protocol.hpp)。原生代码使用 C++ 接口，修改字段时两端需一起运行 [注册表测试](../tests/test_target_registry.py)。
