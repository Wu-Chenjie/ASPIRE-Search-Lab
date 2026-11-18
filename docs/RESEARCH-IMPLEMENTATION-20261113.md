# 研究方案接入 · 2026-11-13

基线：`2a9a0d52fa5e7a045bfbd4cb94aecaf3ac6f3753`，`improve/static-guidance`。实现分支：`implement/research-roadmap-20261113`。工作位于独立 Git worktree，原移交副本和历史实验保持原样。

用户已将本轮范围明确为“代码接入与离线验证”；没有目标域数据、微调权重或 Jetson 资源。本次没有启动训练、历史仿真、飞行或远端推送。附件中的原型只作参考，未整体覆盖生产定位模块。

## 分阶段交付

| 阶段 | 已实现代码 | 验证范围与边界 |
| --- | --- | --- |
| 1. 保留前端信息 | `presence.py` 的可见掩膜去重与关联、0.8 秒 LOST 唯一恢复、4 秒工作缓存回收；单调源 ID；原生活跃源集合与任务内身份账本分离 | 相同框的两个不相交掩膜保留且跨帧接续；漏检打断尚未完成的连续确认；140 次过期来源仍可接收。视觉源数仍不等于真实人数 |
| 2. 状态与几何 | 正交语义/身份/生命周期字段、当前位置空值规则、双时间重投影入口、局部主轴表面门、分项质量及校准版本传输 | 保留原有腐蚀、深度分量、紧凑窗口与多帧融合；合成水平支撑通过，歧义/不足支撑拒绝；协方差明确未标定 |
| 3. 身份与复查 | 分坐标版本的空间网格候选；保留双向唯一、同帧互斥和撤销；原生组件级次数/冷却/证据版本记账；轨迹发布前预留时间与航程 | 旧方位不会因新的可见性事件重新变为有效；合并后预算合并，拆分不会退还预算；不为已确认者单独改善精度而绕行 |
| 4. 人体与痕迹 | DetectorSpec 与可选 CUDA/TensorRT 入口、可见掩膜遮挡增强、数据划分审计、独立痕迹检测入口/记忆/SearchHint | 未获得或训练模型；新痕迹输出默认关闭。方向主轴保留正负歧义，形成时间保持未知，痕迹不计入人数 |
| 5. 多机与机载 | 原始观测 ID、map epoch/clock domain、显式跨机变换及误差/有效期准入；单 worker、batch=1、可配置 RGB 周期与有界队列；独立任务租约 API | 变换需要外部估计并提供。租约模块完成单权威状态机和精确停止回执，尚未接入跨机 DDS 执行交接。未测 Jetson 或分布式飞行 |

## 主要接口

### 活跃缓存和事实账本

Python 的 `tracks`/`at(id)` 只访问活跃工作记录；过期已确认事实进入 `confirmed_history`，`report()` 同时展示这两部分，归档项无当前位置。`RegistryEvidence` 的融合与临时来源缓存随工作记录清理。旧帧索引在回收时清除，不可能指向复用 ID。

C++ 的 `active_sources` 决定 128/256 活跃容量；`records` 是任务内事实/来源账本。过期后释放多帧 history，保留确认、锚点、alias、cannot-link、sequence/revision 和任务引用端点。它不是跨进程数据库：账本和原始观测去重索引仍随任务增长，未实现无限时长的磁盘归档。这样避免通过删除关系端点来换取活跃名额。

短时恢复仍是局部视觉关联，不是全局身份认证。已知相机旋转被补偿；LOST 恢复遇到超过 0.10 m 的已知平移时拒绝纯图像接续，留给原生空间关联。无位姿时只提供保守的图像唯一关联。低置信候选可保留，但不能增加高置信连续确认次数。

### RGB-D

默认 `rgbd_registration_mode=strict` 保留原来已配准输入验证。选择 `reproject` 后：

- `rgbd_image_mode=rectified` 使用 P；`raw_pinhole` 只接受无畸变的 K。
- 16UC1 按毫米、32FC1 按米解码，处理步长、大小端与无效值。
- 按两幅图像的采集时间和对应 CameraInfo 配对；分别记录 RGB/深度时间、skew、calibration ID。
- `lookup_transform_full(target_frame, target_time, source_frame, source_time, 'map')` 查询跨时刻相机运动，再重投影并保留最近支持表面。洞保持无效。
- `rgbd_max_skew_s` 默认 0，上限 0.05；异时输入还要求显式 `rgbd_static_scene=true`。这不补偿人体运动或遮挡变化。
- 原始畸变、ROI/binning、非单位矫正 R、带平移项的立体 P 继续拒绝，须由相机驱动先输出约定清楚的光学帧。没有把缩放图像当作配准。
- 表面输出仍是可见支撑的代表点；中位数不保证落在某个采样点，也不表示人体中心。

质量经 Python 与 C++ 协议传递：支撑点数、有效比例、分量比例、深度跨度、RGB/深度时间差、calibration ID、`covariance_calibrated=false`。共有误差不会按像素数或帧数缩小，像素取整误差也进入代理协方差。

### 复查

`StaticGuidanceEvents` 继续是生产调度入口。默认组件最多 3 次、任务最多 32 次、冷却 0.25 秒；组件路程预留上限 6 m、时间预留上限 30 秒。时间包括观测等待；航程采用 B 样条控制多边形长度作保守预留。原生可达性、碰撞、执行授权与任务时间/能量守卫仍要全部通过。

同一源序列与可见性版本不能重复预留。竞争任务造成的同一次求解最多延期重试一次；撤销不返还已预留预算。合并后的成员共享历史预算，拆分后保守继承。已确认目标的一般主动引导继续取消。

后续修订将延期重试绑定到原预留的目标源、观测序列和可见性版本。强证据升级或新的有效可见性事件需要重新检查冷却与预算。

`StaticReviewBudget` 区分存在确认、身份消歧、痕迹核查与精度任务，拒绝精度任务。当前生产队列仍只生成已有的存在确认任务；新身份消歧和痕迹提示不会自行转化为运动任务。

### 检测器与数据工具

`detector_manifest` 为空时保留官方权重 SHA 与 CPU 两线程路径；非空时按 `DetectorSpec` 校验本地文件 SHA、精确运行时版本、设备、精度、类别映射、输入尺寸和可见 mask 输出合同。适配器固定针对 Ultralytics 8.3.228，不能任填最新版本。

后续修订在 `decoupled` 和 `registry` 模式中使用 `iou=1.0`，让每帧至多 64 个候选进入可见掩膜去重。深度定位仅处理去重后进入跟踪的候选，证据文件保留原始检测索引。此模式要求导出模型提供原始候选，拒绝运行时标记为内置 NMS 的模型。

`config/detectors/yolo11n-seg-locked.json` 是显式锁定配置示例。微调后填写真实 SHA 与版本；TensorRT 还必须记录 CUDA、计算能力、batch、尺寸和精度，INT8 需要目标域校准数据的 SHA。元数据校验不能替代在目标设备运行 engine。

`min_rgb_period_s` 默认 0.2，可配置为 0；`input_queue_limit` 默认 40，可设 2–128。采集年龄守卫、单推理 worker、最新可用帧选择和 batch=1 保留；把周期设为 0 只取消准入节流，不承诺吞吐提升。

```sh
python3 tools/search/audit_person_dataset.py /absolute/path/annotations.json
python3 tools/search/benchmark_detector.py --manifest /absolute/path/model.json \
  --weights /absolute/path/model.pt --images /absolute/path/images \
  --max-frames 100 --output /absolute/path/out/detector-timing.json
```

数据清单是一组 JSON 记录，包含 `observation_id`、`capture_group`、`split`、`source`、`license` 和 `instances`。同一采集组不得跨训练/验证/测试集；实例主类是 person，姿态与遮挡为属性，mask_type 必须为 visible。`partial_human_hint` 是待确认数据标签，不直接作为人数。遮挡增强会同步扣掉被遮住的 mask；实际训练集仍需人工审查与目标域补标。

基准工具只测本地图片解码与推理，排除初始化 warm-up，记录 P50/P95/P99；不是 ROS 端到端或飞行时延。

### 痕迹与多机

配置 `trace_detector_manifest` 和 `trace_weights` 才启用独立痕迹分割器，且要求 registry 模式。输出位于 `/<robot>/search/trace_hints`，人体注册表不消费它。可见支撑经保守空间唯一匹配进入痕迹记忆，歧义保留不同线索；空间阈值尚未实景标定。重复 observation_id 幂等，冲突内容拒绝；新观测不刷新形成时间。

`search.peer_alignment_json` 在原生 ROS 注册表构造时装载一份外部估计：`evidence_id`、`robot`、`source_version`、`target_version`、`source_clock`、`target_clock`、`clock_offset_s`、`clock_error_s`、`position_error_m`、`rotation_error_rad`、`valid_from_s`、`valid_until_s`、`tx/ty/tz`、`r00` 至 `r22`。旋转必须刚性，时钟误差上限 20 ms；没有依据、版本不符或有效期结束就不能保留跨机几何关联。变换误差扩入表面半径，过大时只保留语义事实。记录是准入输入，不是地图对齐估计器，也不是对估计真实性的自动证明。

`review_lease.py` 提供单权威 grant/renew/revoke/acknowledge/reconcile API：到期先要求撤销，收到原 owner 对精确 token 的停止回执前不得转授。它尚未替代原执行器的授权/取消协议，也未实现多权威共识、网络分区恢复或自动接管。

## 复现与验收

修改前，附件清单的 57 个 Git blob SHA 全部吻合；原四个反例复现；主机基线 464 passed / 40 skipped。

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests
PYTHONDONTWRITEBYTECODE=1 python3 tools/search/verify_research_roadmap.py \
  --output /absolute/path/out/regression-summary.json
```

本机 Boost 位于 `/opt/homebrew/include`，测试时通过 `CPATH=/opt/homebrew/include` 使用已有头文件，使真实 JSON 协议测试参与执行。构建和测试产物在代码树外。最终结果记录在本节末尾。

本轮全量结果：**485 passed / 37 skipped，53.69 秒**；真实 JSON 解析、原生注册表、生成工程头文件编译和新增反例均参与验证。新增 18 项测试；相对基线另有 3 项原先缺 Boost 的测试恢复执行。随后补强远端原始输入校验与图像关联快速筛选，执行受影响的定向回归。

最终受影响回归：**35 passed，17.00 秒**。变更文件的 Python AST/JSON 语法和 `git diff --check` 通过。

仍需后续资源验收：ROS2 Jazzy 完整容器编译/通信、真实 RGB-D/相机模式、目标域标注和隔离测试集、微调模型效果、TensorRT 实机兼容与端到端时延、真实地图/时钟估计、跨机租约传输与执行闭合。当前 Docker daemon 无法连接，ROS/Gazebo/PX4 相关跳过项不能算通过。

## 官方实现依据

深度单位和 Z 语义采用 [REP-118](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst)。双时间查询核对了 [ROS2 Jazzy Buffer 源码](https://github.com/ros2/geometry2/blob/jazzy/tf2_ros_py/tf2_ros/buffer.py)。检测参数对照 [Ultralytics 8.3.228 predictor](https://github.com/ultralytics/ultralytics/blob/v8.3.228/ultralytics/engine/predictor.py)，没有按最新文档更换项目运行时。
