# 人员分割帧率与点云投影优化报告

分支 `perf/rgb-throughput-and-projection-stride`（基于 `af2960b`）。所有数字均为本机实测，
CPU-only（`torch 2.5.1+cpu`、`ultralytics 8.3.228`、`yolo11n-seg.pt`，权重 sha256 与
`config/detectors/yolo11n-seg-locked.json` 一致）。

## 结论摘要

1. **纯推理是主要瓶颈。** 640 输入 P50 124.6 ms、320 输入 P50 34.0 ms，占单帧耗时的 96–99%；
   30 FPS 的 33 ms 单帧预算只有 320 勉强接近，本机 CPU 上任何配置都跑不满 30 FPS。
2. **放开采样周期是最有效的一步。** 640 从 4.70 FPS 提到 7.72 FPS（+64%），320 达到 18.96 FPS
   （旧采样率下只有 15.7% 的帧被推理，现在为 63.2%）。
3. **画证据图是第二大开销。** 在 320 上叠加画框+填掩膜把吞吐从 18.96 拉到 14.44 FPS（−24%），
   建议把调试图移到独立线程或按需开关。
4. **点云覆盖能力只由投影步长决定。** 可稳定保留的最薄障碍 ≈ `stride × 距离 / fx`；
   `640×480 + stride 8` 与 `320×240 + stride 4` 覆盖能力完全相同、建图耗时接近，因此推荐
   **维持 320×240 分辨率、把步长 8 降到 4**，不必先动分辨率。

## B. 人员分割帧率

### B1. 测量方法

- 纯推理：`tools/search/benchmark_detector.py`，本地帧、预热 2 帧、取 P50/P95/P99。
- 采样回路：`tools/search/benchmark_presence_loop.py`，30 FPS 输入连续压测 60 s 墙钟
  （1800 帧），走与 registry 节点相同的
  `select_presence_capture` → `discard_superseded_rgb` → `rgb_pixels` → `detect` 路径，
  计时覆盖解码/格式转换/推理/发布。
- 两个已知边界：本机无 ROS/DDS，端到端延迟是"收到 RGB → 发出语义"的进程内等价测量，
  不含 DDS 传输；本机 `torch.cuda.is_available()==False`，CUDA/FP16 项未测。
- 帧计数口径：`rgb_received_total = rgb_inferred_total + rgb_skipped_total + rgb_ingest_dropped_total`
  在 5 组实验中均严格闭合（误差 ≤1 帧，来自压测窗口首尾）。

### B2. 纯推理基准（固定权重，仅改输入尺寸）

| 输入尺寸 | 推理 P50 | P95 | P99 | 推理 FPS |
|---|---|---|---|---|
| 640 | 124.6 ms | 133.5 ms | 136.7 ms | 8.10 |
| 416 | 65.4 ms | 70.5 ms | 71.2 ms | 17.59 |
| 320 | 34.0 ms | 54.4 ms | 54.5 ms | 25.39 |

### B3. 60 s 原速压测

| 配置 | 完成推理 | 完成率 | 跳过帧 | 入队丢弃 | 推理 FPS | 推理 P50/P95 | 端到端 P95 |
|---|---|---|---|---|---|---|---|
| 640，period=0.2（旧默认） | 282 | 15.7% | 1516 | 0 | 4.70 | 125.3/135.9 | 284.4 ms |
| 640，period=0（放开） | 464 | 25.8% | 1335 | 0 | 7.72 | 125.8/136.7 | 312.2 ms |
| 640，period=0 + 画证据图 | 403 | 22.4% | 1397 | 0 | 6.70 | 125.1/133.9 | 332.7 ms |
| 320，period=0 | 1138 | 63.2% | 662 | 0 | 18.96 | 52.2/54.5 | 235.1 ms |
| 320，period=0 + 画证据图 | 867 | 48.2% | 932 | 0 | 14.44 | 53.3/55.1 | 250.1 ms |

- **最快可用配置**：320 输入 + `min_rgb_period_s:=0`，18.96 FPS、63.2% 完成率，比旧默认
  （640/0.2，4.70 FPS）快 4.0 倍。
- 端到端 P95 都在 235–333 ms，因为推理线程串行，排队时间本身就是一帧以上的推理耗时；
  要压到 33 ms 量级必须靠 GPU 或更小输入，靠改采样策略做不到。
- 放开采样后 `rgb_ingest_dropped_total` 为 0：积压由"喜新厌旧"在选帧时立即退休，不再
  撑满 40 帧队列上限。上一版（只有放开周期、没有退休策略）此值高达 1114/1800。

### B4. 改动内容

- `src/person_search/presence_adapter.py`：新增 `discard_superseded_rgb(node,ns)`，选帧后立即
  退休比它更早、且 tracker 没观测过的 RGB 帧并计入 `rgb_skipped_total`；已观测帧保留到深度
  补算窗口（`max_age`）之外再静默退休，避免重复计数。
- `src/person_search/node.py`：新增 `rgb_skipped_total` / `rgb_ingest_dropped_total` 计数；
  `timing()` 输出这两个字段；`publish_semantics()` 输出 `received_wall_s` 与
  `first_semantic_latency_wall_s`（节点收到 RGB → 首次发出语义）。
- `tools/flight/run.py`：新增 `--min-rgb-period-s`（默认 0.2）与 `--map-projection-stride`
  （默认 8），带范围校验，分别透传到 `person_search` 与 `pointcloud_mapping_node`。

## C. 点云投影

### C1. 改动内容

- `src/annual_planner/src/annual_runtime/pointcloud_mapping_node.py`：`projection_stride`
  参数化（默认 8，越界告警回退 8）。
- `src/aspire_camera/depth_projection.py`：`_project_depth` 对非法 stride 抛
  `ValueError('Invalid depth projection stride')`，避免静默出坏点云。

### C2. 对照实验（离线合成，可复算）

`artifacts/agent-runs/declaration/stride_sweep.py` 生成已知场景（4 m 平墙 + 一条已知物理宽度
的竖条障碍，取最坏像素相位），记录每个配置能稳定保留的最薄障碍与单帧 `VoxelMap.integrate()`
耗时。两种分辨率按同一 ~60° 水平视场取内参，因此 fx 分别为 277 / 554。

| 分辨率 | 距离 | stride 8 | stride 4 | stride 2 |
|---|---|---|---|---|
| 320×240 | 1.5 m | 43.3 mm / 4.3 ms | 21.7 mm / 15.8 ms | 10.8 mm / 61.4 ms |
| 320×240 | 4 m | 115.5 mm / 3.9 ms | 57.7 mm / 15.2 ms | 28.9 mm / 61.3 ms |
| 640×480 | 1.5 m | 21.7 mm / 15.0 ms | 10.8 mm / 64.6 ms | 5.4 mm / 261.8 ms |
| 640×480 | 4 m | 57.7 mm / 15.9 ms | 28.9 mm / 77.8 ms | 14.4 mm / 331.9 ms |

（单元格 = 该配置能稳定保留的最薄障碍；斜杠后为单帧建图耗时）

- 步长减半，可保留的最薄障碍也减半；提高分辨率的效果与减半步长等价
  （`640 + stride 8` ≡ `320 + stride 4`），但前者要多 4 倍深度数据，并且要重新对齐
  RGB / 深度 / CameraInfo / TF（`rgbd_alignment.require_registered_pair` 要求四者严格一致）。
- 代价：320 上 stride 8→4 使单帧建图从 3.9 ms 升到 15.2 ms；mapper 以 0.35 s 为周期整合，
  即约 4% 的占空比，可以接受。
- **推荐**：320×240 → stride 4。只有在需要 < 3 cm 级细节时再考虑升分辨率到 640×480 + stride 4
  （77.8 ms/帧，约 22% 占空比）。

### C3. 未能覆盖的验收项

合成场景真值精确，墙面距离误差为 0；真实数据下的墙面 P95 误差、有效 RGB-D 配对率、优化前后
同视角点云对比图都需要 ROS 2 + 真实 rosbag，本机不具备（无 ROS 2 Jazzy 环境、无实机数据）。
这些项留给仿真/实机验证，代码侧只保证：步长可配、非法值显式报错、分辨率变动时
`require_registered_pair` 会拒绝未对齐的四件套。

## 复现

```bash
# 纯推理基准
python tools/search/benchmark_detector.py --manifest config/detectors/yolo11n-seg-locked.json \
  --weights yolo11n-seg.pt --images frames --input-size 320 --repeats 3 --output runs/det.json

# 60 s 采样压测
python tools/search/benchmark_presence_loop.py --manifest config/detectors/yolo11n-seg-locked.json \
  --weights yolo11n-seg.pt --images frames --input-fps 30 --seconds 60 \
  --min-rgb-period-s 0 --input-size 320 --output runs/loop.json

# 步长/分辨率扫描
python artifacts/agent-runs/declaration/stride_sweep.py
```
