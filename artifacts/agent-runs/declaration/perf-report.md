# 人员分割帧率与点云投影优化报告

分支 `perf/rgb-throughput-and-projection-stride`（基于 `af2960b`）。B 节为原作者的本地基准，
CPU-only（`torch 2.5.1+cpu`、`ultralytics 8.3.228`、`yolo11n-seg.pt`，权重 sha256 与
`config/detectors/yolo11n-seg-locked.json` 一致）。

## 结论摘要

1. **纯推理是主要瓶颈。** 640 输入 P50 124.6 ms、320 输入 P50 34.0 ms，占单帧耗时的 96–99%；
   30 FPS 的 33 ms 单帧预算只有 320 勉强接近，本机 CPU 上任何配置都跑不满 30 FPS。
2. **放开采样周期是最有效的一步。** 640 从 4.70 FPS 提到 7.72 FPS（+64%），320 达到 18.96 FPS
   （旧采样率下只有 15.7% 的帧被推理，现在为 63.2%）。
3. **画证据图是第二大开销。** 在 320 上叠加画框+填掩膜把吞吐从 18.96 拉到 14.44 FPS（−24%），
   建议把调试图移到独立线程或按需开关。
4. **步长 4 提高了同视角的表面采样密度。** 在 Depot 07 同一张 320×240 深度图上，
   步长 8 和 4 分别产生 258 和 1,049 个量程内的表面点。C 节给出带无杆对照的合成测量。


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
  是逐帧核对的目标。窗口结束时仍待处理的帧需单独计入。下表 B3 保留原始测量。

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
  入队丢弃只计开始处理前被挤出的帧。各模式的语义消息都带接收墙钟时间及首发延迟，同一帧补发时沿用首发值。
- `tools/flight/run.py`：新增 `--min-rgb-period-s`（默认 0.2）与 `--map-projection-stride`
  （默认 8），带范围校验，分别透传到 `person_search` 与 `pointcloud_mapping_node`。

## C. 点云投影

### C1. 改动内容

- `src/annual_planner/src/annual_runtime/pointcloud_mapping_node.py`：`projection_stride`
  参数化（默认 8，越界告警回退 8）。
- `src/aspire_camera/depth_projection.py`：`_project_depth` 对非法 stride 抛
  `ValueError('Invalid depth projection stride')`，避免静默出坏点云。

### C2. 无杆对照与薄杆占据

合成场景使用 4.4 m 背景墙，竖杆深度分别为 1.5 m 和 4 m。每组先积分一张无杆图，
再在相同内参和采样相位下增加杆宽。杆与墙位于不同的前向体素层。下表给出首次在杆深度层
产生占据格的杆宽，以及不同正杆宽下单次 `VoxelMap.integrate()` 的墙钟时间中位数。
测量使用 0.3 m 体素和 4.5 m 径向量程。杆的左边缘位于采样列的下一像素。

| 分辨率 | 杆深度 | 步长 8 | 步长 4 | 步长 2 |
|---|---|---|---|---|
| 320×240 | 1.5 m | 43.3 mm / 2.3 ms | 21.7 mm / 8.4 ms | 10.8 mm / 33.1 ms |
| 320×240 | 4 m | 115.5 mm / 2.3 ms | 57.7 mm / 8.4 ms | 28.9 mm / 32.7 ms |
| 640×480 | 1.5 m | 21.7 mm / 8.2 ms | 10.8 mm / 33.1 ms | 5.4 mm / 125.7 ms |
| 640×480 | 4 m | 57.7 mm / 8.2 ms | 28.9 mm / 31.9 ms | 14.4 mm / 131.0 ms |

12 组无杆对照在杆深度层均产生 0 个占据格。墙面前向深度误差只统计落在墙面且在量程内的命中点，
所有组合的 P95 均小于 1 µm。该数值反映理想浮点深度输入。积分耗时取自本次 macOS 主机复测。

### C3. Depot 07 闭环观测

在同一场景各运行 180 仿真秒。原作者的默认组合（640 输入、0.2 s、步长 8）接收 1,376 帧，
其中 688 帧完成推理及有效 RGB-D 配准。实验组合（320 输入、0 s、步长 4）对应 1,382 / 1,382 帧。
同一来源首次视觉确认并与人物真值匹配的时刻分别为 67.25 s 和 97.50 s。两组同时调整了三项参数，
各飞行一次。实机墙面误差仍需带参考距离的实测数据。

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
