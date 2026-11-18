# 心跳停止与 GPU 检查（阶段12历史诊断）

用户随后授权的心跳修复见[阶段13](HEARTBEAT-FIX.md)，GPU继续延后。

未修改生产代码、安全阈值、门控或执行架构；未安装推理栈、启动新服务、上传或新增飞行。隔离实验使用真实PX4Bridge类，消息/时钟受控，Node和发布器替身不连接DDS，不能称飞行复现。

## 已记录的触发链

第二轮ULog在84.060 s记录`offboard_control_signal_lost=1`，84.072 s记录`vehicle_status.failsafe=1`并从Offboard转Auto Land；短暂恢复后84.640 s再次进入Auto Land。不是runner单独报错。runner当时没有锁存故障自动退出，后续人工SIGINT停止，结果`KeyboardInterrupt`。

桥接的`yaw_aligned()`同时要求对齐旗标为真、标志仿真年龄≤2 s、墙钟接收年龄<3 s。`local_position()`在此条件不满足时将`position=None`并返回，即使本次输入位置有效且很新。`tick()`随后因同一门控失败，在发送心跳前返回，原因显示为`awaiting_heading_alignment`。因此日志里的“位姿变旧”可由桥接拒收新位置造成，不能据此说PX4估计停止。

PX4源码`EKF2::PublishStatusFlags()`约每1 s仿真时间发布一次，状态变化立即发布。80–85 s的ULog实测标志间隔中位数1.006 s、最大1.008 s；姿态/位置间隔中位数分别4/8 ms，最大12/16 ms，对齐旗标持续为真。DDS配置没有另设该标志限速。

桥接使用`BEST_EFFORT/KEEP_LAST depth=1/VOLATILE`、独立进程的单线程executor；心跳定时器是20 ms的STEADY_TIME，消息年龄还使用ROS仿真钟及Python monotonic墙钟。YOLO在另一进程、两线程executor中，不能直接占住桥接的Python回调，但可争用CPU/内存资源。QoS可能丢样本，第二轮未记录完整接收序列，不能排除。

最后一次心跳空档83.560→84.056 s占墙钟2.58648072 s。按起点同戳的**最后一次**发布精确截取，期间tick仍运行129次，最大间隔24.948 ms，并接受24次轨迹参考；排除这整段executor完全阻塞。初步统计139次包括了同戳更早的正常回调，以129为准。独立记录节点两次标志接收间隔约5.556 s，但不是桥接的精确接收时间。

## 隔离时钟实验

已比较第二轮源码快照与当前诊断类，`yaw_aligned/local_position/tick/estimator_status/vehicle_attitude/vehicle_status`六个方法AST一致。每组400次tick；不改变原门槛。

| 受控输入 | 实测结果 |
|---|---|
| 实时倍率1，标志每1 s仿真时间，姿态/位置/参考持续新鲜 | 0次停发 |
| 实时倍率0.2，同上 | 101次停发；墙钟3.000 s时标志仅老0.600 s仿真时间，唯一失败的heading条件是墙钟年龄；新位置被拒收 |
| 实时倍率0.2，仅在诊断中改为每0.5 s墙钟注入新标志 | 0次停发；这是机制对照，不是已部署修复 |
| 仿真暂停且所有输入停止 | 墙钟0.5 s因输入过期停发，原暂停保护有效 |

**已证明低频仿真消息与固定墙钟门槛之间存在不匹配。** 它与第二轮周期性现象一致；第二轮未记录桥接flags接收年龄/回调起止，无法事后唯一确定每次丢样本和调度延迟的贡献。第三轮减轻RViz后成功是缓解证据，不足以确定RViz或YOLO为唯一负载根因。

建议的最小后续修复候选：先补齐每次门控失败的各输入双时钟年龄和具体失败项；在隔离SITL分支提高**PX4真实状态快照**的发布频率（例如10 Hz仿真时间）并保留现有桥接/飞控门槛，再验证慢速、暂停、断流、乱序和状态转坏。不能把缓存旗标重新盖新时间戳、也不能直接删除3 s检查。更广泛的仿真时钟适配应单独设计；本轮未实施上述改动。

## GPU 实测与未测试范围

| 环境 | 检查结果 |
|---|---|
| 当前Colima Linux aarch64容器，torch 2.5.1 | CUDA版本为空、available=false、device_count=0；MPS built=false、available=false；/dev未发现nvidia/dri/kfd/render/gpu设备 |
| MacBook Air，Apple M4，24 GB，10核GPU | Metal硬件支持 |
| 已检查宿主Python 3.13.9与Homebrew 3.14.5 | 均无torch和ultralytics；常见虚拟环境目录及Spotlight未找到可用环境。未安装大栈，未进行CPU/MPS配对基准 |

锁定Ultralytics 8.3.228源码支持MPS，但不可用时会回退CPU；当前任务适配器还显式指定`device='cpu'`。在该Linux容器里仅改为`mps`不能获得Mac GPU。Apple GPU的PyTorch路径是宿主macOS的[MPS后端](https://docs.pytorch.org/docs/stable/notes/mps.html)，硬件支持不等于当前Python环境可用。

最小准备方案是宿主独立arm64 Python环境（优先核对Python3.12与既有torch2.5.1/torchvision0.20.1轮子兼容）、固定Ultralytics8.3.228及图像依赖，复用同一权重SHA。先离线用同一批原始RGB、conf=.35、imgsz=640、retina_masks、FP32做CPU/MPS预热后的短基准，MPS计时前后同步，检查真实模型设备以防静默CPU回退，比较掩膜尺寸/IoU、框、分数及定位结果。没有环境，因此速度、算子兼容和掩膜一致性均未实测。

若离线通过，宿主推理接容器ROS还需要显式的本机数据交接设计、原采集戳、有限队列和过期拒绝；本轮没有搬进程、配置网络或创建服务。GPU可能减轻CPU负载，但不能根治上述时间基准不匹配，也不能保证解决Gazebo/RViz的软件渲染负担。

可复查脚本、JSON数值和哈希清单保存在代码树外`../task-3-evidence/stage-12-diagnostics/`；脚本依赖阶段11保留的本机环境和`tests/test_px4_bridge.py`夹具。原始第二轮ULog仍在阶段11完整归档中。
