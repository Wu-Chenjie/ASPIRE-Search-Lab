# 验证、依赖与复现边界

## 共同源码单元测试

在仓库根目录、已有依赖环境执行：

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests
```

`requirements.txt` 列出主机依赖，`requirements-segmentation.lock` 列出可选分割依赖。缺失 ROS/Gazebo 依赖导致的跳过项必须单列。此命令不会启动新的搜索仿真。

## 外部层无飞行测试

下面选择可独立执行的捕获审计与原子发布测试；不运行原飞行入口测试中的机器固定路径，不把这个子集当作全栈验收：

```sh
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD:$PWD/src:$PWD/src/annual_planner/src:$PWD/experiments/stage65/scripts:$PWD/experiments/stage66/scripts:$PWD/experiments/stage68/scripts" \
python3 -m pytest -q -p no:cacheprovider \
  experiments/stage65/tests/test_stage65_passive.py \
  experiments/stage65/tests/test_stage65_capture_audit.py \
  experiments/stage65/tests/test_stage65_rejection_replay.py \
  experiments/stage66/tests/test_stage66_capture_audit.py \
  experiments/stage67/tests/test_monitor_inventory.py \
  experiments/stage68/tests/test_stage68_publication.py
```

## 仿真栈

原生 ROS2 RACER 在 `external/racer_ros2_ws/src/racer_core/`；本机环境为 ROS2 Jazzy、Gazebo Harmonic 8.15.0、PX4 SITL、RViz。锁定信息见 `dependencies.lock.json`；Stage68 实际使用的额外二进制、权重和测量依赖哈希见 [PREFLIGHT](results/stage68/PREFLIGHT.json) 与 [入口清单](results/stage68/ENTRY-MANIFEST.json)。历史锁定文件不自动代表最终全部外部运行层。

`experiments/` 保存原始源码以便审计：Stage49 原生公开请求、Stage53/54 元数据与精确公开输出构建、Stage58 观测、Stage62 有界公开反馈、Stage64 ULog 审计、Stage65/66 捕获与握手、Stage67/68 监督和原子发布。没有采用 Stage47 改变积分语义的子步原型。Stage68 的规划/控制数学及入口原守卫 AST 等价记录保留。

这些脚本原样包含历史 `/tmp`、容器和外盘路径，以及只允许执行一次的旧批次 ID。它们是冻结实验源码，**不是可直接重启的演示命令**。本次未把原批次改造成跨机器一键启动器。重新运行需要新的协议与 ID、实际 Linux 容器、已核验依赖、代码树外输出目录及完整前检；不能通过改路径绕过已关闭批次的停止约定。

权重、PX4 构建产物、容器/VM 镜像、原始 RGB-D/ULog/PAX 和完整场景生成结果均未上传。必要静态模型已实体复制。场景真值与离线审计不得接入在线决策。

## 来源与证据

- [Stage45 原冻结清单](provenance/stage45-frozen-source.json)：599 项全部在整理前核对相同。仓库入口文档及 Git 忽略规则单独更新；算法、ROS 接口和测试保持原内容。
- [外部层当前文件](provenance/measurement-files.json)：八个实际入口/测量文件与冻结 SHA-256 相符。
- [导入文件清单](provenance/imported-files.json)：外部源码、历史说明和结果的原路径、大小、哈希。
- `docs/results/` 是独立评测摘要，包含失败和中止。原报告相对链接中未随仓库收录的详细证据仍在本机原归档；见 [归档索引](provenance/LOCAL-ARCHIVES.md)。

原始恢复镜像、工作 shadow、容器实例与服务不由本仓库管理，本次未停启或改动它们。
