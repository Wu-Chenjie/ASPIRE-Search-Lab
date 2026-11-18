# ASPIRE

**面向灾后环境的无人机自主建图与人员搜索系统研究**

基于原生 ROS2 RACER 的单机自主建图与人员搜索研究，采用 ROS2 Jazzy、Gazebo Harmonic 8.15.0、PX4 SITL 和 RViz。当前研究对象为室内静止人员，使用 RGB-D 人体候选、独立定位质量、持久假设与有界方位引导；多机与真实灾后环境验证仍为后续目标。

本仓库为 **Private**，前身为 `aster-search`。2026-10-19 整理版本包含冻结的 **Stage45 共同源码**与 **Stage68 外部输入/取证层**，保留初始 Stage30 Git 历史。原生 RACER 未替换，不把共同工程栈称作未经修改的全栈基线。

## 当前结果

| 项目 | 已核实结果 |
|---|---|
| 固定预算比较 | 六对、12 个主 arm，各 180 仿真秒；另保留 2 次中止 |
| 当前核心与取证门 | 12/12 通过 |
| 全部严格检查 | 9/12 通过；定位表面污染与身份帧未定仍存在 |
| 独立确认人数增益 | 六对 B−A 均为 0，尚无引导组人数收益证据 |
| 关键局限 | 有人场景在启动就绪前已确认两人，不能据此证明隐藏人员搜索优势 |

结果、协议修订和未完成项见 [当前状态](docs/STATUS.md)；原始结论摘录及逐轮数值见 [最终报告](docs/results/stage68/FINAL-REPORT.md)、[比较表](docs/results/stage68/RUNS.csv)。末重复为 0 不等于全过程无重复；表面位置误差不是人体中心误差。GNSS 辅助 SITL 不等于无 GNSS 室内实机定位。

## 目录

- `src/`、`tools/`、`config/`：感知、搜索、执行与仿真工具。
- `external/racer_ros2_ws/`：锁定的原生 ROS2 RACER；其余 `external/` 为必要模型资产。
- `experiments/`：与冻结源码分开的输入、测量、审计层和历史测试；禁止把已关闭批次重新运行。
- `tests/`：共同源码测试。
- `docs/`：状态、复现边界、方法与精简结果；`docs/provenance/` 保存来源哈希。
- `licenses/`、`dependencies.lock.json`：已有许可证和依赖来源，不推定缺失授权。

## 本地验证

在已有依赖环境执行：

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests
```

外部工程层、仿真所需依赖与路径限制见 [复现说明](docs/REPRODUCIBILITY.md)。本仓库整理不新跑仿真；历史测试与本次测试分别列在 [检查记录](docs/REPOSITORY-UPDATE.md)。

构建、权重、运行图像、视频、原始证据和 VM 镜像均保存在代码树外。在线规划与检测不得读取真值，未知空间不能当作自由空间；真值仅用于场景生成和独立评测。不自动追加实验、不运行真实无人机。唯一持续研究记忆仍为本机共享 `task-3/PROJECT_MEMORY.md`，本仓库状态文档是日期明确的交付快照。
