# ASPIRE Search Lab 工作约定

本仓库保存灾后室内人员搜索仿真工程，保持私有。四人职责和运行入口见 README.md，跨模块协议见 docs/TEAM-INTERFACES.md。

- A 负责原生 ROS 2 RACER 规划、仿真运行和执行反馈。B 负责视觉识别与人员观测。C 负责 RGB-D 相机和配准。D 负责地图、遮挡物、人物布置及场景交付。
- 保留原生 RACER 依赖与第三方许可证。历史实验批次和历史证据只读，新试验使用新目录。
- 在线规划和检测只用传感器观测与状态估计。场景真值供生成和独立评测。未知空间保持未知。
- 大体积运行记录、构建产物、虚拟机镜像和权重放代码树外。
- 修改后在现有依赖环境运行 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests。ROS 接口变更另做已配置环境验证。
- 项目状态和历史实验边界见 docs/STATUS.md 和 docs/REPRODUCIBILITY.md。仿真闭环、静态回放和实物飞行分别报告。
