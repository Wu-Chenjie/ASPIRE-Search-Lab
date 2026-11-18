# ASPIRE Search Lab 工作约定

本仓库保存灾后室内人员搜索仿真工程，保持私有。四人职责和运行入口见 README.md，跨模块协议见 docs/TEAM-INTERFACES.md。

- A 负责原生 ROS 2 RACER 规划、仿真运行和执行反馈。B 负责视觉识别与人员观测。C 负责 RGB-D 相机和配准。D 负责地图、遮挡物、人物布置及场景交付。
- 保留原生 RACER 依赖与第三方许可证。新试验使用新运行目录。精简前的实验批次与结果见 README.md 的归档链接。
- 在线规划和检测只用传感器观测与状态估计。场景真值供生成和独立评测。未知空间保持未知。
- 大体积运行记录、构建产物、虚拟机镜像和权重放代码树外。
- 修改后在现有依赖环境运行 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests。ROS 接口变更另做已配置环境验证。
- 运行机配置和验收方式见 docs/REPRODUCIBILITY.md。修改接口时请交接双方共同查看测试结果。
