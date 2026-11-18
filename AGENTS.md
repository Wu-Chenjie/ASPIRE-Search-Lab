# ASPIRE 工作约定

- 本仓库研究“面向灾后环境的无人机自主建图与人员搜索系统研究”，保持 Private。
- 使用锁定的原生 ROS2 RACER，不引入替代探索规划器。当前共同源码为 Stage45；外部输入/取证层单列 experiments，不冒称全栈未修改。
- 原 Annual-Planner、历史阶段和原始证据只读保留。只保留任务源码、必要模型与许可证、测试和精简文档。
- 构建、日志、运行图像、权重和记录放代码树外；不得复制 VM 镜像或大原件到仓库。
- 规划及检测不得读取人体或障碍真值；真值仅用于场景生成和独立评测。未知空间不可当作自由空间。
- 静态相机、回放、理想运动不能称自主飞行；视觉 ID 不等于独立真实人数，表面点不冒称人体中心。
- 用户已授权 2026-10-19 整理、重命名 ASPIRE 并继续上传到现有私有仓库；不默认授权后续推送、公开发布或实飞。
- 所有旧动态批已关闭，不能执行 experiments 中的原批次入口来重开旧 ID，不自动新跑仿真或清理历史。
- 修改后执行 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests；ROS 接口变更另做本机已有容器验证。
- 先读 docs/STATUS.md 与 docs/REPRODUCIBILITY.md。唯一持久研究记忆为本机 ../task-3/PROJECT_MEMORY.md，完整规划为 ../task-3/docs/RESEARCH-PLAN.md；仓库内状态仅是有日期的交付快照。
