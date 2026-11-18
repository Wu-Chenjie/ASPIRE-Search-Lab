# 可选候选边与保守人员对应（Stage37）

2026-10-17 UTC，仅离线修复版本，从task-3-stage36d1独立建立；本版本未飞行或新跑仿真。唯一项目记忆见[共享入口](../../task-3/PROJECT_MEMORY.md)。[完整结果](../../task-3-evidence/stage-37-association-eligible-edges/RESULTS.md)与[最小反例](../../task-3-evidence/stage-37-association-eligible-edges/ROOT-CAUSE.md)为本轮证据。

`tools/flight/target_registry.hpp`只调整`associate`候选图入边：距离须同时满足原几何门和严格小于`.6m`不匹配成本，再参与双向唯一性。共现、坐标版本、当前定位TTL、历史静止锚、既有alias重验、确认与人数接口原样保留。两个合法候选仍保持歧义，无XYZ/仅方位不产生身份别名；历史锚不当成当前可导航定位。

原D1固定896批确认假设末10→2，独立保存标签下重复8→0/已知错合并0，仍16批短暂重复、峰值额外2组。旧B1的561批组件图全部不变、末4组/重复2，无XYZ仍未知；同帧检测实例不保证不同真人。算法仍输出独立真实人数null，仅在证据有效条件下给已确认集合范围。本次结果不承诺运动/检测闭环或搜索收益，也没有已累积任意组件的一般合并算法。

新增`tests/association_eligibility_probe.cpp`与pytest编译驱动覆盖成本/门/双向竞争、近邻双人、重现、无XYZ、乱序与失败预算。全宿主297/28、新原生prefix下Jazzy426/6，另实际ROS13+4检查通过。复查首次epoch/用量/冷却和全局上限未重置；新增alias合并继承已用预算，无法辨识的源不靠真值绑到某人。

构建和测试临时文件均在代码树外；容器新源码的两个资产软链接映射到原有`/tmp/racer-stage23-src/external/`，宿主仍使用原相对链接，资产不复制修改。冻结D1和原Annual-Planner保持不动。没有新的动态运行授权，不从这里自动启动B1、D1或其它仿真。
