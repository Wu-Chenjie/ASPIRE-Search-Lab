# 冻结外部工程层

这里保存 Stage45 基础源码之外、最终比较实际用到的工程层及其直接支持文件。Stage68 是当前冻结入口；Stage65–67 相关文件用于前缀继承、原反例和回归测试，不是多个同时活动的系统。

脚本保持原样和原哈希，历史硬编码路径尚未迁移成通用启动器。旧 batch_driver/run_one 和 flight-entry 不能直接用于重开已结束的实验；本次整理不执行这些入口。

无飞行测试及机器依赖见 [复现说明](../docs/REPRODUCIBILITY.md)，实际运行文件对应见 [来源清单](../docs/provenance/measurement-files.json)。真值审计只用于独立事后评测。
