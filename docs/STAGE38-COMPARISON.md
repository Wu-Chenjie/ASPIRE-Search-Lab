# Stage38表面支持与公平RACER对照

2026-10-18。独立于冻结Stage37，深度支持分割边界2→3px（mask_border_px），其余质量/确认/身份/安全门不变；D1/B1只作开发回放。运行时C++算法与Stage37相同；同证据关闭引导夹具增强后建立新的原生安装，两arm用同一二进制。完整回归及新动态以[证据](../../task-3-evidence/stage-38-racer-comparison/)为准。

先单轮双人稳定性门，过门后预设6对12轮。对照为RACER-ROS2＋共同修复＋相同感知，只切bearing_bias。实际prepared深度/RGB30Hz（更正预备15Hz文字），固定场景和seed123；失败停止余批不补跑。详见[执行协议](../../task-3-evidence/stage-38-racer-comparison/PROTOCOL.md)。定位仍是可见表面，算法视觉假设不等于独立真实人数。
