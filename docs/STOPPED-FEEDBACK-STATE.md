# 停止反馈与原生FSM状态解耦

2026-10-18，来源Stage38 V2 P01-B。原生前沿已覆盖可使EXEC_TRAJ先进入PLAN_TRAJ；stop-and-plan仍等旧执行完成。终端观察超时按原1.5秒关闭，并在原6秒窗口交回原生规划，但旧active=-1只在EXEC_TRAJ清理executed_id，PLAN状态会永久被旧ID阻挡。

新overlay仅对已拥有executed_id的有效新鲜active<0反馈清理旧ID、done内部停止标志和静态根；只在EXEC_TRAJ时转PLAN，其他状态保持。没有把失败观察标成成功，没有新目标、覆盖奖励或真值输入；消息长度/时效过滤、.6m净空、地图/跟踪/控制、1.5秒观察、6秒授权、3次失败预算保持。

旧FSM实际对象加保存字段的原生反例失败，新同一反例须解除等待并到空地图原生IDLE；过期停止反馈仍拒绝，无旧ID的重复反馈不唤醒IDLE/WAIT_TRIGGER。完整回归/新门控/比较结果以[证据](../../task-3-evidence/stage-39-stopped-feedback-state/)为准。
