# 静态引导取消协议 v1 与可选 C2 终态 · 默认关闭

源码连接已实现：原生FSM→DDS请求→授权代理→执行器→取消回执→原生反馈核对。
宿主合同验证协议及安全回调分支；没有完整ROS/Boost编译、真实DDS时序或闭环验证。
原GPU故障尚未修复，本功能不放行新实验。

## 三个阶段

| 取消时刻 | 行为 | 资源释放条件 |
|---|---|---|
| 尚未发布 | S7作废提案，恢复原目标/原执行轨迹；优先任务拥有目标时不覆盖 | 本地未提交事务已结束 |
| 已发布但授权代理尚未发送命令 | 撤销pending或为晚到提案保留墓碑，拒绝以后激活 | 精确终态回执，加原生反馈中该提案不活动 |
| 命令已发送或已执行 | 转交执行器；未激活则墓碑拒绝，已执行则等待原曲线的已认证静止端点 | 执行器终态回执，加原生反馈中该提案不活动 |

执行中取消不调用现有通用stop/cancel：该接口会清空曲线、重设参考点，不符合
本协议的连续性要求。此版本保持原曲线继续受原执行安全检查；到固定端点后，
通过原certified_stopped_reference、原地图年龄/跟踪/200ms计算和证据写入门，
才置arrived并发送retired_at_safe_rest。曲线、token和参考点仍保留，原静止保持
检查继续运行。observation_proof清空，不给观察/发现信用；失败和安全故障不发终态成功。
这不是立即停止或主动提前刹停。可选移动C2已另接入，见 STATIC-MOVING-C2.md；
只有实际安全切换成功后才可将原任务记为 superseded_by_c2，否则仍可能完成原安全轨迹。

## 身份与消息

task包含session、task（调度work token）、proposal（原生trajectory ID）和map_revision。
session为每次运行独立生成的48bit正整数十进制字符串，只作命名空间，不是鉴权凭据。
同一proposal不能更换task/map版本。取消是不可逆revision=1；重试完全相同。

- 边界元数据原4字段不变，静态取消启用时追加session、task、map_revision。
- `/baseline/static_cancel_N`：Float64MultiArray
  `[1, session, task, proposal, map_revision, 1]`。
- `/baseline/static_cancel_ack_N`：上述6字段再加状态：
  1等待安全静止，2启动前撤销，3安全静止后退役，4经验证C2替换。全部标识必须可被double精确表示。
- 授权代理→执行器沿既有view_command发
  `{"static_cancel":{"version":1,"task":{...},"revision":1}}`；
  正常静态命令带同一static_task和原executor token。
- 执行器在既有view_execution中附static_cancel_ack；
  终态安全退役含原安全检查证明、checked_at和observation_complete=false。

代理核对session/task/map、已发送executor token、新鲜执行报告及安全保持ready；
迟到、不同任务、不同版本和错误token的结果不能结清事务。重复终态回执保留幂等；
发送过命令的记录不能因本地intent已清空就伪称“从未执行”。
取消单独启用时原生FSM在等待回执期间保留在途槽；C2启用后，实际接受反馈
将任务转入独立执行监视、释放唯一计算槽。发布待接受的提案仍不能被覆盖；EXEC继续
原执行状态处理。回执先于或晚于inactive反馈均可，只有两者齐全才释放。超时不等于取消成功。

每session最多256个协议记录，不逐帧创建。墓碑不自动淘汰，以免迟到命令重新激活；
满额后拒绝新增静态任务，不能通过清空记录恢复。一般探索仍使用原接口。
这是一项保守资源上限，不是经实测确定的最佳容量，也不保证无限时长可用。
新运行必须新session；本协议不提供进程崩溃后的持久恢复或跨session自动接管。
启用时，所有view_command（包括普通取消/扫描）由授权代理添加authorization_session；
执行器在epoch及任何命令副作用之前核对。旧会话即使带更大epoch也不能接管，
缺字段同样拒绝。该命名空间检查不替代DDS鉴权，不实现自动重连或状态恢复。

## 配置与验证

STATIC-SETTINGS.template.json的static_cancel_enabled默认false。两组均采用相同
布尔值；启用只能经显式static-guidance-settings，仍须单机、registry、原execution
safety及stop-and-plan。run.py产生session，并写代码树外static-cancellation-session.json，
同一会话传给授权代理、执行器和原生参数。session不进入共同算法配置哈希。
不手工继承旧session；默认关闭时入口清除继承的该环境变量。

宿主检查：

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -ra -p no:cacheprovider tests/test_static_cancellation.py
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -ra -p no:cacheprovider tests
```

测试真实ledger/gateway、C++事务门、生成后的连接位置，以及提取的真实
check_execution_safety回调；后者使用真实曲线和地图几何，替换ROS、捕获装饰器、
记录I/O与时钟。不证明真实传输、线程、磁盘时延或物理飞行。测试覆盖未知几何、
过期地图、计算超200ms、证据写入超时仍拒绝终态。

明确区分：

1. **代码尚缺**：确认后主动生成提前认证刹停曲线、跨进程崩溃恢复。
   本版保守端点退役不能称这些能力已实现。
2. **代码已接入，原生未验证**：上述取消链路及可选移动C2/终态回执、消息乱序、保持/取消与原反馈、
   长时间满额行为及原真实执行器的安全时序。恢复已有ROS/Boost环境后应按
   STATIC-REPLAN-ACCEPTANCE.md建新目录，逐项验证；不启动原冻结批。
3. **参数待独立数据**：导航/弱动作/预算/可见性/事件间隔与误报上限；不影响本地
   协议实现，但仍阻止部署参数冻结和收益声明。
