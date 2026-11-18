# 同一请求的有界原生复查（stage25，默认关闭）

`review_continuation_overlay.py` 只修改生成构建中的原生 FSM 调度与回执；不修改锁定 RACER 规划器、地图值、原净空、运动限制或执行授权。原来的 `active_review` 一次消费逻辑在新开关关闭时保留。

## 根因与衔接

此前 `task_review_pending_` 在首次规划时无条件清除。原生 `planTrajToView()` 可只生成通向视点的一个分段；段末的普通探索分支依赖几何前沿，因而可能转入 IDLE 或规划其他前沿，同一视点没有续段依据。

新模式保留请求身份、固定目标和总预算。每段均调用原生 `planTrajToView()`，并继续经过原执行器的最新地图、完整路径、动力学、净空和制动授权。只有新鲜且 ID 匹配的执行完成回执，结合新鲜实际里程计，才消费该段完成：实际位置距原请求点 `<=0.25m` 时报告 `arrived`；否则最新原生地图中的端点仍为 FREE、无膨胀且预算可用，才以同一请求进入下一段。几何前沿为空不会独自丢失已接受的复查。

`arrived` 只说明到达复查视点；人员确认仍需感知端检查新的视觉证据，不能把执行状态或参考轨迹当作人体证据。真实到达后保留已完成执行的悬停至多 5 秒，阻止新规划以等待新图像；该窗口受原请求总预算限制，不重置预算，也不修改执行安全控制。

## 参数与边界

| ROS 参数 | 默认 | 含义 |
|---|---:|---|
| `search.review_continuation` | `false` | 显式启用，且要求 `active_review`、`stop_and_plan`、授权执行 |
| `search.review_max_segments` | 3 | 单请求最多原生规划段；重复请求不补充预算 |
| `search.review_max_duration_s` | 60 | 从接受请求起的仿真时钟总预算；等待也计时 |
| `search.review_max_path_m` | 6 | 由该请求执行和到达后观察段的实际里程计增量累计；新段曲线路长不得超剩余预算 |
| `search.review_observation_hold_s` | 5 | 真实到达后的观察窗口，允许 0–30 秒；总请求预算优先 |

参数通过运行配置、`live_ros2` 的 hello 和 `configuration.py` 传入原生节点。新段经过原有可行性缩放后的时长还必须落在剩余时间内。段数允许范围为 1–20、时长为 (0,300] 秒、路径为 (0,30] 米；无效配置失败退出。观察窗口到期自动释放，成功证据回执可提前释放；返回、执行改变或实际位置离开请求点 0.25 米范围则退出。

预算是继续规划的上限，不是瞬时撤控命令。取消或耗尽发生在已发布/已授权段内时，禁止补段并保留原安全执行，等匹配的安全完成回执后结束。跟踪误差、里程计采样和等待完成可产生路径/时长超额，应在评测中如实报告。授权/完成回执中断时不伪造完成，也不越过旧反馈新鲜度门槛；仍由原执行安全逻辑处理，原生可能保持 `cancel_pending`，直到安全完成或执行丢失被实际报告。

## 请求与回执

请求仍为 `/search/review_goal` 的 `PoseStamped`，frame 为 `map`；`header.stamp` 的整数纳秒作为请求 ID。请求年龄最多 1 秒、位姿有效、目标点当前 FREE 且无膨胀才接受。同一活动 ID 的重复消息不修改目标、时刻或预算；另一个活动请求拒绝为 busy；已完成 ID 的重放拒绝。新请求 ID 在当前原生节点会话内必须递增。

`/search/review_status` 为 `std_msgs/String` JSON；原生日志同时记录 `TASK_REVIEW_STATUS {json}`。字段为：

- `request_id`：十进制整数字符串，避免浮点丢精度。
- `native_plan_id`、`segment_index`：当前段原生计划 ID、从 1 开始的段号。
- `state`、`reason`、`time`：状态、原因、仿真时间。
- `goal`、`distance_m`、`path_used_m`：固定请求坐标、最新实际位置距离、请求已执行路径；无有效里程计时距离为 `null`。
- `max_segments`、`max_duration_s`、`max_path_m`、`observation_hold_s`：本请求的实际配置。

通常序列为 `accepted → planning → planned → executing → segment_completed → continuing`，到达时报告 `arrived`；随后有限观察窗口释放时报告 `released`，或以失败状态结束。`planned` 仅在本段全部原生出站检查通过后报告；`executing` 只由实际授权回执产生。状态更新时间、plan ID 或任意新 token 都不能单独代替同一请求证据关联。

`released` 表示到达后观察保持已结束，不是人员确认状态。失败终态包括 `cancelled`、`timeout`、`unreachable`、`invalid`、`budget_exhausted`、`rejected`。`cancel_pending` 表示等待原安全段结束，不是终态。原生规划失败、执行授权拒绝、目标当前变为未知/占据/膨胀、返回中、时钟异常和执行丢失都有独立原因。

取消话题 `/search/review_cancel` 为 `std_msgs/String`，严格的两个字符串字段 JSON：`{"request_id":"123456789","reason":"target_invalid"}`，字段顺序可交换。错误 ID 或无效 JSON 被忽略。观察窗口内成功理由 `active_multiview_confirmed` 或 `reliable_direct_confirmed` 的对应取消报告 `released`，其余取消报告 `cancelled`。目标失效、直接确认等取消理由由感知端给出；原生层不读取人体真值，也不猜测人体是否仍存在。

## 验证范围

`tests/review_continuation_test.cpp` 编译生产使用的同一生命周期逻辑，覆盖多段身份保持、预算耗尽、错误/过期完成、取消和精确整数协议。`tests/test_review_continuation.py` 检查默认兼容、参数传递和构建 overlay 组合，并在 ROS 环境运行 `task_review_continuation_probe`。

原生 probe 使用真实生成 FSM 的回调和未观测地图，覆盖新鲜实际到达、严格 0.25 米门槛、未知端点拒绝、延迟取消，以及上一段残留 `live_sent_` 不得把未发布的新段困在等待中；还验证 5 秒观察保持、原总预算截断、成功证据/返回释放。其生命周期/里程计是测试输入，不是飞行成功证据。真实同请求多段到达与后续人体新证据仍须以独立 Gazebo/PX4 运行记录核验。
