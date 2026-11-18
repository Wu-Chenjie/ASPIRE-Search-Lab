# 静态事件重规划源码验收（2026-10-20）

默认关闭。当前交付是源码连接和可执行宿主合同，不是ROS编译或运动中C2换轨验证、
真实飞行或发现数收益证明。此前原200ms安全故障未修复；禁止新仿真放行。

S8已补默认关闭的发布后取消链路，见[取消协议](STATIC-CANCELLATION.md)；
以下原S4–S6流程保留；S9续接的C2窗口、独立执行监视和终态回执以
[移动C2协议](STATIC-MOVING-C2.md)为准。实际接受释放计算槽，执行监视另等终态。

## 当前连接

运行配置 → hello → configuration → `search.static_event_replan`，只有原
execution-safety构建包含overlay。开启还要求单机、原安全模块、
authorized execution、stop_and_plan、target_registry、bearing_bias、
static_event_audit，以及显式正有限的static_event_min_interval_s。
间隔参数尚无独立标定值，测试中的0.5秒仅为合同输入，不是部署建议。
所有现有配置默认false；原速度/加速度/膨胀/安全阈值未改。

事件回调保留资格 → 全局唯一在途工作 → 新鲜反馈/里程计及切换间隔 →
运动时提前原生候选评估（无发布、无消费选择预算） → PLAN_TRAJ请求 →
原stop-and-plan/恢复/返航门 → 原生完整规划 → 事件/地图/前驱/反馈版本复核 →
同次发布边界元数据及B-spline → 原外部安全授权 → 匹配接受/拒绝反馈释放资源。

IDLE没有保证已初始化的探索目标，直接唤醒原生规划器建立基线，不用旧ED目标
做预评估。重复事件只保留一项，正常探索阶段不重复评估全部cue；事件成本
计算限定获准实体。预评估与实际规划是两个不同阶段，后者从当前地图/起点
重算，不能复用未经校验的旧轨迹。

发布前失效方案回队列（仅仍有效的事件），不替换executor拥有的轨迹。
边界元数据也延迟到发布检查后发送，避免传输层积累未配对元数据。
提交之后仍等待原executor的明确反馈，不能把本地publish当已执行。
若提交后确认状态发生变化，本层不能自行取消已发布/已授权轨迹；不会因此
重置执行状态或绕开原停止机制。这一时间窗需要原生取消/终止协议独立验收。

## 本机已具备的可执行检查

```sh
cd /Users/wuchenjie/Documents/Codex/2026-10-20/task-2/ASPIRE
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -ra -p no:cacheprovider tests/test_static_replan.py tests/test_native_static_events.py
```

测试编译真正的StaticReplanGate/StaticGuidanceEvents，以及真正的
static_replan_methods.cpp；后一项ROS时钟/原生规划接口使用宿主替身，
只能证明调度和状态恢复，不能证明真实路径、轨迹、ROS线程或安全授权。
另生成完整overlay并检查门控位置。完整JSON解析器因缺Boost而跳过。

## 恢复原生验证的精确条件与命令

必须已有可用的ROS2/Boost/原生依赖、当前代码可访问，且已批准该执行位置。
本任务不安装依赖、启动旧VM、上传代码或写I盘。下列是以后原生验证的命令，
本轮没有执行；输出目录必须不存在，且先盘点空间和预计构建增量。

```sh
python3 tools/search/build_native.py ../native-static-replan-validation --execution-safety --consistent-inflation
cd ../native-static-replan-validation
colcon build --packages-select racer_interfaces racer_core --cmake-args -DCMAKE_BUILD_TYPE=Release
```

原生验收必须记录实际配置、源码提交、依赖版本和日志SHA；逐项检查：

1. 真实Python JSON（含support_policy）进入ROS注册表，随后产生一次事件。
2. 同实体连续帧、别名、弱帧穿插，仅一份pending、一份在途；记录solver峰值。
3. 正在执行旧轨迹时，新资格的preview时间早于旧轨迹完成，且没有提前发布。
4. 默认stop-and-plan仍要求原认证停止；可选C2只经显式开关/正lead/取消session
   和接收、实际切换双重复核。原生验证前不启用，不得把stop-and-plan设false绕过。
5. 确认、身份拆分、frame变化、地图版本更新、反馈过期、前驱变化发生于规划
   或发布之间时，旧结果不发布，原执行保持；没有孤立boundary缓存增长。
6. 发布后必须出现原外部安全授权与匹配execution feedback，才计为执行和释放
   计算槽。延迟/丢失接受反馈不能开第二项计算。检查执行中计算新线索、峰值1、
   旧任务确认失效、先回执/先非活动、连续两次换轨及晚到取消精确终态。
7. 返航、原no-path恢复、人工review优先级不被事件覆盖；预评估异常必须留下
   明确失败证据，不算成功观察。
8. 原200ms安全预算在有新事件负载下独立测量。新频率可能增加CPU争用，
   宿主收件箱微基准不能替代路径评估/优化/安全检查耗时。
9. 全量原生pytest执行后逐项列出skip；不能把缺依赖当通过。

## 尚可本地推进的工作及真实验证差距

- 独立审查仍待只读子代理授权或主线程安排；作者复核不计独立审查。
- map revision目前随每次融合导出递增，空/不变帧也会使方案保守失效，可能
  造成重复重算或饥饿；需实际速率测量后决定局部地图版本合同，不能删门避免失败。
- 第四阶段已在event_driven_static分支绕开旧DiscoveryMemory年龄/分组/次数门，
  不按时间衰减或永久次数封禁，不在排序前用1.5倍硬门删候选。旧模式完全保留。
  静态模式只受获准收件箱实体范围约束，再比较全部已计算安全候选的非负成本。
  这不是已标定的搜索收益模型。现已加显式任务预算准入，边界见下文。
- 表面区域可见性变化已源码接入事件队列；独立几何变化触发、预算变化唤醒和完整人体可见性仍未实现。
- 保守深度可见性仍缺真实capture-time校准来源验证；未知空间不能用于反证。
- 确认/身份评估、冻结回放、独立FP上限和最终配对搜索实验仍待完成。


## 静态成本分支的合同边界

static_event_replan启用时才设置event_driven_static。时间戳保持原值，未来帧
仍拒绝；没有重写capture伪装新观测。资格及确认/身份/frame版本由收件箱控制，
静态证据不用旧20秒first年龄或2秒当前帧TTL来判不存在。无新事件不会重复触发。

保留原生ViewNode非负路径成本，安全不可达候选仍拒绝；只在已评估候选上比较
cost*(1-discount*utility)，且静态模式要求0<=discount<1。原Astar/优化器的
代价和剪枝没有改，不向它们注入负奖励。当前discount仍是既有0.5启发式，
不代表独立验证通过。native cost包含距离/速度、方向变化、偏航代价，不能直接
当作已认证飞行时长。当前发布前另用实际B-spline时长检查任务余额；观察/切换/
返回所需预留仍须独立验证，不能删除安全门弥补预算模型不足。

合成合同证明：旧模式拒绝的1.6倍安全成本候选在高相对效用时可赢；3倍成本
或继续探索本身具有同等观察价值时仍继续探索。它们是代数合同，不是实验收益。
64槽旧cue缓存满时，获准的第65实体使用临时单实体范围评分，结束后恢复原缓存，
身份仍由注册表保留；不会因cue缓存把一个事件误删为不存在。


## 第五阶段：弱证据、可见性和预算接线

全部保持显式启用。`static_event_replan`除原安全条件外，现在要求
`static_guidance_budget`对象：`duration_s`、`reserve_time_s`、`observation_s`、
`initial_energy_j`、`reserve_energy_j`、`power_upper_w`和`model_id`。
这里不提供部署数值；合同测试值仅为合成输入。时钟从原FSM实际接受任务触发时
开始一次，全实体共用，不在发现目标/重获/重规划时重置。开始计算前检查余额，
发布前以实际生成轨迹getTimeSum检查：

- elapsed + trajectory + observation + time reserve <= duration（秒）；
- (elapsed + trajectory + observation) * power upper + energy reserve <= initial energy（焦耳）。

该功率上界必须来自独立验证。它是新增引导动作的保守准入模型，不是电池遥测，
不控制基线的全任务终止/紧急返航，不能声称已认证整机续航。切换开销、原执行
安全协议可能追加的动作及返航必须计入经验证的预留；现阶段没有该验证数据。
原始200ms门、速度/加速度与停止/授权门不变。

`static_weak_enabled`默认false；启用须正有限的`static_weak_extra_cost`
（原生启发式代价单位）、`static_weak_extra_path_m`（米）、
`static_weak_trajectory_s`（秒）。前三项中的额外成本与路径相对继续探索比较，
最后一项约束实际生成轨迹时长；不把原生代价当秒。单帧事件升级为多帧资格会
失效旧计算并合并同实体pending，全局在途上限仍1；没有另起并行solver。

`static_visibility_config`和`static_visibility_model_id`可从prepared runtime
经tools/flight/run.py传入感知节点。config必须显式包含相机/成像验证值：
min_range_m、max_range_m、max_capture_age_s、min_pixels、depth_margin_m，
可指定max_pixels。无配置默认关闭。只有已校验注册RGB-D和捕获时刻TF才生成消息。
每帧最多64区域，轮转避免后部身份长期饥饿，共享像素扫描上限；超过预算保留unknown。
轮转使用dict迭代，遍历成本随历史记录位置增长，未声称常数复杂度或硬实时。

区域来自历史可见表面及其不确定度，不是人体中心或完整人体包围盒。
clear只表示该表面区域在有效成像条件下有整片后方深度；未知/无深度/前景遮挡
不计有效区域漏检。原生端复核身份、捕获时刻、frame版本、锚点位置和尺寸，
同帧同实体阳性优先。区域漏检只计审计，不删除静态位置/身份或撤销确认。
clear/occluded状态转换可重开一项保留资格，重复同状态不逐帧重算；已确认仍无主动动作。
缺少有效历史表面或只有方位射线的目标不会获得伪造三维可见性。

`tools/search/evaluate_static_claim.py`为离线声明门，协议和配对完整结果分开输入。
false_positive_cap仍null；本阶段不得生成正式收益结论。详见STATIC-EVALUATION.md。
上述新增接口通过宿主合同不等于原生ROS、Boost JSON解析和PX4执行验证。
