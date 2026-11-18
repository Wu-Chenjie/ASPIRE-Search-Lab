# 返航与临时恢复状态修复（阶段15）

阶段14在临时退回点附近进入FINISH，实际仍距原起点超过3 m。本轮只修复这段状态衔接；原生RACER前沿、A*、轨迹优化、净空、动力学、PX4保护和CPU YOLO配置保持原样。

## 行为

- 首次开始返航时保存原始起点，之后不可被临时退回目标覆盖。
- 临时恢复或截短安全轨迹执行完成后，恢复原始返航目标，继续原生规划。
- 仅匹配的已授权执行段完成、执行反馈与里程计均不超过0.5 s，且实测估计位置距原返航点不超过0.15 m时，进入`returned`和FINISH。计划曲线端点接近目标不构成到达。
- 返航任务累计最多8次规划尝试，间隔至少0.5 s；成功激活临时轨迹也不重置次数。无合格实测目标或预算耗尽分别进入`waiting_return_no_goal`、`waiting_return_budget`，不宣称返航成功。
- 已接受执行丢失时进入`execution_lost_latched`；新鲜消息恢复也不会重新开始本次返航。桥接/执行器的既有failsafe锁存继续生效。

补丁在构建副本应用：`tools/flight/return_policy.hpp`管理任务状态，`return_methods.cpp`核对实际完成，`return_overlay.py`连接原生FSM；`tools/search/build_native.py`调用覆盖层。锁定迁移源码和原Annual-Planner未改。`return_audit.py`只读配对原生状态、执行器观测证明和估计位姿，不读取场景真值。

## 验证

精确编译同一C++策略的测试覆盖正常返航、主目标失败后临时恢复、恢复后继续原目标、持续不可达、曲线到端但机体未到、陈旧/错误段反馈、非有限位姿、故障退役锁存及重复触发不重置8次预算。离线审计另用明确标注的合成日志检查提前FINISH、缺少完成证明和终止后激活；这些测试不冒充动力学飞行。

初始宿主78通过/12跳过，隔离原生149通过/无跳过（54.54 s）。加入独立审计回归后，宿主82通过/12跳过。最终原生153项通过/无跳过（最终复跑50.73 s）。所有测试仍执行原有安全断言。

正常无注入回归完成420.000 s仿真/522.325 s墙钟，50段激活/50次末端观测完成，估计路径50.415 m。没有自然触发返航，明确标为该分支未覆盖；armed期间无Offboard丢失/failsafe，心跳最大墙钟间隔89.385 ms，无新鲜度拒绝快照，ready后0条接触，26进程正常清理。

独立测试构建仅在显式`RACER_FLIGHT_FIXTURE=return-home|return-primary|return-all`时触发：真实运动完成、离开原起点超过1.5 m后请求返航；失败测试还要求至少3个已观察历史根点。`return-primary`只将主返航A*预算设为0，`return-all`对恢复也设为0；执行真实A*超时分支，不伪造返回码、地图或运动。测试不改变正常构建，也不能称为自然复现。

`return-home`实际150 s仿真/187.848 s墙钟：67.272 s触发，67.456 s执行器激活id2，77.04 s完成到达后传感器观测，77.412 s才进入returned/FINISH；当时实测估计位置距原点9.24 mm。累计一次规划，之后观察72.588 s无新激活。独立审计无违规；无Offboard丢失/failsafe/新鲜度拒绝，ready后0接触，26进程清理完成。该距离是对本次估计起点的相对误差，不是真实定位精度。

`return-primary`实际180 s仿真/259.971 s墙钟：101.412 s触发；3次主返航A*超时后，第4次规划使用原有前沿恢复候选，103.460 s收到激活反馈，112.472 s在安全截短段完成后恢复原始返航点。随后再经历3次主返航失败，第8次规划使用历史退回点，114.464 s激活，123.252 s完成并进入waiting_return_budget。终止时距原点2.976 m，之后观察56.748 s无新规划/激活，无FINISH；两段执行完成证明均配对通过。第一次恢复段完成位置未必等于原临时目标，不能误写为原目标到达。无Offboard丢失/failsafe/新鲜度拒绝，ready后0接触，26进程清理完成。

这是受控计算预算故障，不证明场景中原点物理不可达。

`return-all`实际180 s仿真/214.677 s墙钟，把主目标和临时恢复搜索预算均置零，但原生A*先检查近邻到达再检查超时，因此第4次规划仍有一次合法临时恢复，不能称为“全部规划失败”。第8次恢复失败后，87.188 s进入waiting_return_budget，距原点2.654 m，之后观察92.812 s无新激活/FINISH。独立审计无违规、无Offboard丢失/failsafe、ready后0接触，26进程清理完成；有1条position_present保护拒绝快照，等待期最大记录跟踪误差0.1745 m，不能声称所有输入始终有效或悬停误差为零。

返航期间flags断流专项：第3次返航提案实际激活后，74.868–79.180 s仿真期间丢弃42条flags，实际11.9988 s墙钟；位姿/姿态仍接收。2.642 s墙钟后触发flags_wall_age保护，PX4在76.360 s报Offboard丢失、76.372 s进入failsafe、82.252 s落锁；FSM于76.460 s进入execution_lost_latched，无FINISH和后续提案。恢复后接收100条新flags，观察30 s墙钟，0心跳、0重新解锁指令、无armed Offboard。总89.696 s仿真/150.339 s墙钟，27进程正常清理，全部12项故障审计断言通过。测试包装脚本只等待实际返航激活后调用既有断流夹具，不修改保护阈值。

故障降落后2662条接触记录为79.104–89.748 s的一段持续地板接触；首次底部横杆距近水平地板不超过1.85 mm、机体倾斜0.0595°。最后一条缺少同步真值括号，未外推评分。它不是2662次空中碰撞，也不构成完整着陆安全认证。真值仅用于此独立接触判读。

所有运行均无强制杀进程或本轮残留PID。独立进程检查未见活跃的相关仿真进程；容器保留17条更早的僵尸进程记录，未擅自重启容器处理。自然420秒运行没有触发返航，关键返航分支由明确标注的受控触发验证，不能声称自然复现旧600秒轨迹或证明场景完全覆盖。定位仍是仿真GNSS辅助，未扩展主动搜索策略或GPU。

## 复现

使用本机Docker context `colima-racer-gvp`的`annual-px4-research`，工作副本`/tmp/racer-search-task3-stage15-return`，隔离原生构建`/tmp/racer-stage15-native`。固定PX4构建复用`/tmp/racer-stage13-px4/source/build/px4_sitl_default`。

```sh
# 容器内，构建和运行输出必须尚不存在
bash /tmp/racer-stage15-build-native.sh /tmp/NEW_BUILD --map-probe
# 测试构建另用 --flight-fixture，不能替换正常构建
bash /tmp/racer-stage15-tests.sh
bash /tmp/racer-stage15-flight.sh --seconds 420 --wall-watchdog-seconds 1800 --output /tmp/NEW_RUN
RACER_FLIGHT_FIXTURE=return-home bash /tmp/racer-stage15-fixture-flight.sh --seconds 150 --wall-watchdog-seconds 650 --output /tmp/NEW_HOME
RACER_FLIGHT_FIXTURE=return-primary bash /tmp/racer-stage15-fixture-flight.sh --seconds 180 --wall-watchdog-seconds 750 --output /tmp/NEW_PRIMARY
RACER_FLIGHT_FIXTURE=return-all bash /tmp/racer-stage15-fixture-flight.sh --seconds 180 --wall-watchdog-seconds 750 --output /tmp/NEW_ALL
RACER_FLIGHT_FIXTURE=return-home bash /tmp/racer-stage15-return-fault.sh --fault-injection flags --seconds 180 --wall-watchdog-seconds 750 --output /tmp/NEW_FAULT
# 已加载同一脚本中的ROS和Python环境后
python3 -m tools.flight.return_audit /tmp/NEW_RUN
python3 -m tools.flight.heartbeat_audit /tmp/NEW_RUN
python3 -m tools.hm3d.flight_audit /tmp/NEW_RUN
python3 -m tools.flight.contact_audit /tmp/NEW_RUN
```

持久证据目录为`../task-3-evidence/stage-15-return-fix/`；修改前源码及哈希保存在`before/`；正常运行初始源码为`normal-source.tar.gz`。最终源码、原始运行、生成的C++源码/二进制和可读报告独立归档，逐文件哈希见`manifest.json`。所有构建、日志和图像均在代码树外。

原生正常二进制SHA256：`7e34d280d44c3fa71551ad7c59dcd264692a33c2eec306a97aac0057c2eb2f4b`；受控测试二进制：`0c49f504188cad191c9a0ef9f7b33e944c5c7aef250c43fe20b32c7dc0cd9874`。两者共享完全相同的返航策略。固定PX4仍为`c359b7f3fde4678476d984a75b60d0d258146a90b0d1a3ad66e6a5b142c7968c`。
