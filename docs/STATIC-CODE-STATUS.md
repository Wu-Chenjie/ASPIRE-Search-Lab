2026-10-21 06:08 UTC：ec3af05开启C2的动力学仿真完成240sim/577.635wall，实际换轨0；后审计确认871次实际检测均无候选，完整C2触发验收未覆盖。IMU主记录审计通过，ULog次级记录有缺样。详见[原生验证](STATIC-NATIVE-VALIDATION.md)。

# 本地静态搜索代码收尾 · 2026-10-20

2026-10-21 04:40 UTC 更新：实际原生优化器暴露并修复移动 C2 软边界缺口；原生 580 通过/7 跳过，Mac 444 通过/36 跳过。原生曲线的真实 DDS 切换/取消通过；唯一 240 秒 GPU 基础闭环无故障，但未开启 C2，旧 200ms 超时仍未解决。完整范围见 [原生验证](STATIC-NATIVE-VALIDATION.md)。
2026-10-21：新增[原生构建、回归和 DDS 有限验证](STATIC-NATIVE-VALIDATION.md)。下文的环境/禁止仿真描述为此前交接状态；本轮按最新用户授权分阶段执行，旧批次不重开。

此分支在独立仓库基于2250c27继续；原冻结仓库、外部RACER源码和旧实验未改。
S9续接补齐默认关闭的[移动 C2 与执行监视](STATIC-MOVING-C2.md)，取消边界见[发布后取消协议](STATIC-CANCELLATION.md)。
默认引导仍关闭。本次完整宿主回归结果和哈希在任务目录evidence/S10-VALIDATION.json；
宿主合同不能代替ROS线程、原生轨迹授权或闭环搜索。

| 已确认需求 | 当前代码与检查 | 尚需验证的边界 |
|---|---|---|
| 导航弱证据与可靠确认分离 | presence/registry_evidence双支持；同一确认代码用于两组 | 导航参数与误报上限独立校准 |
| 实体与任务分别去重 | native注册表组件关联、每实体pending、全局1计算事务；C2接受后转独立执行监视；强快照评分后恢复缓存 | ROS并发、长期关联成本及地图规模 |
| 短暂遮挡不抹除身份 | 静态资格保留、无帧不触发负证据；短暂强弱波动不重复排队 | 真实遮挡重获闭环 |
| 单帧只争取低成本动作 | 默认关闭；开启须额外原生成本/路径/实际轨迹时间三限额 | 无推荐部署值，必须独立验证 |
| 有效可见漏检 | 可选注册RGB-D/捕获TF及历史表面区域；unknown不算无遮挡 | 不是完整人体不可见性证明 |
| 新线索提前重评估 | 执行旧轨迹时先preview；可选C2未来边界、接收/切换复核已接入 | 默认关闭且未原生验证；地图频繁更新可能使版本检查饥饿 |
| 确认后不主动精化 | Python/native组件抑制；未发布失效恢复旧目标；默认关闭的取消链路已接入 | 取消可在原曲线认证端点退役，或实际C2替换后终态结清；提前刹停尚缺，真实链路未验证 |
| 与继续探索比较 | 非负原生成本折扣排序，保留安全路径检查，显式全任务时间/能量准入 | 非整机续航认证；候选效用/参数未校准 |
| 公平基线与计数 | 配对入口只变两个引导开关；独立标注计数及协议门 | 原日志规范化、真实标注与闭环配对尚未取得 |

本轮修复有旧代码失败证据：较新弱帧会阻止加载保留强快照；短暂强弱震荡会
反复失效同一实体计算。新增测试直接编译相关C++头和方法，保留原失败日志。
取消未发布方案现在恢复旧导航目标；返航/避障/恢复拥有目标时不覆盖它。
取消 v1 单独启用时，已提交方案占槽直到原完成或终态与非活动反馈。C2启用后，
匹配接受反馈释放计算槽并转执行监视；监视独立等待完成/终态，不以本地取消假装停止。

## 配置准备（不启动实验）

复制STATIC-SETTINGS.template.json到代码树外的新文件，用独立验证值填写。
null是刻意阻断，不能直接用于部署。weak关闭时不填限额；
开启时添加static_weak_trajectory_s、static_weak_extra_cost、
static_weak_extra_path_m三项正有限值。visibility关闭时保留null，开启时按
STATIC-REPLAN-ACCEPTANCE.md填写成像配置及model_id。间隔和预算始终必填。
模型ID及参数必须有实际验证证据，不可仅用字符串声称已验证。

输入runtime须来自同一已准备场景，单机、confirmation_mode=registry、
stop_and_plan=true、execution_safety=true；两组模型与资源设置相同。

```sh
python3 tools/search/prepare_static_pair.py \
  --runtime /absolute/shared-scene/runtime.json \
  --settings /absolute/validated-static-settings.json \
  --output /absolute/new-pair-configs
```

只写两个配置文件；拒绝已存在输出目录。common_runtime_sha256排除两个引导
开关和arm元数据，其余字段均参与；生成后的实际运行配置还应重新哈希和冻结。
生成配置不冻结协议、不证明资源相同，也不授权实验。

实际飞行入口现支持`--static-guidance-settings FILE`，A不带`--bearing-bias`，
B带该选项；两组均需`--prepared SHARED_SCENE --confirmation-mode registry
--detector yolo11n-seg --segmentation-weights EXISTING_WEIGHTS --stop-and-plan
--budget-clock sim --seconds VALIDATED_DURATION`及原`RACER_EXECUTION_SAFETY=1`。
原入口所需PX4、场景、依赖和原安全门仍全部适用。此处不是可直接启动的批次命令：
原安全故障未修复、容量未满足且协议未冻结，当前禁止启动。

## 验证顺序

1. 仓库根目录执行
   `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -ra -p no:cacheprovider tests`。
2. 具备已有ROS/Boost环境后，按STATIC-REPLAN-ACCEPTANCE.md生成新的代码树外
   native验证目录并colcon构建；禁止覆盖旧目录或安装大环境来跳过当前限制。
3. 原生事件→preview→未来边界/原安全停止→规划→授权→实际切换→匹配反馈→
   计算槽释放与终态回执逐项通过。缺真实接口和执行证据，不能只靠宿主替身宣称完成。
4. 独立验证参数、误报上限与协议；连接、逐卷容量、旧进程闭合及原200ms
   故障修复均证实后，才可讨论新ID闭环配对。离线回放不是反事实收益。

本轮未访问Windows、启动仿真、安装、上传、推送、清理数据或启动独立审查代理。

本次三类剩余工作及精确运行边界见 STATIC-MOVING-C2.md；主动提前刹停/崩溃恢复未扩展，
C2源码完成但未原生验证，参数及误报上限仍待独立数据。
