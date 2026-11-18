# Stage30 视觉存在、方位与定位独立记录

日期：2026-10-16 UTC。活动源码task-3-stage30继承stage29 v2。[阶段证据](../../task-3-evidence/stage-30-presence-localization/RESULTS.md)与[预注册协议](../../task-3-evidence/stage-30-presence-localization/protocol.json)单独保存。显式启用decoupled；同版本hybrid/strict/legacy保持原行为。

## 状态契约

| 维度 | 状态与含义 |
|---|---|
| visual_state | person_candidate / visually_confirmed；连续可信RGB的算法判断，不是真值 |
| localization_state | unknown缺证据、unreliable质量拒绝、reliable当前合格、expired超时 |
| bearing_state | unknown / unavailable / fresh / expired；需要采集标定和世界TF，不需要深度 |
| position / xyz | 仅当前定位reliable时非空，含原采集时间、处理时间、frame与quality |
| last_reliable_location_history | 原好点带原时间和history_only=true；不是当前人员位置或导航输入 |

视觉确认：置信度≥.70、至少5个不同采集时间、跨度≥.8秒、支持窗及间隔≤2秒。深度、TF、掩膜/框比例、裁剪与前景代理不能否决RGB存在判断。实际RGB漏检或低分清空近期支持；已经建立的视觉历史不因深度/TF失败撤销，活跃性另记。对真正反证的争议/撤销流程尚未实现，历史标签不代表永远正确。

定位独立要求：配准同时间RGB-D/标定、map采集TF父子帧及时间有效（静态0时间允许）、原估计位姿括号和原新鲜度门；原localize_masks几何门保持。质量要求为≥120深度支持、有效率≥.80、跨度≤.45米、外环有效率≥.80、外环近前景≤.08、框内近前景≤.08；至少3合格帧、跨度≥.8秒、2秒窗内中位点偏差≤.25米。2秒过期。掩膜比例与裁剪仅记录提示。代理通过不等于误差≤.5米。

代表点固定`median_visible_mask_surface_not_body_centre`，不补人体中心/脚点/距离。当前点失效立即清空，历史好点不会填回当前XYZ。图像采集、视觉确认采集、实际observation_proof.completed_at、处理收到与当前发布时刻分开记录。

## 关联与人数

同帧按置信度排序，IoU≥.60抑制重复框；跨帧2秒以内IoU≥.30一对一关联，最多128条历史轨迹。外部检测ID不参与。同一短时轨迹只产生一次visual_confirmed事件。丢失再出现、显著换视角或相邻人仍可能重复或错关联，未实现独立身份去重；容量耗尽记录memory_full，不能保证完整召回。

schema_version=2区分raw_rgb_candidates_latest、post_nms_candidates_latest、local_visual_track_records、visually_confirmed_track_records、reliable_localizations_current和same_frame_duplicates_suppressed_total。raw_rgb_candidates_latest指检测器返回、进入本地额外NMS前的候选；YOLO自身已做内部NMS，本阶段未记录网络pre-NMS logits，不能把该字段称为原始网络候选。在线independent_true_people始终null。独立评测器结束后另读真值，给evaluator_ground_truth_people_total；无一对一身份复核时confirmed_people为null。空场中任何视觉确认是误确认；不能将本地ID数量直接当真实人数。

## 原生接口与RViz

视觉已确认但定位不足时，仍可向原生RACER发布新鲜世界射线/角锥。数值接口保持v1的17字段；无目标距离、无伪造3D人员点。原生TTL20秒、每线索2次、全任务4次/60秒、偏置后强制原探索不改；发布端要求角度和图像均≤2秒，历史确认不会让发布端旧射线续命。原生已收到的角度缓存仍按原TTL20秒保留，停止发布并非撤回；因此不能声称原生每次选择也满足2秒图像年龄。净空、动力学、轨迹和执行安全门不变。

/search/tracks、/search/semantic_candidates、/search/presence_events和/search/bearings共用状态定义。/search/targets每次先DELETEALL，当前可靠点才画表面点，否则只画新鲜相机射线；射线显示长2米不是人体距离，无新鲜几何不画世界点。状态标签明确视觉确认和定位质量。

启用：`--detector yolo11n-seg --confirmation-mode decoupled --bearing-bias`。decoupled禁止旧基于XYZ的`--active-review`，使用原生方位观察路径；尚未统一旧复查状态机。默认模式不静默替换。

## 验证边界

单元与真实ROS消息fixture覆盖重复框/时间、短检测ID变化、低分与漏检、定位恢复再丢失/过期、错误及陈旧TF、无位置标记和方位延续。fixture缺深度/NaN是显式感知故障注入，建图与安全深度不改，不是自然性能或飞行验收。离线回放用冻结保存模型输出和原几何，处理时刻设capture+.01s，只验状态逻辑，不重现原时延。动态实验使用Gazebo/PX4实际动力学，独立于回放；最多三轮各180仿真秒、硬故障结束全批次、无调参或追加运行强行通过。原始失败保留。

人体局部模型、结构/遗留物引导、多机、跨视角身份、无GNSS定位和统计收益均未由本阶段实现或验收。
