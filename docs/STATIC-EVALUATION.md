# 静态搜索配对验收协议（未冻结）

使用 [协议模板](STATIC-GUIDANCE-PROTOCOL.json) 创建新实验。先填入独立验证得出的误报上限、
预算和数据划分，再记录冻结时间及模型与规则哈希。宣称搜索收益前须取得完整的闭环配对证据。

## 冻结与评测顺序

1. 在独立validation_ids上选择并说明误报容忍上限，以及弱证据成本、
   观察时间、时间/能量预留、功率上界和成像可见性参数。
2. 先列出完整test_ids和每个场景/种子的两个arm，固定时间预算。
   两组使用同一原生基础规划器、检测权重、最终确认和身份去重规则。
   实验中失败或中止的配对不得从清单删除以制造提升。
3. 冻结协议：status=frozen、带时区frozen_at、验证证据SHA256、
   detector_sha256、confirmation_sha256、identity_sha256、common_stack_sha256、
   resource_budget_sha256；case_specs逐项冻结seed、scene_sha256及common_runtime_sha256。
   资源清单包括CPU/GPU绑定、频率/并发、仿真时钟与采集/录像选项。规则哈希应来自
   明确文件清单的规范化manifest；独立复核完整清单，不能只哈希一个标签。
4. 原生接口验证通过后，以新 ID 运行独立闭环配对实验。
5. 仅离线评测读真值，将可靠确认后的身份与真实静态目标匹配，计算真实
   去重发现数。不能把视觉track数或表面点数当人数；定位误差单列。

当前误报上限定义为**每次完整运行的误报发现数上限，两组均适用**。
上限的可接受值仍需用户目标与独立验证共同确定；不得在看过测试结果后填入。
出现新的定义（例如按场景比例）必须预先修订协议和代码，不能混用量纲。

## 可执行声明门

```sh
python3 tools/search/evaluate_static_claim.py \
  --protocol docs/STATIC-GUIDANCE-PROTOCOL.json \
  --runs /absolute/path/to/new-batch/paired-results.json
```

runs为JSON数组；每行包含：

| 字段 | 定义 |
|---|---|
| test_id / arm | 冻结测试项及baseline或guided |
| protocol_sha256 | evaluation_gate.protocol_digest对完整冻结协议计算的SHA256 |
| time_budget_s | 与协议相同的秒数 |
| detector_sha256 / confirmation_sha256 / identity_sha256 / common_stack_sha256 / resource_budget_sha256 | 与协议完全相同的共享模型、规则、基础栈和资源清单哈希 |
| seed / scene_sha256 / common_runtime_sha256 | 与该test_id的冻结case_specs相同 |
| guidance_enabled / original_safety_gates_passed | arm对应的布尔引导开关，以及原安全门全部通过的布尔证据声明 |
| started_at | 带时区时间，严格晚于冻结时间 |
| execution_mode / outcome | closed_loop及complete；回放/失败不能通过收益声明 |
| evidence_sha256 | 本次完整原始证据manifest的SHA256 |
| unique_true_discoveries / false_positives / duplicates | 离线真值匹配后的非负整数 |
| confirmation_latency_s / reacquisitions | 确认时延摘要与重获次数；同时保留逐目标事件。预算内无视觉确认（真发现、误报、重复及未决身份计数均为 0）时，时延可显式填 `null` |
| extra_path_m / compute_s / identity_uncertain | 有符号路径增量、非负计算耗时、非负整数不确定身份数 |

门禁拒绝缺配对、重复运行条目、非闭环证据、未知误报上限、测试早于冻结、
共享规则不一致、无效指标和超上限。只有完整配对发现数总增量为正且所有检查
通过，才返回claim_allowed=true/exit0；其余exit2。它检查提交的声明，
不会认证哈希指向文件真实存在、时间戳真实、数据划分独立或标注正确；
以上均需证据核验。总增量为正也不是统计显著性证明，不应宣传泛化收益。

正式报告还应给出每对及汇总分布、误报/重复、确认时延、遮挡重获率/时延、
覆盖率、额外路径/时间、有效可见区域漏检、事件到动作时延、身份关联及调度
成本、峰值并发、失效结果、日志丢弃、连续交接与安全故障、单独定位精度。
CLI当前只强制表中最小字段，不替代该报告完整性核查。

## 离线发现计数

`evaluate_discoveries.py`接收三个JSON数组。events每项含稳定event_id、
算法identity_id、任务相对仿真capture_s、visual_state（person_candidate或
visually_confirmed）；annotations每项含identity_id、status（matched、
false_positive或uncertain）、evidence_sha256，matched另含真实truth_id；
truth-ids只列本场景独立评测真值ID。它们只供离线使用，不输入导航。

```sh
python3 tools/search/evaluate_discoveries.py \
  --events /absolute/new-batch/confirmed-events.json \
  --annotations /absolute/new-batch/independent-identity-review.json \
  --truth-ids /absolute/new-batch/truth-ids.json \
  --time-budget-s "$VALIDATED_DURATION_S" \
  --output /absolute/new-batch/discovery-score.json
```

计数只取预算内首次可靠确认。重复传输事件幂等；同一算法身份重复报告单列。
多个算法身份匹配同一真实人，真实发现只计1，额外身份计duplicates；
缺标注或不确定匹配只计identity_uncertain。输入不使用定位成败或XYZ评分。
输出文件必须不存在。CLI不自动从原日志抽取事件，不认证标注或证据哈希；
输入准备须记录时间原点，并按冻结的最终身份关联规则规范化别名，保留原始
合并/拆分过程以另行评价过程重复，不能把别名改写当新的发现或删除错误记录。
该结果与时延、资源、安全及闭环证据一起组成上表runs，不单独放行收益声明。
