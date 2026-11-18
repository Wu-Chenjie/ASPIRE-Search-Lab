# 新 HM3D 场景下载与账号访问

后续已完成官方包下载、完整性核验及 10 个新场景解析，实际几何和飞行结果见[新地图实测](NEW-MAP-TESTS.md)。以下为阶段 17/18 的历史访问记录，下载中和登录前状态不代表当前状态。

## 用户登录后的配置复核

用户登录后，已通过本机 Safari 核实 Matterport 工作台可访问。旧 `/settings/account/devtools` 地址在此账号下回到 Account Settings，Account/Features/Add-ons 中未看到 Developer Tools 或 HM3D 权限面板；不能据此判断账号没有下载权限。

随后用同一浏览器访问官方 `hm3d-minival-glb-v0.2.tar` 链接，成功转入官方 S3 下载，已允许本次下载。**现有浏览器登录足以启动这个包的下载，无须新建 API 令牌。** 未创建或保存令牌、修改账号权限、提取 Cookie，也未把下载签名复制到文件/命令。浏览器自身接收官方重定向；记录不保存其带签名查询串。

本轮配置结束时，Safari 显示总量约 487.2 MB，文件仍在 `/Users/wuchenjie/Downloads/hm3d-minival-glb-v0.2.tar.download/` 写入；尚未完成下载、校验或提取，不能宣称取得完整新场景或通过筛选。无需用户再输入凭据。后续先核验下载完成、TAR 可完整读取及 SHA256，再逐场景清点、少量筛选；不要把 `.download` 中间文件当完整模型使用。脱敏配置记录位于代码树外 `../task-3-evidence/stage-18-browser-configuration/`。

## 登录前访问记录（历史）

用户已授权少量新场景下载；沿用此前已确认的 HM3D 学术用途许可。本轮实际取得 **0 个新场景、0 字节新模型/纹理**，没有新几何筛选或飞行结果。宿主约 59 GiB、既有 `annual-px4-research` 容器约 20 GiB 可用，空间不是当前阻碍。

官方最小新 GLB 包 `hm3d-minival-glb-v0.2.tar` 的 HEAD 返回 405；限量 GET 经 307、308 重定向后返回 `https://authn.matterport.com/login`，标题 `Login | Matterport`，HTTP 200 的内容是 HTML 登录页，不是下载成功。Python urllib 连接失败后已用系统 curl 完成复核，没有关闭 TLS 校验。未保存登录页、Cookie、查询串或认证秘密。

宿主和容器均未发现 HM3D/Matterport 专用环境变量或 `.netrc` 文件；这不代表用户没有账号或浏览器登录。未搜索其他秘密存储。官方说明需要账号在 Settings → Developer Tools 中申请 HM3D 访问，并使用 API token ID/secret 进行程序下载；此前公开 example 包不证明已有完整数据集下载权限。

最小下一步：用户在 [Matterport Developer Tools](https://my.matterport.com/settings/account/devtools) 登录，确认 HM3D 访问权限并完成本地凭据配置，或将通过官方账号取得的小批 GLB 放入代码树外后告知路径。不要将 token/secret 发到聊天中。若账号出现新的协议、费用或审批，先报告。这里需要账号访问操作，不需要重复确认已接受的同一学术协议。

## 已完成的公开元数据初选

下载自 Matterport 官方仓库 `metadata.csv`，65916 字节，SHA256 `77c5724a86610178e69aa1cdbd63c5e4ba107aba9c01becf2aab576fe77ec2b4`。排除现有三幅图，筛选单层、至少三房间、可导航面积 ≥150 m²，以及 large_holes/portals_blocked/doors_scanned_closed/navmesh_quality 字段均为 0 的记录；按 scene_clutter 升序、面积降序、场景 ID 排序。仅为候选优先级，没有修改在线 RACER 参数。

| 初选场景 | 元数据房间数 | 可导航面积 / m² | scene_clutter |
|---|---:|---:|---:|
| 00777-N17ddiDvJr9 | 17 | 219.86 | 1.79 |
| 00772-zJ3fVx3BZYR | 7 | 227.59 | 1.86 |
| 00559-53jtKd53a1X | 3 | 311.94 | 1.92 |

三者在 train split；不能据此声称位于 minival。房间数可能有误差，官方可导航面积使用半径 0.1 m、高 1.5 m 的圆柱，不满足本项目净空证明。尚未查看候选实际几何/预览，未确定宽通道、真实连通房间、起点或人体摆位。

官方清单列出 minival GLB 约 464M、train GLB 约 32G，没有在该清单提供逐场景入口。未下载这些完整包；鉴权后再检查是否支持范围读取 TAR 成员或官方逐场景取得，不能现在保证支持。若不支持少量获取，先报告，不能直接下载全库。取得候选后，0.75/0.90 m 保守格点筛选只能提供诊断；有希望的路线须核对真实连续净空和原生安全约束，不能以筛选失败冒充真实不可达。

证据持久保存于 `../task-3-evidence/stage-17-new-scene-access/`：公开元数据及哈希、初选规则、脱敏 HTTP 状态、空间/配置存在性记录、访问脚本、报告与回归日志；`manifest.json` 为逐文件 SHA256，`archive-validation.json` 记录完整归档逐成员比对与 gzip CRC。没有启动仿真进程、上传资料或使用第三方镜像。

官方来源：[下载清单](https://github.com/matterport/habitat-matterport-3dresearch)、[账号访问流程](https://matterport.com/partners/meta)、[Habitat 下载认证说明](https://github.com/facebookresearch/habitat-sim/blob/main/DATASETS.md)、[元数据定义](https://aihabitat.org/datasets/hm3d/)。
