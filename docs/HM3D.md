# HM3D 接入与验证（stage 8历史记录）

后续收敛诊断及局部膨胀修复见 [stage 9](HM3D-DIAGNOSIS.md)。

**以下状态截至stage 8；当前NBg单例跨区域通过，其余两场景受所选门洞净空限制，多场景尚未通过，详见上方stage 9。** 以下所有飞行是本机 Gazebo/PX4/原生 ROS2 RACER，使用 GNSS 辅助 EKF2；不等于无 GNSS 室内定位验收。

使用已许可的本机 HM3D example v0.2 三个 GLB，原文件只读。保留米制 Z-up、纹理和全部碰撞三角形；视觉与碰撞使用同一平移，不补洞、不封门。归属：Habitat-Matterport 3D Research Dataset / Matterport；[使用许可](https://matterport.com/legal/matterport-end-user-license-agreement-academic-use-model-data)。资产副本、网格、构建与运行记录均在代码树外。

## 固定门槛与结果

静态门槛：四朝向各30对320×240 RGB-D，时间戳、内参和光学坐标一致；独立网格轴向深度残差中位数≤5 cm、至少95%在20 cm内；另用下落球验证实际地面接触。HM3D四朝向沿用旧runner名称，不代表四种人体遮挡，场内未放人物。

| 场景与静态记录 | 各朝向残差中位数范围 | 20 cm内最低比例 | 配准/几何 |
|---|---:|---:|---|
| GLAQ sensors-02 | 0.000034–0.005697 m | 98.408% | 通过 |
| NBg sensors-01 | 0.000822–0.001836 m | 98.674% | 通过 |
| CFV sensors-01，旧起点 | 0.000143–0.002826 m | 97.701% | 通过；不代表起飞可行 |
| CFV verified-sensors-02，新起点 | 0.000020–0.001217 m | 99.204% | 通过 |

这些是静态传感器对照误差，不是定位精度。GLAQ第一次接触探针订错话题，第二次修正；后三场及GLAQ复验均取得球体与扫描地面的实际接触。

飞行门槛：固定seed123、`consistent`、原安全包络、未知空间检查；关闭人员模块，采用逐段执行及有限无路恢复。240 s仿真预算含起飞扫描；至少3条完成轨迹、实际穿过门洞进入相邻房间并取得新观测；连续60 s无完成进展判失败。三场有既往使用历史，不声称盲测。

| 实际运行 | 激活/完成轨迹 | 累计位移 | 结果 |
|---|---:|---:|---|
| glaq-flight-01 | 0/0 | 7.201 m | 扫描后FINISH；202.556 s人工中止 |
| glaq-flight-diagnostic-02 | 1/1 | 6.969 m | 150 s；末尾83.188 s无完成，未跨房间 |
| glaq-flight-vertical-03 | 9/8 | 13.219 m | 240 s；末尾110.756 s无完成，未跨房间 |
| nbg-flight-01 | 12/12 | 21.049 m | 240 s；末尾84.2 s无完成，未证实跨房间 |
| cfv-flight-01，旧起点 | 0/0 | 25.800 m | 起飞扫描碰撞、持续偏离；200.004 s中止 |
| cfv-flight-02，新起点 | 0/0 | 8.516 m | 240 s；就绪后182.6 s无完成，未跨房间 |

位移包括起飞、扫描和漂移，不能当作有效搜索路径。早期进度曾把GLAQ diagnostic-02说成0条，完整日志修正为1条；旧flight-01的中止异常文本为空，不能将空error解释为成功。GLAQ diagnostic、vertical及NBg的起飞后记录未出现接触；旧CFV有大量实际碰撞，绝不列为成功。

## 已修复与精确限制

- 原C++采样诊断记录边界、膨胀占据、未知和可见性拒绝。可选`--vertical-samples`在原采样圆上增加0、±0.3、±0.6、±0.9 m高度；其余检查和规划链保持。GLAQ仍失败，因此默认关闭，属于任务扩展。
- 人员模块开关与`--stop-and-plan`、前沿恢复解耦。前三轮曾用`--active-review --no-perception`取得相同执行方式；实际人员模块未启动，旧result里的请求字段不代表模块启用。
- 旧CFV稀疏射线检查漏掉升高路径中的结构：离线查得z=1.85 m附近表面距离仅0.140 m。生成器现沿0.7–1.8 m扫描段每≤2.5 cm取样，利用距离函数1-Lipschitz性质验证整个连续段≥0.6 m。旧起点被拒绝，替代层起点下界0.684 m。此检查仅用于生成仿真初始条件，不向决策节点提供网格。
- 新增基于实测执行误差的SITL中止检查：起飞旋转扫描时误差>0.65 m持续3 s则停止本次仿真。回放旧CFV在45.8 s触发，其余四次未触发；这是测试工具保护，不是飞控或实机安全认证。
- **0.6 m不是门宽**：原生RACER以0.3 m体素、每轴±2格作立方膨胀；Python额外执行检查的legacy椭球水平半径0.6 m、竖直半径0.3 m。两者均保持，未启用较小的可选包络。
- GLAQ所查入口截面y=6.75、x∈[3.8,6.0]、z∈[0.7,3.0]，301次自适应距离查询给出整个矩形的最大净空上界0.599963 m，小于0.6 m球半径，原生立方膨胀也不能通过。**只证明该入口，不证明全场景没有其他路线**；真值审计不进入规划。
- 新CFV起飞后无接触记录，位置误差中位数0.0310 m、P95 0.0520 m；就绪后新增37个观测体素，但位移范围仅约0.076×0.103×0.044 m。原生采样仍大量被膨胀占据和未知检查拒绝。不得把FINISH解释为整个房间已搜索，也不得将未知改为自由来推进。

## 代码与复现

`tools/hm3d/prepare.py`生成场景；`audit.py`、`portal_audit.py`、`flight_audit.py`仅事后评测；`frontier_diagnostics.py`、`vertical_sampling.py`只修改代码树外的生成构建副本。锁定的迁移源码未改。

C++负责前沿/观察点、A*及B样条优化；Python负责实测深度地图、额外执行安全授权、执行状态机和PX4接口。已从真实进程路径、二进制符号及加载库确认C++在运行，ROS2 DDS负责交互。生产/诊断/多高度构建的完整源码、二进制哈希和日志随证据保存；多高度版本默认不启用高度扩展。

容器内（Jazzy/Harmonic，本机已有依赖）：

```bash
source /opt/ros/jazzy/setup.bash
source /planner/external/ros_ws/install/setup.bash
source /tmp/racer-search-task3-native-stage8-vertical/install/setup.bash
cd /tmp/racer-search-task3-stage8-clean
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$PWD:$PWD/src:$PWD/src/annual_planner/src:/tmp/racer-stage8-python:$PYTHONPATH"
python3 -m tools.hm3d.prepare --source /planner/external/datasets/hm3d-example-v0.2/extracted/00861-GLAQ4DNUx5U/GLAQ4DNUx5U.glb --output /tmp/new-glaq-static --models /planner/external/PX4-gazebo-models/models --static
python3 -m tools.rgbd_scene.run --prepared /tmp/new-glaq-static --output /tmp/new-glaq-capture
python3 -m tools.hm3d.audit /tmp/new-glaq-static /tmp/new-glaq-capture
# 飞行另选新目录生成，去掉 --static，然后：
python3 -m tools.flight.run --prepared /tmp/new-glaq-flight-scene --no-perception --stop-and-plan --recover-no-path --budget-clock sim --seconds 240 --output /tmp/new-glaq-flight
python3 -m tools.hm3d.flight_audit /tmp/new-glaq-flight
```

每个输出目录必须不存在。生成依赖为固定Linux `trimesh==4.7.4`、`rtree==1.4.1`，存于`/tmp/racer-stage8-python`；原工程里的macOS rtree不能用于容器。几何读取跳过材质解码，视觉仍保留原GLB；射线分8条批次以控制内存。原始内存溢出和失败日志保留。

本机6 vCPU、约7.7 GiB容器内存；软件渲染CFV约占5.1 GiB RSS，场景串行运行，未使用外部机器。当前没有待批准的权限或安装阻碍；主要未解决项是兼容现有安全包络的跨房间可达区域和已知空间中的有效观察点。

基座通过后再启用官方YOLO nano实例分割。后续已做的依赖、扫描人体及接口准备见[准备记录](SEGMENTATION-PREP.md)，不代表门槛通过。Artec官网链接CC BY 4.0，但下载包内为CC BY 3.0，已保留原文；未采用限制Computer Vision Research的Renderpeople模型。

## 回归与保存

宿主最终回归64通过、10跳过；Jazzy容器原生回归118通过，含保留的原四视角采集回归。原工程351个来源文件及30个飞行来源文件哈希均未变化；迁移RACER及资产锁定验证通过。六次飞行的自有进程全部清理，无强杀或残留。

证据在项目旁`../task-3-evidence/stage-8/`，含六次飞行、静态采集、源场景派生资产、三套原生构建、最终任务源码、统一审计及SHA256清单。原CFV碰撞失败记录和修改前日志一并保留，不以新结果覆盖。
