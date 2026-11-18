# 人体模块准备（启用前历史）

用户随后明确选择范围A，已进入[实际YOLO人员搜索](YOLO-SEARCH.md)。以下保留当时准备阶段的状态，不能代替最新结果。

多场景HM3D基座未通过，默认ROS节点和飞行runner仍为`hog`。以下是可关闭的准备代码，不构成YOLO检测验收或自主搜索成功；不能提前用于对外宣称的新闭环。

- 选定官方YOLO11n-seg，`ultralytics==8.3.228`、CPU PyTorch 2.5.1、torchvision 0.20.1。理由是轻量、现有ARM容器兼容和固定版本可复现，不宣称最新型号。官方权重SHA256：`55ed65c56c91713d23e8402371c6c49a6fd84f257f7dce452e8d70e41dcbe152`。推理接口固定person类、conf=0.35、imgsz=640、retina_masks、CPU；启动前校验文件/版本，关闭自动依赖安装及遥测，不自动下载模型。
- `/search/semantic_candidates`保留RGB二维证据；缺深度或采集时刻TF时`xyz=null`。只将有可靠深度的位置送入原去重/确认器与有限复查。深度限于腐蚀后的实例掩膜，拒绝跨度过大或支持不足，不读框中心、不从背景连通组件借深度。定位仍是可见表面位置，不是人体中心。
- 5个纯几何回归覆盖无效深度、框中心遮挡板、7米混合深度、掩膜外背景和无检测。ROS接口用明确标记的检测桩验证，不冒充实际YOLO推理。原HOG接口原生回归保留。
- 真实人体来自[Artec Body scan](https://www.artec3d.com/3d-models/body-scan)，ZIP SHA256 `a931d512a7ee87970d61d6731f556ed1d841af2140c7bead6c3ff38909f304b2`。包内CC BY 3.0全文保存在`licenses/Artec-body-scan-CC-BY-3.0.txt`，官网当前链接CC BY 4.0，二者差异如实记录。署名Artec Group inc.；仅将毫米转米、居中XY并使脚底Z=0，保留764896个三角形及原纹理。显式设非金属粗糙PBR，修复首次转换默认金属导致的发黑；失败样本保留。最终高度1.78942 m是按模型单位转换结果，不是另行测得人体身高。
- 正例和实际移除人体的负例各采集四个手动静态视角，每视角30对320×240配准RGB-D。没有飞控、没有自主运动，也尚无实际YOLO识别结果。HM3D飞行仍不启用此资产或检测器。

资产、Python依赖、图像均位于代码树外。`tools/rgbd_scene/scanned_person.py`按ZIP哈希重建模型；`--mesh`生成静态夹具，`--negative`实际移除人体；`tools/rgbd_scene/evaluate_segmentation.py`为门槛满足后待执行的离线评测脚本。运行环境需保留ROS的原PYTHONPATH，不能覆盖后再期望rclpy可用。Linux ARM64/Python 3.12可选依赖完整锁为`requirements-segmentation.lock`；安装发现setuptools 84与colcon要求冲突，已固定79.0.1并保留旧元数据，未修改系统环境。CPU torch/torchvision NMS兼容性检查通过，不是YOLO模型测试。

待完成：多场景基座验收、实际YOLO四视角/无人识别与深度评估、HM3D内扫描人体布置及动态搜索、同预算策略对照。Visual Jev仍未接入，不影响几何基线准备。

当前回归：宿主70通过/11跳过，Jazzy原生126通过，包含HOG旧行为与检测桩的RGB单独到达、无效深度和有效定位。默认节点/runner没有改为YOLO。
