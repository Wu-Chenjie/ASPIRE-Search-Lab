# 研究依赖来源

- RACER: Robotics-STAR-Lab/RACER，提交 049c332e3634ef72d8beb155b4c13dc91ca52916。保留源文件版权注释及上游 README；本地副本未见根目录 LICENSE，生成 package.xml 的 TODO 不构成授权。本仓库用于用户授权的私有研究备份；不作公开发布，也不新增或推定上游授权。
- LKH: 原 RACER 随附 lkh/，保留全部源文件注释；未新增或推定许可证。
- PX4 Gazebo models: e05f4312d3f28aa621157610584a4870406cb6d3，保留根 LICENSE 及所需各模型 LICENSE。
- person_standing: osrf/gazebo_models@8163eb4b5e7e21985c6591d1c0bfb56468c0093f，作者 Marina Kollmitz / MakeHuman，保留 model.config 和文件原貌、assets.lock.json。现有源资产未附独立 LICENSE，不虚构授权。
- OpenCV HOG: 使用已有 Ubuntu OpenCV 4.6.0 的内置 Dalal–Triggs 人体分类器；无下载权重。运行环境自带依赖许可证由系统包保留。
- Artec Body scan: Artec Group inc.，https://www.artec3d.com/3d-models/body-scan 。包内CC BY 3.0 Unported全文随本任务保留；官网当前标CC BY 4.0，未将差异隐去。模型仅代码树外本地准备，单位、坐标和非金属PBR转换见`docs/SEGMENTATION-PREP.md`，原网格/纹理保持。与旧MakeHuman模型分开。
- 可选Ultralytics YOLO11n-seg: 官方AGPL-3.0版本，固定8.3.228及权重SHA256，隔离依赖准备；默认仍为HOG，实际SITL实验通过显式参数启用YOLO。原始依赖许可证随代码树外安装包保留。

原工程及其 Git 历史是未裁剪的可恢复来源。这里不包含其他探索规划器或 PX4 构建产物。
