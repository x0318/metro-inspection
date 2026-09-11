# Metro Inspection 代码详细说明

本文按实际使用顺序解释代码：**打开入口 → 读取设置 → 启动后台 → 加载仿真与算法 → 显示画面和记录 → 移动车辆 → 查看报告 → 停止程序**。

首次安装见[README](../README.md#环境与首次部署)。

## 目录

- [1. 打开程序：找到项目](#1-打开程序找到项目)
  - [1.1 桌面打开流程](#11-桌面打开流程)
  - [1.2.1读取设置并决定启动或连接](#121读取设置并决定启动或连接)
  - [1.2.2命令行直接打开三个窗口](#122命令行直接打开三个窗口)
- [2. 决定本次启动进程](#2-决定本次启动进程)
  - [2.1 launch 解析设置](#21-launch-解析设置)
  - [2.2 主要启动参数](#22-主要启动参数)
- [3. 场景出现：车辆与传感器加载流程](#3-场景出现车辆与传感器加载流程)
  - [3.1 sensors 脚本顺序](#31-sensors-脚本顺序)
  - [3.2 车辆模型](#32-车辆模型)
  - [3.3 坐标和时间后续处理](#33-坐标和时间后续处理)
- [4. 平台就绪：画面、记录与报告如何显示](#4-平台就绪画面记录与报告如何显示)
  - [4.1 HTTP 与 ROS 在哪里连接](#41-http-与-ros-在哪里连接)
  - [4.2 网页怎样持续更新](#42-网页怎样持续更新)
  - [4.3 记录怎样写入数据库](#43-记录怎样写入数据库)
  - [4.4 RViz 红点与报告输出](#44-rviz-红点与报告输出)
  - [4.5 远程打开同一平台](#45-远程打开同一平台)
- [5. 开始巡检：移动、停车与视点切换](#5-开始巡检移动停车与视点切换)
  - [5.1 指令怎样到达车辆](#51-指令怎样到达车辆)
  - [5.2 watchdog 怎样处理超时与急停](#52-watchdog-怎样处理超时与急停)
- [6 .识别过程二维转三维](#6-识别过程二维转三维)
  - [6.1 病害命名及类型](#61-病害命名及类型)
  - [6.2 六路配置与输出](#62-六路配置与输出)
  - [6.3 点云定位的文件与职责](#63-点云定位的文件与职责)
  - [6.4 坐标如何变成里程和环号](#64-坐标如何变成里程和环号)
  - [6.5 打开建图流程：里程计与地图保存](#65-打开建图流程里程计与地图保存)
  - [6.6 运行与保存](#66-运行与保存)
- [7. 打开岔轨平台：独立导航流程](#7-打开岔轨平台独立导航流程)
- [8. 入口对照](#8-入口对照)

## 1. 打开程序：找到项目

### 1.1 桌面打开流程

在仓库根目录执行：

```bash
bash scripts/open_inspection_app.sh
```

Windows 双击 `Open Metro Inspection.vbs` 时，代码先经过 PowerShell 和 WSL，再调用 Linux 侧的桌面入口：

```text
Open Metro Inspection.vbs / Open Metro Inspection.cmd
  -> scripts/open_inspection_windows.ps1
  -> WSL 中的 scripts/open_inspection_app.sh
  -> /usr/bin/python3 -m metro_dashboard_bridge.desktop_app
  -> main() -> InspectionWindow
```

[open_inspection_windows.ps1](../scripts/open_inspection_windows.ps1) 负责 Windows 到 WSL 的启动转换；[open_inspection_app.sh](../scripts/open_inspection_app.sh) 根据自身位置求出仓库目录，用 `exec` 进入 Python 桌面程序。日志写入 `~/.local/state/metro-inspection/desktop.log`。

### 1.2.1读取设置并决定启动或连接

[desktop_app.py](../ros_ws/src/metro_dashboard_bridge/metro_dashboard_bridge/desktop_app.py) 的 `InspectionWindow.__init__()` 创建工具栏、`ReportView`、日志面板和定时器，读取 `~/.config/metro-inspection/desktop.json`。

可在打开vbs之后的运行设置部分选择需要打开内容和切换识别模型，重启即是所需配置。

### 1.2.2命令行直接打开三个窗口

按之前的使用方式，可直接打开平台、Gazebo 和 RViz。

```bash
bash scripts/run_inspection_backend.sh \
  guided_yolo:=true \
  yolo_model_path:="/你的实际路径/best.pt" \
  gui:=true rviz:=true qt:=true
```

## 2. 决定本次启动进程

### 2.1 launch 解析设置

源码：[inspection_platform.launch.py](../ros_ws/src/metro_bringup/launch/inspection_platform.launch.py)。

```text
generate_launch_description()
  -> DeclareLaunchArgument 声明默认值
  -> OpaqueFunction 调用 _launch_platform(context)
       -> LaunchConfiguration.perform(context) 读取本次参数
       -> _as_bool() 转换开关并校验互斥模式
       -> 检查脚本、权重、模型、配置和端口范围
       -> 设置 ROS_DOMAIN_ID、Gazebo 地址等环境
       -> 创建 ExecuteProcess / Node / TimerAction
       -> 注册关键进程 OnProcessExit 回调
```

### 2.2 主要启动参数

| 参数 | 默认值 | 行为 |
| --- | --- | --- |
| `detection` / `localization` | `true` / `true` | 常规检测和 Odin1 点云定位 |
| `model_demo` / `guided_yolo` | `false` / `false` | 模型参考模式的两个独立入口 |
| `yolo_model_path` | `METRO_YOLO_MODEL_PATH` 或开发机默认文件 | 新机器应指定本机权重；模型标注不需要 |
| `gui` / `rviz` / `qt` | 均为 `true` | 直接 launch 打开三个窗口；桌面默认覆盖 GUI/RViz 为关闭 |
| `open_browser` | `false` | 可改用浏览器；关闭 Qt 时用 `qt:=false` |
| `yolo_auto_drive` | `false` | 常规覆盖实验的有限行驶，桌面固定关闭 |
| `dashboard_port` / `ros_domain_id` | `8088` / `70` | 主巡检平台 HTTP 与 DDS 域 |
| `gazebo_master_uri` | `http://127.0.0.1:11370` | 主仿真 master |
| `initial_pitch_deg` | `45` | Pitch 关节初始角，单位度 |
| `inspection_session_id` | 空，按时间生成 | 每次启动新会话；显式命名可再次打开同一会话 |
| `database_path` / `defect_topic` | 按模式选择 | 区分处理结果来源 |

## 3. 场景出现：车辆与传感器加载流程

### 3.1 sensors 脚本顺序

源码：[open_subway_tunnel_v2_sensors.sh](../ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_sensors.sh)。

```text
读取 subway_v2_model_scale.txt
  -> scale_subway_v2_urdf.py：生成临时缩放 URDF
  -> check_urdf：检查模型结构
  -> 启动 robot_state_publisher
  -> 启动 camera_info_calibrator
  -> 启动 cmd_vel_watchdog.py
  -> 启动 odometry_fusion.launch.py
  -> 启动 Pitch controller spawner 与初始角脚本
  -> gazebo_ros/gazebo.launch.py 加载 subway_tunnel_v2_sensors.world
```

### 3.2 车辆模型

`subway_v2` 由 V6 CAD 归一化为 ROS 2 URDF；运行 SDF 使用约 `3.0063795853269537` 的统一缩放匹配轨道。原始 URDF 尺寸和仿真尺寸应分别解释。

四轮由 `libgazebo_ros_diff_drive.so` 驱动，通过轮轨接触移动。正前方为 `base_footprint +X`。Pitch 关节范围 `-15°～+45°`，默认 `+45°` 对应光轴朝前上方约 `30°`；Yaw 锁定。转换脚本和安装修正见 [metro_description](../ros_ws/src/metro_description/README.md)。

### 3.3 坐标和时间后续处理

- 相机光学坐标用于点云投影，图像 header 保留源时间戳；检测框坐标均对应原图像素。
- EKF 独占 `odom -> base_footprint`；图 SLAM 独占 `map -> odom`；ICP 和 Gazebo 差速插件不再重复发布这些 TF。
- 仿真节点使用 `/clock`，watchdog、网页心跳与部分限频使用稳态时钟，避免仿真暂停让超时保护停止计时。

## 4. 平台就绪：画面、记录与报告如何显示

### 4.1 HTTP 与 ROS 在哪里连接

```text
main()
  -> rclpy.init()
  -> DefectEventBridgeNode()
       -> DefectStore：打开 SQLite 和本次会话
       -> CameraFrameStore：建立相机缓存
       -> 订阅 DefectEvent、原图和处理图
       -> 建立 /inspection/defect_markers 发布器
  -> SingleThreadedExecutor + 后台线程：执行 ROS 回调
  -> create_app(store, statistics, camera_store, ...)
  -> Uvicorn：在 127.0.0.1:8088 提供 HTTP
```

### 4.2 网页怎样持续更新

[dashboard.js](../dashboard/assets/dashboard.js) 每 2 秒更新一次数据。`refreshData()` 先确定会话，再并行请求健康、病害和相机接口，把结果写入页面响应式状态；请求序号用于避免旧请求覆盖新选择。

当前页面每 500 ms 更新地址中的版本参数，使 `<img>` 重新加载图像。

### 4.3 记录怎样写入数据库

源码入口：[defect_event_node.py](../ros_ws/src/metro_dashboard_bridge/metro_dashboard_bridge/defect_event_node.py)、[event_conversion.py](../ros_ws/src/metro_dashboard_bridge/metro_dashboard_bridge/event_conversion.py)、[defect_store.py](../ros_ws/src/metro_dashboard_bridge/metro_dashboard_bridge/defect_store.py)。

SQLite 包含 `inspection_sessions` 和 `defect_events` 两张表， 每会话默认仅保留最近更新的 500 条，超出部分会从数据库删除。

`list_sessions()` 提供历史目录，`list()` / `get()` 查询选定会话；浏览历史不会改变 ROS 当前写入会话。主 launch 自动新建会话。

### 4.4 RViz 红点与报告输出

[defect_markers.py](../ros_ws/src/metro_dashboard_bridge/metro_dashboard_bridge/defect_markers.py) 的 `report_markers()` 对当前会话事件按 ID 去重，每条有效坐标生成一个红色球点，并先清除旧列表。这些标记来自报告记录，离开视野仍保留。

前端 [dashboard.js](../dashboard/assets/dashboard.js) 读取接口，提供筛选、模式标签、历史与图表。`printReport()` 调用网页打印；Qt 的 [ReportView](../ros_ws/src/metro_dashboard_bridge/metro_dashboard_bridge/report_view.py) 接收打印请求。

### 4.5 远程打开同一平台

Tailscale 转发的是已经启动的 HTTP 服务。远程设备打开 Serve 地址后，仍由相同的 `dashboard.js` 请求会话、记录和图像接口；手机不直接订阅 ROS 话题。配置命令见 [README 的 Tailscale 部分](../README.md#tailscale-远程访问)。浏览器打印使用浏览器自身功能，Qt 本地导出文件保存在选择的本机路径。

## 5. 开始巡检：移动、停车与视点切换

### 5.1 指令怎样到达车辆

默认打开时车辆静止。

| 场景 | 速度链 | 保护实现 |
| --- | --- | --- |
| 完整 V2 巡检 | `/cmd_vel_raw -> collision_monitor -> /cmd_vel_safe -> /cmd_vel_drive` | Nav2 碰撞监控 + `metro_sim` watchdog |
| 岔轨训练场 | Nav2 `/cmd_vel -> /cmd_vel_drive` | `metro_navigation_demo` 自身 watchdog + Web 心跳急停 |

### 5.2 watchdog 怎样处理超时与急停

cmd_vel_watchdog.py负责消息、定时器、服务与实际零速度输出。默认 `timeout=0.5 s`、输出 `20 Hz`；检查消息新鲜度，点云几何是否有效还由对应处理模块判断。

岔轨使用独立 [drive_watchdog.py](../ros_ws/src/metro_navigation_demo/metro_navigation_demo/drive_watchdog.py)

## 6 .识别过程二维转三维

### 6.1 病害命名及类型

| 中文类别 | 拼音显示标签 |
| --- | --- |
| 裂缝 | `liefeng` |
| 渗漏水 | `shenloushui` |
| 管片破损掉块 | `guanpianposundiaokuai` |
| 扣件断裂 | `koujianduanlie` |
| 扣件缺失 | `koujianqueshi` |
| 扣件松动歪斜 | `koujiansongdongwaixie` |
| 管线支架松脱 | `guanxianzhijiasongtuo` |
| 异物入侵 | `yiwuruqin` |

### 6.2 六路配置与输出

[yolov8_coverage.yaml](../ros_ws/src/metro_detection/config/yolov8_coverage.yaml) 配置六路常规推理：置信度阈值 `0.35`、IoU `0.45`、推理尺寸 `640`、每相机上限 `3 Hz`、压缩质量 `75`，图像 QoS 使用 `reliable` 适配 Gazebo。上限不等于实际帧率。

### 6.3 点云定位的文件与职责

| 文件 | 核心对象 / 函数 | 职责 |
| --- | --- | --- |
| [damage_localizer.py](../ros_ws/src/metro_mapping/metro_localization/metro_localization/damage_localizer.py) | `DamageLocalizer`、`DetectionLocalization` | ROS 同步、每目标处理、调试显示、事件发布 |
| [cloud_projector.py](../ros_ws/src/metro_mapping/metro_localization/metro_localization/cloud_projector.py) | `project_camera_points()`、`select_bbox_points()`、`dbscan_median_point()` | 可见点投影、ROI 和鲁棒坐标估计 |
| [geometry_utils.py](../ros_ws/src/metro_mapping/metro_localization/metro_localization/geometry_utils.py) | `transform_points()`、`quaternion_to_rotation_matrix()` | 旋转与点坐标变换 |
| [spatial_event_tracking.py](../ros_ws/src/metro_mapping/metro_localization/metro_localization/spatial_event_tracking.py) | `SpatialEventTracker` | 同类目标的一对一关联、多帧确认、稳定 ID |
| [semantic_geometry.py](../ros_ws/src/metro_mapping/metro_localization/metro_localization/semantic_geometry.py) | `TunnelSemanticProjector` | 坐标转里程、环号、时钟方位和结构区域 |

### 6.4 坐标如何变成里程和环号

`TunnelSemanticProjector.project()` 默认沿 `+X` 计算，起始里程 `12000 m`、起始环 `1000`、环长 `1.2 m`，断面中心 `y=0、z=1.75 m`：

```text
相对里程 d = chainage_sign × 指定轴坐标
里程 s = chainage_start_m + d
环索引 n = floor(d / segment_length_m)
环号 = segment_start_id + n
环内偏移 = d - n × segment_length_m
时钟方位 = normalize_degrees(atan2(-(y-y0), z-z0)) / 30
```

例如 `x=10.2 m` 对应 `K12+010.200`、环 `1008`、环内偏移约 `0.6 m`。顶部为 0 点（12 点），右侧为 3 点，底部为 6 点，左侧为 9 点。输出明确标为“仿真环号”。当前为直线轴向映射，不包含真实线路曲线里程、测量控制网和变环宽模型。

### 6.5 打开建图流程：里程计与地图保存

独立启动的建图流程，不会因为打开主平台 RViz 就自动执行。先运行本节启动脚本，观察点云和里程计，完成采集后调用保存服务。

### 6.6 运行与保存

另开终端运行 `bash ros_ws/src/metro_sim/scripts/open_subway_v2_mapping_rviz.sh`。保存时先加载 ROS 与本工作空间，设置 `ROS_DOMAIN_ID=70`.

默认输出 `results/maps/subway_v2_optimized.pcd` 和 `subway_v2_rtabmap.db`。PCD 是优化后点云，数据库保存关键帧和图约束。

## 7. 打开岔轨平台：独立导航流程

这一入口启动独立的训练场和控制平台。已经完成完整工作空间构建后，从仓库根目录执行：

```bash
cd ros_ws
bash ../scripts/open_route_choice_platform.sh
```

打开 `http://127.0.0.1:8090`，等待里程计在线，选择目的地或“到岔口询问”，提交任务后才开始行驶。下面按该操作跟踪代码。

### 8. 入口对照

| 用途 | 仓库根目录命令 |
| --- | --- |
| Qt 桌面管理 | `bash scripts/open_inspection_app.sh` |
| 三窗口主平台 | `bash scripts/run_inspection_backend.sh yolo_model_path:="/本机/best.pt" gui:=true rviz:=true qt:=true` |
| 浏览器主平台 | 上述命令改为 `gui:=false rviz:=false qt:=false open_browser:=true` |
| 仅传感器平台 | `bash scripts/run_inspection_backend.sh detection:=false` |
| 模型标注 | `bash scripts/run_inspection_backend.sh model_demo:=true` |
| 引导式 YOLO | `bash scripts/run_inspection_backend.sh guided_yolo:=true yolo_model_path:="/本机/best.pt"` |
| 独立网页桥接 | `bash scripts/open_defect_dashboard.sh`，不会启动仿真和检测 |
| 岔轨平台 | `bash scripts/open_route_choice_platform.sh`，需要 RViz 加 `rviz:=true` |
