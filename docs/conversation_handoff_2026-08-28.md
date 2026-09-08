# 地铁巡检项目对话迁移与开发交接记录

> 状态快照：2026-08-28（Asia/Shanghai）
> 用途：将此前长对话中的项目背景、设计判断、已完成工作、验证结果、命令和后续任务迁移到新对话。
> 新对话应先完整阅读本文件，再通过只读命令核对现场状态，不要从早期模型方向、相机数量或端口问题重新开始。

## 0. 新对话最先需要知道的结论

1. 当前正式开发仓库是：

   ```text
   /home/jo/my-project/metro-inspection
   ```

2. 当前分支和提交是：

   ```text
   branch: feature/localization-fusion
   HEAD:   ac26c95 fix(localization): anchor EKF with lidar pose
   ```

3. 当前分支没有配置远端跟踪分支。未经用户明确要求，不要创建分支、合并、提交或推送。

4. 当前工作区不是干净状态。三维建图稳定性改动已经实现并完成本地验收，但仍未提交。

5. `results/` 当前约 `9.8 GB`，包含 rosbag、RTAB-Map 数据库和 PCD，禁止直接执行：

   ```bash
   git add -A
   ```

6. 当前保存的工作区测试记录为：

   ```text
   164 tests, 0 errors, 0 failures, 17 skipped
   ```

7. 当前真实 YOLO 尚未接入。队友以后提供二维检测节点；HSV 红色检测器只是默认关闭的通信占位节点，不能作为病害识别成果。

8. 正式平台目前是五路相机和病害事件的只读平台。旧项目中的导航控制、暂停、急停和路线选择尚未迁入正式平台。

9. 最新三维地图已经生成并通过当前验收规则，但短距离结果不能证明长距离隧道中的回环鲁棒性。仍需完成 `20～50 m` 往返压力测试。

10. 技术报告最终交付是“只补充第七章”的 DOCX/PDF，不是此前误生成的全报告补充版。

## 1. 用户的工作方式与协作要求

- 默认使用中文交流。
- 用户正在边开发边学习，需要提供可直接执行的完整命令，并解释关键参数和运行现象。
- 用户会把终端输出或截图发回；下一步应基于最新结果继续，不要重复已经完成的验证。
- 修改模型、TF、相机方向或运动方向前必须先检查当前实现，不能凭画面印象再次翻转。
- 不要把“接口已打通”写成“真实算法已完成”。
- 不要删除 rosbag、地图、模型压缩包、未跟踪文档或其他用户成果。
- Git 操作前先报告当前分支、HEAD、工作区状态和目标分支。
- 大功能按“最小验证环境 → 正式项目接口 → 运行验收 → 实车迁移”推进。

## 2. 两个仓库的职责

### 2.1 正式项目：metro-inspection

```text
/home/jo/my-project/metro-inspection
├── dashboard/                              正式五路巡检网页
├── docs/                                   项目报告和交接文档
├── results/                                本地 bag、数据库和地图，不直接提交
├── scripts/open_defect_dashboard.sh        正式平台启动脚本
└── ros_ws/src/
    ├── metro_description/                  V6机器人URDF、网格和导入脚本
    ├── metro_sim/                          Gazebo模型、世界、启动与验收脚本
    ├── metro_mapping/
    │   ├── metro_closed_loop/              2D检测适配、CameraInfo校准
    │   ├── metro_localization/             2D转3D、EKF融合定位
    │   └── metro_pointcloud_mapping/       累计点云与RTAB-Map三维SLAM
    ├── metro_inspection_interfaces/        DefectEvent消息
    └── metro_dashboard_bridge/             ROS 2到Web平台桥接
```

后续真实检测、建图、平台和实车迁移工作应以该仓库为准。

### 2.2 旧导航学习项目：subway-patrol-robot-sim

```text
/home/jo/my-project/subway-patrol-robot-sim/
subway-patrol-robot-sim-simulation
```

该仓库保存：

- 单分岔最小训练环境；
- 两种路径选择模式；
- 可直接控制导航的旧平台原型；
- 路线 JSON、Nav2 和安全看门狗；
- 早期模型、脚本和学习文档。

当前分支为 `simulation-demo`，相对 `origin/simulation-demo` 超前 2 个提交，并存在大量未提交文件。不要把该仓库的 world、机器人包或 ROS Domain 与正式项目混合启动。

## 3. 对话中的开发过程概览

### 3.1 路径选择与单分岔训练

最初尝试直接在隧道中增加分叉，但出现了视觉轨道搭接、碰撞体中间堵塞、模型被额外改造等问题。之后按用户要求脱离正式隧道，只保留：

- 一辆小车；
- 一条直行主轨；
- 一条连续接通的分叉轨；
- 不可见连续承载面；
- 不增加额外障碍物。

在该最小环境中完成了：

- `straight` 主轨路线；
- `branch` 单分叉路线；
- “到岔口后询问选择”；
- “按最终目的地自动选路”；
- Nav2 `NavigateThroughPoses` 路线执行；
- 暂停、继续、取消、锁存急停、复位和浏览器心跳；
- `/odom`、动作进度和五路相机的平台展示。

该成果属于旧导航原型，主要用于证明路径选择原理。正式 `metro-inspection` 平台尚未开放这些控制接口。

### 3.2 平台与远程访问

旧平台完成可操作导航；正式平台完成五路相机和病害结果只读展示。用户已在手机端通过 Tailscale 验证连接和实时画面。

平台访问概念已经明确：

```text
127.0.0.1     只有运行服务的本机能访问
0.0.0.0       服务监听所有网卡，配合防火墙可供局域网访问
Tailscale     授权设备间的加密私网访问，可用 Serve 提供HTTPS入口
公网长期开放  需要Funnel、VPS或云服务，并补身份认证和合规方案
```

`*.ts.net` 是 Tailscale MagicDNS/证书名称，不是 GitHub 子域名。GitHub 可以托管代码或静态前端，但不能直接暴露某台电脑本地的 ROS 动态 API。

### 3.3 正式项目扫描与接口设计

进入 `metro-inspection` 后先只读扫描，再新增统一病害消息：

```text
metro_inspection_interfaces/msg/DefectEvent.msg
```

它用于把一次病害事件的二维框、相机、置信度、三维坐标、定位方法、线路里程、区段和时钟方位封装为统一接口。检测节点可以先发布二维结果，定位节点使用相同 `event_id` 更新三维和语义字段，平台按 `event_id` 去重和更新。

二维检测节点将由队友提供，因此当时跳过真实 YOLO 开发，只完成接口、定位链和平台接收端。

### 3.4 最新模型、传感器和方向修正

项目已植入 `tunnel_8.10` 和 V6 机器人，完成法兰轮、轨距、传感器、TF 和轮轨运动。Pitch 相机和车头方向曾多次出现视觉判断与坐标定义不一致，最终通过提交 `6180c7a` 统一。

### 3.5 三维建图与定位融合

先建立有限体素累计点云基线，再加入轮速和 IMU 的 EKF，之后升级为 ICP + EKF + RTAB-Map 位姿图。运行中发现车辆静止时点云仍持续送入 RTAB-Map，导致临时节点和地图装配反复增长，因此新增运动关键帧门控、地图变化门控、体素存储和图验收脚本。

### 3.6 技术报告

最初误生成过整份报告补充版。用户随后明确“只填写第七章”，最终以原始框架为底稿，仅替换第七章，并把路径选择导航并入 `7.6`；其他章节正文保持原样，目录自动更新。

## 4. 当前模型与仿真状态

### 4.1 隧道

```text
模型：tunnel_8.10
Gazebo模型：subway_tunnel_v2
模型比例：1 1 1（原生米制）
有效轨道长度：约31.2 m
左轨中心：y=+0.754 m
右轨中心：y=-0.754 m
```

复杂 DAE 用于视觉外观，简化中部支撑面和左右钢轨碰撞用于动力学。这样可以避免薄面、孔洞和视觉搭接但碰撞不连续的问题。184 MB DAE 由 Git LFS 管理。

### 4.2 V6机器人和法兰轮

```text
原始轮面中心距：0.5016 m
隧道轨距：      1.508 m
统一缩放系数：  3.0063795853269537
缩放后轮面宽度：约0.0571 m
法兰内侧间隙：  约3.6 mm
```

- 四轮已拆分为固定安装件和旋转法兰轮；
- 车辆由 `libgazebo_ros_diff_drive.so` 驱动；
- 已移除直接修改位姿的 `planar_move`；
- 差速插件发布 `/wheel/odom_raw`，但不发布 odom TF；
- 车头、正向速度、Pitch视角和 `base_footprint +X` 已统一；
- `base_link.STL` 已包含 Odin1 外壳，独立 `leida.STL` 不再重复显示外观。

涉及方向的最新已提交修正：

```text
6180c7a fix(sim): align robot nose with forward motion
ac26c95 fix(localization): anchor EKF with lidar pose
```

再次修改方向前必须同时核对：网格车头、`base_footprint +X`、正 `cmd_vel.linear.x`、雷达前向和相机画面。

### 4.3 分模式运行

```text
Preview   检查模型、方向和碰撞
Sensors   启动完整传感器
Fusion    启动图像点云定位所需传感器
Mapping   关闭六路RGB和GUI，集中资源运行建图
```

Gazebo 实时倍率取决于物理步长、传感器数量、GPU/CPU和GUI负载。六路高分辨率相机、GPU Ray 点云和复杂隧道会显著降低实时倍率，建图时不应把所有相机同时打开。

### 4.4 照明仿真判断

当前隧道采用分布式顶灯、低强度环境光和带材质反射的网格。严谨试验应根据实测数据设置多组：

- 照度；
- 灯具间距；
- 色温；
- 相机曝光和增益；
- 运动速度；
- 阴影、反光和暗区。

每组工况分别统计检测率、误检率和三维定位误差，不能只选择视觉上“好看”的灯光。

## 5. 传感器、相机和TF

### 5.1 Odin1

```text
点云：/odin1/cloud_raw
类型：sensor_msgs/msg/PointCloud2
frame_id：lidar_link
水平/垂直采样：240 / 180
单帧理论点数：43200
水平/垂直FOV：120° / 90°
量程：0.2～30 m
请求频率：10 Hz
距离噪声标准差：0.03 m

IMU：/odin1/imu
frame_id：imu_link
请求频率：100 Hz
```

Odin1 RGB 当前使用设备 `O1-P070100024` 的已有内参：

```text
fx=736.9688
fy=737.0365
skew=0.2058
cx=766.6570
cy=642.9091
```

完整畸变系数尚未提供，仿真暂按零畸变。Gazebo 插件 CameraInfo 只作近似，校准节点发布权威矩阵。

### 5.2 五路巡检相机

```text
XJ1、XJ2：侧壁区域
XJ3、XJ4：轨道区域
Pitch：正立的巡检视角
仿真分辨率：1440×1080
仿真请求频率：30 Hz
```

五路相机都有独立 `camera_link` 和 `optical_frame`，但目前复用一套内参用于仿真集成。这不代表五台实物已完成独立标定。

### 5.3 当前唯一TF树

```text
map                              RTAB-Map发布
└── odom
    └── base_footprint           EKF发布
        └── base_link
            ├── lidar_link
            │   ├── imu_link
            │   └── odin1_rgb_optical_frame
            ├── xj1_optical_frame
            ├── xj2_optical_frame
            ├── xj3_optical_frame
            ├── xj4_optical_frame
            └── pitch_camera_optical_frame
```

TF所有权必须保持：

```text
EKF：      odom -> base_footprint
RTAB-Map： map -> odom
ICP：      不发布TF
Gazebo：   publish_odom_tf=false
```

## 6. 病害识别与二维到三维定位

### 6.1 当前接口链

```text
/odin1/rgb/image_raw
/odin1/rgb/camera_info
/odin1/cloud_raw
/damage_detections
/tf 和 /tf_static
  -> 图像/点云近似同步
  -> 点云投影到图像
  -> 检测框或掩码内点云筛选
  -> DBSCAN与中值估计
  -> /damage_point_camera
  -> TF转换
  -> /damage_point_global
  -> DefectEvent
  -> 正式平台
```

队友检测节点约定：

```text
topic: /damage_detections
type:  vision_msgs/msg/Detection2DArray
```

当前需要保持的边界：

- 真实 YOLO 尚未接入；
- HSV 检测器 `run_placeholder_detector=false`，只用于测试通信；
- 当前隧道没有正式病害真值；
- 原演示环境的 `truth_xyz`、里程和区段参数不能直接沿用；
- 没有真值时不能编造定位精度。

### 6.2 相机和雷达视场问题

五路相机并不都与前向雷达在同一时刻具有视场重叠。需要区分：

1. 相机看到某处时，雷达同一帧也能看到该表面：可以直接做同步投影。
2. 相机先看到、车辆移动后雷达才看到：需要可靠时间戳、位姿和累计点云地图，把不同时刻的数据变换到统一坐标系。
3. 某个表面始终不在雷达视场内：累计地图也不会凭空产生深度，需要调整传感器方向、与隧道曲面求交，或增加深度传感器。

因此“积累点云地图”可以复用雷达过去已经测到的表面，但不能解决从未被雷达观测的区域。

## 7. 正式五路相机平台

### 7.1 架构与接口

```text
五路 CompressedImage + /defect_events
  -> metro_dashboard_bridge
  -> FastAPI只读API和MJPEG
  -> dashboard网页
```

平台已完成：

- 桌面端五路相机；
- 移动端只连接当前选中相机；
- 相机在线、离线和更新时间；
- 病害列表、相机/类型/等级筛选；
- 二维框、三维坐标、定位方法、里程和隧道语义；
- 类型/等级统计和打印报告；
- 按 `event_id` 校验、去重和更新；
- 不生成随机病害。

API：

```text
GET /api/health
GET /api/defects
GET /api/defects/{event_id}
GET /api/cameras
GET /api/cameras/{camera_id}/stream.mjpg?fps=6
```

当前API只读，病害只能由 ROS 2 `DefectEvent` 进入。记录最多保留500条，存储在进程内存中，服务重启后清空。

### 7.2 图像码率参数

```text
预览最大尺寸：640×360，保持宽高比
JPEG质量：55
默认输出FPS：6
最大输出FPS：10
相机离线超时：3 s
移动端：单路连接
```

降低 FPS 会减少每秒传输的帧数；降低 JPEG 质量会减少单帧字节数。二者影响网络码率和移动端解码负载，但不改变源相机本身的采集 FPS。没有客户端观看时，桥接节点不执行 OpenCV 解码和二次编码。

### 7.3 端口和Tailscale

默认平台端口是 `8088`；如果已被其他进程占用，可以使用 `8089` 或其他空闲端口。端口号不是永久绑定，Gazebo 也不会天然占用 `8088`。占用端口的是正在监听该端口的具体进程，例如旧的 HTTP 服务或另一个平台实例。

检查端口：

```bash
ss -ltnp | grep ':8088'
```

本机平台通过 Tailscale Serve 暴露到授权 tailnet 的典型命令：

```bash
tailscale serve --bg http://127.0.0.1:8088
tailscale serve status
```

如果平台实际运行在 `8089`，Serve 的目标也要改成 `8089`。手机端已经完成连接和实时画面验证。

别人 clone 仓库后可以在自己的电脑安装依赖并启动自己的本地平台，不需要与用户处于同一网络。若要访问用户电脑上正在运行的实例，则需要局域网连通或加入授权 Tailscale 网络。

## 8. 旧导航训练原型

### 8.1 两种模式

```text
到岔口询问：公共段 -> 停车 -> waiting_choice -> 人工选直行/分叉 -> 继续
最终目的地：选择 main_end/branch_end -> 映射路线 -> 一次提交完整路径
```

当前岔口“识别”由到达路线公共段等待点触发。正式环境以后可用视觉或激光岔口识别替换事件来源，平台接口和路线执行器无需重写。

### 8.2 路线和状态机

```text
idle -> starting -> running -> completed
                       ├──> waiting_choice -> running
                       ├──> canceling -> paused -> starting
                       └──> canceling -> idle
任意运动状态 -> estop -> reset -> idle
```

路线由 JSON 的 `frame_id` 和 waypoints 组成，上层决定路线，Nav2 负责路径连接和速度输出。正式 SLAM 环境迁移时应把 `frame_id` 从 `odom` 改为 `map`，并重新测量轨道中心线和岔口坐标。

Nav2 没有原生暂停语义。原型通过取消当前 Action、保存剩余路径点，并在继续时从当前位置重新提交。取消会清空任务。

### 8.3 安全控制

```text
Nav2 /cmd_vel
  -> tunnel_obstacle_guard
  -> /cmd_vel_safe
  -> cmd_vel_watchdog（0.5 s速度超时，20 Hz零速度输出）
  -> /cmd_vel_drive
  -> 差速底盘
```

- 急停会取消 Action，并持续输出零速度；
- 急停期间忽略新速度；
- 复位只恢复安全许可，不自动恢复运动；
- 运动期间平台心跳超过3秒会锁存急停；
- 平台不应直接绕过安全链发布底盘速度。

### 8.4 启动旧联合原型

```bash
cd ~/my-project/subway-patrol-robot-sim/subway-patrol-robot-sim-simulation
bash scripts/open_latest_branch_platform.sh
```

```text
HTTP：http://127.0.0.1:8088
ROS_DOMAIN_ID=31
GAZEBO_MASTER_URI=http://127.0.0.1:11356
```

该命令只用于旧原型验收，不能与正式 `metro-inspection` 的 Domain 70 和 world 混用。

## 9. 当前三维建图与定位方案

### 9.1 为什么需要轮速、IMU、ICP和回环

- 轮式编码器提供连续里程，但轮径误差、左右轮差、打滑和轨面污染会累计漂移。
- Odin1 IMU 提供短时角速度和运动约束，帮助 EKF 在轮速不稳定时保持姿态连续。
- ICP（Iterative Closest Point）通过相邻点云配准估计局部运动，提供与轮速不同来源的位姿约束。
- EKF 融合轮速、IMU和激光里程计，输出连续 `odom -> base_footprint`。
- RTAB-Map 识别重复经过区域并优化整个位姿图，用于修正累计漂移和历史点云位置。

编码器是里程计的重要来源，但不是无误差的绝对定位。现实系统仍需要 IMU、激光/视觉里程计、地图约束和必要的回环校正。

### 9.2 当前数据流

```text
/odin1/cloud_raw
  -> cloud_gate
  -> /mapping/cloud_valid
     ├──> ICP全部有效帧 -> /lidar/odom
     └──> motion_cloud_gate -> /mapping/cloud_keyframe -> RTAB-Map

/wheel/odom_raw + /odin1/imu + /lidar/odom
  -> EKF
  -> /odometry/filtered
  -> odom -> base_footprint

RTAB-Map mapData
  -> map_data_gate
  -> map_assembler
  -> /mapping/cloud_map
  -> /mapping/save_map
```

### 9.3 当前关键参数

| 模块 | 参数 | 当前值 |
|---|---|---:|
| `cloud_gate` | 最小点数 | 5000 |
| `motion_cloud_gate` | 平移阈值 | 0.35 m |
| `motion_cloud_gate` | 旋转阈值 | 0.10 rad |
| `motion_cloud_gate` | 最大里程计年龄 | 0.25 s |
| ICP | 输入体素 | 0.10 m |
| RTAB-Map | 检测频率 | 2 Hz |
| RTAB-Map | 线性更新阈值 | 0.30 m |
| RTAB-Map | 角更新阈值 | 0.08 rad |
| RTAB-Map | 节点点云体素 | 0.05 m |
| `map_data_gate` | 平移/旋转阈值 | 0.01 m / 0.005 rad |
| 最终地图 | Grid CellSize | 0.05 m |

`minimum_points=5000` 来源于现有仿真 bag 统计，是数据集专用保护值。实车必须先统计真实有效点数分布，再重新设置。

### 9.4 静止内存增长问题与修复

早期车辆静止时仍约10 Hz向 RTAB-Map 输入点云，导致临时节点反复创建/丢弃，`map_assembler` 也重复装配，最终可能 OOM。

当前未提交改动增加：

- `motion_cloud_gate`：只有达到运动阈值才向 RTAB-Map 送关键帧；
- `map_data_gate`：只有图位姿显著变化才触发地图装配；
- 节点存储点云使用 `0.05 m` 体素；
- RTAB-Map 线性更新阈值调整为 `0.30 m`；
- `map_assembler` 异常退出后最多重启1次，并有15秒延迟；
- 累计 MapGraph 检查脚本和自动验收测试。

### 9.5 最新验收记录

```text
实际移动：3.860 m
运动关键帧：10

静止两分钟：
RTAB-Map      395008 -> 395264 KiB
map_assembler 594048 -> 594304 KiB
两者各波动256 KiB，无持续线性增长

恢复已有数据库后的第一次图更新：
总RSS一次性增加约872 MiB，之后90秒保持稳定

MapGraph：
poses：112
轨迹范围：9.452 m
proximity links：103
near-revisit links：52
最小邻近约束平移：0.001 m

优化PCD：268239 points
自动验收：SLAM_ACCEPTANCE=PASS
测试：164 tests, 0 errors, 0 failures, 17 skipped
```

这些结果证明当前短距离图满足现有验收规则，并且静止内存不再线性增长；它们不能单独证明重复隧道中的长距离回环完全可靠。

### 9.6 当前地图成果

```text
/home/jo/my-project/metro-inspection/results/maps/subway_v2_optimized.pcd
大小约3.2 MB

/home/jo/my-project/metro-inspection/results/maps/subway_v2_rtabmap.db
大小约128 MB
```

PCD 是优化后的点云成果；数据库保存关键帧、约束和图状态，用于检查或继续建图。继续已有数据库时必须显式设置 `SUBWAY_MAPPING_RESET_DATABASE=false`。

## 10. 正式项目常用命令

### 10.1 每个新终端的环境

```bash
source /opt/ros/humble/setup.bash
source ~/my-project/metro-inspection/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=70
```

### 10.2 构建和测试

```bash
cd ~/my-project/metro-inspection/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

### 10.3 完整传感器仿真

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_sensors.sh
```

需要GUI参数时必须完整输入 `gui:=true`，不能写成 `gui:=tr`。

### 10.4 正式五路平台

另一个终端：

```bash
cd ~/my-project/metro-inspection
bash scripts/open_defect_dashboard.sh
```

```text
http://127.0.0.1:8088
```

健康检查：

```bash
curl http://127.0.0.1:8088/api/health
```

### 10.5 图像点云融合

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_fusion.sh
```

另一个终端打开RViz：

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_v2_perception_rviz.sh
```

脚本必须从仓库根目录运行；此前在 `/mnt/c/Users/jo199` 直接使用相对路径失败，是因为当前目录不包含 `ros_ws`。

### 10.6 新建三维地图

终端1：

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_mapping.sh
```

默认配置：

```text
ROS_DOMAIN_ID=70
GAZEBO_MASTER_URI=http://127.0.0.1:11372
GUI=false
reset_database=true
```

终端2：

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_v2_mapping_rviz.sh
```

### 10.7 继续已有地图

```bash
cd ~/my-project/metro-inspection
SUBWAY_MAPPING_RESET_DATABASE=false \
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_mapping.sh
```

### 10.8 控制车辆移动和停止

新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/my-project/metro-inspection/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=70

ros2 topic pub --rate 10 /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

按 `Ctrl+C` 结束连续发布后，再明确发送零速度：

```bash
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist '{}'
```

控制小车移动的目的不是“让地图看起来会动”，而是提供不同位置的点云关键帧，验证 ICP、EKF、位姿图和回访约束。车辆静止时不应持续生成新关键帧。

### 10.9 保存和验收地图

```bash
ros2 service call /mapping/save_map std_srvs/srv/Trigger '{}'

cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/check_subway_v2_slam.sh
```

验收脚本检查：

- 8个核心节点；
- 有效点云、ICP和融合里程计；
- `odom -> base_footprint` 与 `map -> odom`；
- 持久化 MapGraph；
- 邻近/回访约束；
- PCD与数据库；
- 最终 `SLAM_ACCEPTANCE=PASS`。

## 11. 常见问题和已经确认的原因

### 11.1 `Address already in use`

表示某个进程已经监听该端口，不表示 Gazebo 每启动一次就占用一个网页端口。检查：

```bash
ss -ltnp | grep ':8088'
```

可以关闭旧服务或为新平台选择另一个端口。

### 11.2 话题不存在

如果 `/odom`、`/odin1/imu` 或 `/odin1/cloud_raw` 不存在，通常是仿真/节点未启动、工作空间未 source 或 `ROS_DOMAIN_ID` 不一致。先确认启动终端仍在运行，再执行：

```bash
ros2 node list
ros2 topic list
```

### 11.3 RViz没动但ICP日志一直变化

ICP是在处理连续点云和尝试扫描配准，不是病害识别。即使车辆静止，传感器仍在发布带噪点云；没有运动门控时算法会重复计算。当前关键帧门控用于阻止静止点云继续扩展位姿图。

### 11.4 黄字持续滚动

通常是 RTAB-Map/ICP 的警告、状态或匹配信息。应读取具体消息，不能仅凭颜色判断故障。若RViz无位姿变化，同时 `/mapping/cloud_valid` 正常，先检查 `/lidar/odom`、`/odometry/filtered` 和TF。

### 11.5 MVS与RTAB-Map库冲突

海康 MVS SDK 自带旧版 `libusb`，如果 `/opt/MVS` 位于 `LD_LIBRARY_PATH` 前部，PCL/RTAB-Map 会出现 `libusb_set_option` 未定义。提交 `2ac9e46` 已让 Mapping 模式优先使用 Ubuntu 系统库，并在启动前运行 `ldd -r` 检查。

### 11.6 ROS软件包安装TLS失败

曾使用清华 ROS 2 镜像时出现 `Could not handshake`。之后已确认以下包可被 ROS 2 找到：

```text
robot_localization
rtabmap_slam
rtabmap_odom
rtabmap_util
```

不要因为旧的 apt 输出再次重复安装，先运行 `ros2 pkg prefix` 核对。

### 11.7 模型方向又反了

该问题出现多次的原因是 CAD 网格自身朝向、URDF根链接姿态、Gazebo模型初始yaw、正速度方向和相机视角被分开修改。当前已经统一，任何再次修改必须进行完整方向链验收，而不是只旋转一个 mesh。

### 11.8 Gazebo启动慢或实时倍率低

主要负载来自184 MB隧道、六路RGB、GPU Ray点云、复杂碰撞和GUI。应选择对应运行模式，不要为了建图同时开启全部相机和GUI。

### 11.9 Tailscale手机登录或网页无响应

需要分别确认：手机Tailscale客户端已登录、设备加入同一tailnet、Serve已启用、平台本机端口可访问、Serve目标端口正确。手机浏览器不能直接使用电脑的 `127.0.0.1`。

## 12. 当前Git现场

### 12.1 已提交的关键提交

```text
ac26c95 fix(localization): anchor EKF with lidar pose
6180c7a fix(sim): align robot nose with forward motion
2ac9e46 fix(mapping): isolate MVS runtime libraries
4018110 feat(mapping): add loop-closing 3d slam
d4a6f3b feat(localization): fuse wheel odometry and imu
b3725f5 feat(mapping): add accumulated point cloud baseline
6f5157d fix(sim): face v2 robot along patrol direction
69d09b4 fix(sim): align pitch camera and vehicle forward
1664983 fix(platform): match simulation ROS domain
f181f34 feat(platform): add five-camera inspection dashboard
6cb69a3 feat(sim): integrate V6 patrol robot and perception pipeline
```

### 12.2 当前未提交的建图稳定性改动

已修改：

```text
metro_pointcloud_mapping/CMakeLists.txt
metro_pointcloud_mapping/README.md
metro_pointcloud_mapping/config/graph_slam.yaml
metro_pointcloud_mapping/launch/graph_slam.launch.py
metro_pointcloud_mapping/package.xml
metro_pointcloud_mapping/test/test_graph_slam_config.py
metro_sim/README.md
metro_sim/scripts/check_subway_v2_slam.sh
```

新增：

```text
map_graph_change_detector.hpp/.cpp
planar_motion_gate.hpp/.cpp
map_data_gate_node.cpp
motion_cloud_gate_node.cpp
scripts/check_rtabmap_graph.py
test_map_graph_change_detector.cpp
test_planar_motion_gate.cpp
test_rtabmap_graph_checker.py
```

另有未跟踪内容：

```text
docs/biweekly_report_2026-08-05_to_2026-08-18.md
docs/conversation_handoff_2026-08-26.md
docs/conversation_handoff_2026-08-28.md
results/
```

提交前必须逐项暂存代码和测试，不要把 `results/` 一起加入。

固定检查命令：

```bash
cd ~/my-project/metro-inspection
git branch --show-current
git status -sb
git diff --stat
git log -5 --oneline --decorate
git remote -v
```

## 13. 已生成文档和报告

### 13.1 本交接文件

```text
/home/jo/my-project/metro-inspection/docs/conversation_handoff_2026-08-28.md
```

### 13.2 技术报告最终第七章版

```text
C:\Users\jo199\Desktop\地铁多领域病害综合智能巡检系统研究技术报告框架1(1)-第七章补充完善版.docx
C:\Users\jo199\Desktop\地铁多领域病害综合智能巡检系统研究技术报告框架1(1)-第七章补充完善版.pdf
```

最终版只替换第七章，并更新自动目录：

```text
7.1 总体设计
7.2 隧道数字孪生
7.3 V6机器人
7.4 多源传感器与TF
7.5 三维点云建图与融合定位
7.6 路径选择导航与运行安全控制
7.7 病害检测、三维定位与数据闭环
7.8 五路相机可视化与远程访问
7.9 系统集成与命令
7.10 测试结果
7.11 当前边界
7.12 本章小结
```

PDF共26页，第七章位于第12～23页；DOCX结构和PDF页面已检查，无空白页和文字越界。

以下文件是此前误生成的全报告版本，不是最终交付物，不要把它当作用户要求的结果：

```text
...-补充完善版.docx
...-补充完善版.pdf
```

## 14. 下一阶段建议顺序

### P0：保护现有成果

1. 先只读检查当前 diff。
2. 用户确认后，把建图稳定性代码、测试和文档按明确文件列表提交到 `feature/localization-fusion`。
3. 制定 `results/` 的忽略、压缩、备份和地图制品发布策略。

### P0：完成建图严谨性验证

1. 新数据库完成 `20～50 m` 往返路线。
2. 重复至少3次独立运行。
3. 检查真回环、错误回环、`map -> odom`变化和最终重叠误差。
4. 记录CPU、RSS、点云频率、实际实时倍率和地图点数随距离增长的曲线。
5. 再决定当前阈值是否可作为正式仿真参数。

### P0：接入队友YOLO

1. 获取检测节点包、依赖、模型权重和类别表。
2. 确认输入图像、输出消息、时间戳、相机名称和坐标约定。
3. 将输出适配为 `Detection2DArray` 或直接发布 `DefectEvent`。
4. 先单路相机联调，再扩展五路并处理资源调度。
5. 使用当前隧道的正式仿真病害真值做误差评价。

### P1：标定与实车迁移

1. 五台相机逐台标定内参、畸变和外参。
2. 标定 Odin1 雷达、RGB、IMU 和车体之间的外参。
3. 检查硬件时间同步、QoS、有效点数和协方差。
4. 根据实车 bag 重新调整点云门槛、ICP和EKF。
5. 保持上层话题和 `DefectEvent` 不变，通过 remap 替换仿真驱动。

### P1：正式平台功能

1. 增加数据库、截图归档、历史查询和正式报告导出。
2. 接入导航状态、里程和安全状态。
3. 经过权限、心跳、急停和实车制动距离审查后，再迁移控制接口。
4. 不允许网页直接绕过安全链发布底盘速度。

## 15. 新对话建议开场提示

把下面内容发给新对话即可：

```text
请先完整阅读：
/home/jo/my-project/metro-inspection/docs/conversation_handoff_2026-08-28.md

当前正式仓库是 /home/jo/my-project/metro-inspection，分支为
feature/localization-fusion，记录时HEAD为ac26c95。工作区包含已经验收但尚未提交的
建图稳定性改动，以及约9.8 GB的results。不要切换或创建分支，不要运行git add -A，
不要删除results。先执行只读的git status、git diff --stat、git log和colcon test-result
核对现场，然后再继续任务。回答使用中文，给出完整命令并解释关键现象。
```

## 16. 一句话状态总结

当前项目已经具备 `tunnel_8.10 + V6法兰轮巡检车 + Odin1/IMU/五路相机 + 唯一TF + 轮速/IMU/ICP融合 + RTAB-Map优化点云 + DefectEvent + 五路远程平台 + 单分岔导航原型`；核心剩余工作是真实YOLO、正式病害真值、长距离回环压力测试、逐台标定、平台数据持久化和经过安全审查的正式导航控制迁移。
