# metro_sim

`metro_sim` 提供 Metro Inspection 的 Gazebo 隧道仿真、巡检车辆、激光雷达、Nav2 直行导航和轨道障碍安全停车能力。

导航与安全模块由独立验证仓库 `subway-demo` 对接，当前同步基线：

```text
b04bcb3 feat: add tunnel-safe navigation guard
1de43a2 docs: add navigation safety test checklist
```

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic / `gazebo_ros`
- Nav2
- Python 3.10

## 仅预览隧道

```bash
cd ~/my-project/metro-inspection/ros_ws/src/metro_sim
./scripts/open_subway_tunnel.sh
```

## V6 完整传感器与建图模式

四种 V2 场景都在 `x=19.2、yaw=pi` 放置车辆，使 Pitch 相机端朝隧道
巡检前进方向。正 `linear.x` 沿车头行驶时，世界坐标 `x` 会减小。

完整传感器模式保留 Odin1、IMU 和六路 RGB 相机，使用端口 `11370`：

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_sensors.sh
```

该入口同时把 Gazebo Classic 的近似 Odin1 标定从
`/odin1/rgb/camera_info_gazebo` 转换为真实标定
`/odin1/rgb/camera_info`，与融合模式使用同一个接口。

点云建图模式从同一份 `subway_v2/model.sdf` 临时派生，只关闭六路
RGB 相机并默认不启动 Gazebo GUI；雷达的 `240 x 180` 分辨率、FOV、
量程、噪声和 10 Hz 请求值保持不变。默认端口为 `11372`：

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_mapping.sh
```

需要观察模型时可临时打开 GUI，但会降低高密度 GPU 雷达频率：

```bash
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_mapping.sh gui:=true
```

完整 sensors 仿真和融合节点运行后，使用统一入口查看 Odin1 原始图像、
带检测框/点云投影/定位十字的识别调试图、三维点云和病害位置标记：

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_v2_perception_rviz.sh
```

该脚本会自动加载 ROS 2 和当前工作空间，因此 RViz 可以解析
`package://metro_description/...` 的机器人网格。不要只在未加载工作空间的
终端直接运行 `rviz2 -d ...`，否则 RobotModel 会显示资源查找错误。

建图模式只在 `/tmp` 写入轻量 SDF/config，退出时自动删除；不会复制
`subway_v2` 的 STL，也不会在仓库中累积新模型。

## 启动导航与安全演示

```bash
cd ~/my-project/metro-inspection/ros_ws/src/metro_sim
bash scripts/open_nav2_demo.sh
```

默认使用 demo 参数：障碍持续 10 秒后报告，速度 watchdog 超时为 2 秒。

生产候选参数：

```bash
TUNNEL_GUARD_PROFILE=production bash scripts/open_nav2_demo.sh
```

生产候选配置使用 30 秒障碍报告和 0.5 秒 watchdog。上真机前必须根据实际通信抖动、制动距离和硬件急停链重新验收。

## 安全接口契约

```text
Nav2 /cmd_vel
→ tunnel_obstacle_guard
→ /cmd_vel_safe
→ cmd_vel_watchdog
→ /cmd_vel_drive
→ subway_v2_diff_drive（V2/V6 车辆）
→ 车辆
```

`open_nav2_demo.sh` 仍使用旧 `gazebo_train` 演示模型及其
`train_planar_move`；V2 传感器、建图和融合入口使用四轮
`subway_v2_diff_drive`，车轮会按轮轨接触真实旋转。

- `/scan`：前向障碍与雷达存活输入。
- `/odom`：车辆当前位置和相对导航目标基准。
- `/cmd_vel`：Nav2 原始速度，不允许底盘直接订阅。
- `/cmd_vel_safe`：守卫审核后的正向直行速度。
- `/cmd_vel_drive`：watchdog 输出的最终底盘速度，底盘唯一软件入口。
- `/navigation/obstacle_error`：持续阻挡错误报告。

轨道约束：

- 强制 `angular.z=0`。
- 禁止负向速度和后退恢复。
- 最大正向速度 `0.35 m/s`。
- 前方约 2 米出现障碍时停车，障碍退到约 2.5 米外后恢复。
- 雷达断流或安全命令流失联时进入停车状态。

## 快速验收

检查 Nav2 和安全链：

```bash
source /opt/ros/humble/setup.bash
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
ros2 topic info /cmd_vel_safe
ros2 topic info /cmd_vel_drive
```

发送 1 米相对导航目标：

```bash
cd ~/my-project/metro-inspection/ros_ws/src/metro_sim
bash scripts/send_nav_goal_forward.sh 1.0 120
```

检查 clearing 和 watchdog 参数：

```bash
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.inf_is_valid
ros2 param get /global_costmap/global_costmap obstacle_layer.scan.inf_is_valid
ros2 param get /cmd_vel_watchdog timeout
```

车辆位于初始位置 `x≈-20` 时，可提前生成前方障碍：

```bash
python3 scripts/manage_nav_test_obstacle.py spawn --x -15.0 --y 0.0
ros2 topic echo /navigation/obstacle_error
```

随后发送会穿过障碍位置的目标：

```bash
bash scripts/send_nav_goal_forward.sh 7.0 120
```

收到持续阻挡报告后删除障碍：

```bash
python3 scripts/manage_nav_test_obstacle.py delete
```

预期车辆不转向、不后退；障碍清除后继续完成原导航目标。

## 目录

- `models/subway_tunnel/`：Metro 原有隧道视觉与碰撞资源。
- `models/gazebo_train/`：巡检车辆、雷达和 Gazebo 插件配置。
- `worlds/subway_tunnel.world`：仅隧道预览世界。
- `worlds/subway_track_tunnel.world`：隧道、轨道行驶面和巡检车辆导航世界。
- `urdf/gazebo_train_tf.urdf`：RViz/TF 使用的车辆结构描述。
- `config/nav2_odom_params.yaml`：Nav2、Footprint、Obstacle Layer 与 clearing 参数。
- `config/tunnel_guard_demo.yaml`：WSL 仿真参数。
- `config/tunnel_guard_production.yaml`：生产候选参数。
- `scripts/open_nav2_demo.sh`：完整系统启动入口。
- `scripts/tunnel_obstacle_guard.py`：轨道直行与障碍停车守卫。
- `scripts/cmd_vel_watchdog.py`：守卫失联停车保护。
- `scripts/send_nav_goal_forward.py`：相对导航 Action 客户端。
- `scripts/manage_nav_test_obstacle.py`：测试障碍生成与删除工具。

## 对接边界

当前迁移的是已在 Subway 仿真中完成运行验收的软件导航与安全链。真实 Metro 车辆仍需提供：

- 与 `/cmd_vel_drive` 等价的底盘命令适配层；
- 可靠的 `/odom`、`/scan` 和完整 TF；
- 底盘固件通信超时、机械制动与硬件急停；
- 正式 dashboard/任务管理系统对 `/navigation/obstacle_error` 的订阅和持久化。
## 迁移验收记录

2026-07-19 已在 Metro 源码路径下完成独立运行验收：

- `bt_navigator` lifecycle 为 `active [3]`。
- `/cmd_vel_safe` 与 `/cmd_vel_drive` 均为 1 个发布者和 1 个订阅者。
- 从约 `x=-20.000` 向前导航 1 米，剩余距离持续下降并返回 `Navigation succeeded`。
- 前方测试障碍在约 `1.939 m` 处触发停车，车辆未转向、未后退。
- 持续阻挡实际报告：`NAVIGATION_BLOCKED: reason=front_obstacle duration=10.1s front_distance=1.939m`。
- 删除障碍后 Costmap 正常 clearing，车辆恢复原导航任务并成功完成。
