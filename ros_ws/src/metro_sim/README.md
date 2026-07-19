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
→ train_planar_move
→ 车辆
```

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
