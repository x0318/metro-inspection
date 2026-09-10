# 导航复用与停车保护评估

> 历史评估：本文描述评估时的状态。2026-09-10 已迁入独立的
> `metro_navigation_demo` 岔轨平台，并删除本项目废弃的旧导航入口、旧世界和旧车模型。
> 当前运行说明见 [岔轨导航测试模块](../ros_ws/src/metro_navigation_demo/README.md)。

## 1. 评估范围

导航参考工作区为 `subway-patrol-robot-sim-simulation` 的
`simulation-demo` 分支。本次按工作区当前文件内容只读检查，没有切换分支、覆盖未提交
改动或合并分支，也没有迁移旧世界、旧机器人模型、网页或训练数据。

目标工作区为 `metro-inspection`。本次只修改自动巡检里程计保护、最终速度 watchdog、
相应参数和测试，没有修改 YOLO、三维定位、病害事件、网页、隧道或机器人模型。

## 2. 参考导航已经实现的内容

当前可复用的已实现能力包括：

- 从 JSON 读取、校验命名路线和连续 waypoints；
- 命令行预先选择 `straight` 或 `branch`；
- 调用 Nav2 `NavigateThroughPoses`，处理服务器等待、反馈、结果和超时取消；
- 在 `odom` 坐标系运行的 `NavigateThroughPoses` 行为树；
- Nav2 参数、障碍守卫、命令超时 watchdog 和软件急停的原型；
- 单岔口训练世界中的一次分叉路线端到端验证。

以下内容只存在于文档设计中，不能按“已经实现”迁移：

- 到达岔口后自动停车并向平台询问方向；
- 根据最终目的地搜索轨道拓扑并自动决定多个岔口方向；
- 常驻路线管理节点及稳定的 Service/Action 接口；
- 平台的模式切换、路线提交和完整导航状态推送。

`send_nav_route.py` 目前只是一次性 Action 客户端。它只等待收到一帧 `/odom`，没有
检查数值、消息时间戳或持续新鲜度，因此不能单独承担定位安全保护。

## 3. 两个项目的关键差异

| 项目 | 参考导航工作区 | `metro-inspection` 当前机器人 |
| --- | --- | --- |
| 机器人主体 | V5，缩放约 `4.58965` | V6，运行时缩放约 `3.00638` |
| 碰撞尺寸 | 主体约 `2.7009 x 2.5447 m`；Nav2 footprint 为 `2.72 x 2.56 m` | 运行时主体水平投影约 `1.40 x 1.43 m`，传感器碰撞还会扩大外廓；现有 Nav2 footprint 为 `1.30 x 1.45 m`，必须重新测量并留裕量 |
| 运动模型 | `gazebo_ros_planar_move`，运动学直接赋值 | 四轮差速驱动，运行时轮距 `1.508 m`、轮径约 `0.36077 m`，存在动力学、加速度和打滑影响 |
| 转向能力 | 接收前进和 `angular.z` | 差速转向可用，但目标项目现有 `tunnel_obstacle_guard.py` 强制 `angular.z=0`，不能直接用于分叉导航 |
| 原始里程计 | `/odom` | `/wheel/odom_raw` |
| 融合里程计 | 无单独融合层 | EKF 融合 `/wheel/odom_raw` 和 `/odin1/imu`，输出 `/odometry/filtered` |
| TF 所有者 | Gazebo 插件发布 `odom -> base_footprint` | 差速插件不发布 TF，EKF 发布 `odom -> base_footprint` |
| 巡检相机 | XJ1-XJ4、Pitch；另有 Odin1 RGB | XJ1-XJ4、Pitch；另有 Odin1 RGB，外参和模型不同 |
| 激光数据 | 配置中存在 `/scan` 和 Odin1 点云 | V6 实际输出 `/odin1/cloud_raw` `PointCloud2`，不输出 `/scan` |
| 仿真时间 | Nav2 使用 `use_sim_time=true` | 仿真节点使用 `use_sim_time=true`；安全超时必须改用 steady clock |
| 最终底盘入口 | `/cmd_vel_drive` | `/cmd_vel_drive` |

目标项目现有 `metro_sim/config/nav2_odom_params.yaml` 仍订阅 `/odom` 和 `/scan`，而 V6
的有效接口是 `/odometry/filtered` 和 `/odin1/cloud_raw`。现有
`metro_sim/scripts/open_nav2_demo.sh` 启动的也是旧 `gazebo_train` 和旧世界，并非 V6
导航入口。因此这些文件只能作为旧演示参考，不能视为导航已接入当前巡检机器人。

## 4. 启动入口实际启用的保护

| 启动入口 | 实际速度链 | 保护结论 |
| --- | --- | --- |
| 参考 `open_nav2_demo.sh` | Nav2 `/cmd_vel` -> 障碍守卫 -> `/cmd_vel_safe` -> watchdog -> `/cmd_vel_drive` | 同时有激光守卫和命令超时/软件急停 |
| 参考 `open_route_choice_training.sh` | Nav2 `/cmd_vel` -> watchdog -> `/cmd_vel_drive` | 没有启动障碍守卫 |
| 参考 `send_nav_route.py` | 只发送 Action | 是否有保护完全取决于已启动的 bringup |
| 目标 V2 sensors/fusion/mapping 启动脚本 | `/cmd_vel_safe` -> watchdog -> `/cmd_vel_drive` | 都启动最终 watchdog |
| 目标 `open_subway_tunnel_v2_yolo_coverage.sh` | 自动巡检 -> `/cmd_vel_safe` -> sensors 脚本中的 watchdog -> `/cmd_vel_drive` | 有本轮两层停车保护 |
| 单独运行目标 `open_yolo_coverage.sh` | 只启动检测、评估及可选自动巡检 | 不启动 watchdog，必须配合机器人 bringup |

参考行为树包含 `BackUp` 恢复动作，但参考 `open_nav2_demo.sh` 的障碍守卫禁止倒车，
所以该入口下 `BackUp` 不会真正后退。移植时需要让行为树和安全策略保持一致。

## 5. 独立导航包接入方案

建议新建标准 ROS 2 包 `metro_navigation`，不把导航代码继续放在 `metro_sim` 的散落
脚本中。建议结构如下：

```text
metro_navigation/
  package.xml
  setup.py
  metro_navigation/
    route_config.py
    route_manager.py
  config/
    nav2_v6.yaml
    routes.yaml
  behavior_trees/
    navigate_through_poses.xml
  launch/
    navigation.launch.py
  test/
```

可以复用：

- `send_nav_route.py` 的路线配置校验、Pose 生成、Action 反馈、结果和超时取消逻辑；
- `navigate_through_poses_odom.xml` 的规划/跟踪/清图结构；
- 命名路线和 waypoints 的数据组织方式；
- watchdog 的命令超时、软件急停和旧命令清除语义。

必须针对 V6 重配：

- 将 Nav2 里程计设为 `/odometry/filtered`，并验证唯一的
  `odom -> base_footprint` TF 所有者是 EKF；
- 从运行时碰撞外廓重新生成 footprint，不能复制 V5 footprint，也不应继续使用当前偏小
  的目标项目 footprint；
- 按四轮差速实测转弯半径、角速度、加减速度和停止距离调整控制器；
- 将障碍层改为消费 `/odin1/cloud_raw`，或增加经过高度/轨道过滤的
  PointCloud2-to-LaserScan 节点；
- 将目标项目障碍守卫改为支持差速转向，或由 Nav2 costmap/collision monitor 统一负责；
- 用正式隧道中心线重新测量路线点、岔口等待点和目的地，明确使用 `odom` 还是 `map`；
- 所有节点通过 launch 参数统一切换仿真时间和真实时间；
- Nav2 输出改为 `/cmd_vel_nav`，不得直接发布最终底盘话题。

## 6. 统一速度来源规则

未来速度链建议固定为：

```text
Nav2          -> /cmd_vel_nav
自动巡检      -> /cmd_vel_auto
手动控制      -> /cmd_vel_teleop
                      |
              唯一模式管理器 / twist_mux
                      |
              障碍、定位和运动约束安全门
                      |
                 /cmd_vel_safe
                      |
             最终 watchdog + 软件急停
                      |
                 /cmd_vel_drive
                      |
                     底盘
```

规则如下：

- 同一时刻只授权一个来源，不允许三个节点同时向 `/cmd_vel_safe` 发布；
- 手动接管前先取消 Nav2 Action 或停止自动巡检，再清除缓存命令；
- 急停和安全故障撤销所有来源授权；
- 模式切换、里程计 rearm、急停解除和 watchdog rearm 后，只接受操作之后到达的新命令；
- `/cmd_vel_drive` 始终只能有最终 watchdog 一个发布者；
- 在 mux 落地前，自动巡检、旧 Nav2 demo 和手动速度发布器必须互斥启动。

## 7. 本轮停车保护

自动巡检现在检查里程计位置、速度、协方差、四元数和时间戳是否有效，同时检查：

- 第一帧里程计是否按时到达；
- 接收流是否超过 `odometry_timeout_sec`；
- 消息是否超过 `odometry_max_age_sec`；
- 消息是否来自允许范围之外的未来；
- 时间戳是否严格递增。

任一检查失败都会持续发布零速度并进入锁存的 `fault_stop`。数据恢复不会自动恢复
行驶。必须先恢复连续有效里程计，再调用：

```bash
ros2 service call /simulation_coverage_driver/rearm std_srvs/srv/Trigger "{}"
```

rearm 后还要收到一帧时间戳更新的有效里程计，并重新经过启动延时。状态可查看：

```bash
ros2 topic echo /simulation/coverage_driver_state
```

最终 watchdog 保留命令超时停车，新增 `/safety/estop_active` 和状态话题。急停置为
`false` 只表示停车条件已经解除，仍需显式重新授权：

```bash
ros2 service call /cmd_vel_watchdog/rearm std_srvs/srv/Trigger "{}"
ros2 topic echo /safety/cmd_vel_watchdog_state
```

rearm 前和 rearm 过程中到达的速度均会被丢弃，rearm 后必须收到新命令才会输出。
保护定时器使用 steady clock，因此 Gazebo 暂停、`/clock` 停止推进时仍能按真实经过
时间停车。

参考项目的软件急停可复用的是 Bool 话题、持续零速度和清除旧命令的语义。参考
watchdog 自身不锁存操作者急停，也没有独立 rearm；目标项目在此基础上增加了显式
rearm。生产系统仍需一个常驻安全管理节点以可靠保持急停状态，并在真实机器人上使用
硬件急停或驱动器 STO；ROS 软件急停不能替代安全认证的硬件回路。

## 8. 验证结果

- `metro_detection` 包构建成功，包内 24 个测试通过；
- 里程计保护 12 个离线测试通过；
- 最终 watchdog 6 个离线测试通过；
- 隔离 ROS 域节点测试验证了正常输出、命令失联、仿真时钟冻结、急停、解除、rearm
  和新命令恢复；
- 隔离 ROS 域节点测试验证了自动巡检正常行驶、里程计中断、锁存停车、数据恢复仍停车、
  rearm 后等待新里程计及重新启动；
- Python 语法、Flake8、XML 和 `git diff --check` 均通过。

本轮没有在现有 Gazebo 会话或真实底盘上测试，也没有接管正在运行的 ROS 图。

## 9. 导航接入尚未完成的工作

1. 创建 `metro_navigation` 包并迁入通用路线客户端逻辑；
2. 完成 V6 footprint、差速运动约束、融合里程计和点云障碍层配置；
3. 实现唯一速度来源选择器，并让所有控制模式通过本轮最终 watchdog；
4. 在目标隧道中重新采集路线点，先验证单分叉，再扩展拓扑寻路；
5. 将“岔口询问”和“最终目的地”实现为路线管理节点接口；
6. 最后才将这些 ROS 接口接入平台，不从网页直接操作底盘速度。
