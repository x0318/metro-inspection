# 岔轨导航测试模块

从旧 `subway-patrol-robot-sim-simulation` 项目迁入已有的
`route_choice_training.world`、蓝色简易小车、两条路线和选路平台。
所有运行资源随本包安装，不需要旧项目目录或旧工作空间。
小车使用原有平面运动插件，轨道为可视几何加连续承载面；用于导航流程测试，
不是正式巡检车的动力学或避障性能验收。

在 `metro-inspection` 根目录运行：

```bash
bash scripts/open_route_choice_platform.sh
```

浏览器访问 `http://127.0.0.1:8090`。默认打开 Gazebo，不打开 RViz；启动后静止等待任务。
首次运行自动构建本包，需要 ROS 2 Humble、Gazebo Classic 11、Nav2、FastAPI、Uvicorn、
WebSockets、OpenCV。依赖可通过 `rosdep install --from-paths ros_ws/src --ignore-src -r -y --rosdistro humble` 安装。

```bash
# 后台仿真，保留网页相机
bash scripts/open_route_choice_platform.sh gui:=false
# 额外打开 RViz
bash scripts/open_route_choice_platform.sh rviz:=true
```

默认 ROS 域为 74、Gazebo 端口为 11374、网页端口为 8090，与正式 Qt 巡检平台分开。
用启动参数 `domain:=74 port:=8090 gazebo_port:=11374` 可以修改，整套启动会统一传递。
端口占用时明确退出，避免打开页面后连接到另一台仿真小车。

## 平台操作

- 到岔口询问：启动任务，行驶到预设等待点，停车后选择直行或分岔。
- 最终目的地：选择主轨终点或分岔终点，再启动。
- 暂停、继续、取消、急停沿用迁入平台的接口；急停复位不会自动重新启动任务。
- 四路画面分别来自前向、左前轨道、右前轨道和近处轨道相机。前向相机下俯约 22°；
  左右相机下俯约 37°、向内偏转约 11°；近处轨道相机升高并下俯约 60°，覆盖两根钢轨和多根轨枕。
  四路原始画面均为 16:9，与网页预览比例一致。修改模型相机参数后需要重启场景。
- 每轮测试从初始位置开始；再次测试另一条完整路线前关闭并重启场景。

## 接口

| 功能 | HTTP 接口 |
| --- | --- |
| 状态、里程计、导航进度 | `GET /api/status` 或 `/ws/status` |
| 可选模式、目的地、岔路 | `GET /api/navigation/options` |
| 提交任务 | `POST /api/navigation/start` |
| 岔路选择 | `POST /api/navigation/junction-choice` |
| 暂停、继续、取消 | `POST /api/navigation/pause`、`resume`、`cancel` |
| 急停、复位 | `POST /api/safety/estop`、`reset` |
| 相机列表 | `GET /api/cameras` |
| 相机画面 | `GET /api/camera/{front,left,right,ground}.mjpeg` |

写接口需要 `GET /api/session` 返回的 token 放入 `X-Session-Token`。
页面每秒调用 `/api/session/heartbeat`；执行任务期间心跳中断超过 3 秒触发急停。
状态监控和心跳使用稳态时钟，导航目标时间戳使用 Gazebo 仿真时间。

速度链为 Nav2 `/cmd_vel` → 命令超时 watchdog → `/cmd_vel_drive` → 小车。
预设等待点与路线选择不代表自动识别岔轨；本测试场未启用病害检测和障碍物避让。
旧平台的病害表不会自动生成测试记录，也不接入正式巡检数据库。

## 验证

```bash
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
python3 -m pytest ros_ws/src/metro_navigation_demo/test -q
```

资源来自用户的旧项目；第三方网页依赖及许可见 `web/inspection_dashboard/THIRD_PARTY_NOTICES.md`。

### 2026-09-10 迁移验证

- 包构建和 13 项接口、状态、路线及安装资源测试通过。
- 从当前包的安装目录启动完整仿真，四路相机均返回实际图像。
- 到岔口询问模式：暂停后速度归零，继续到达等待点，再选择分岔并到达 `(10.903, 10.923)`；
  配置终点为 `(11, 11)`。这些是仿真里程计位置，不是病害定位预测结果。
- 浏览器实际点击目的地模式和启动按钮，直行到达 `(11.888, 0.032)`；配置终点为 `(12, 0)`。
- 页面急停、复位通过，复位后保持静止；四路图片均为 640×360，浏览器无脚本错误，
  390 像素移动端页面无横向溢出。
- 本包关键进程退出会联动关闭该次启动；本机 Gazebo Classic 退出曾触发约 15 秒的超时清理，
  清理后端口释放。未进行长期稳定性或真机导航验收。
