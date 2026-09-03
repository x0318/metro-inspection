# metro_dashboard_bridge

订阅 `metro_inspection_interfaces/msg/DefectEvent`、五路原始相机图像和五路 YOLO
带框图像，托管巡检平台
页面，并提供只读病害记录与相机流 API。本包不运行二维识别算法，也不会生成随机或演示
病害。

## 数据流

```text
队友检测节点（以后接入）
  -> /defect_events
  -> metro_dashboard_bridge
  -> 按 event_id 校验、去重和更新
  -> GET /api/defects
  -> 巡检平台病害列表与统计模块
```

同一个 `event_id` 可以先发布二维结果，随后由三维定位或工程语义节点发布包含更多字段的
更新结果。存储层会替换该事件，而不是重复生成多条病害记录。默认最多保留 500 条记录，
并持久化到 `~/.local/share/metro-inspection/defects.sqlite3`；服务重启后仍会加载原记录。
`inspection_session_id` 用于隔离不同巡检批次，同一数据库中的不同会话不会互相覆盖。

```text
/subway_v2/{xj1,xj2,xj3,xj4,pitch_camera}/image_raw/compressed
以及 /damage_detection/{xj1,xj2,xj3,xj4,pitch}/annotated_image/compressed
  -> metro_dashboard_bridge（每路只保留最新压缩帧）
  -> 保持宽高比缩放、JPEG 质量和输出 FPS 限制
  -> GET /api/cameras/{camera_id}/stream.mjpg
  -> 桌面五画面 / 手机单画面
```

相机图像不会写入病害存储。原始画面和 YOLO 带框画面分别显示在“相机监控”和
“YOLO 识别监控”页面；没有网页观看时不会进行 OpenCV 解码和二次 JPEG 编码。

## 启动

首次在 Ubuntu 22.04 上使用时安装网页后端依赖：

```bash
sudo apt update
sudo apt install python3-fastapi python3-uvicorn python3-opencv python3-numpy \
  ros-humble-image-transport-plugins
```

推荐从仓库根目录一键构建并启动：

```bash
bash scripts/open_defect_dashboard.sh
```

也可以手动构建并启动：

```bash
cd /home/jo/my-project/metro-inspection/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to metro_dashboard_bridge
source install/setup.bash
export ROS_DOMAIN_ID=70
export METRO_DASHBOARD_DIR=/home/jo/my-project/metro-inspection/dashboard
ros2 run metro_dashboard_bridge defect_event_bridge
```

默认监听：

```text
ROS topic: /defect_events
HTTP:      http://127.0.0.1:8088
ROS domain: 70
```

打开 HTTP 根路径即可进入巡检工作台。当前页面只使用真实存在的健康状态、相机和病害
查询接口；导航和病害截图模块将在后续步骤单独迁移。

一键启动脚本默认使用与完整传感器仿真一致的 `ROS_DOMAIN_ID=70`。脚本参数可以通过环境
变量覆盖，例如为一次新巡检使用独立的会话名称：

```bash
ROS_DOMAIN_ID=70 \
METRO_DASHBOARD_BIND_ADDRESS=0.0.0.0 \
METRO_DASHBOARD_PORT=8089 \
METRO_DASHBOARD_DATABASE_PATH=~/.local/share/metro-inspection/defects.sqlite3 \
METRO_INSPECTION_SESSION_ID=inspection-2026-09-02 \
bash scripts/open_defect_dashboard.sh
```

直接使用 `ros2 run` 时，应通过 ROS 参数修改数据库、会话、话题和相机配置：

```bash
ros2 run metro_dashboard_bridge defect_event_bridge \
  --ros-args \
  -p defect_topic:=inspection/defect_events \
  -p database_path:=/home/jo/.local/share/metro-inspection/defects.sqlite3 \
  -p inspection_session_id:=inspection-2026-09-02 \
  -p camera_topics.xj1:=/robot/xj1/image/compressed \
  -p camera_preview_jpeg_quality:=55 \
  -p camera_stream_default_fps:=6.0
```

公网或 Tailscale 访问时仍建议让服务绑定 `127.0.0.1`，再由经过身份验证的反向代理转发，
不要直接向公网暴露 ROS 2 或无认证的 HTTP 服务。

## API

```text
GET /api/health
GET /api/defects
GET /api/defects/{event_id}
GET /api/cameras
GET /api/cameras/{camera_id}/stream.mjpg?fps=6
```

接口是只读的。真实病害只能由 ROS 2 `DefectEvent` 进入，避免网页端伪造检测结果。
相机流只允许访问节点参数中明确配置的五路原始画面和五路 YOLO 画面，`fps` 最终不会超过
`camera_stream_max_fps`。

## 主要参数

| 参数 | 默认值 |
|---|---|
| `database_path` | `~/.local/share/metro-inspection/defects.sqlite3` |
| `inspection_session_id` | `simulation` |
| `camera_topics.xj1` | `/subway_v2/xj1/image_raw/compressed` |
| `camera_topics.xj2` | `/subway_v2/xj2/image_raw/compressed` |
| `camera_topics.xj3` | `/subway_v2/xj3/image_raw/compressed` |
| `camera_topics.xj4` | `/subway_v2/xj4/image_raw/compressed` |
| `camera_topics.pitch_camera` | `/subway_v2/pitch_camera/image_raw/compressed` |
| `camera_timeout_seconds` | `3.0` |
| `camera_preview_width` / `camera_preview_height` | `640` / `360` |
| `camera_preview_jpeg_quality` | `55` |
| `camera_stream_default_fps` / `camera_stream_max_fps` | `6.0` / `10.0` |

## 手工验收消息

在服务运行时打开另一个已经 source 工作空间的终端：

```bash
ros2 topic pub --once /defect_events \
  metro_inspection_interfaces/msg/DefectEvent \
  "{header: {frame_id: xj1_optical_frame}, event_id: defect-demo-001,
  detection_id: box-001, camera_name: xj1, class_name: crack,
  confidence: 0.93, severity: 2,
  bbox: {center: {position: {x: 320.0, y: 180.0}, theta: 0.0},
  size_x: 120.0, size_y: 80.0}, image_width: 640, image_height: 360,
  has_3d_position: true,
  position: {header: {frame_id: odom}, point: {x: 10.2, y: -1.1, z: 2.4}},
  localization_method: 1, localization_confidence: 0.82,
  model_name: simulation-yolo-v1, has_semantic_location: true,
  chainage_m: 12010.2, chainage: K12+010.200, segment_name: ring,
  segment_id: 1008, segment_offset_m: 0.6,
  clock_position_hours: 2.5, structure_area: right_wall}"

curl http://127.0.0.1:8088/api/health
curl http://127.0.0.1:8088/api/defects
curl http://127.0.0.1:8088/api/cameras
```
