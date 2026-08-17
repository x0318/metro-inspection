# Metro Inspection

地铁多领域病害综合智能巡检系统。

## Environment

- Ubuntu 22.04
- ROS2 Humble
- Gazebo / Ignition
- Python 3.10

## Modules

- metro_sim: Gazebo 仿真、小车模型、导航
- metro_detection: 病害检测
- metro_mapping: 三维坐标同化
- metro_dashboard_bridge: 病害事件与五路相机的网页桥接
- dashboard: 五路相机监控、病害记录与巡检报告
- configs: 配置文件
- docs: 技术文档
- hardware: 实物小车资料
