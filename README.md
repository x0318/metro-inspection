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

## 巡检平台与 RViz

启动巡检可视化平台：

```bash
cd /home/jo/my-project/metro-inspection
bash scripts/open_defect_dashboard.sh
```

浏览器访问：

```text
http://127.0.0.1:8088
```

打开已经准备好点云、TF、机器人模型和病害识别显示项的 RViz：

```bash
cd /home/jo/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_v2_perception_rviz.sh
```

Tailscale Serve 可用 `tailscale serve --bg http://127.0.0.1:8088` 将本机平台安全提供给已授权的 tailnet 设备，但仍需单独运行上面的平台启动命令。
