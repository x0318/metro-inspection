# 地铁巡检平台

这里是从旧项目迁移到 `metro-inspection` 的平台页面。当前阶段只启用已经具有真实后端
数据的巡检工作台：

- 读取 `metro_dashboard_bridge` 的运行状态；
- 监控 XJ1～XJ4 和 Pitch 五路实时相机；
- 显示每路相机在线状态、源帧率和最新帧时间；
- 展示 `/defect_events` 形成的病害记录；
- 按五路相机和病害等级筛选；
- 展示二维框、三维定位和隧道语义结果；
- 生成等级/类型统计和可打印巡检报告。

桌面端相机页同时显示五路画面，并可放大一路查看。手机端只连接当前选择的一路，减少
Tailscale 带宽和解码压力。导航控制和病害截图接口尚未迁移，页面不会生成随机病害。

## 启动

从仓库根目录执行：

```bash
bash scripts/open_defect_dashboard.sh
```

浏览器访问：

```text
http://127.0.0.1:8088
```

需要修改端口时：

```bash
METRO_DASHBOARD_PORT=8089 bash scripts/open_defect_dashboard.sh
```

脚本会构建 `metro_dashboard_bridge` 及其接口依赖，然后启动 ROS 2 订阅节点和 FastAPI。
脚本默认使用完整传感器仿真一致的 `ROS_DOMAIN_ID=70`。接入其他 ROS 2 系统时，可以用
`ROS_DOMAIN_ID=<目标值>` 覆盖。病害记录目前保存在进程内存中，服务重启后清空。

相机预览默认不超过 `640×360`，保持源图像宽高比，以 JPEG 质量 55、每路 6 FPS 输出。
Gazebo 未启动或三秒内没有新帧时，对应相机会显示离线。预览参数和五路压缩图像话题可
通过 `metro_dashboard_bridge` 的 ROS 2 参数覆盖。

第三方浏览器依赖固定在 `vendor/` 中，不需要连接公网 CDN。版本和许可证见
[THIRD_PARTY_NOTICES.md](./THIRD_PARTY_NOTICES.md)。
