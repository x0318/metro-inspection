# metro_inspection_interfaces

地铁巡检系统各模块共用的 ROS 2 接口包。当前只定义
`metro_inspection_interfaces/msg/DefectEvent`，不包含检测、三维定位或平台节点。

一条 `DefectEvent` 表示某一路相机在某一帧图像中发现的一处病害。消息把后续流水线需要的
信息放在同一份记录中：

- 二维识别阶段填写来源相机、类别、置信度、检测框和图像尺寸。
- 三维定位成功后填写 `position`，并将 `has_3d_position` 设为 `true`。
- 里程和隧道结构语义计算完成后填写对应字段，并将
  `has_semantic_location` 设为 `true`。

因此，当前没有队友提供的二维识别节点也不影响接口包的独立构建。后续每个处理节点只补充
自己负责的字段，不需要再通过无约束的 JSON 字符串传递核心数据。

## 字段约定

- `header`：原始图像的时间戳和相机光学坐标系。
- `event_id`：完整病害事件的唯一标识，供平台、记录和去重使用。
- `detection_id`：二维检测器产生的单帧检测标识。
- `camera_name`：相机逻辑名称，例如 `xj1`、`xj2`、`xj3`、`xj4` 或
  `pitch_camera`。
- `severity`：工程病害等级。检测阶段无法严谨判断时使用
  `SEVERITY_UNKNOWN`，后续评估节点可以使用同一个 `event_id` 更新等级。
- `bbox`：原始图像像素坐标中的二维检测框。
- `position.header`：三维点所属坐标系及其对应时间戳。
- `snapshot_uri`：可选的截图路径或 URL；消息本身不传输整幅图片。
- `chainage_m`：以米表示的数值里程；`chainage` 是供人阅读的格式，例如
  `K12+002.961`。
- `clock_position_hours`：隧道断面上的连续时钟方位，范围建议为 `[0, 12)`，
  其中 `0` 表示 12 点方向。

三维定位尚未成功时，发布者必须设置 `has_3d_position=false` 和
`localization_method=LOCALIZATION_NONE`。工程语义尚未生成时，必须设置
`has_semantic_location=false`。接收者应先检查这两个布尔字段，再读取对应的可选结果。

## 构建与检查

在 `ros_ws` 目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select metro_inspection_interfaces
source install/setup.bash
ros2 interface show metro_inspection_interfaces/msg/DefectEvent
```

其他 ROS 2 包使用该消息时，需要在自身的 `package.xml` 和构建配置中声明对
`metro_inspection_interfaces` 的依赖。消息定义发生变更后，需要重新构建本包及所有依赖包。
