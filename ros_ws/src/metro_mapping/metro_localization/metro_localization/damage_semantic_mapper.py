"""空间语义映射节点。

输入：
    /damage_point_global  geometry_msgs/msg/PointStamped
        damage_localizer 输出的病害全局三维坐标。

输出：
    /damage_semantic_result  std_msgs/msg/String
        JSON字符串，包含三维坐标、里程、区段/环号、时钟方位、结构区域等。

当前假设：
    /damage_point_global 已经在 odom/tunnel_frame 下。
    后续如果单独建立 tunnel_frame，可以先通过 TF 把点从 odom 转到 tunnel_frame，
    再使用本文件中的里程、区段和时钟方位计算公式。


订阅 /damage_point_global
→ 读取病害全局三维坐标 x, y, z
→ 计算里程
→ 计算区段号/环号
→ 计算区段内偏移
→ 计算时钟方位
→ 判断结构区域
→ 发布JSON结果
→ 保存CSV记录
"""

import csv
import json
import math
import os
from dataclasses import dataclass
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class Point3D:
    """用于让几何计算代码更清楚的小型三维点结构。"""

    x: float
    y: float
    z: float


class DamageSemanticMapper(Node):
    """把病害三维坐标转换为工程语义位置。

    这个节点不重新做检测，也不重新做点云融合，只负责把已经算出来的
    三维点解释成巡检报告能看懂的结果，例如：

        K12+002.961、1002号区段、区段内偏移0.561 m、3点钟方向。
    """

    def __init__(self):
        super().__init__("damage_semantic_mapper")

        # 话题参数
        self.declare_parameter("input_topic", "/damage_point_global")
        self.declare_parameter("output_topic", "/damage_semantic_result")

        # 坐标系参数。当前你的 /damage_point_global 是 odom 坐标。
        self.declare_parameter("tunnel_frame", "odom")
        self.declare_parameter("allow_frame_mismatch", False)

        # 里程参数：chainage = chainage_start_m + chainage_sign * point[axis]
        # 例如 K12+000 对应 12000.0 m。
        self.declare_parameter("chainage_start_m", 12000.0)
        self.declare_parameter("chainage_axis", "x")
        self.declare_parameter("chainage_sign", 1.0)

        # 区段/环号参数。当前仿真里可以理解为虚拟区段；
        # 如果后续是盾构管片隧道，也可以把 segment_id 理解为 ring_id。
        self.declare_parameter("segment_start_id", 1000)
        self.declare_parameter("segment_length_m", 1.2)
        self.declare_parameter("segment_name", "segment")

        # 时钟方位参数。按照面向 +X 方向看隧道断面：
        # 12点 = +Z，3点 = -Y，6点 = -Z，9点 = +Y。
        self.declare_parameter("clock_center_y_m", 0.0)
        self.declare_parameter("clock_center_z_m", 0.65)

        # 目前使用红色假病害，先给默认类别和置信度。
        # 后续接入真实YOLO时，可以把类别、置信度、bbox一起传过来。
        self.declare_parameter("default_class_name", "red_damage")
        self.declare_parameter("default_confidence", 1.0)

        # 是否保存CSV，便于后续网页/报告读取。
        self.declare_parameter("save_csv", True)
        self.declare_parameter("result_csv_path", "results/damage_semantic_records.csv")

        # 防止同一个静止假病害每一帧都疯狂写入CSV。
        self.declare_parameter("min_publish_interval_sec", 0.2)
        self.declare_parameter("log_every_n", 10)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.tunnel_frame = str(self.get_parameter("tunnel_frame").value)
        self.allow_frame_mismatch = bool(self.get_parameter("allow_frame_mismatch").value)
        self.chainage_start_m = float(self.get_parameter("chainage_start_m").value)
        self.chainage_axis = str(self.get_parameter("chainage_axis").value).lower()
        self.chainage_sign = float(self.get_parameter("chainage_sign").value)
        self.segment_start_id = int(self.get_parameter("segment_start_id").value)
        self.segment_length_m = float(self.get_parameter("segment_length_m").value)
        self.segment_name = str(self.get_parameter("segment_name").value)
        self.clock_center_y_m = float(self.get_parameter("clock_center_y_m").value)
        self.clock_center_z_m = float(self.get_parameter("clock_center_z_m").value)
        self.default_class_name = str(self.get_parameter("default_class_name").value)
        self.default_confidence = float(self.get_parameter("default_confidence").value)
        self.save_csv = bool(self.get_parameter("save_csv").value)
        self.result_csv_path = os.path.expanduser(str(self.get_parameter("result_csv_path").value))
        self.min_publish_interval_sec = float(self.get_parameter("min_publish_interval_sec").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)

        self.validate_parameters()

        self.result_pub = self.create_publisher(String, self.output_topic, 10)
        self.point_sub = self.create_subscription(
            PointStamped,
            self.input_topic,
            self.on_damage_point,
            10,
        )

        self.result_count = 0
        self.last_publish_time = None

        if self.save_csv:
            self.prepare_csv_file()

        self.get_logger().info(
            "damage_semantic_mapper started: "
            f"input={self.input_topic}, output={self.output_topic}, "
            f"frame={self.tunnel_frame}, chainage_start={self.chainage_start_m:.3f} m, "
            f"segment_start={self.segment_start_id}, segment_length={self.segment_length_m:.3f} m"
        )

    def validate_parameters(self):
        """检查参数，防止后面计算出现隐蔽错误。"""

        if self.chainage_axis not in {"x", "y", "z"}:
            raise ValueError("chainage_axis must be one of: x, y, z")
        if self.segment_length_m <= 0.0:
            raise ValueError("segment_length_m must be positive")
        if self.min_publish_interval_sec < 0.0:
            raise ValueError("min_publish_interval_sec must be non-negative")
        if self.log_every_n <= 0:
            self.log_every_n = 10

    def prepare_csv_file(self):
        """创建CSV文件。文件不存在或为空时写入表头。"""

        directory = os.path.dirname(self.result_csv_path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        need_header = (
            not os.path.exists(self.result_csv_path)
            or os.path.getsize(self.result_csv_path) == 0
        )
        if need_header:
            with open(self.result_csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=self.csv_fieldnames())
                writer.writeheader()

    @staticmethod
    def csv_fieldnames():
        return [
            "damage_id",
            "timestamp_sec",
            "frame_id",
            "class_name",
            "confidence",
            "x_m",
            "y_m",
            "z_m",
            "chainage_m",
            "chainage_text",
            "segment_id",
            "segment_offset_m",
            "clock_hour",
            "clock_position",
            "clock_angle_deg",
            "structure_area",
        ]

    def on_damage_point(self, msg: PointStamped):
        """收到病害三维点后，计算工程语义并发布JSON。"""

        if not self.allow_frame_mismatch and msg.header.frame_id != self.tunnel_frame:
            self.get_logger().warn(
                f"Ignore point in frame '{msg.header.frame_id}', expected '{self.tunnel_frame}'. "
                "If this is intended, set allow_frame_mismatch:=true or add TF conversion."
            )
            return

        now = self.get_clock().now()
        if self.last_publish_time is not None:
            dt = (now - self.last_publish_time).nanoseconds / 1e9
            if dt < self.min_publish_interval_sec:
                return
        self.last_publish_time = now

        self.result_count += 1
        point = Point3D(
            x=float(msg.point.x),
            y=float(msg.point.y),
            z=float(msg.point.z),
        )

        timestamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        relative_chainage_m = self.compute_relative_chainage(point)
        chainage_m = self.chainage_start_m + relative_chainage_m
        chainage_text = self.format_chainage(chainage_m)

        segment_id, segment_offset_m = self.compute_segment(relative_chainage_m)

        clock_angle_deg, clock_hour, clock_position = self.compute_clock(point)
        structure_area = self.classify_structure_area(clock_angle_deg)

        result = {
            "damage_id": f"damage_{self.result_count:04d}",
            "timestamp": round(timestamp_sec, 9),
            "class_name": self.default_class_name,
            "confidence": round(self.default_confidence, 4),
            "global_point": {
                "frame_id": msg.header.frame_id,
                "x": round(point.x, 4),
                "y": round(point.y, 4),
                "z": round(point.z, 4),
            },
            "chainage_m": round(chainage_m, 4),
            "chainage": chainage_text,
            "segment_name": self.segment_name,
            "segment_id": int(segment_id),
            "segment_offset_m": round(segment_offset_m, 4),
            "clock_angle_deg": round(clock_angle_deg, 2),
            "clock_hour": int(clock_hour),
            "clock_position": clock_position,
            "structure_area": structure_area,
        }

        self.result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))

        if self.save_csv:
            self.append_csv(result)

        if self.result_count == 1 or self.result_count % self.log_every_n == 0:
            self.get_logger().info(
                f"{result['damage_id']}: {chainage_text}, "
                f"{self.segment_name}_id={segment_id}, "
                f"offset={segment_offset_m:.3f} m, "
                f"clock={clock_position}, area={structure_area}"
            )

    def compute_relative_chainage(self, point: Point3D) -> float:
        """计算病害相对隧道起点的纵向距离。"""

        axis_value = {
            "x": point.x,
            "y": point.y,
            "z": point.z,
        }[self.chainage_axis]
        return self.chainage_sign * axis_value

    def compute_segment(self, relative_chainage_m: float) -> Tuple[int, float]:
        """根据纵向距离计算区段/环号和区段内偏移。"""

        segment_index = math.floor(relative_chainage_m / self.segment_length_m)
        segment_id = self.segment_start_id + segment_index
        segment_offset_m = relative_chainage_m - segment_index * self.segment_length_m
        return segment_id, segment_offset_m

    def compute_clock(self, point: Point3D) -> Tuple[float, int, str]:
        """计算时钟方位。

        面向 +X 方向观察隧道断面时：
            12点：+Z
            3点：-Y
            6点：-Z
            9点：+Y

        使用 atan2(-dy, dz) 的原因是：
            dz 为正时应该是 12点方向；
            -dy 为正时应该向 3点方向旋转。
        """

        dy = point.y - self.clock_center_y_m
        dz = point.z - self.clock_center_z_m
        angle_deg = (math.degrees(math.atan2(-dy, dz)) + 360.0) % 360.0

        # 每30度对应一个小时。0度和360度都显示为12点。
        hour = int(round(angle_deg / 30.0)) % 12
        if hour == 0:
            hour = 12

        return angle_deg, hour, f"{hour}点"

    @staticmethod
    def classify_structure_area(clock_angle_deg: float) -> str:
        """根据时钟角度粗略判断病害所在结构区域。"""

        if clock_angle_deg < 45.0 or clock_angle_deg >= 315.0:
            return "拱顶/顶部"
        if 45.0 <= clock_angle_deg < 135.0:
            return "右侧边墙"
        if 135.0 <= clock_angle_deg < 225.0:
            return "底部/道床附近"
        return "左侧边墙"

    @staticmethod
    def format_chainage(chainage_m: float) -> str:
        """把米制里程格式化成 K12+002.961 这种形式。"""

        sign = "-" if chainage_m < 0 else ""
        abs_m = abs(chainage_m)
        km = int(abs_m // 1000.0)
        meter = abs_m - km * 1000.0
        return f"{sign}K{km}+{meter:07.3f}"

    def append_csv(self, result: Dict[str, object]):
        """把语义结果追加保存到CSV，后续网页和报告可以读取。"""

        row = {
            "damage_id": result["damage_id"],
            "timestamp_sec": result["timestamp"],
            "frame_id": result["global_point"]["frame_id"],
            "class_name": result["class_name"],
            "confidence": result["confidence"],
            "x_m": result["global_point"]["x"],
            "y_m": result["global_point"]["y"],
            "z_m": result["global_point"]["z"],
            "chainage_m": result["chainage_m"],
            "chainage_text": result["chainage"],
            "segment_id": result["segment_id"],
            "segment_offset_m": result["segment_offset_m"],
            "clock_hour": result["clock_hour"],
            "clock_position": result["clock_position"],
            "clock_angle_deg": result["clock_angle_deg"],
            "structure_area": result["structure_area"],
        }
        with open(self.result_csv_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.csv_fieldnames())
            writer.writerow(row)


def main():
    rclpy.init()
    node = DamageSemanticMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
