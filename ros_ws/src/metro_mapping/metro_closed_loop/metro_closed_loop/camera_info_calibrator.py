"""Publish the calibrated Odin1 CameraInfo unsupported by Gazebo Classic."""

import rclpy
import signal
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class CameraInfoCalibrator(Node):
    """Preserve sensor timing while replacing Gazebo's approximate calibration."""

    WIDTH = 1600
    HEIGHT = 1296
    FX = 736.9688
    FY = 737.0365
    SKEW = 0.2058
    CX = 766.6570
    CY = 642.9091

    def __init__(self):
        super().__init__("camera_info_calibrator")
        self.declare_parameter("input_topic", "/odin1/rgb/camera_info_gazebo")
        self.declare_parameter("output_topic", "/odin1/rgb/camera_info")
        self.publisher = self.create_publisher(
            CameraInfo,
            self.get_parameter("output_topic").value,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.subscription = self.create_subscription(
            CameraInfo,
            self.get_parameter("input_topic").value,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Publishing calibrated Odin1 CameraInfo: "
            f"{self.WIDTH}x{self.HEIGHT}, fx={self.FX}, fy={self.FY}, "
            f"skew={self.SKEW}, cx={self.CX}, cy={self.CY}."
        )

    def on_camera_info(self, source: CameraInfo):
        if source.width != self.WIDTH or source.height != self.HEIGHT:
            self.get_logger().error(
                "Ignoring CameraInfo with unexpected resolution "
                f"{source.width}x{source.height}; expected "
                f"{self.WIDTH}x{self.HEIGHT}."
            )
            return

        output = CameraInfo()
        output.header = source.header
        output.height = self.HEIGHT
        output.width = self.WIDTH
        output.distortion_model = "plumb_bob"
        # The supplied calibration omitted distortion coefficients. Gazebo's
        # rendered image is ideal, so zero distortion is the honest model.
        output.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        output.k = [
            self.FX, self.SKEW, self.CX,
            0.0, self.FY, self.CY,
            0.0, 0.0, 1.0,
        ]
        output.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        output.p = [
            self.FX, self.SKEW, self.CX, 0.0,
            0.0, self.FY, self.CY, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.publisher.publish(output)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = CameraInfoCalibrator()
    try:
        # Python signal handlers must also run after camera publishers stop.
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
