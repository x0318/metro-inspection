import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class DamageDetector(Node):
    """Detects the red Gazebo patch and publishes a standard Detection2DArray."""

    def __init__(self):
        super().__init__("damage_detector")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/damage_detections")
        self.declare_parameter("mask_topic", "/damage_mask")
        self.declare_parameter("class_id", "damage_patch")
        self.declare_parameter("score", 0.99)
        self.declare_parameter("min_area_px", 80.0)
        self.declare_parameter("morph_kernel", 5)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            Detection2DArray,
            self.get_parameter("detections_topic").value,
            qos_profile_sensor_data,
        )
        self.mask_pub = self.create_publisher(
            Image,
            self.get_parameter("mask_topic").value,
            qos_profile_sensor_data,
        )
        self.sub = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Red-patch placeholder detector started; publishing bbox and /damage_mask."
        )

    def on_image(self, image: Image):
        output = Detection2DArray()
        output.header = image.header

        try:
            bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge raises different exception types by version
            self.get_logger().error(f"Failed to convert camera image: {exc}")
            self.pub.publish(output)
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        low_red = cv2.inRange(hsv, np.array([0, 90, 70]), np.array([10, 255, 255]))
        high_red = cv2.inRange(
            hsv, np.array([170, 90, 70]), np.array([179, 255, 255])
        )
        mask = cv2.bitwise_or(low_red, high_red)

        kernel_size = max(1, int(self.get_parameter("morph_kernel").value))
        if kernel_size > 1:
            kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_msg.header = image.header
        self.mask_pub.publish(mask_msg)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        min_area = float(self.get_parameter("min_area_px").value)
        valid = [contour for contour in contours if cv2.contourArea(contour) >= min_area]

        if valid:
            contour = max(valid, key=cv2.contourArea)
            x, y, width, height = cv2.boundingRect(contour)

            detection = Detection2D()
            detection.header = image.header
            detection.id = "gazebo_red_damage"
            detection.bbox.center.position.x = float(x + width / 2.0)
            detection.bbox.center.position.y = float(y + height / 2.0)
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(width)
            detection.bbox.size_y = float(height)

            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = str(
                self.get_parameter("class_id").value
            )
            result.hypothesis.score = float(self.get_parameter("score").value)
            detection.results.append(result)
            output.detections.append(detection)

        self.pub.publish(output)


def main():
    rclpy.init()
    node = DamageDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
