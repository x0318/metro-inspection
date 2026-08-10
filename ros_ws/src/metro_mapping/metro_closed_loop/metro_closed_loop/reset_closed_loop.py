import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Empty


class ResetClosedLoop(Node):
    def __init__(self):
        super().__init__("reset_closed_loop")
        self.zero_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def call_empty(self, service_name):
        client = self.create_client(Empty, service_name)
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"{service_name} is not available.")
            return False
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.done() and future.result() is not None

    def set_robot_pose(self):
        client = self.create_client(SetEntityState, "/set_entity_state")
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("/set_entity_state is not available.")
            return False

        state = EntityState()
        state.name = "inspection_car"
        state.reference_frame = "world"
        state.pose.position.x = 0.0
        state.pose.position.y = 0.0
        state.pose.position.z = 0.12
        state.pose.orientation.w = 1.0

        request = SetEntityState.Request()
        request.state = state
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.done() and future.result() is not None and future.result().success

    def run(self):
        for _ in range(5):
            self.zero_pub.publish(Twist())
        self.call_empty("/reset_world")
        self.set_robot_pose()
        self.get_logger().info(
            "Closed loop reset requested. Restart auto_driver or relaunch to follow the path again."
        )


def main():
    rclpy.init()
    node = ResetClosedLoop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
