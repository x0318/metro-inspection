from typing import Any, Protocol, Set

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class StatusSink(Protocol):

    def update_ros_graph(
        self,
        external_node_count: int,
        graph_error: bool = False,
    ) -> None:
        ...

    def mark_odom_received(self) -> None:
        ...


class RosStatusNode(Node):
    """Collect the small, read-only ROS status exposed by the dashboard."""

    def __init__(self, status_store: StatusSink, **node_kwargs: Any) -> None:
        super().__init__('metro_navigation_demo', **node_kwargs)
        self._status_store = status_store
        self._own_fully_qualified_name = self.get_fully_qualified_name()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self._handle_odom,
            sensor_qos,
        )
        self._graph_timer = self.create_timer(1.0, self._refresh_ros_graph)
        self._refresh_ros_graph()
        self.get_logger().info('Read-only dashboard status bridge initialized')

    @staticmethod
    def _fully_qualified_name(name: str, namespace: str) -> str:
        clean_namespace = namespace.rstrip('/')
        if not clean_namespace:
            return '/' + name
        return clean_namespace + '/' + name

    def _handle_odom(self, _message: Odometry) -> None:
        self._status_store.mark_odom_received()

    def _refresh_ros_graph(self) -> None:
        try:
            external_nodes: Set[str] = {
                self._fully_qualified_name(name, namespace)
                for name, namespace in self.get_node_names_and_namespaces()
                if self._fully_qualified_name(name, namespace)
                != self._own_fully_qualified_name
            }
            self._status_store.update_ros_graph(len(external_nodes))
        except Exception as error:  # ROS middleware failures must not stop HTTP.
            self._status_store.update_ros_graph(0, graph_error=True)
            self.get_logger().warning(f'ROS graph check failed: {error}')
