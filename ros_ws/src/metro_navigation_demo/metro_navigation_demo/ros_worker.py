import json
import os
import queue
import threading
from pathlib import Path
from typing import Dict

import rclpy
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from .ros_control_node import PatrolControlNode
from rclpy.parameter import Parameter
from .route_config import RouteCatalog


class PipeStatusSink:
    """Forward bounded, low-rate ROS state updates to FastAPI."""

    def __init__(self, status_fd: int) -> None:
        self._stream = os.fdopen(status_fd, 'w', encoding='utf-8', buffering=1)
        self._lock = threading.Lock()

    def _write(self, update: Dict[str, object]) -> None:
        try:
            with self._lock:
                self._stream.write(json.dumps(update, separators=(',', ':')) + '\n')
        except (BrokenPipeError, OSError):
            pass

    def update_ros_graph(self, external_node_count: int, graph_error: bool = False) -> None:
        self._write({
            'type': 'graph',
            'external_node_count': max(0, int(external_node_count)),
            'graph_error': bool(graph_error),
        })

    def update_odom(self, data: Dict[str, object]) -> None:
        self._write({'type': 'odom', 'data': data})

    def update_navigation(self, data: Dict[str, object]) -> None:
        self._write({'type': 'navigation', 'data': data})

    def update_safety(self, data: Dict[str, object]) -> None:
        self._write({'type': 'safety', 'data': data})

    def update_camera(self, camera_name: str, data: Dict[str, object]) -> None:
        self._write({'type': 'camera', 'camera_name': camera_name, 'data': data})

    def close(self) -> None:
        self._stream.close()


def _receive_commands(command_fd: int, commands: 'queue.Queue[Dict[str, object]]') -> None:
    try:
        with os.fdopen(command_fd, 'r', encoding='utf-8') as stream:
            for line in stream:
                if len(line) > 16384:
                    continue
                try:
                    command = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(command, dict):
                    commands.put(command)
    except OSError:
        pass


def main() -> None:
    status_fd = int(os.environ['SUBWAY_PATROL_STATUS_FD'])
    command_fd = int(os.environ['SUBWAY_PATROL_COMMAND_FD'])
    route_config = Path(os.environ['PLATFORM_ROUTE_CONFIG'])
    camera_directory = Path(os.environ['PLATFORM_CAMERA_CACHE_DIR'])
    behavior_tree_value = os.environ.get('PLATFORM_NAV_BEHAVIOR_TREE')
    behavior_tree = Path(behavior_tree_value).resolve() if behavior_tree_value else None

    status_sink = PipeStatusSink(status_fd)
    command_queue: 'queue.Queue[Dict[str, object]]' = queue.Queue(maxsize=200)
    command_thread = threading.Thread(
        target=_receive_commands,
        args=(command_fd, command_queue),
        name='subway-patrol-command-receiver',
        daemon=True,
    )
    command_thread.start()

    ros_context = Context()
    ros_node = None
    executor = None
    try:
        routes = RouteCatalog(route_config)
        rclpy.init(context=ros_context)
        ros_node = PatrolControlNode(
            status_sink,
            command_queue,
            routes,
            camera_directory,
            behavior_tree=behavior_tree,
            context=ros_context,
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        executor = MultiThreadedExecutor(num_threads=3, context=ros_context)
        executor.add_node(ros_node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        status_sink.update_ros_graph(0, graph_error=True)
        print(f'ROS 2 control worker failed: {error}', flush=True)
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=5.0)
        if ros_node is not None:
            ros_node.destroy_node()
        if ros_context.ok():
            ros_context.shutdown()
        status_sink.close()


if __name__ == '__main__':
    main()
