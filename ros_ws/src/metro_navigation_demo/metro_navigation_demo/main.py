import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import List, Optional

import uvicorn
from ament_index_python.packages import get_package_share_directory

from .command_gateway import CommandGateway
from .defect_store import DefectStore
from .route_config import RouteCatalog
from .status_store import StatusStore
from .web_app import create_app


def _read_port() -> int:
    port = int(os.environ.get('PLATFORM_PORT', '8088'))
    if not 1 <= port <= 65535:
        raise ValueError('PLATFORM_PORT must be between 1 and 65535')
    return port


def _read_odom_timeout() -> float:
    timeout = float(os.environ.get('PLATFORM_ODOM_TIMEOUT_SECONDS', '3.0'))
    if timeout <= 0.0:
        raise ValueError('PLATFORM_ODOM_TIMEOUT_SECONDS must be positive')
    return timeout


def _read_dashboard_dir() -> Path:
    configured_dir = os.environ.get('PLATFORM_DASHBOARD_DIR')
    if configured_dir:
        return Path(configured_dir)
    candidate = Path(get_package_share_directory('metro_navigation_demo')) / 'web' / 'inspection_dashboard'
    if (candidate / 'index.html').is_file():
        return candidate
    raise ValueError('PLATFORM_DASHBOARD_DIR is not set and dashboard was not found')


def _read_route_config() -> Path:
    configured_path = os.environ.get('PLATFORM_ROUTE_CONFIG')
    if configured_path:
        path = Path(configured_path)
    else:
        path = Path(get_package_share_directory('metro_navigation_demo')) / 'config' / 'route_choice_training_routes.json'
    if not path.is_file():
        raise ValueError(f'route config not found: {path}')
    return path.resolve()


def _start_ros_worker(
    status_store: StatusStore,
    route_config: Path,
    camera_directory: Path,
) -> tuple[subprocess.Popen, threading.Thread, threading.Event, CommandGateway]:
    status_read_fd, status_write_fd = os.pipe()
    command_read_fd, command_write_fd = os.pipe()
    worker_environment = os.environ.copy()
    worker_environment.update({
        'SUBWAY_PATROL_STATUS_FD': str(status_write_fd),
        'SUBWAY_PATROL_COMMAND_FD': str(command_read_fd),
        'PLATFORM_ROUTE_CONFIG': str(route_config),
        'PLATFORM_CAMERA_CACHE_DIR': str(camera_directory),
    })
    stop_requested = threading.Event()
    first_update_received = threading.Event()

    worker = subprocess.Popen(
        [sys.executable, '-m', 'metro_navigation_demo.ros_worker'],
        env=worker_environment,
        pass_fds=(status_write_fd, command_read_fd),
    )
    os.close(status_write_fd)
    os.close(command_read_fd)
    command_gateway = CommandGateway(command_write_fd)

    def receive_status() -> None:
        try:
            with os.fdopen(status_read_fd, 'r', encoding='utf-8') as status_pipe:
                for line in status_pipe:
                    update = json.loads(line)
                    if not isinstance(update, dict):
                        continue
                    first_update_received.set()
                    status_store.apply_ros_update(update)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            if not stop_requested.is_set():
                print(f'ROS 2 status channel failed: {error}', flush=True)
        finally:
            if not stop_requested.is_set():
                status_store.update_ros_graph(0, graph_error=True)

    receiver = threading.Thread(
        target=receive_status,
        name='subway-patrol-status-receiver',
        daemon=True,
    )
    receiver.start()

    def detect_initialization_timeout() -> None:
        if not first_update_received.wait(timeout=5.0) and not stop_requested.is_set():
            status_store.update_ros_graph(0, graph_error=True)

    threading.Thread(
        target=detect_initialization_timeout,
        name='subway-patrol-ros-startup-watchdog',
        daemon=True,
    ).start()
    return worker, receiver, stop_requested, command_gateway


def main(args: Optional[List[str]] = None) -> None:
    del args
    host = os.environ.get('PLATFORM_BIND_ADDRESS', '127.0.0.1')
    status_store = StatusStore(odom_timeout_seconds=_read_odom_timeout())
    route_config = _read_route_config()
    routes = RouteCatalog(route_config)
    defect_store = DefectStore()
    port = _read_port()

    with tempfile.TemporaryDirectory(prefix='subway_patrol_camera_') as camera_cache:
        camera_directory = Path(camera_cache)
        worker, receiver, stop_requested, command_gateway = _start_ros_worker(
            status_store, route_config, camera_directory
        )
        app = create_app(
            status_store,
            _read_dashboard_dir(),
            command_gateway,
            routes,
            defect_store,
            camera_directory,
        )
        try:
            config = uvicorn.Config(
                app,
                host=host,
                port=port,
                log_level='info',
                reload=False,
                workers=1,
            )
            uvicorn.Server(config).run()
        finally:
            try:
                command_gateway.send({'command': 'estop'})
                time.sleep(0.2)
            except RuntimeError:
                pass
            command_gateway.close()
            stop_requested.set()
            if worker.poll() is None:
                worker.terminate()
                try:
                    worker.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    worker.kill()
            receiver.join(timeout=1.0)


if __name__ == '__main__':
    main()
