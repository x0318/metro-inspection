import os
import threading
from pathlib import Path

import rclpy
import uvicorn
from rclpy.executors import SingleThreadedExecutor

from .api import create_app
from .defect_event_node import DefectEventBridgeNode


def _read_port() -> int:
    port = int(os.environ.get("METRO_DASHBOARD_PORT", "8088"))
    if not 1 <= port <= 65535:
        raise ValueError("METRO_DASHBOARD_PORT must be between 1 and 65535")
    return port


def _read_dashboard_dir() -> Path:
    configured = os.environ.get("METRO_DASHBOARD_DIR")
    if configured:
        candidates = [Path(configured)]
    else:
        candidates = [Path.cwd() / "dashboard", Path.cwd().parent / "dashboard"]

    for candidate in candidates:
        if (candidate / "index.html").is_file():
            return candidate.resolve()
    raise ValueError(
        "METRO_DASHBOARD_DIR is not set and dashboard/index.html was not found"
    )


def main(args=None) -> None:
    host = os.environ.get("METRO_DASHBOARD_BIND_ADDRESS", "127.0.0.1")
    port = _read_port()
    rclpy.init(args=args)
    node = DefectEventBridgeNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin,
        name="metro-dashboard-ros-executor",
        daemon=True,
    )
    spin_thread.start()

    app = create_app(
        node.store,
        node.statistics,
        _read_dashboard_dir(),
        node.camera_store,
        node.camera_timeout_seconds,
        node.camera_stream_config,
    )
    try:
        config = uvicorn.Config(
            app,
            host=host,
            port=port,
            log_level="info",
            reload=False,
            workers=1,
        )
        uvicorn.Server(config).run()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=5.0)


if __name__ == "__main__":
    main()
