"""Configuration and owned process groups for the desktop application."""

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
from dataclasses import asdict, dataclass, fields
from datetime import datetime


@dataclass
class DesktopOptions:
    model_path: str = ""
    detection: bool = True
    localization: bool = True
    gazebo_gui: bool = False
    rviz: bool = False
    port: int = 8088
    domain: int = 70
    gazebo_port: int = 11370

    def validate(self):
        for name in ("port", "gazebo_port"):
            value = getattr(self, name)
            if type(value) is not int or not 1024 <= value <= 65535:
                raise ValueError(f"{name}: expected a port between 1024 and 65535")
        if self.port == self.gazebo_port:
            raise ValueError("HTTP and Gazebo ports must be different")
        if type(self.domain) is not int or not 0 <= self.domain <= 101:
            raise ValueError("ROS domain must be between 0 and 101")
        for name in ("detection", "localization", "gazebo_gui", "rviz"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name}: expected a boolean")
        if not isinstance(self.model_path, str):
            raise ValueError("model_path: expected a path")


def load_options(path: Path) -> DesktopOptions:
    if not path.exists():
        return DesktopOptions(
            model_path=os.environ.get(
                "METRO_YOLO_MODEL_PATH",
                str(Path.home() / "incoming/yolov8n_sim_demo_best(1).pt"),
            )
        )
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Desktop settings must be a JSON object")
    names = {field.name for field in fields(DesktopOptions)}
    options = DesktopOptions(
        **{key: value for key, value in data.items() if key in names}
    )
    options.validate()
    return options


def save_options(path: Path, options: DesktopOptions):
    options.validate()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(asdict(options), indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def preflight(project: Path, options: DesktopOptions):
    """Check local resources without importing ROS or loading the model."""
    options.validate()
    required = [
        Path("/opt/ros/humble/setup.bash"),
        project / "ros_ws/install/setup.bash",
        project / "dashboard/index.html",
        project / "scripts/run_inspection_backend.sh",
        project
        / "ros_ws/src/metro_sim/models/subway_tunnel_v2/meshes/subway_tunnel_v2.dae",
    ]
    if options.detection:
        required.extend(
            [
                Path(options.model_path).expanduser()
                if options.model_path
                else project / "models/best.pt",
                project / ".venv-yolo/bin/python",
                project / ".yolo-ros-compat/numpy",
                project / ".yolo-ros-compat/cv2",
            ]
        )
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise ValueError("Missing runtime files:\n" + "\n".join(missing))
    mesh = (
        project
        / "ros_ws/src/metro_sim/models/subway_tunnel_v2/meshes/subway_tunnel_v2.dae"
    )
    with mesh.open("rb") as stream:
        if stream.read(128).startswith(b"version https://git-lfs.github.com/spec/v1"):
            raise ValueError(
                "隧道模型尚未下载：当前文件是 Git LFS 指针。"
                "请在项目目录运行 git lfs pull 后重新启动。"
            )
    for name, port in (("Dashboard", options.port), ("Gazebo", options.gazebo_port)):
        ensure_port_free(name, port)


def ensure_port_free(name, port):
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError as error:
            raise ValueError(
                f"{name} port {port} is in use. Stop the existing instance or change the port."
            ) from error


def backend_command(project: Path, options: DesktopOptions):
    options.validate()
    flag = lambda value: "true" if value else "false"
    return [
        "/bin/bash",
        str(project / "scripts/run_inspection_backend.sh"),
        f"project_dir:={project}",
        "qt:=false",
        "open_browser:=false",
        "dashboard:=true",
        "yolo_auto_drive:=false",
        f"gui:={flag(options.gazebo_gui)}",
        f"rviz:={flag(options.rviz)}",
        f"detection:={flag(options.detection)}",
        f"localization:={flag(options.localization)}",
        f"yolo_model_path:={Path(options.model_path).expanduser()}",
        f"dashboard_port:={options.port}",
        f"ros_domain_id:={options.domain}",
        f"gazebo_master_uri:=http://127.0.0.1:{options.gazebo_port}",
    ]


class OwnedProcess:
    """Keep logs and stop only this application's process group."""

    def __init__(self, log_dir: Path, clock=time.monotonic):
        self.log_dir = log_dir
        self.clock = clock
        self.process = None
        self.log_path = None
        self.started_at = None
        self.stopping_at = None
        self.stop_stage = 0
        self.returncode = None
        self._reader = None
        self._writer = None

    @property
    def active(self):
        return self.process is not None

    def start(self, command, cwd: Path, environment=None):
        if self.active:
            raise RuntimeError("An operation is already running")
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.log_dir / (
            datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".log"
        )
        self._writer = self.log_path.open("wb")
        environment = dict(os.environ if environment is None else environment)
        environment["METRO_RUNTIME_STATUS_PATH"] = str(
            self.log_path.with_suffix(".status.json")
        )
        try:
            self.process = subprocess.Popen(
                ["/usr/bin/setpriv", "--pdeathsig", "SIGINT", "--", *command],
                cwd=str(cwd),
                env=environment,
                stdout=self._writer,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception:
            self._writer.close()
            self._writer = None
            raise
        self._reader = self.log_path.open("rb")
        self.started_at = self.clock()
        self.stopping_at = None
        self.stop_stage = 0
        self.returncode = None

    def read_output(self):
        if self._reader is None:
            return ""
        return self._reader.read(65536).decode("utf-8", errors="replace")

    def _signal(self, number):
        if self.process is not None:
            try:
                os.killpg(self.process.pid, number)
            except ProcessLookupError:
                pass

    def _group_alive(self):
        if self.process is None:
            return False
        try:
            os.killpg(self.process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    def stop(self):
        if self.active and self.stopping_at is None:
            self.stopping_at = self.clock()
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
            else:
                self._signal(signal.SIGINT)

    def poll(self):
        if not self.active:
            return None
        code = self.process.poll()
        if code is not None and self.stopping_at is None:
            # A launcher may exit while a grandchild still owns a camera or port.
            self.stop()
        if self.stopping_at is not None:
            elapsed = self.clock() - self.stopping_at
            if elapsed >= 40 and self.stop_stage < 2:
                self._signal(signal.SIGKILL)
                self.stop_stage = 2
            elif elapsed >= 30 and self.stop_stage < 1:
                self._signal(signal.SIGTERM)
                self.stop_stage = 1
        if code is None or (self._group_alive() and self.stop_stage < 2):
            return None
        self.returncode = code
        self.process = None
        self._writer.close()
        self._writer = None
        return code

    def close_log(self):
        if self._reader is not None:
            self._reader.close()
            self._reader = None

    def failure_detail(self):
        if self.log_path is None:
            return ""
        try:
            status = json.loads(self.log_path.with_suffix(".status.json").read_text())
            labels = {
                "detection": "YOLO 检测",
                "simulation": "Gazebo 仿真",
                "localization": "三维定位",
                "dashboard": "平台数据服务",
            }
            component = labels.get(status["component"], status["component"])
            return f"{component}进程已退出（退出码 {status['exit_code']}）。"
        except (OSError, ValueError, KeyError, TypeError):
            return ""
