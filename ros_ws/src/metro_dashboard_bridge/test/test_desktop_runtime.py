import os
from pathlib import Path
import signal
import socket
import sys
import time

import pytest

from metro_dashboard_bridge.desktop_runtime import (
    DesktopOptions,
    OwnedProcess,
    backend_command,
    ensure_port_free,
    load_options,
    preflight,
    save_options,
)


def wait_for(predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("Timed out")


def test_settings_reject_invalid_values_and_preserve_model_path(tmp_path):
    path = tmp_path / "config/desktop.json"
    options = DesktopOptions(model_path="/tmp/model ' $(example).pt", detection=False)
    save_options(path, options)
    assert load_options(path) == options
    with pytest.raises(ValueError):
        DesktopOptions(port=70, domain=999).validate()
    with pytest.raises(ValueError):
        DesktopOptions(detection="false").validate()


def test_launch_arguments_are_literal_and_never_enable_driving():
    project = Path("/tmp/project with spaces")
    options = DesktopOptions(model_path="/tmp/a $(touch bad).pt")
    command = backend_command(project, options)
    assert "yolo_auto_drive:=false" in command
    assert "qt:=false" in command
    assert "yolo_model_path:=/tmp/a $(touch bad).pt" in command
    assert f"project_dir:={project}" in command


def test_preflight_explains_missing_lfs_model(tmp_path, monkeypatch):
    for name in (
        "ros_ws/install/setup.bash",
        "dashboard/index.html",
        "scripts/run_inspection_backend.sh",
        "ros_ws/src/metro_sim/models/subway_tunnel_v2/meshes/subway_tunnel_v2.dae",
    ):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("version https://git-lfs.github.com/spec/v1\noid sha256:example\n")
    exists = Path.exists
    monkeypatch.setattr(
        Path, "exists",
        lambda path: str(path) == "/opt/ros/humble/setup.bash" or exists(path),
    )
    with pytest.raises(ValueError, match="git lfs pull"):
        preflight(tmp_path, DesktopOptions(detection=False))


def test_port_check_distinguishes_listener_from_closed_connections():
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
        listener.listen()
        with pytest.raises(ValueError, match="in use"):
            ensure_port_free("Dashboard", port)
        with socket.create_connection(("127.0.0.1", port)) as client:
            connection, _ = listener.accept()
            connection.close()
            assert client.recv(1) == b""
    ensure_port_free("Dashboard", port)


def test_failed_process_keeps_log_and_can_restart(tmp_path):
    runner = OwnedProcess(tmp_path)
    runner.start(
        [
            sys.executable,
            "-c",
            "print('missing model', flush=True); raise SystemExit(7)",
        ],
        tmp_path,
    )
    wait_for(lambda: runner.poll() is not None)
    assert runner.returncode == 7
    assert "missing model" in runner.read_output()
    assert runner.log_path.read_text().strip() == "missing model"
    runner.close_log()
    runner.start([sys.executable, "-c", "print('restarted')"], tmp_path)
    wait_for(lambda: runner.poll() is not None)
    assert runner.returncode == 0
    runner.close_log()


def test_stop_escalates_only_for_owned_process_group(tmp_path):
    now = [0.0]
    runner = OwnedProcess(tmp_path, clock=lambda: now[0])
    runner.start(
        [
            sys.executable,
            "-c",
            "import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); "
            "signal.signal(signal.SIGTERM, signal.SIG_IGN); print('ready',flush=True); time.sleep(60)",
        ],
        tmp_path,
    )
    try:
        wait_for(lambda: "ready" in runner.log_path.read_text())
        pid = runner.process.pid
        assert os.getpgid(pid) == pid
        runner.stop()
        now[0] = 31
        runner.poll()
        assert runner.stop_stage == 1
        now[0] = 41
        runner.poll()
        wait_for(lambda: runner.poll() is not None)
        assert runner.returncode == -signal.SIGKILL
    finally:
        if runner.active:
            os.killpg(runner.process.pid, signal.SIGKILL)
            runner.process.wait()
        runner.close_log()
