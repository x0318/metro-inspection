"""Opt-in signal shutdown checks in an isolated ROS domain."""

import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("METRO_TEST_ROS") != "1", reason="requires sourced ROS 2 runtime"
)


@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM])
def test_watchdog_signal_shutdown_is_clean(stop_signal):
    script = Path(__file__).resolve().parents[1] / "scripts/cmd_vel_watchdog.py"
    environment = dict(os.environ, ROS_DOMAIN_ID="73", ROS_LOCALHOST_ONLY="1")
    process = subprocess.Popen(
        [sys.executable, "-u", str(script), "--ros-args",
         "-p", "input_topic:=/shutdown_test/input",
         "-p", "output_topic:=/shutdown_test/output"],
        env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True,
    )
    output = ""
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            deadline = time.monotonic() + 10
            while "Drive watchdog active" not in output and time.monotonic() < deadline:
                if selector.select(timeout=0.2):
                    output += process.stdout.readline()
                if process.poll() is not None:
                    break
        assert "Drive watchdog active" in output, output
        process.send_signal(stop_signal)
        tail, _ = process.communicate(timeout=5)
        output += tail
        assert process.returncode == 0, output
        assert "Traceback" not in output
        assert "context is not valid" not in output
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)
