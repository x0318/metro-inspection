import os
import select
import signal
import subprocess
import sys
import time

import pytest


@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize("idle_seconds", [0.0, 0.4])
def test_calibrator_exits_without_camera_messages(stop_signal, idle_seconds):
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            "from metro_closed_loop.camera_info_calibrator import main; main()",
        ],
        env={**os.environ, "ROS_DOMAIN_ID": "100"},
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        ready, _, _ = select.select([process.stdout], [], [], 10)
        assert ready, "Calibrator did not start"
        startup = process.stdout.readline()
        assert "Publishing calibrated Odin1 CameraInfo" in startup
        time.sleep(idle_seconds)
        process.send_signal(stop_signal)
        output, _ = process.communicate(timeout=5)
        assert process.returncode == 0, startup + output
        assert "Traceback" not in output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stdout.close()
