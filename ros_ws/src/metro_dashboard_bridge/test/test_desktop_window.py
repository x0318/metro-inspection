"""Opt-in tests requiring a working Qt display (METRO_TEST_QT=1)."""

import os
from pathlib import Path
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("METRO_TEST_QT") != "1", reason="Qt display test"
)


@pytest.fixture(scope="module")
def qt_app():
    from metro_dashboard_bridge.desktop_app import QApplication

    app = QApplication.instance() or QApplication([])
    app.setQuitOnLastWindowClosed(False)
    yield app
    app.processEvents()


def test_backend_failure_keeps_window_and_restart_available(tmp_path, qt_app):
    from PyQt5.QtTest import QTest
    from metro_dashboard_bridge.desktop_app import InspectionWindow

    project = Path(__file__).resolve().parents[4]
    window = InspectionWindow(
        project, tmp_path / "settings.json", tmp_path / "logs", False
    )
    window.show()
    window.start_operation(
        [sys.executable, "-c", "print('model missing'); raise SystemExit(7)"], "backend"
    )
    deadline = time.monotonic() + 6
    while window.runner.active and time.monotonic() < deadline:
        QTest.qWait(50)
    assert not window.runner.active
    assert window.isVisible()
    assert window.start_action.isEnabled()
    assert "7" in window.message_body.text()
    assert "model missing" in window.logs.toPlainText()
    assert window.log_dock.isVisible()
    window.close()
    qt_app.processEvents()


def test_renderer_failure_does_not_close_application(tmp_path, qt_app):
    from metro_dashboard_bridge.desktop_app import InspectionWindow

    window = InspectionWindow(
        Path.cwd(), tmp_path / "settings.json", tmp_path / "logs", False
    )
    window.show()
    window.on_renderer_exit(2, 9)
    assert window.isVisible()
    assert window.pages.currentWidget() is window.message_page
    assert "9" in window.message_body.text()
    window.close()
    qt_app.processEvents()


def test_running_settings_are_saved_for_next_start(tmp_path, qt_app, monkeypatch):
    from PyQt5.QtTest import QTest
    from metro_dashboard_bridge import desktop_app
    from metro_dashboard_bridge.desktop_runtime import DesktopOptions, load_options

    updated = DesktopOptions(port=8099, gazebo_gui=True, rviz=True)
    titles = []

    class AcceptedSettings:
        def __init__(self, options, parent):
            self.initial_options = options

        def setWindowTitle(self, title):
            titles.append(title)

        def exec_(self):
            return desktop_app.QDialog.Accepted

        def options(self):
            return updated

    monkeypatch.setattr(desktop_app, "SettingsDialog", AcceptedSettings)
    window = desktop_app.InspectionWindow(
        Path.cwd(), tmp_path / "settings.json", tmp_path / "logs", False
    )
    window.show()
    original = window.options
    try:
        window.start_operation(
            [sys.executable, "-c", "import time; time.sleep(60)"], "build"
        )
        assert window.settings_action.isEnabled()
        window.settings_action.trigger()
        assert titles == ["运行设置（下次启动生效）"]
        assert window.options == original
        assert window.url.endswith(str(original.port))
        assert window.pending_options == updated
        assert load_options(window.config_path) == updated

        window.stop_backend()
        deadline = time.monotonic() + 6
        while window.runner.active and time.monotonic() < deadline:
            QTest.qWait(50)
        assert not window.runner.active
        assert window.options == updated
        assert window.pending_options is None
    finally:
        if window.runner.active:
            window.runner.process.kill()
            window.runner.process.wait()
            window.runner.poll()
        window.close()
        qt_app.processEvents()
