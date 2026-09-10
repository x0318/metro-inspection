"""Opt-in tests requiring a working Qt display (METRO_TEST_QT=1)."""

import os
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

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


def test_existing_backend_opens_page_without_starting_or_stopping_process(tmp_path, qt_app, monkeypatch):
    from metro_dashboard_bridge import desktop_app
    monkeypatch.setattr(desktop_app, "dashboard_is_online", lambda url: True)
    monkeypatch.setattr(desktop_app, "preflight", lambda *args: pytest.fail("must reuse existing backend"))
    window = desktop_app.InspectionWindow(Path.cwd(), tmp_path / "settings.json", tmp_path / "logs", False)
    monkeypatch.setattr(window, "check_health", lambda: None)
    monkeypatch.setattr(window.runner, "start", lambda *args: pytest.fail("must not launch duplicate backend"))
    monkeypatch.setattr(window.runner, "stop", lambda: pytest.fail("must not stop external backend"))
    window.show()
    window.start_backend()
    window.on_page_loaded(True)
    assert window.attached_backend and not window.runner.active
    assert window.pages.currentWidget() is window.browser
    assert window.reload_action.isEnabled() and window.settings_action.isEnabled()
    assert not window.start_action.isEnabled() and not window.stop_action.isEnabled()
    window.last_health -= 16
    window.tick()
    assert not window.attached_backend
    assert window.start_action.isEnabled()
    window.close()
    qt_app.processEvents()


def test_demo_settings_preserve_yolo_options_and_disable_inference_controls(qt_app):
    from metro_dashboard_bridge.desktop_app import SettingsDialog
    from metro_dashboard_bridge.desktop_runtime import DesktopOptions

    options = DesktopOptions(model_path="/example/best.pt", detection=True, localization=True)
    dialog = SettingsDialog(options, None)
    dialog.checks["model_demo"].setChecked(True)
    assert not dialog.checks["detection"].isEnabled()
    assert not dialog.checks["localization"].isEnabled()
    saved = dialog.options()
    assert saved.model_demo and saved.detection and saved.localization
    assert saved.model_path == options.model_path
    dialog.checks["model_demo"].setChecked(False)
    assert dialog.checks["detection"].isEnabled()
    assert dialog.checks["localization"].isEnabled()
    dialog.close()


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


@pytest.fixture
def monitored_window(tmp_path, qt_app, monkeypatch):
    from metro_dashboard_bridge import desktop_app

    window = desktop_app.InspectionWindow(
        Path.cwd(), tmp_path / "settings.json", tmp_path / "logs", False
    )
    window.timer.stop()
    monkeypatch.setattr(window, "check_health", lambda: None)
    monkeypatch.setattr(window, "reload_page", lambda: None)
    window.options.model_demo = True
    window.start_operation([sys.executable, "-c", "import time; time.sleep(60)"], "backend")
    yield window
    if window.runner.active:
        window.runner.process.kill()
        window.runner.process.wait()
        window.runner.poll()
    window.close()
    qt_app.processEvents()


def report_camera_health(window, online, total):
    class Reply:
        def error(self):
            return False

        def readAll(self):
            return json.dumps({"service": "metro_dashboard_bridge", "status": "online",
                               "cameras": {"online": online, "total": total}}).encode()

        def deleteLater(self):
            pass

    window.on_health(Reply())


@pytest.mark.parametrize("previously_ready", [True, False])
def test_camera_timeout_keeps_backend_and_recovers_actions(monitored_window, monkeypatch, previously_ready):
    from metro_dashboard_bridge import desktop_app

    window = monitored_window
    now = window.runner.started_at + 200
    monkeypatch.setattr(desktop_app, "time", SimpleNamespace(monotonic=lambda: now))
    if previously_ready:
        report_camera_health(window, 12, 12)
        assert all(action.isEnabled() for action in window.demo_actions)
        window.last_complete_streams = now - 25
    report_camera_health(window, 11, 12)
    assert not any(action.isEnabled() for action in window.demo_actions)
    window.tick()
    assert window.runner.active
    assert not window.requested_stop
    assert not window.failure_reason
    assert window.stream_warning
    window.tick()
    diagnostics = window.runner.log_path.with_suffix(".diagnostics.jsonl")
    events = [json.loads(line)["event"] for line in diagnostics.read_text().splitlines()]
    assert events.count("camera_stream_timeout") == 1
    report_camera_health(window, 12, 12)
    assert not window.stream_warning
    assert all(action.isEnabled() for action in window.demo_actions)


def test_health_timeout_still_stops_and_persists_reason(monitored_window, monkeypatch):
    from metro_dashboard_bridge import desktop_app

    window = monitored_window
    now = window.runner.started_at + 200
    monkeypatch.setattr(desktop_app, "time", SimpleNamespace(monotonic=lambda: now))
    report_camera_health(window, 12, 12)
    window.last_health = now - 16
    window.tick()
    assert window.requested_stop
    assert "15 秒" in window.failure_reason
    diagnostics = window.runner.log_path.with_suffix(".diagnostics.jsonl")
    records = [json.loads(line) for line in diagnostics.read_text().splitlines()]
    stop = next(record for record in records if record["event"] == "stop_requested")
    assert stop["reason"] == window.failure_reason
    assert stop["cameras"] == {"online": 12, "total": 12}


def test_web_print_opens_native_output_dialog_and_can_cancel(qt_app):
    from PyQt5.QtTest import QSignalSpy, QTest
    from PyQt5.QtWidgets import QMessageBox
    from metro_dashboard_bridge.report_view import ReportView

    view = ReportView()
    loaded = QSignalSpy(view.loadFinished)
    view.setHtml("<html><body>Inspection report</body></html>")
    assert loaded.wait(10000)
    view.page().runJavaScript("window.print()")
    deadline = time.monotonic() + 5
    # Poll inside Qt's event loop so the modal dialog can also be dismissed.
    from PyQt5.QtCore import QTimer
    seen = []
    timer = QTimer()

    def cancel_dialog():
        for widget in qt_app.topLevelWidgets():
            if isinstance(widget, QMessageBox) and widget.windowTitle() == "打印 / 导出 PDF":
                seen.append([button.text() for button in widget.buttons()])
                widget.reject()
        if time.monotonic() > deadline:
            timer.stop()

    timer.timeout.connect(cancel_dialog)
    timer.start(25)
    try:
        while not seen and time.monotonic() < deadline:
            QTest.qWait(50)
        assert seen and "导出 PDF" in seen[0] and "打印" in seen[0]
        assert not view.output_busy
    finally:
        timer.stop()
        view.close()


def test_report_pdf_export_and_cancel(tmp_path, qt_app, monkeypatch):
    from PyQt5.QtTest import QSignalSpy
    from metro_dashboard_bridge import report_view

    view = report_view.ReportView()
    loaded = QSignalSpy(view.loadFinished)
    view.setHtml("<html><body><h1>巡检报告</h1><p>裂缝 (1.2, 3.4, 5.6)</p></body></html>")
    assert loaded.wait(10000)
    target = tmp_path / "report.pdf"
    monkeypatch.setattr(report_view.QFileDialog, "getSaveFileName", lambda *args: (str(target), "PDF (*.pdf)"))
    notices = []
    monkeypatch.setattr(report_view.QMessageBox, "information", lambda *args: notices.append(args[2]))
    finished = QSignalSpy(view.page().pdfPrintingFinished)
    view.export_pdf()
    assert finished.wait(10000)
    assert finished[0][1]
    assert target.read_bytes().startswith(b"%PDF-")
    assert not view.output_busy and notices
    monkeypatch.setattr(report_view.QFileDialog, "getSaveFileName", lambda *args: ("", ""))
    view.output_busy = True
    view.export_pdf()
    assert not view.output_busy
    view.close()


def test_native_print_keeps_printer_until_finished(tmp_path, qt_app, monkeypatch):
    from PyQt5.QtTest import QSignalSpy, QTest
    from metro_dashboard_bridge import report_view

    view = report_view.ReportView()
    loaded = QSignalSpy(view.loadFinished)
    view.setHtml("<html><body>Inspection report</body></html>")
    assert loaded.wait(10000)
    target = tmp_path / "printer.pdf"

    class PrintToFileDialog:
        def __init__(self, printer, parent):
            assert printer.resolution() == 300
            printer.setOutputFormat(report_view.QPrinter.PdfFormat)
            printer.setOutputFileName(str(target))

        def setWindowTitle(self, title):
            pass

        def exec_(self):
            return report_view.QDialog.Accepted

    monkeypatch.setattr(report_view, "QPrintDialog", PrintToFileDialog)
    failures = []
    monkeypatch.setattr(report_view.QMessageBox, "warning", lambda *args: failures.append(args[2]))
    view.print_report()
    assert view.output_busy and view.printer is not None
    deadline = time.monotonic() + 10
    while view.output_busy and time.monotonic() < deadline:
        QTest.qWait(25)
    assert not view.output_busy and view.printer is None and not failures
    assert target.read_bytes().startswith(b"%PDF-")
    view.close()
