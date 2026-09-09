"""Qt application owning the inspection backend and its diagnostics."""

import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
import traceback

from PyQt5.QtCore import QLockFile, QTimer, QUrl, Qt
from PyQt5.QtGui import QDesktopServices, QFont
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkRequest
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import (
    QAction,
    QApplication,
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QDockWidget,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QStackedWidget,
    QStyle,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from .desktop_runtime import (
    DesktopOptions,
    OwnedProcess,
    backend_command,
    load_options,
    preflight,
    save_options,
)


class SettingsDialog(QDialog):
    def __init__(self, options, parent):
        super().__init__(parent)
        self.setWindowTitle("运行设置")
        self.setMinimumWidth(620)
        layout = QFormLayout(self)
        self.model = QLineEdit(options.model_path)
        browse = QPushButton(self.style().standardIcon(QStyle.SP_DirOpenIcon), "", self)
        browse.setToolTip("选择模型权重")
        browse.clicked.connect(self.choose_model)
        row = QHBoxLayout()
        row.addWidget(self.model)
        row.addWidget(browse)
        layout.addRow("模型权重", row)
        self.checks = {}
        for name, title in (
            ("detection", "病害识别"),
            ("localization", "三维定位"),
            ("gazebo_gui", "Gazebo 窗口"),
            ("rviz", "RViz 窗口"),
        ):
            check = QCheckBox(title)
            check.setChecked(getattr(options, name))
            self.checks[name] = check
            layout.addRow(check)
        self.checks["localization"].setEnabled(options.detection)
        self.checks["detection"].toggled.connect(self.checks["localization"].setEnabled)
        self.numbers = {}
        for name, title, limits in (
            ("port", "平台端口", (1024, 65535)),
            ("domain", "ROS 域", (0, 101)),
            ("gazebo_port", "仿真端口", (1024, 65535)),
        ):
            spin = QSpinBox()
            spin.setRange(*limits)
            spin.setValue(getattr(options, name))
            self.numbers[name] = spin
            layout.addRow(title, spin)
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Save).setText("保存")
        buttons.button(QDialogButtonBox.Cancel).setText("取消")
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def choose_model(self):
        name, _ = QFileDialog.getOpenFileName(
            self, "选择模型权重", self.model.text(), "YOLO (*.pt)"
        )
        if name:
            self.model.setText(name)

    def options(self):
        return DesktopOptions(
            model_path=self.model.text().strip(),
            **{name: widget.isChecked() for name, widget in self.checks.items()},
            **{name: widget.value() for name, widget in self.numbers.items()},
        )


class InspectionWindow(QMainWindow):
    def __init__(self, project, config_path, log_dir, autostart=True):
        super().__init__()
        self.project = project
        self.config_path = config_path
        self.runner = OwnedProcess(log_dir)
        self.operation = None
        self.pending_options = None
        self.requested_stop = False
        self.closing = False
        self.failure_reason = ""
        self.backend_seen = False
        self.page_loaded = False
        self.last_health = None
        self.last_complete_streams = None
        self.reply = None
        self.next_health = 0.0
        self.network = QNetworkAccessManager(self)
        config_error = ""
        try:
            self.options = load_options(config_path)
        except (OSError, ValueError, TypeError) as error:
            self.options = DesktopOptions()
            config_error = str(error)
        self.setWindowTitle("地铁多领域病害综合智能巡检系统")
        self.resize(1440, 920)
        self.setMinimumSize(760, 540)
        self.setWindowIcon(self.style().standardIcon(QStyle.SP_ComputerIcon))
        self.setStyleSheet(
            "QToolBar { spacing: 8px; padding: 5px; } QStatusBar { padding: 3px; }"
        )

        toolbar = QToolBar("运行控制", self)
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        self.start_action = self.add_tool(
            toolbar, QStyle.SP_MediaPlay, "启动平台", self.start_backend
        )
        self.stop_action = self.add_tool(
            toolbar, QStyle.SP_MediaStop, "停止平台", self.stop_backend
        )
        toolbar.addSeparator()
        self.reload_action = self.add_tool(
            toolbar, QStyle.SP_BrowserReload, "重新加载页面", self.reload_page
        )
        self.settings_action = self.add_tool(
            toolbar, QStyle.SP_FileDialogDetailedView, "运行设置", self.configure
        )
        self.add_tool(
            toolbar, QStyle.SP_FileDialogContentsView, "运行日志", self.toggle_logs
        )
        self.add_tool(toolbar, QStyle.SP_DirOpenIcon, "打开日志目录", self.open_logs)
        toolbar.addSeparator()
        self.state_label = QLabel("未启动")
        toolbar.addWidget(self.state_label)

        menu = self.menuBar().addMenu("系统")
        self.build_action = QAction("准备工作空间", self)
        self.build_action.triggered.connect(self.build_workspace)
        menu.addAction(self.build_action)
        menu.addAction("运行设置", self.configure)
        menu.addAction("在浏览器中打开", lambda: QDesktopServices.openUrl(QUrl(self.url)))
        menu.addSeparator()
        menu.addAction("退出", self.close)

        self.pages = QStackedWidget()
        self.setCentralWidget(self.pages)
        self.message_page = QWidget()
        layout = QVBoxLayout(self.message_page)
        layout.setContentsMargins(40, 32, 40, 32)
        self.message_title = QLabel("地铁多领域病害综合智能巡检系统")
        self.message_title.setStyleSheet("font-size: 22px; font-weight: 600;")
        self.message_title.setWordWrap(True)
        self.message_body = QLabel("准备启动")
        self.message_body.setWordWrap(True)
        self.message_body.setTextFormat(Qt.PlainText)
        self.message_body.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.message_title)
        layout.addWidget(self.message_body)
        layout.addStretch()
        self.pages.addWidget(self.message_page)
        self.browser = QWebEngineView(self)
        self.browser.loadFinished.connect(self.on_page_loaded)
        self.browser.page().renderProcessTerminated.connect(self.on_renderer_exit)
        self.pages.addWidget(self.browser)

        self.logs = QPlainTextEdit()
        self.logs.setReadOnly(True)
        self.logs.setMaximumBlockCount(1800)
        self.log_dock = QDockWidget("运行日志", self)
        self.log_dock.setWidget(self.logs)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)
        self.log_dock.hide()
        self.statusBar().showMessage("自动行驶关闭")
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(200)
        self.update_actions()
        if config_error:
            self.show_message("运行设置无法读取", config_error)
        elif autostart:
            QTimer.singleShot(300, self.start_backend)

    @property
    def url(self):
        return f"http://127.0.0.1:{self.options.port}"

    def add_tool(self, toolbar, icon, title, callback):
        action = QAction(self.style().standardIcon(icon), title, self)
        action.setToolTip(title)
        action.triggered.connect(callback)
        toolbar.addAction(action)
        return action

    def show_message(self, title, detail):
        self.message_title.setText(title)
        self.message_body.setText(detail)
        self.pages.setCurrentWidget(self.message_page)

    def update_actions(self):
        active = self.runner.active
        self.start_action.setEnabled(not active and not self.closing)
        self.stop_action.setEnabled(active and not self.requested_stop)
        self.settings_action.setEnabled(not self.closing)
        self.build_action.setEnabled(not active)
        self.reload_action.setEnabled(self.backend_seen and active)

    def start_operation(self, command, operation):
        self.runner.close_log()
        self.logs.clear()
        self.failure_reason = ""
        self.backend_seen = False
        self.page_loaded = False
        self.last_health = None
        self.last_complete_streams = None
        self.requested_stop = False
        self.operation = operation
        try:
            self.runner.start(command, self.project, os.environ.copy())
        except OSError as error:
            self.show_message("无法启动", str(error))
            self.state_label.setText("启动失败")
            return
        self.state_label.setText("正在准备" if operation == "build" else "正在启动")
        self.statusBar().showMessage(str(self.runner.log_path))
        self.update_actions()

    def start_backend(self):
        if self.runner.active:
            return
        try:
            preflight(self.project, self.options)
        except (OSError, ValueError) as error:
            self.show_message("启动检查未通过", str(error))
            self.state_label.setText("启动失败")
            return
        self.browser.setUrl(QUrl("about:blank"))
        self.show_message("正在启动巡检平台", "正在检查运行环境并等待相机数据。")
        self.start_operation(backend_command(self.project, self.options), "backend")

    def build_workspace(self):
        if self.runner.active:
            return
        self.show_message("正在准备工作空间", "编译结果将显示在运行日志中。")
        self.log_dock.show()
        self.start_operation(
            [
                "/bin/bash",
                str(self.project / "scripts/run_inspection_backend.sh"),
                "--build",
            ],
            "build",
        )

    def stop_backend(self):
        if not self.runner.active:
            return
        self.requested_stop = True
        self.state_label.setText("正在停止")
        self.runner.stop()
        self.update_actions()

    def configure(self):
        dialog = SettingsDialog(self.pending_options or self.options, self)
        if self.runner.active:
            dialog.setWindowTitle("运行设置（下次启动生效）")
        if dialog.exec_() == QDialog.Accepted:
            try:
                options = dialog.options()
                save_options(self.config_path, options)
                if self.runner.active:
                    self.pending_options = options
                    self.statusBar().showMessage("设置已保存，下次启动平台时生效。")
                else:
                    self.options = options
                    self.pending_options = None
                    self.show_message("设置已保存", "平台未启动。")
            except (OSError, ValueError) as error:
                QMessageBox.warning(self, "无法保存设置", str(error))

    def reload_page(self):
        if self.backend_seen and self.runner.active:
            self.page_loaded = False
            self.browser.setUrl(QUrl(self.url))

    def on_page_loaded(self, ok):
        if self.browser.url().scheme() not in ("http", "https"):
            return
        if ok and self.runner.active and not self.failure_reason:
            self.page_loaded = True
            self.pages.setCurrentWidget(self.browser)
        elif self.runner.active:
            self.show_message("页面加载失败", "后台状态和日志仍可查看。可重新加载页面或在浏览器中打开。")

    def on_renderer_exit(self, status, code):
        self.page_loaded = False
        self.logs.appendPlainText(f"[Qt renderer] status={status}, exit={code}")
        self.show_message("显示进程已退出", f"退出码：{code}。后台仍由应用管理，可重新加载页面或停止平台。")
        self.log_dock.show()

    def toggle_logs(self):
        self.log_dock.setVisible(not self.log_dock.isVisible())

    def open_logs(self):
        self.runner.log_dir.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.runner.log_dir)))

    def check_health(self):
        if self.reply is not None:
            return
        reply = self.network.get(QNetworkRequest(QUrl(self.url + "/api/health")))
        self.reply = reply
        deadline = QTimer(reply)
        deadline.setSingleShot(True)
        deadline.timeout.connect(reply.abort)
        deadline.start(1500)
        reply.finished.connect(lambda: self.on_health(reply))

    def on_health(self, reply):
        try:
            if not self.runner.active or self.requested_stop:
                return
            if reply.error():
                return
            data = json.loads(bytes(reply.readAll()))
            if (
                data.get("service") != "metro_dashboard_bridge"
                or data.get("status") != "online"
            ):
                return
            self.last_health = time.monotonic()
            cameras = data.get("cameras", {})
            expected = 10 if self.options.detection else 5
            online = cameras.get("online", 0)
            complete = online >= expected
            if complete:
                self.last_complete_streams = self.last_health
            self.state_label.setText(
                f"{'图像流就绪' if complete else '等待图像流'}  |  {online} / {expected}"
            )
            if not self.backend_seen:
                self.backend_seen = True
                self.reload_page()
                self.update_actions()
        except (ValueError, TypeError, AttributeError):
            pass
        finally:
            self.reply = None
            reply.deleteLater()

    def tick(self):
        output = self.runner.read_output()
        if output:
            self.logs.appendPlainText(output.rstrip())
        code = self.runner.poll()
        if code is not None:
            remaining = self.runner.read_output()
            if remaining:
                self.logs.appendPlainText(remaining.rstrip())
            self.runner.close_log()
            if self.pending_options is not None:
                self.options = self.pending_options
                self.pending_options = None
            self.backend_seen = False
            self.last_complete_streams = None
            if self.closing:
                self.close()
                return
            if self.failure_reason:
                self.show_message("运行中断", self.failure_reason)
                self.state_label.setText("运行故障")
                self.log_dock.show()
            elif self.requested_stop:
                self.show_message("平台已停止", "本次运行日志已保留。")
                self.state_label.setText("已停止")
            elif self.operation == "build" and code == 0:
                self.show_message("工作空间已准备", "可以启动平台。")
                self.state_label.setText("准备完成")
            else:
                self.show_message(
                    "后台进程已退出",
                    (self.runner.failure_detail() or f"退出码：{code}")
                    + f"\n日志：{self.runner.log_path}",
                )
                self.state_label.setText("运行故障")
                self.log_dock.show()
            self.update_actions()
        if (
            self.runner.active
            and self.operation == "backend"
            and not self.requested_stop
        ):
            now = time.monotonic()
            if now >= self.next_health:
                self.next_health = now + 2
                self.check_health()
            elapsed = now - (self.last_health or self.runner.started_at)
            limit = 15 if self.backend_seen else 180
            if elapsed > limit:
                self.failure_reason = (
                    "平台连接中断超过 15 秒。" if self.backend_seen else "平台启动超过 180 秒仍未就绪。"
                )
                self.stop_backend()
            if not self.requested_stop:
                stream_elapsed = now - (
                    self.last_complete_streams or self.runner.started_at
                )
                stream_limit = 20 if self.last_complete_streams else 180
                if stream_elapsed > stream_limit:
                    self.failure_reason = "相机或识别图像流未就绪或持续中断，请查看运行日志。"
                    self.stop_backend()

    def closeEvent(self, event):
        if self.runner.active:
            event.ignore()
            self.closing = True
            self.stop_backend()
            self.show_message("正在退出", "正在停止本次启动的后台进程并保存日志。")
        else:
            self.runner.close_log()
            event.accept()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-dir", required=True, type=Path)
    parser.add_argument("--no-autostart", action="store_true")
    parser.add_argument("--config", type=Path)
    options = parser.parse_args(args)
    app = QApplication([sys.argv[0]])
    app.setApplicationName("Metro Inspection")
    app.setFont(QFont("Noto Sans CJK SC", 10))
    state = (
        Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state")))
        / "metro-inspection"
    )
    state.mkdir(parents=True, exist_ok=True)
    lock = QLockFile(str(state / "desktop.lock"))
    lock.setStaleLockTime(0)
    if not lock.tryLock(0):
        QMessageBox.information(None, "Metro Inspection", "巡检应用已经打开。")
        return 0
    config = (
        options.config
        or Path(os.environ.get("XDG_CONFIG_HOME", str(Path.home() / ".config")))
        / "metro-inspection/desktop.json"
    )
    window = InspectionWindow(
        options.project_dir.resolve(), config, state / "runs", not options.no_autostart
    )
    window.show()

    def report_exception(error_type, error, trace):
        detail = "".join(traceback.format_exception(error_type, error, trace))
        print(detail, file=sys.stderr, flush=True)
        window.logs.appendPlainText(detail)
        window.failure_reason = str(error)
        window.show_message("运行异常", str(error))
        window.log_dock.show()
        window.stop_backend()

    sys.excepthook = report_exception
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    code = app.exec_()
    lock.unlock()
    return code


if __name__ == "__main__":
    sys.exit(main())
