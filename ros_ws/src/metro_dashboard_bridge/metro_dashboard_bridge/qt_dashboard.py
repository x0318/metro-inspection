import argparse
import signal
import sys
import time
from typing import Optional, Sequence
from urllib.error import URLError
from urllib.request import Request, urlopen


def _health_url(dashboard_url: str) -> str:
    return f"{dashboard_url.rstrip('/')}/api/health"


def wait_for_dashboard(dashboard_url: str, timeout_seconds: float) -> bool:
    deadline = time.monotonic() + timeout_seconds
    request = Request(_health_url(dashboard_url), method="GET")
    while time.monotonic() < deadline:
        try:
            with urlopen(request, timeout=1.0) as response:
                if response.status == 200:
                    return True
        except (OSError, URLError):
            pass
        time.sleep(0.25)
    return False


def _parse_arguments(args: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open the Metro Inspection dashboard in a Qt window."
    )
    parser.add_argument("--url", default="http://127.0.0.1:8088")
    parser.add_argument("--title", default="地铁多领域病害综合智能巡检系统")
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    return parser.parse_args(args)


def main(args: Optional[Sequence[str]] = None) -> int:
    options = _parse_arguments(args)
    if options.startup_timeout <= 0.0:
        raise ValueError("startup timeout must be positive")

    if not wait_for_dashboard(options.url, options.startup_timeout):
        print(
            f"Dashboard did not become ready at {_health_url(options.url)}",
            file=sys.stderr,
        )
        return 1

    try:
        from PyQt5.QtCore import QTimer, QUrl
        from PyQt5.QtWidgets import QApplication, QMainWindow
        from .report_view import ReportView
    except ImportError as error:
        print(
            "Qt dashboard requires python3-pyqt5.qtwebengine. Install it with: "
            "sudo apt install python3-pyqt5.qtwebengine",
            file=sys.stderr,
        )
        print(f"Import error: {error}", file=sys.stderr)
        return 2

    application = QApplication([sys.argv[0]])
    application.setApplicationName("Metro Inspection Dashboard")

    window = QMainWindow()
    window.setWindowTitle(options.title)
    window.resize(1440, 900)
    browser = ReportView(window)
    browser.setUrl(QUrl(options.url))
    window.setCentralWidget(browser)
    window.show()

    def stop_application(_signal_number, _frame) -> None:
        application.quit()

    signal.signal(signal.SIGINT, stop_application)
    signal.signal(signal.SIGTERM, stop_application)
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(250)

    return application.exec_()
