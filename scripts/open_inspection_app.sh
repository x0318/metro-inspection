#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# The shell must open even when the ROS workspace has not been built yet.
export PYTHONPATH="${PROJECT_DIR}/ros_ws/src/metro_dashboard_bridge${PYTHONPATH:+:${PYTHONPATH}}"
export PYTHONUNBUFFERED=1
if [[ -d /mnt/wslg ]]; then
  export DISPLAY="${DISPLAY:-:0}"
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
  export QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS:---disable-gpu}"
  export QT_OPENGL="${QT_OPENGL:-software}"
fi

if ! /usr/bin/python3 -c 'from PyQt5.QtWidgets import QApplication; from PyQt5.QtWebEngineWidgets import QWebEngineView' 2>/dev/null; then
  MESSAGE="Metro Inspection requires Qt. Install: sudo apt install python3-pyqt5.qtwebengine fonts-noto-cjk"
  echo "${MESSAGE}" >&2
  if command -v xmessage >/dev/null && [[ -n "${DISPLAY:-}" ]]; then
    xmessage -center "${MESSAGE}" || true
  fi
  exit 2
fi

LOG_DIR="${XDG_STATE_HOME:-${HOME}/.local/state}/metro-inspection"
mkdir -p "${LOG_DIR}"
exec /usr/bin/python3 -m metro_dashboard_bridge.desktop_app \
  --project-dir "${PROJECT_DIR}" "$@" \
  >>"${LOG_DIR}/desktop.log" 2>&1
