#!/usr/bin/env python3
"""Install per-user Linux and optional Windows desktop launchers."""

import argparse
import os
from pathlib import Path
import subprocess


def desktop_quote(value):
    value = str(value).replace("%", "%%")
    for character in ("\\", '"', "`", "$"):
        value = value.replace(character, "\\" + character)
    return '"' + value + '"'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--windows-desktop", type=Path)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    apps = (
        Path(os.environ.get("XDG_DATA_HOME", str(Path.home() / ".local/share")))
        / "applications"
    )
    apps.mkdir(parents=True, exist_ok=True)
    entry = apps / "metro-inspection.desktop"
    entry.write_text(
        "[Desktop Entry]\nType=Application\nName=Metro Inspection\n"
        "Name[zh_CN]=地铁巡检系统\nTerminal=false\nCategories=Science;Robotics;\n"
        f"Exec=/bin/bash {desktop_quote(project / 'scripts/open_inspection_app.sh')}\n"
        "Icon=applications-science\n",
        encoding="utf-8",
    )
    entry.chmod(0o755)
    print(entry)
    if args.windows_desktop:
        if not args.windows_desktop.is_dir():
            raise ValueError("Windows Desktop directory does not exist")
        windows_script = subprocess.check_output(
            ["wslpath", "-w", str(project / "scripts/open_inspection_windows.ps1")],
            text=True,
        ).strip()
        if any(c in windows_script for c in ('"', "\n", "\r", "%")):
            raise ValueError("Unsupported character in shortcut path")
        command = f'powershell.exe -NoProfile -ExecutionPolicy Bypass -File "{windows_script}"'
        launcher = args.windows_desktop / "Metro Inspection.vbs"
        escaped = command.replace('"', '""')
        launcher.write_text(
            'Set shell = CreateObject("WScript.Shell")\r\n'
            f'code = shell.Run("{escaped}", 0, True)\r\n',
            encoding="utf-16",
        )
        print(launcher)
    subprocess.run(["desktop-file-validate", str(entry)], check=True)


if __name__ == "__main__":
    main()
