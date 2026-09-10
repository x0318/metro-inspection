# 桌面应用

运行环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。Windows 使用安装了
Ubuntu 22.04 和 WSLg 的 WSL 2。应用默认启动静止仿真、YOLO、三维定位和网页桥接，
在 Qt 窗口内显示平台；不自动行驶，默认不额外打开 Gazebo 和 RViz 窗口。

## 打开

- Windows：双击仓库根目录的 `Open Metro Inspection.vbs`。仓库放在 WSL 的
  `\\wsl.localhost\Ubuntu-22.04\...` 目录时，会自动识别发行版和 Linux 路径。
  `Open Metro Inspection.cmd` 为保留控制台的诊断入口。
- 已安装的 Windows 桌面入口：双击 `Metro Inspection.vbs`，无须打开终端。
- Linux：在应用菜单中打开 `Metro Inspection`。

如果配置端口已有正常的平台后台，入口会直接连接它，避免重复启动造成端口冲突。
此时仍可刷新页面和保存运行设置；设置下次启动生效。后台由原入口管理，关闭新窗口
不会停止仿真，新窗口的停止后台和演示视点按钮不启用。

安装个人应用菜单入口：

```bash
python3 scripts/install_inspection_shortcut.py
```

WSL 中还可指定 Windows 桌面目录生成无控制台入口：

```bash
python3 scripts/install_inspection_shortcut.py --windows-desktop /mnt/c/Users/用户名/Desktop
```

## 首次部署

代码仓库不包含本机编译结果、Python 虚拟环境和模型权重。首次使用仍需安装依赖；
完成后日常启动不需要输入 launch 命令。

本版位于 `feature/simulation` 分支。建议在 WSL 的 Linux 主目录中重新克隆，
使用 Git LFS 下载隧道网格；网页下载 ZIP 可能只得到模型指针文件。

```bash
sudo apt install git git-lfs
git lfs install
git clone --branch feature/simulation https://github.com/x0318/metro-inspection.git
cd metro-inspection
git lfs pull
```

以下命令在新克隆的项目根目录执行，要求已经安装 ROS 2 Humble：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  python3-pyqt5.qtwebengine python3-venv fonts-noto-cjk \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-controllers ros-humble-robot-localization liburdfdom-tools
source /opt/ros/humble/setup.bash
# rosdep 尚未初始化时先执行 sudo rosdep init
rosdep update
rosdep install --from-paths ros_ws/src --ignore-src -r -y --rosdistro humble
bash scripts/setup_yolo_environment.sh
```

打开应用，在“系统 / 准备工作空间”中编译。成功后进入“运行设置”选择本机 `.pt`
权重，再启动平台。尚未准备检测环境时可关闭“病害识别”，先验证相机采集。
更换权重不会自动重新启动或驱动车辆。

每台电脑需要在自己的项目目录编译，不能复制其他人的 `build/`、`install/`、
`log/` 或虚拟环境。这些目录包含本机路径、软链接或二进制依赖。如果之前复制过
整个工作空间，使用上面的新目录部署，可以保留旧目录排查。

YOLO 权重不随 Git 仓库上传。由项目负责人提供当前使用的 `.pt` 文件，在“运行设置”
中选择该文件的本机路径；不要沿用其他人的 `/home/jo/...` 路径。

首次编译也可在终端运行一次以下命令。它会同时构建自定义消息包和平台依赖：

```bash
bash scripts/run_inspection_backend.sh --build
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
ros2 pkg prefix metro_bringup
ros2 pkg prefix metro_inspection_interfaces
```

曾出现的 `install/setup.bash: No such file` 表示尚未在该目录完成编译；
缺少 `metro_inspection_interfaces/.../local_setup.bash` 表示编译产物不完整或
来自另一份工作空间。中文显示为方框时，安装 `fonts-noto-cjk` 后重新打开 Qt。

运行期间也可打开“运行设置”。保存的修改在下一次启动平台时生效，当前画面和
后台连接继续使用原设置。要立即启用 Gazebo 或 RViz 窗口，保存后停止并重新启动平台。

## 打印与导出 PDF

在“统计报告”中点击“打印 / 导出 PDF”，Qt 会弹出输出方式选择。
选择“导出 PDF”后指定保存位置；报告使用 A4 横向排版，保留病害类别、相机和坐标。
选择“打印”则打开 Qt 打印对话框。WSL 中没有配置打印机时，可先导出 PDF，
再由 Windows 打开文件打印。文件保存在所选目录，不会自动下载到远程设备。
普通浏览器和 Tailscale 浏览器入口仍使用浏览器自身的打印功能。

## 故障处理

- 启动前检查安装目录、模型文件和端口。缺失资源直接显示在主窗口。
- 后台 ROS 包及消息类型支持检查失败时，错误保留在日志面板。
- YOLO 等待六路相机实际收到图像后启动；发布者存在不代表相机已经可用。
- Qt 窗口独立于 ROS launch。关键后台进程退出时停止该次运行，保留窗口、日志和
  重新启动入口。不会在故障后自动恢复行驶。
- HTTP 连接持续中断时结束本次后台运行并显示原因。
- 图像流启动超过 180 秒仍未就绪，或曾就绪后连续 20 秒未能恢复全部在线时，
  显示持续告警并保留后台和页面；图像恢复后自动清除告警。演示视点仅在图像全部
  在线时可用。行驶所需传感器与速度指令的停车保护仍由独立 watchdog 执行。
  “图像流就绪”只表示图像正在更新，不表示检测正确或三维定位已有有效结果。
- Qt 网页渲染进程退出时显示错误，可重新加载页面或使用系统浏览器查看。
- 关闭应用先通知启动进程执行正常退出，30 秒后仍未退出则发送终止信号，40 秒后强制清理
  本次运行的进程组。不会用全局 `pkill` 终止其他项目。
- 桌面主进程意外退出时，Linux 父进程退出信号会通知后台启动进程结束运行。
- WSL 默认使用 Qt 软件 OpenGL 和禁用 WebEngine GPU 的兼容配置，保留 Chromium
  沙箱。系统仍需要可用的 WSLg 图形环境。

设置保存在 `~/.config/metro-inspection/desktop.json`。
桌面启动日志为 `~/.local/state/metro-inspection/desktop.log`。
每次后台运行日志在 `~/.local/state/metro-inspection/runs/`，界面可直接打开目录。
后台故障模块和退出码另存为同名 `.status.json`。Windows 入口自身的错误记录在
`%LOCALAPPDATA%\MetroInspection\launcher.log`。
桌面监控另存同名 `.diagnostics.jsonl`，记录停止原因、图像在线数量变化、渲染进程退出，
以及每 10 秒的可用内存、交换空间和 HTTP 健康状态。它是周期采样，不能排除采样间的瞬时内存峰值。
原有 SQLite 病害数据位置和存储方式不变。

## 验证

```bash
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
python3 -m pytest ros_ws/src/metro_dashboard_bridge/test/test_desktop_runtime.py
METRO_TEST_QT=1 python3 -m pytest ros_ws/src/metro_dashboard_bridge/test/test_desktop_window.py
python3 -m pytest ros_ws/src/metro_mapping/metro_closed_loop/test
```

Qt 测试需要有效的 DISPLAY。实际相机与点云效果还需使用目标机器完成仿真运行验证；
本地通过不代表任意显卡驱动和 WSL 配置都不会出错。

### 2026-09-09 本机验收

- 工作空间七个相关包构建通过，发布前启动、桥接、检测、定位、相机校准和停车保护
  的 104 项测试通过，含运行中修改设置、Git LFS 指针检查和无相机消息时的退出测试。
- 实际双击 Windows 桌面入口启动成功，五路原图和五路检测图均在线。
- 静止仿真实测约 50 秒图像流就绪，一轮正常关闭后 2.6 秒结束后台。
  另一次 Gazebo 退出阻塞触发超时清理，约 11 秒结束；清理后无后台残留。
  Gazebo Classic 的偶发退出阻塞仍是已知限制。
- 主动终止 YOLO 后，窗口保留错误、日志和重启入口；再次启动恢复全部图像流。
- 未完成长时间连续运行、队友电脑部署和行驶过程中故障的验收。
  本次结果验证启动与退出链路，不代表渗漏水、异物识别准确率已经达标。

### 后续识别工作

V4 在固定观察位置、当前在线参数下未达到两类病害稳定检出的要求，V5 的独立
视角评测也没有改善。桌面默认权重暂未替换。第二阶段需要重新核验二维检测框、
目标的实际可见表面和定位数据来源，再接入模型辅助确认；不能用图像流在线数或
参考里程命中数代替识别准确率。已修正显示用拼音标签进入下游事件的问题，
ROS 消息保持统一的英文病害类别，平台据此显示中文。
本轮代码整理限于桌面入口和故障处理相关改动，
尚未进行全项目清理。
