# 岔轨测试平台页面

本页面随 `metro_navigation_demo` 安装，由同一个进程提供静态资源、相机接口和导航控制接口。
在当前项目根目录运行 `bash scripts/open_route_choice_platform.sh`，访问 `http://127.0.0.1:8090`。
不能通过直接打开 HTML 文件控制机器人。

页面显示训练场小车的前向、左侧、右侧和轨道四路图像，支持到岔口询问、目的地选择、
暂停、继续、取消及急停。场景未启用病害检测，病害表保持空白，除非通过接口明确写入记录。

启动参数、接口和迁移说明见包根目录的 `README.md`。
第三方网页依赖及许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
