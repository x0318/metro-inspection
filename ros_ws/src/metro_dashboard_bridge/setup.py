from setuptools import find_packages, setup


package_name = "metro_dashboard_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jo0625",
    maintainer_email="jo0625@users.noreply.github.com",
    description="ROS 2 defect event and camera streaming bridge.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "defect_event_bridge = metro_dashboard_bridge.main:main",
            "dashboard_qt = metro_dashboard_bridge.qt_dashboard:main",
            "inspection_desktop = metro_dashboard_bridge.desktop_app:main",
        ],
    },
)
