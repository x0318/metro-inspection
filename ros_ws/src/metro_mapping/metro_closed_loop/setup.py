from glob import glob
from setuptools import setup

package_name = "metro_closed_loop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf.xacro")),
        (f"share/{package_name}/worlds", glob("worlds/*.world")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yang",
    maintainer_email="yang@example.com",
    description="Gazebo camera/lidar closed loop for metro damage localization.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "auto_driver = metro_closed_loop.auto_driver:main",
            "damage_detector = metro_closed_loop.damage_detector:main",
            "reset_closed_loop = metro_closed_loop.reset_closed_loop:main",
        ],
    },
)

