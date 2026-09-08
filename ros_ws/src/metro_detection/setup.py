from glob import glob

from setuptools import find_packages, setup


package_name = "metro_detection"


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
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jo0625",
    maintainer_email="jo0625@users.noreply.github.com",
    description="YOLOv8 2D damage detection for the Metro inspection system.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_detector = metro_detection.yolo_detector:main",
        ],
    },
)
