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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yang",
    maintainer_email="yang@example.com",
    description="Detection adapter for the existing Metro simulation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_info_calibrator = metro_closed_loop.camera_info_calibrator:main",
            "damage_detector = metro_closed_loop.damage_detector:main",
        ],
    },
)
