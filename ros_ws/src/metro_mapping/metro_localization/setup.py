from glob import glob
from setuptools import setup

package_name = "metro_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yang",
    maintainer_email="yang@example.com",
    description="Image-point-cloud fusion and 3D damage localization.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "damage_localizer = metro_localization.damage_localizer:main",
            "localization_evaluator = metro_localization.localization_evaluator:main",
            "damage_semantic_mapper = metro_localization.damage_semantic_mapper:main",
        ],
    },
)
