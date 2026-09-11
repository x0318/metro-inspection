from setuptools import find_packages, setup
from pathlib import Path


package_name = 'metro_navigation_demo'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ] + [
        ('share/' + package_name + '/' + str(directory),
         [str(path) for path in directory.iterdir() if path.is_file()])
        for root in ('launch', 'config', 'worlds', 'urdf', 'models', 'web')
        for directory in [Path(root), *[p for p in Path(root).rglob('*') if p.is_dir()]]
        if directory.exists()
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jo0625',
    maintainer_email='jo0625@users.noreply.github.com',
    description=(
        'Web navigation and safety bridge for the subway patrol dashboard.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspection_dashboard_bridge = '
            'metro_navigation_demo.main:main',
            'drive_watchdog = metro_navigation_demo.drive_watchdog:main',
        ],
    },
)
