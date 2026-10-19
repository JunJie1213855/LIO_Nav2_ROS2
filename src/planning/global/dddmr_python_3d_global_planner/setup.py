from glob import glob
from setuptools import find_packages, setup

setup(
    name='dddmr_ros2', version='0.1.0',
    packages=['dddmr_ros2'] + find_packages(where='dddmr_py', include=['dddmr_py*']),
    package_dir={'dddmr_py': 'dddmr_py/dddmr_py'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/dddmr_ros2']),
        ('share/dddmr_ros2', ['package.xml', 'ROS2_README.md']),
        ('share/dddmr_ros2/launch', glob('launch/*.launch.py')),
        ('share/dddmr_ros2/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy>=1.21', 'scipy>=1.7'],
    zip_safe=True,
    maintainer='ros', maintainer_email='ros@example.com',
    description='DDDMR ROS2 global planner bridge', license='Apache-2.0',
    entry_points={'console_scripts': ['planner_node = dddmr_ros2.node:main']},
)
