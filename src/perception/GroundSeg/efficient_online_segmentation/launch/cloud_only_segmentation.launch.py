"""Standalone point-cloud input, no odometry subscription or TF dependency."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('efficient_online_segmentation')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'ground_separation.launch.py')),
        launch_arguments={'mode': 'cloud_only'}.items())])
