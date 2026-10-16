import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('efficient_online_segmentation')
    default_params = os.path.join(pkg_dir, 'config', 'segmentation_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'launch', 'efficient_online_segmentation.rviz')

    params_file = LaunchConfiguration('params_file')

    segmentation_node = Node(
        package='efficient_online_segmentation',
        executable='efficient_online_segmentation_node',
        name='efficient_online_segmentation',
        parameters=[params_file],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    def launch_bag_conditionally(context):
        bag = context.launch_configurations.get('bag_filename', '')
        if not bag:
            return []
        return [
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag],
                output='screen',
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the segmentation parameter YAML (e.g. config/robosense_airy.yaml).',
        ),
        DeclareLaunchArgument(
            'bag_filename',
            default_value='',
            description='Absolute path to a rosbag to replay (leave empty to skip).',
        ),
        segmentation_node,
        rviz_node,
        OpaqueFunction(function=launch_bag_conditionally),
    ])
