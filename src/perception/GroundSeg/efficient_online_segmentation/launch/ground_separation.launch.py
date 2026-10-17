import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('efficient_online_segmentation')
    default_params = os.path.join(pkg_dir, 'config', 'gazebo_mid360_sep.yaml')

    params_file = LaunchConfiguration('params_file')

    ground_separation_node = Node(
        package='efficient_online_segmentation',
        executable='ground_separation_node',
        name='ground_separation',
        parameters=[params_file, {
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='机器人模型已发布传感器外参时应关闭，避免重复 TF'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='ground_separation 节点参数 YAML。',
        ),
        ground_separation_node,
    ])
