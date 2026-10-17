import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio_robosense')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    def launch_lio(context):
        overrides = {'use_sim_time': use_sim_time}
        point_filter = LaunchConfiguration('point_filter_num').perform(context)
        if point_filter:
            point_filter = int(point_filter)
            if point_filter < 1:
                raise ValueError('point_filter_num must be at least 1')
            overrides['point_filter_num'] = point_filter
        dense = LaunchConfiguration('dense_publish_en').perform(context)
        if dense:
            overrides['publish.dense_publish_en'] = dense == 'true'
        return [Node(
            package='fast_lio_robosense',
            executable='fastlio_mapping',
            parameters=[PathJoinSubstitution([config_path, config_file]), overrides],
            output='screen'
        )]
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(DeclareLaunchArgument(
        'point_filter_num', default_value='',
        description='Override preprocessing point decimation; 1 keeps all valid points, empty uses YAML'))
    ld.add_action(DeclareLaunchArgument(
        'dense_publish_en', default_value='', choices=['', 'true', 'false'],
        description='Override publication of pre-voxel-filtered undistorted scan; empty uses YAML'))

    ld.add_action(OpaqueFunction(function=launch_lio))
    ld.add_action(rviz_node)

    return ld
