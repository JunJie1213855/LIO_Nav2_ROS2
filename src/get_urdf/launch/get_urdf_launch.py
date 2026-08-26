import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    pkg_share_path = get_package_share_directory('get_urdf')

    # robot 参数: 选择 model/<robot>.urdf（默认 simple_car，可选 diff_robot_2d）
    robot = LaunchConfiguration('robot').perform(context)
    urdf_file_path = os.path.join(pkg_share_path, 'model', f'{robot}.urdf')

    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    rviz_config_path = os.path.join(pkg_share_path, 'rviz', 'nav2.rviz')
    default_world_path = os.path.join(pkg_share_path, 'worlds', 'indoor_office.world')

    # Gazebo 模型搜索路径
    dataset_models_path = '/home/ros/dataset/gazebo_models_worlds_collection/models'
    pkg_models_path = os.path.join(pkg_share_path, 'models')
    gazebo_model_path = f'{pkg_models_path}:{dataset_models_path}'

    return [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                 LaunchConfiguration('world_path')],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='urdf_spawner',
                    output='screen',
                    arguments=['-entity', robot, '-topic', 'robot_description',
                               '-timeout', '30.0'],
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    pkg_share_path = get_package_share_directory('get_urdf')
    default_world_path = os.path.join(pkg_share_path, 'worlds', 'indoor_2d.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_path',
            default_value=default_world_path,
            description='Gazebo 世界模型文件路径 (.world)'),

        DeclareLaunchArgument(
            'robot',
            default_value='simple_car',
            description='URDF 模型名（model/<robot>.urdf），例如 diff_robot_2d'),

        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='是否启动 RViz2'),

        OpaqueFunction(function=launch_setup),
    ])
