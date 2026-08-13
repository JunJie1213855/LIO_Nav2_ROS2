import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 动态获取 get_urdf 包在 install 目录下的绝对路径
    pkg_share_path = get_package_share_directory('get_urdf')

    rviz_config_path = os.path.join(pkg_share_path, 'rviz', 'nav2.rviz')
    default_world_path = os.path.join(pkg_share_path, 'worlds', 'field.world')

    # Gazebo 模型搜索路径（冒号分隔）
    #   1. 包内 models 目录（如果有的话）
    #   2. gazebo_models_worlds_collection 中的室内/室外模型
    dataset_models_path = '/home/ros/dataset/gazebo_models_worlds_collection/models'
    pkg_models_path = os.path.join(pkg_share_path, 'models')
    gazebo_model_path = f'{pkg_models_path}:{dataset_models_path}'

    # 拼接出准确的 URDF 路径
    urdf_file_path = os.path.join(pkg_share_path, 'model', 'simple_car.urdf')

    # 2. 读取 URDF 文件内容
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # ── 声明 launch 参数 ──────────────────────────────────────────
        DeclareLaunchArgument(
            'world_path',
            default_value=default_world_path,
            description='Gazebo 世界模型文件路径 (.world)'),

        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='是否启动 RViz2'),

        # ── 设置环境变量 ──────────────────────────────────────────────
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        # ── 启动 Gazebo 仿真引擎 ──────────────────────────────────────
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                 LaunchConfiguration('world_path')],
            output='screen'),

        # ── robot_state_publisher ─────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]),

        # ── 延迟 3 秒生成机器人 ───────────────────────────────────────
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='urdf_spawner',
                    output='screen',
                    arguments=['-entity', 'simple_car', '-topic', 'robot_description',
                               '-timeout', '30.0'],
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),

        # ── RViz2（可通过 rviz:=false 关闭）───────────────────────────
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
        )

    ])