import math
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
    world_path = os.path.abspath(LaunchConfiguration('world_path').perform(context))
    spawn_pose = []
    if os.path.basename(world_path) == 'rough_terrain.world':
        # Spiral entry: radius 23 m, theta=-pi/2. Face the inward spiral
        # tangent (dx/dt, dy/dt)=(69*pi, 14), with 5 cm ground clearance.
        spawn_pose = ['-x', '0', '-y', '-23', '-z', '0.05',
                      '-Y', str(math.atan2(14, 69 * math.pi))]
        for mesh_name in ['rough_terrain_mesh.stl', 'rough_terrain_fence.stl']:
            mesh_path = os.path.join(os.path.dirname(world_path), mesh_name)
            if not os.path.isfile(mesh_path):
                raise RuntimeError(
                    f'缺少地形或围栏网格: {mesh_path}。请运行源码 worlds/ 目录中的 '
                    'generate_rough_terrain.py，再 colcon build --packages-select get_urdf。')

    # Resolve package assets while retaining Gazebo's built-in shaders/materials.
    # Setting GAZEBO_RESOURCE_PATH replaces Gazebo's fallback system paths, even
    # when the parent terminal has not sourced /usr/share/gazebo/setup.sh.
    resource_paths = [pkg_share_path, os.path.dirname(os.path.dirname(world_path))]
    resource_paths.extend(filter(None, context.environment.get('GAZEBO_RESOURCE_PATH', '').split(os.pathsep)))
    install_prefix = context.environment.get('GAZEBO_INSTALL_PREFIX', '/usr')
    for path in [os.path.join(install_prefix, 'share', 'gazebo-11'),
                 '/usr/share/gazebo-11', '/usr/share/gazebo']:
        if os.path.isdir(path):
            resource_paths.append(path)
    gazebo_resource_path = os.pathsep.join(dict.fromkeys(resource_paths))

    # Gazebo 模型搜索路径
    dataset_models_path = '/home/ros/dataset/gazebo_models_worlds_collection/models'
    pkg_models_path = os.path.join(pkg_share_path, 'models')
    gazebo_model_path = f'{pkg_models_path}:{dataset_models_path}'

    return [
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_resource_path),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                 world_path],
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
                               '-timeout', '30.0'] + spawn_pose,
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
    default_world_path = os.path.join(pkg_share_path, 'worlds', 'rough_terrain.world')

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
