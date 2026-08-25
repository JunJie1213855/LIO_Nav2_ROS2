import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    me_share_path = get_package_share_directory('nav2_planner_bringup')

    params_file = os.path.join(me_share_path, 'config', 'nav2_params.yaml')
    map_yaml_file = os.path.join(me_share_path, 'map', 'test_map__2.yaml')
    rviz_file = os.path.join(me_share_path, 'rviz', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')

    # composition 容器参数
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True),
        allow_substs=True)

    # composition 容器（必须先于 navigation 启动）
    container_cmd = Node(
        condition=IfCondition(use_composition),
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        emulate_tty=True,
        parameters=[configured_params],
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'use_composition': use_composition,
            'container_name': 'nav2_container'
        }.items()
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file,
                    {'yaml_filename': map_yaml_file},
                    {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager_map_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation /clock time'))
    ld.add_action(DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Launch Nav2 in shared container (~200MB saved)'))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=map_yaml_file,
        description='Full path to map yaml file'))
    ld.add_action(container_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(lifecycle_manager_map_cmd)
    ld.add_action(rviz_cmd)

    return ld
