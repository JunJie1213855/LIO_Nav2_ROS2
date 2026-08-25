import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """RoboSense Airy + FAST-LIO 的 Cartographer 2D 建图 launch。

    实机: use_sim_time:=False (默认)
    bag 回放: use_sim_time:=True (需 bag 带 /clock)

    依赖的 TF (由其他节点提供):
      map -> odom            cartographer 自己发布 (provide_odom_frame=true)
      odom -> base_footprint sensor_scan_generation
      base_footprint -> chassis -> livox_frame   gld_robot_description (URDF)
    """
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time')

    pkg_share = get_package_share_directory('nav2_planner_bringup')
    config_dir = os.path.join(pkg_share, 'config', 'cartographer')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_airy_2d.lua',
        ],
        output='screen',
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': 0.05},
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node,
        occupancy_grid_node,
    ])
