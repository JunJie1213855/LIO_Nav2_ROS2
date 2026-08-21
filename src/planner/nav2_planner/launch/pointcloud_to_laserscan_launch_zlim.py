import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('nav2_planner'),
        'config',
        'Pointcloud2d_3d_zlim.yaml'
    )

    # use_sim_time: 实机 false (默认); bag 回放时传 true (需 bag 带 /clock)
    # 这里显式传给节点, 覆盖 config 文件里硬编码的值。
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time (bag 回放时传 true)'
    )

    # 定义 pointcloud_to_laserscan 节点
    pc2l_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='Pointcloud2d_3d',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('cloud_in', '/registered_scan'),   # 输入3d点云
            ('scan', '/scan')               # 输出2d点云
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        pc2l_node
    ])