from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Super-LIO 输出：
    #   里程计 /lio/odom          (frame_id="world", world→imu)
    #   点云   /lio/cloud_world   (frame_id="world")
    # lio_interface 把它们桥接到标准 odom 帧，
    # 发布 /registered_odometry + /registered_scan。

    lio_interface_node = Node(
        package='lio_interface',
        executable='lio_interface_node',
        namespace='',
        output='screen',
        emulate_tty=True,  # 开启提示颜色
        parameters=[{
            'use_sim_time': True,
            'odometry_sub': '/lio/odom',
        }],
        remappings=[
            ('/cloud_registered', LaunchConfiguration('cloud_topic')),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_topic', default_value='/lio/cloud_world',
            description='Super-LIO 输出点云 topic'),
        lio_interface_node,
    ])
