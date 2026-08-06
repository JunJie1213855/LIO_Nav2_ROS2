import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share = get_package_share_directory('gld_robot_description')

    default_urdf = os.path.join(pkg_share, 'model', 'robosenseAiry.urdf')
    # rviz_file = os.path.join(pkg_share, 'rviz', 'gld_robot_description.rviz')

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf,
        description='URDF model file path'
    )

    # use_rviz_arg = DeclareLaunchArgument(
    #     'rviz', default_value='true',
    #     description='Launch RViz2'
    # )

    def load_urdf(context):
        urdf_path = LaunchConfiguration('urdf_file').perform(context)
        with open(urdf_path, 'r') as f:
            robot_desc = f.read()
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc}]
            ),
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="screen",
            #     arguments=["-d", rviz_file],
            #     condition=IfCondition(LaunchConfiguration('rviz')),
            # )
        ]

    return LaunchDescription([
        urdf_file_arg,
        # use_rviz_arg,
        OpaqueFunction(function=load_urdf)
    ])
