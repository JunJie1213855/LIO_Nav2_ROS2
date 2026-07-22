import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_share = get_package_share_directory('gld_robot_description')

    # 默认使用 gld_robot_description.urdf, 可通过 launch 参数切换为 my_description.urdf
    default_urdf = os.path.join(pkg_share, 'model', 'gld_robot_description.urdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'gld_robot_description.rviz')

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf,
        description='URDF 模型文件路径. 例: urdf_file:=/ws/src/gld_robot_description/model/my_description.urdf'
    )

    # LaunchConfiguration 不能直接传给 os.path 和 open(),
    # 所以用 OpaqueFunction 在 runtime 读取文件内容
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
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_file],
            )
        ]

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        urdf_file_arg,
        OpaqueFunction(function=load_urdf)
    ])
