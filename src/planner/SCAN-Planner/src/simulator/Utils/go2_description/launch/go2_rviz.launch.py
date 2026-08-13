"""Inspect the Go2 model and joints in RViz2 without a physics simulator."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("go2_description")
    model = PathJoinSubstitution([share, "xacro", "robot.xacro"])
    rviz_config = PathJoinSubstitution([share, "launch", "check_joint.rviz"])
    robot_description = {"robot_description": Command(["xacro ", model, " use_gazebo:=false"])}
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
