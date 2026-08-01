"""Spawn Go2 in Gazebo Fortress with gz_ros2_control."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # xacro package:// URIs are converted by Gazebo to
    # model://go2_description/..., so Gazebo must search the parent of this
    # package's share directory.  Without this, the robot is spawned but its
    # DAE visual meshes cannot be rendered.
    resource_root = os.path.dirname(get_package_share_directory("go2_description"))
    description_share = FindPackageShare("go2_description")
    ros_gz_share = FindPackageShare("ros_gz_sim")
    model = PathJoinSubstitution([description_share, "xacro", "robot.xacro"])
    world = PathJoinSubstitution([description_share, "worlds", "empty.sdf"])
    bridge_config = PathJoinSubstitution([description_share, "config", "bridge.yaml"])
    robot_description = {
        "robot_description": Command(["xacro ", model, " use_gazebo:=true"]),
        "use_sim_time": True,
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r -v 3 ", world], "on_exit_shutdown": "true"}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "go2",
            "-allow_renaming", "true",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster, trajectory_controller],
        )
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="go2_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    return LaunchDescription(
        [
            # Fortress uses IGN_GAZEBO_RESOURCE_PATH.  Set the newer alias as
            # well, keeping this launch usable with newer Gazebo releases.
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[resource_root, os.pathsep,
                       EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[resource_root, os.pathsep,
                       EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.5"),
            gazebo,
            state_publisher,
            bridge,
            spawn,
            start_controllers,
        ]
    )
