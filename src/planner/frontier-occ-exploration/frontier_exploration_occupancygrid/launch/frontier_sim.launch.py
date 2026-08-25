#!/usr/bin/env python3
"""TurtleBot3 simulation for the frontier_exploration ROS2 port.

Brings up:
  1. Gazebo (headless gzserver by default; optional gzclient GUI via gui:=true)
     with the TurtleBot3 burger + slam_toolbox SLAM + Nav2 navigation
  2. RViz with the exploration displays (map, inflated map, frontiers, centroids, goal, home)
  3. frontier_planner: the exploration node (sends goals directly to Nav2 /navigate_to_pose)

Usage:
  ros2 launch frontier_exploration frontier_sim.launch.py
  # other worlds (pass a full .world path):
  ros2 launch frontier_exploration frontier_sim.launch.py \
      world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world
  # open the Gazebo GUI as well:
  ros2 launch frontier_exploration frontier_sim.launch.py gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_desc = get_package_share_directory('turtlebot3_description')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')
    frontier_exploration = get_package_share_directory('frontier_exploration')
    rviz_config = os.path.join(frontier_exploration, 'rviz', 'frontier_exploration.rviz')

    print("path of rviz config : ", rviz_config)
    model = LaunchConfiguration('model', default='burger')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration(
        'world', default=os.path.join(tb3_gazebo, 'worlds', 'turtlebot3_world.world'))
    gui = LaunchConfiguration('gui', default='false')
    params_file = LaunchConfiguration(
        'params_file', default=os.path.join(tb3_nav2, 'param', 'humble', 'burger.yaml'))

    return LaunchDescription([
        # DeclareLaunchArgument('model', default_value='burger',
        #                       description='TurtleBot3 model (burger/waffle/waffle_pi)'),
        # DeclareLaunchArgument('use_sim_time', default_value='true'),
        # DeclareLaunchArgument('world', default_value=os.path.join(tb3_gazebo, 'worlds', 'turtlebot3_world.world'),
        #                       description='Full path to a Gazebo .world file'),
        # DeclareLaunchArgument('gui', default_value='false',
        #                       description='Open the Gazebo GUI (gzclient)'),
        # DeclareLaunchArgument('params_file', default_value=os.path.join(tb3_nav2, 'param', 'humble', 'burger.yaml')),

        # SetEnvironmentVariable('TURTLEBOT3_MODEL', model),
        # SetEnvironmentVariable(
        #     'GAZEBO_MODEL_PATH',
        #     os.path.join(tb3_gazebo, 'models') + ':' + os.path.join(tb3_desc, 'meshes')),

        # # 1a) Gazebo server (headless; use gui:=true to also open gzclient)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(PathJoinSubstitution([gazebo_ros, 'launch', 'gzserver.launch.py'])),
        #     launch_arguments={'world': world}.items(),
        # ),
        # # 1b) Gazebo GUI (optional)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(PathJoinSubstitution([gazebo_ros, 'launch', 'gzclient.launch.py'])),
        #     condition=IfCondition(gui),
        # ),
        # # 1c) TurtleBot3 robot state publisher + spawn
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(PathJoinSubstitution([tb3_gazebo, 'launch', 'robot_state_publisher.launch.py'])),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(PathJoinSubstitution([tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py'])),
        #     launch_arguments={'x_pose': '-2.0', 'y_pose': '-0.5'}.items(),
        # ),

        # 2) slam_toolbox + Nav2 navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': 'True',
                'map': os.path.join(tb3_nav2, 'map', 'map.yaml'),  # required arg; unused in slam mode
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'True',
                'use_composition': 'False',
            }.items(),
        ),

        # 3) RViz with the exploration displays (map, inflated map, frontiers, centroids, goal, home)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ),

        # 4) frontier exploration node
        Node(
            package='frontier_exploration',
            executable='frontier_planner',
            name='frontier_planner',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'obstacle_inflation': 0.3},
                        {'map_revolution': 0.1},
                        {'cmd_topic': 'cmd_vel'},
                        {'robot_base_frame': 'base_link'},
                        {'goal_tolerance': 0.3},
                        {'obstacle_tolerance': 0.5},
                        {'rotate_speed': 0.5}],
        ),
    ])
