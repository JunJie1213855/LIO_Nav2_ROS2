"""Paired or cloud-only segmentation; never starts or depends on TF publishers."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_nodes(context):
    standalone = LaunchConfiguration('mode').perform(context) == 'cloud_only'
    overrides = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    for arg, parameter in [
        ('cloud_topic', 'sub_cloud_topic'), ('odometry_topic', 'odometry_topic'),
        ('pub_odometry_topic', 'pub_odometry_topic'), ('ground_topic', 'pub_ground_cloud_topic'),
        ('nonground_topic', 'pub_no_ground_cloud_topic'), ('body_frame_id', 'body_frame_id'),
        ('sensor_frame_id', 'sensor_frame_id')]:
        value = LaunchConfiguration(arg).perform(context)
        if value:
            overrides[parameter] = value
    for parameter in ['input_cloud_is_world', 'odometry_is_sensor_pose']:
        value = LaunchConfiguration(parameter).perform(context)
        if value:
            overrides[parameter] = value == 'true'
    if standalone:
        overrides['input_cloud_is_world'] = False
        if 'sub_cloud_topic' not in overrides:
            overrides['sub_cloud_topic'] = '/livox/lidar'
    share = get_package_share_directory('efficient_online_segmentation')
    return [
        Node(package='efficient_online_segmentation',
             executable='cloud_only_segmentation_node' if standalone else 'ground_separation_node',
             name='ground_separation', parameters=[LaunchConfiguration('params_file'), overrides], output='screen'),
        Node(package='rviz2', executable='rviz2', name='ground_seg_rviz',
             arguments=['-d', os.path.join(share, 'launch', 'efficient_online_segmentation.rviz')],
             parameters=[{'use_sim_time': overrides['use_sim_time']}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('efficient_online_segmentation')
    args = [
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'gazebo_mid360.yaml')),
        DeclareLaunchArgument('mode', default_value='paired', choices=['paired', 'cloud_only']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]
    for name in ['cloud_topic', 'odometry_topic', 'pub_odometry_topic', 'ground_topic',
                 'nonground_topic', 'body_frame_id', 'sensor_frame_id']:
        args.append(DeclareLaunchArgument(name, default_value='', description='Empty uses the selected YAML profile'))
    for name in ['input_cloud_is_world', 'odometry_is_sensor_pose']:
        args.append(DeclareLaunchArgument(name, default_value='', choices=['', 'true', 'false'],
                                          description='Empty uses YAML; cloud-only always requires sensor-frame input'))
    return LaunchDescription(args + [OpaqueFunction(function=launch_nodes)])
