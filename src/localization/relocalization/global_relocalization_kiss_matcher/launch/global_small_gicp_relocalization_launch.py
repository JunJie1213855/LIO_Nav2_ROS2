import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pcd_path = os.path.join(
        get_package_share_directory("nav2_planner_bringup"),
        "pcd", "nav_test_4_27.pcd"
    )

    node = Node(
        package="global_relocalization_kiss_matcher",
        executable="global_small_gicp_relocalization_exec",
        namespace="",
        output="screen",
        emulate_tty=True,
        remappings=remappings,
        parameters=[
            {
                "num_threads": 4,
                "num_neighbors": 10,
                "global_leaf_size": 0.25,
                "registered_leaf_size": 0.25,
                "max_dist_sq": 1.0,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "lidar_frame": "livox_frame",
                "robot_base_frame": "base_footprint",
                "prior_pcd_file": pcd_path,
                "input_cloud_topic": "/registered_scan",
            }
        ],
    )

    return LaunchDescription([node])
