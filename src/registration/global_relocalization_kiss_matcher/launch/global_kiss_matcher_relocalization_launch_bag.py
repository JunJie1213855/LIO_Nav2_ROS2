import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # 优先使用稠密地图 (Ctrl+C 时自动保存)，fallback 到旧稀疏地图
    dense_pcd = os.path.join(
        get_package_share_directory("nav2_planner"),
        "mcu", "robo_map.pcd"
    )
    legacy_pcd = os.path.join(
        get_package_share_directory("nav2_planner"),
        "pcd", "arobo_map.pcd"
    )
    pcd_path = dense_pcd if os.path.exists(dense_pcd) else legacy_pcd
    print("pcd path : ", pcd_path)

    # 节点
    node = Node(
        package="global_relocalization_kiss_matcher",
        executable="global_kiss_matcher_relocalization_exec",
        namespace="",
        output="screen",
        emulate_tty=True,
        remappings=remappings,
        parameters=[
            {
                "num_threads": 1,
                "num_neighbors": 5,
                "global_leaf_size": 0.5,
                "registered_leaf_size": 0.5,
                "max_dist_sq": 1.0,
                "voxel_resolution": 0.5,
                "use_global_initialization": True,
                "use_kiss_recovery": True,
                "gicp_max_consecutive_failures": 2,
                "recovery_min_points": 1000,
                "recovery_cooldown_sec": 2.0,
                "verify_kiss_with_gicp": True,
                "loop.num_inliers_threshold": 3,
                "loop.overlap_threshold": 80.0,
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
