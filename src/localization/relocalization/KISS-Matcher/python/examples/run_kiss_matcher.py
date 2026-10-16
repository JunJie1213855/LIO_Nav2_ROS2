import argparse

import kiss_matcher
import numpy as np
import viser
from kiss_matcher.io_utils import read_bin, read_pcd, read_ply


def remove_nan_from_point_cloud(point_cloud):
    return point_cloud[np.isfinite(point_cloud).any(axis=1)]


def rotate_point_cloud_yaw(point_cloud, yaw_angle_deg):
    yaw_angle_rad = np.radians(yaw_angle_deg)
    rotation_matrix = np.array([
        [np.cos(yaw_angle_rad), -np.sin(yaw_angle_rad), 0],
        [np.sin(yaw_angle_rad),
         np.cos(yaw_angle_rad), 0],
        [0, 0, 1],
    ])
    return (rotation_matrix @ point_cloud.T).T


if __name__ == "__main__":
    # Just tips for checking whether pybinding goes sucessfully or not.
    # attributes = dir(kiss_matcher)
    # for attr in attributes:
    #     print(attr)

    parser = argparse.ArgumentParser(
        description="Point cloud registration using KISS-Matcher.")
    parser.add_argument(
        "--src_path",
        type=str,
        required=True,
        help="Source point cloud file (.bin, .pcd, or .ply)",
    )
    parser.add_argument(
        "--tgt_path",
        type=str,
        required=True,
        help="Target point cloud file (.bin, .pcd, or .ply)",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        required=True,
        help="Resolution for processing (e.g., voxel size)",
    )
    parser.add_argument(
        "--yaw_aug_angle",
        type=float,
        required=False,
        help="Yaw augmentation angle in degrees",
    )

    args = parser.parse_args()

    # Load source point cloud
    if args.src_path.endswith(".bin"):
        src = read_bin(args.src_path)
    elif args.src_path.endswith(".pcd"):
        src = read_pcd(args.src_path)
    elif args.src_path.endswith(".ply"):
        src = read_ply(args.src_path)
    else:
        raise ValueError("Unsupported file format for src. Use .bin, .pcd, or .ply")

    # Apply yaw augmentation if provided
    if args.yaw_aug_angle is not None:
        src = rotate_point_cloud_yaw(src, args.yaw_aug_angle)

    # Load target point cloud
    if args.tgt_path.endswith(".bin"):
        tgt = read_bin(args.tgt_path)
    elif args.tgt_path.endswith(".pcd"):
        tgt = read_pcd(args.tgt_path)
    elif args.tgt_path.endswith(".ply"):
        tgt = read_ply(args.tgt_path)
    else:
        raise ValueError("Unsupported file format for tgt. Use .bin, .pcd, or .ply")

    # Remove NaN values from the point cloud.
    # See https://github.com/MIT-SPARK/KISS-Matcher/pull/16
    src = remove_nan_from_point_cloud(src)
    tgt = remove_nan_from_point_cloud(tgt)

    print(f"Loaded source point cloud: {src.shape}")
    print(f"Loaded target point cloud: {tgt.shape}")
    print(f"Resolution: {args.resolution}")
    print(f"Yaw Augmentation Angle: {args.yaw_aug_angle}")

    params = kiss_matcher.KISSMatcherConfig(args.resolution)
    matcher = kiss_matcher.KISSMatcher(params)
    result = matcher.estimate(src, tgt)
    matcher.print()

    num_rot_inliers = matcher.get_num_rotation_inliers()
    num_final_inliers = matcher.get_num_final_inliers()
    # NOTE(hlim): By checking the final inliers, we can determine whether
    # the registration was successful or not. The larger the threshold,
    # the more conservatively the decision is made.
    # See https://github.com/MIT-SPARK/KISS-Matcher/issues/24
    thres_num_inliers = 5
    if (num_final_inliers < thres_num_inliers):
        print("\033[1;33m=> Registration might have failed :(\033[0m\n")
    else:
        print("\033[1;32m=> Registration likely succeeded XD\033[0m\n")

    # ------------------------------------------------------------
    # Visualization with Viser
    # ------------------------------------------------------------
    # Apply transformation to src
    rotation_matrix = np.array(result.rotation)
    translation_vector = np.array(result.translation)
    transformed_src = (rotation_matrix @ src.T).T + translation_vector

    # Create Viser server for visualization
    server = viser.ViserServer()
    print("Viser server started. Open the web interface to view the point clouds.")
    print("Server URL: http://localhost:8080")

    # Add point clouds to viser
    server.scene.add_point_cloud(
        "/source_cloud",
        points=src.astype(np.float32),
        colors=np.array([[200, 200, 200] for _ in range(src.shape[0])], dtype=np.uint8),  # Gray
        point_size=0.03,
    )

    server.scene.add_point_cloud(
        "/target_cloud",
        points=tgt.astype(np.float32),
        colors=np.array([[90, 165, 230] for _ in range(tgt.shape[0])], dtype=np.uint8),  # Cyan
        point_size=0.03,
    )

    server.scene.add_point_cloud(
        "/transformed_source",
        points=transformed_src.astype(np.float32),
        colors=np.array([[255, 160, 0] for _ in range(transformed_src.shape[0])], dtype=np.uint8),  # Orange
        point_size=0.03,
    )

    print("Point clouds added to visualization:")
    print("- Gray: Original source cloud")
    print("- Cyan: Target cloud")
    print("- Orange: Transformed source cloud")
    print("\nPress Ctrl+C to exit...")

    try:
        # Keep server running
        while True:
            import time
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nVisualization stopped.")
        server.stop()
