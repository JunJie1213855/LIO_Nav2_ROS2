# Lidar_nav2_ws Environment Setup

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22313F?logo=ros)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](https://releases.ubuntu.com/22.04/)
[![Nav2](https://img.shields.io/badge/Nav2-Navigation2-4B8BBE)](https://navigation.ros.org/)
[![LiDAR](https://img.shields.io/badge/LiDAR-Livox%20MID--360-00A6D6)](https://www.livoxtech.com/mid-360)

`Lidar_nav2_ws` is a ROS 2 Humble-based 3D LiDAR autonomous navigation workspace that supports both Gazebo simulation and Livox MID-360 real-robot deployment. This document only covers the environment setup process for this project and assumes that ROS 2 Humble has already been installed.

## Table of Contents

- [1. Requirements](#1-requirements)
- [2. Get the Code](#2-get-the-code)
- [3. Install Dependencies](#3-install-dependencies)
- [4. Optional Dependencies](#4-optional-dependencies)
- [5. Build](#5-build)
- [6. Quick Checks](#6-quick-checks)
- [7. Run](#7-run)
- [8. Switching the LIO Backend](#8-switching-the-lio-backend)
- [9. Troubleshooting](#9-troubleshooting)
- [10. Common Commands](#10-common-commands)

## 1. Requirements

Recommended environment:

| Item | Version / Description |
| --- | --- |
| Operating system | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill |
| Gazebo | Gazebo Classic 11 / ROS 2 Gazebo plugins |
| Build tools | colcon, CMake, GCC/G++ |
| LiDAR | Livox MID-360, required only for real-robot mode |

Source the ROS 2 environment before building or running:

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

Expected output:

```text
humble
```

Install basic tools:

```bash
sudo apt update
sudo apt install -y \
  git wget curl vim gnupg lsb-release software-properties-common \
  build-essential cmake ninja-build pkg-config \
  python3-pip python3-colcon-common-extensions python3-rosdep
```

If `rosdep` has not been initialized:

```bash
sudo rosdep init
rosdep update
```

If `rosdep update` is slow in the current network environment, use `rosdepc`:

```bash
sudo pip3 install rosdepc
sudo rosdepc init
rosdepc update
```

## 2. Get the Code

Clone the workspace:

```bash
cd ~
git clone https://github.com/Ikunio/Lidar_nav2_ws.git
cd Lidar_nav2_ws
```

If the workspace already exists, enter it directly:

```bash
cd ~/Lidar_nav2_ws
```

Expected directory structure:

```text
Lidar_nav2_ws/
├── src/
└── scripts/
    ├── build.sh
    ├── mapping_sim.sh
    ├── nav2_sim.sh
    ├── mapping_real.sh
    ├── nav2_real.sh
    ├── save_map.sh
    ├── save_pcd.sh
    ├── show_tf_tree.sh
    └── RUN.sh
```

## 3. Install Dependencies

Prefer using `rosdep` to install dependencies according to `package.xml`:

```bash
cd ~/Lidar_nav2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

You can also use `rosdepc`:

```bash
cd ~/Lidar_nav2_ws
source /opt/ros/humble/setup.bash
rosdepc install --from-paths src --ignore-src --rosdistro humble -r -y
```

`rosdep` covers most ROS 2, PCL, OpenCV, GTSAM, OpenMP, glog, and message dependencies declared in `package.xml`. If the build still reports missing system libraries, install them according to the error message. Common extra packages are:

```bash
sudo apt install -y \
  libtbb-dev \
  libboost-all-dev \
  qtbase5-dev qtbase5-private-dev \
  python3-tk
```

If Gazebo or navigation-related packages are missing:

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-tf2-tools
```

## 4. Optional Dependencies

### 4.1 Livox-SDK2

`src/livox_ros_driver2` already includes a prebuilt Livox-SDK2, so installing the SDK separately is usually unnecessary.

Before real-robot deployment, check the MID-360 network configuration:

```bash
vim src/livox_ros_driver2/config/MID360_config.json
```

Confirm the following:

- The host IP in `host_net_info` matches the LiDAR network interface.
- The LiDAR IP in `lidar_configs` matches the actual MID-360.
- The host and LiDAR are on the same subnet.

### 4.2 KISS-Matcher

`scripts/nav2_sim.sh` and `scripts/nav2_real.sh` use `global_relocalization_kiss_matcher` by default. If the build reports missing `kiss_matcher` or `robin`, install the C++ libraries:

```bash
cd ~/Lidar_nav2_ws/src/registration/KISS-Matcher
make deps
make cppinstall
```

If ROBIN is already installed and `make cppinstall` fails:

```bash
make cppinstall_matcher_only
```

### 4.3 small_gicp

If the build reports missing `small_gicp`, install it from source:

```bash
cd /tmp
git clone https://github.com/koide3/small_gicp.git
cd small_gicp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
```

## 5. Build

Build the workspace:

```bash
cd ~/Lidar_nav2_ws
source /opt/ros/humble/setup.bash
cd scripts
./build.sh
```

`build.sh` is equivalent to:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

After the build completes, source the workspace:

```bash
source ~/Lidar_nav2_ws/install/setup.bash
```

Optional: Add it to `~/.bashrc` so new terminals source the workspace automatically:

```bash
echo "source ~/Lidar_nav2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 6. Quick Checks

Check whether key packages can be found by ROS 2:

```bash
ros2 pkg list | grep -E "me_nav2_bringup|lio_interface|fast_lio|livox_ros_driver2|global_relocalization_kiss_matcher"
```

Check launch-file arguments:

```bash
ros2 launch me_nav2_bringup my_nav2_launch.py --show-args
ros2 launch lio_interface lio_interface_launch.py --show-args
```

Check the TF tool:

```bash
ros2 run tf2_tools view_frames
```

## 7. Run

### 7.1 Simulation Mapping

Start simulation mapping:

```bash
cd ~/Lidar_nav2_ws
source install/setup.bash
cd scripts
./mapping_sim.sh
```

This script starts Gazebo, GUI teleoperation, FAST-LIO, `lio_interface`, `sensor_scan_generation`, `pointcloud_to_laserscan`, `slam_toolbox`, and Nav2.

Runtime checks:

```bash
ros2 topic hz /scan
ros2 topic hz /registered_scan
ros2 run tf2_ros tf2_echo odom base_footprint
```

After driving the robot through the environment, save the maps:

```bash
./save_map.sh
./save_pcd.sh
```

Map output locations:

```text
2D map: src/me_nav2_bringup/map/
3D PCD: src/me_nav2_bringup/pcd/
```

### 7.2 Simulation Navigation

Before navigation, check the map paths:

```bash
cd ~/Lidar_nav2_ws
vim src/me_nav2_bringup/launch/my_nav2_launch.py
vim src/registration/global_relocalization_kiss_matcher/launch/global_kiss_matcher_relocalization_launch.py
vim src/registration/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py
```

Confirm the following:

- `map_yaml_file` points to the `.yaml` map to be used.
- `prior_pcd_file` points to the `.pcd` point-cloud map to be used.
- Only one node is allowed to publish `map -> odom` at the same time.

Start simulation navigation:

```bash
cd ~/Lidar_nav2_ws
source install/setup.bash
cd scripts
./nav2_sim.sh
```

Runtime checks:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /registered_scan
```

Use `Nav2 Goal` in RViz to send a navigation goal.

### 7.3 Real-Robot Run

Before real-robot mapping or navigation, confirm:

- MID-360 network connectivity: `ping <MID360_IP>`.
- `src/livox_ros_driver2/config/MID360_config.json` is configured correctly.
- Robot power, LiDAR, IMU, chassis controller, emergency stop, and manual takeover are working correctly.
- The workspace has been sourced: `source ~/Lidar_nav2_ws/install/setup.bash`.

Real-robot mapping:

```bash
cd ~/Lidar_nav2_ws/scripts
./mapping_real.sh
```

Real-robot navigation:

```bash
cd ~/Lidar_nav2_ws/scripts
./nav2_real.sh
```

## 8. Switching the LIO Backend

The current scripts use FAST-LIO by default. To switch to Point-LIO, edit the launch blocks in these scripts:

```bash
vim scripts/mapping_sim.sh
vim scripts/nav2_sim.sh
vim scripts/mapping_real.sh
vim scripts/nav2_real.sh
```

Backend differences:

| Item | FAST-LIO | Point-LIO |
| --- | --- | --- |
| Simulation LiDAR input | Directly uses PointCloud2 | Requires `ign_sim_pointcloud_tool` |
| Real-robot LiDAR driver | `fast_lio_msg_MID360_launch.py` | `point_lio_msg_MID360_launch.py` |
| Interface launch file | `fastlio_lio_interface_launch.py` | `pointlio_lio_interface_launch.py` |
| Point-LIO configuration | Not used | `mid360_sim.yaml` / `mid360_real.yaml` |

Do not mix the `lidar_type` configurations of FAST-LIO and Point-LIO. Their enum definitions are different.

## 9. Troubleshooting

### ROS 2 Package Not Found

```text
Package 'xxx' not found
```

Solution:

```bash
cd ~/Lidar_nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep xxx
```

If it still cannot be found:

```bash
cd ~/Lidar_nav2_ws/scripts
./build.sh
source ../install/setup.bash
```

### Stale Gazebo Processes

```bash
killall -9 gzserver gzclient
cd ~/Lidar_nav2_ws/scripts
./mapping_sim.sh
```

### Missing `kiss_matcher` or `robin`

```bash
cd ~/Lidar_nav2_ws/src/registration/KISS-Matcher
make deps
make cppinstall
cd ~/Lidar_nav2_ws/scripts
./build.sh
```

### Missing `small_gicp`

```bash
cd /tmp
git clone https://github.com/koide3/small_gicp.git
cd small_gicp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig

cd ~/Lidar_nav2_ws/scripts
./build.sh
```

### No Data on `/scan` or `/registered_scan`

Check topics in order:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /registered_scan
ros2 topic hz /scan
```

Common causes:

- The LiDAR driver has not started, or the network configuration is wrong.
- The LIO node is not publishing odometry.
- `lio_interface` is subscribing to the wrong LIO topic.
- `sensor_scan_generation` is not receiving point clouds or TF.
- The height-slicing parameters in `pointcloud_to_laserscan` are unsuitable.

### Nav2 Cannot Plan or the Robot Does Not Move

Check TF and velocity commands:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic hz /scan
ros2 topic echo /cmd_vel
```

Common causes:

- `map -> odom` is missing because relocalization has not succeeded.
- `odom -> base_footprint` is missing because LIO or the bridge is abnormal.
- The 2D map and PCD point cloud are not from the same mapping run.
- The RViz initial pose or goal point is outside the traversable area.
- The chassis is not consuming `/cmd_vel`, or the safety state has not been released.

## 10. Common Commands

```bash
# Build
cd ~/Lidar_nav2_ws
source /opt/ros/humble/setup.bash
cd scripts
./build.sh
source ../install/setup.bash

# Simulation
./mapping_sim.sh
./nav2_sim.sh

# Real robot
./mapping_real.sh
./nav2_real.sh

# Save maps
./save_map.sh
./save_pcd.sh

# TF tree
./show_tf_tree.sh
```
