# FAST_LIO_RoboSense

ROS2 Humble version of FAST_LIO with support for RoboSense Airy LiDAR.

## Features

- ✅ Based on FAST_LIO ROS2 Humble version
- ✅ Support for RoboSense Airy LiDAR (lidar_type: 5)

## Dependencies

- ROS2 Humble
- PCL
- Eigen3
- ikd-Tree (git submodule)

## Installation

1. Initialize git submodule:
```bash
cd src/fast_lio_robosense/include
rm -rf ikd-Tree
git clone --depth 1 --branch fast_lio https://github.com/hku-mars/ikd-Tree.git
```

2. Build:
```bash
colcon build --packages-select fast_lio_robosense
source install/setup.bash
```

## Usage

### Launch RoboSense Airy LiDAR Mapping

```bash
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py
```

### Save Map

```bash
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py map_file_path:=/path/to/save/map.pcd

# In another terminal
ros2 service call /map_save std_srvs/srv/Trigger
```

### Parameter Configuration

Configuration file is located at `config/robosenseAiry.yaml`. Main parameters:

- `preprocess.lidar_type: 5` - RoboSense Airy LiDAR type
- `preprocess.scan_line: 96` - Number of scan lines
- `common.lid_topic: "/rslidar_points"` - Point cloud topic
- `common.imu_topic: "/rslidar_imu_data"` - IMU topic

## Sample Effect
<img width="1304" height="765" alt="图片" src="https://github.com/user-attachments/assets/5387aa39-e742-4ec1-b108-0adb5a838c18" />

