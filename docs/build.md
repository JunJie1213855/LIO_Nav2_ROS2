# 仿真导航全链路搭建指南

从零开始在 Ubuntu 22.04 上搭建完整的 3D LiDAR 仿真导航环境。

## 1. 环境准备

### 1.1 系统要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 (Jammy) |
| ROS 2 | Humble Hawksbill |
| 仿真引擎 | Gazebo Fortress |
| CMake | ≥ 3.10（推荐 3.22+） |
| C++ 标准 | C++14 / C++17 |
| Python | 3.10（系统自带） |

### 1.2 安装 ROS 2 Humble

```bash
# 添加 ROS 2 源
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs
```

### 1.3 安装系统依赖

```bash
sudo apt install -y \
  libpcl-dev \
  libeigen3-dev \
  libboost-all-dev \
  libtbb-dev \
  libopencv-dev \
  python3-tk \
  python3-pip \
  tmux \
  wmctrl
```

### 1.4 安装 ROS 2 功能包依赖

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-tools \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-gtsam \
  ros-humble-sophus \
  ros-humble-rmw-cyclonedds-cpp
```

---

## 2. 克隆工作空间

```bash
mkdir -p ~/rosws && cd ~/rosws
git clone https://github.com/your-org/3d_nav_ws.git
cd 3d_nav_ws
```

### 2.1 初始化子模块

```bash
cd src/localization/FAST_LIO_ROBOAIRY
git submodule update --init --recursive
cd ~/rosws/3d_nav_ws
```

> FAST_LIO_ROBOAIRY 内置了 ikd-Tree 子模块，需要初始化。

---

## 3. 安装外部依赖

### 3.1 small_gicp（所有重定位包依赖）

```bash
cd ~/rosws
git clone https://github.com/koide3/small_gicp.git
cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### 3.2 KISS-Matcher + ROBIN（全局重定位依赖）

```bash
cd ~/rosws/3d_nav_ws/src/registration/KISS-Matcher
mkdir -p cpp/kiss_matcher/build && cd cpp/kiss_matcher/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_SYSTEM_ROBIN=OFF
make -j$(nproc)
sudo make install
sudo cmake --install _deps/robin-build
```

### 3.3 Livox-SDK2（实机 MID-360 驱动依赖）

已预编译在 `src/livox_ros_driver2/3rdparty/`，仿真模式无需额外操作。实机需要：

```bash
cd ~/rosws/3d_nav_ws/src/livox_ros_driver2/Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

---

## 4. 构建工作空间

### 4.1 配置环境

```bash
# 务必先退出 conda，否则库冲突会导致链接错误
conda deactivate 2>/dev/null || true

source /opt/ros/humble/setup.bash
cd ~/rosws/3d_nav_ws
```

### 4.2 完整构建

```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

> `CMAKE_POLICY_VERSION_MINIMUM=3.5` 是 CMake 4.x 兼容性所必需的。

### 4.3 增量构建

```bash
# 修改源码后重新构建
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5

# 只构建单个包（更快）
colcon build --symlink-install --packages-select fast_lio_robosense \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5

# 清理重建
rm -rf build/ install/ log/
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

---

## 5. 构建后验证

```bash
source install/setup.bash

# 确认关键包可发现
ros2 pkg list | grep -E "fast_lio_robosense|lio_interface|sensor_scan_generation|nav2_planner|global_relocalization"
# 预期输出:
#   fast_lio_robosense
#   global_relocalization_kiss_matcher
#   lio_interface
#   nav2_planner
#   sensor_scan_generation

# 确认可执行文件存在
ros2 pkg executables fast_lio_robosense        # 预期: fastlio_mapping
ros2 pkg executables lio_interface             # 预期: lio_interface_node
ros2 pkg executables global_relocalization_kiss_matcher  # 预期: global_kiss_matcher_relocalization_exec
```

---

## 6. 首次配置

### 6.1 确认仿真世界文件

```bash
ls src/get_urdf/worlds/
# 应包含 test_world.world (或其他 .world 文件)
```

如需切换仿真地图，编辑 `src/get_urdf/launch/get_urdf_launch.py` 第 14 行：

```python
world_file_path = os.path.join(pkg_share_path, 'worlds', '你的地图.world')
```

修改后重新部署：

```bash
colcon build --symlink-install --packages-select get_urdf
source install/setup.bash
```

### 6.2 建图模式（无需预配置）

建图模式不需要任何预配置文件——SLAM Toolbox 在线构建地图，直接启动即可。

### 6.3 导航模式（需要预存地图和 PCD）

导航模式需要已完成建图并保存了以下文件：

```bash
ls src/nav2_planner/map/*.yaml   # 2D 占用栅格地图
ls src/nav2_planner/pcd/*.pcd    # 3D 点云地图
```

确认启动文件中的路径正确：
- **2D 地图**: `src/nav2_planner/launch/my_nav2_launch.py` → `map_yaml_file`
- **3D 点云**: `src/registration/global_relocalization_kiss_matcher/launch/global_kiss_matcher_relocalization_launch.py` → `prior_pcd_file`

---

## 7. 启动仿真

### 7.1 仿真建图

```bash
source install/setup.bash
./scripts/mapping_sim_tmux.sh
```

tmux 窗口布局：

| 窗口 | Pane | 节点 |
|------|------|------|
| `core` | GUI控制 | `gui_teleop` (WASD 驾驶) |
| `core` | FAST-LIO | LiDAR-IMU 里程计 |
| `core` | lio_interface | TF 桥接 |
| `core` | Gazebo | 仿真环境 + 机器人模型 |
| `nav` | sensor_scan | 点云组装 + 里程计发布 |
| `nav` | 3d→2d | pointcloud_to_laserscan |
| `nav` | slam_toolbox | 在线 2D SLAM 建图 |
| `nav` | Nav2 | 导航栈加载（不发送目标） |

建图完成后保存：

```bash
./scripts/save_map.sh    # → src/nav2_planner/map/
./scripts/save_pcd.sh    # → 手动 cp 到 src/nav2_planner/pcd/
```

关闭：

```bash
./scripts/kill_mapping_sim.sh
```

### 7.2 仿真导航

```bash
source install/setup.bash
./scripts/nav2_sim_tmux.sh
```

tmux 窗口布局（与建图模式差异）：

| 窗口 | 节点 | 说明 |
|------|------|------|
| `nav` | KISS+GICP | 替代 slam_toolbox，负责 map→odom TF |
| `nav` | Nav2 | 全功能导航（map_server 加载静态地图） |

在 RViz 中点击 **"Nav2 Goal"** 发送导航目标，或用命令：

```bash
ros2 run nav2_planner send_goal.py --ros-args -p x:=3.0 -p y:=-1.0 -p yaw:=0.0
```

---

## 8. CycloneDDS 性能优化

工作空间自带 `config/cyclonedds.xml`，启动脚本已自动加载 `CYCLONEDDS_URI`。

核心优化：

| 参数 | 默认值 | 优化值 | 效果 |
|------|--------|--------|------|
| `MaxMessageSize` | 64 KiB | 2 MiB | PointCloud2 不再分片 |
| `WhcHigh` | 自动 | 512 KiB | 发送窗口扩容 |

手动启动单个节点时需加环境变量：

```bash
CYCLONEDDS_URI=file:///home/ros/rosws/3d_nav_ws/config/cyclonedds.xml \
  ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py
```

---

## 9. 常见构建问题

### conda 库冲突

**症状**: `libpython3.10.so` 或 `libz.so` 链接错误。

```bash
conda deactivate
source /opt/ros/humble/setup.bash
# 重新构建
```

### CMake 4.x 兼容性

**症状**: `Compatibility with CMake < 3.5 has been removed`

已在构建命令中加 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`。

### KISS-Matcher -fPIC 链接错误

```bash
cd src/registration/KISS-Matcher
rm -rf cpp/kiss_matcher/build
mkdir -p cpp/kiss_matcher/build && cd cpp/kiss_matcher/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_SYSTEM_ROBIN=OFF
make -j$(nproc)
sudo make install
sudo cmake --install _deps/robin-build
```

### rtabmap_sync 警告（无害）

```
rtabmap_sync/local_setup.bash: not found
```

`rtabmap_sync` 是 CMake-only 包，不需要处理。

---

## 10. 快速命令速查

```bash
# 初始化
source /opt/ros/humble/setup.bash
conda deactivate

# 构建
cd ~/rosws/3d_nav_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source install/setup.bash

# 仿真建图
./scripts/mapping_sim_tmux.sh
./scripts/save_map.sh
./scripts/save_pcd.sh
./scripts/kill_mapping_sim.sh

# 仿真导航
./scripts/nav2_sim_tmux.sh
ros2 run nav2_planner send_goal.py --ros-args -p x:=3.0 -p y:=-1.0 -p yaw:=0.0

# 检查系统状态
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /scan
ros2 topic list | grep -E "cloud|odom|scan|map"
```
