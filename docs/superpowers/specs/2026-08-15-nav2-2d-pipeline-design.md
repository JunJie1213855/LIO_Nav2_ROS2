# 2D 单线 LiDAR 导航 Pipeline 设计（2026-08-15）

## 目标

在现有 LIO_Nav2_ROS2 项目下新增一套 2D 单线 LiDAR 仿真导航 pipeline：
Gazebo 中新建标准 2D 差分小车（单线 LiDAR 输出 LaserScan + IMU），
配合 Cartographer 2D SLAM 在线建图定位 + Nav2 在线导航，一键启动，RViz 点 2D Goal Pose 导航。

## 背景

- 现有 pipeline 均为 3D：simple_car（50 线 MID360，输出 PointCloud2）+ FAST-LIO + SCAN-Planner/TARE。
- 项目已有 2D 软件栈：`nav2_planner` 含 Cartographer 配置、Nav2 配置、pointcloud_to_laserscan 桥。
- 现有 cartographer_simple.lua 是"无外部里程计"模式（Cartographer 自发布 odom TF），
  与差分小车自带 diff_drive 里程计的场景不匹配。
- 全项目无任何 LaserScan 输出的单线 LiDAR URDF（已确认）。

## 架构

```
Gazebo (indoor_2d.world 封闭室内场景)
   └─ 2D 差分小车 (diff_robot_2d.urdf 新建)
         ├─ libgazebo_ros_diff_drive.so → /odom + odom→base_footprint TF
         ├─ 单线 LiDAR (ray, vertical=1) → /scan (LaserScan)
         └─ IMU → /imu
              └─> Cartographer 2D SLAM (在线)
                    ├─ 订阅 /scan /imu /odom
                    ├─ 发布 /map + map→odom TF
                    └─> Nav2 (在线导航，跳过 map_server)
                          ├─ global_costmap ← /map
                          ├─ local_costmap  ← /scan
                          ├─ NavFn 全局规划 + DWB 局部规划
                          └─> /cmd_vel → diff_drive 插件
```

## 文件清单

### 新建

| 文件 | 内容 |
|------|------|
| `src/get_urdf/model/diff_robot_2d.urdf` | 2D 差分小车（2 驱动轮 + 2 万向轮 + 单线 LiDAR + IMU + diff_drive 插件） |
| `src/get_urdf/worlds/indoor_2d.world` | 封闭室内场景（四面墙 + 内部障碍物，为 2D SLAM 提供特征） |
| `src/planner/nav2_planner/config/cartographer_2d.lua` | Cartographer 2D 配置（use_odometry=true + use_imu_data=true） |
| `src/planner/nav2_planner/launch/cartographer_2d_launch.py` | cartographer_node + occupancy_grid_node |
| `src/planner/nav2_planner/launch/nav2_online_launch.py` | Nav2 在线导航（无 map_server/AMCL，复用 nav2_params.yaml） |
| `scripts/nav2_2d_sim.sh` | 一键启动（tmux：Gazebo / Carto / Nav2 / RViz） |
| `docs/nav2_2d_run.md` | 构建/运行/问题文档 |

### 修改

| 文件 | 内容 |
|------|------|
| `src/get_urdf/launch/get_urdf_launch.py` | 加 `robot` 参数选择 URDF（默认 simple_car，不影响 3D 版） |

## 关键技术决策

1. **单线 LiDAR**：水平 360°、720 samples、10Hz、range 0.05~12m、安装于 base_link 上方 0.2m。
   Gazebo ray 传感器 `vertical samples=1` + `<output_type>sensor_msgs/LaserScan</output_type>`。
2. **Cartographer 用真实里程计**：`use_odometry=true`、`provide_odom_frame=false`（diff_drive 发布 odom→base_footprint TF）。
   与现有 cartographer_simple.lua（use_odometry=false）分开成独立文件，不破坏原配置。
3. **Nav2 在线导航**：跳过 map_server/AMCL，costmap 直接订阅 Cartographer 的 /map。
   控制器沿用 nav2_params.yaml 的 NavFn + DWB。
4. **新建 indoor_2d.world**：验证发现 test_world.world 稀疏箱子场景对 2D SLAM 特征不足，
   Cartographer 定位漂移（map→odom 8s 漂移 1.65m）。改用封闭室内场景（四面墙 + 内部障碍物），定位稳定。
5. **TF 树**：map→odom（Cartographer）、odom→base_footprint（diff_drive）、base_footprint→base_link→laser（静态/URDF）。
6. **IMU**：室内场景下 use_imu_data=true 稳定（Roll≈1.8°）；空旷场景需关闭（use_imu_data=false）。

## 验证结果（2026-08-15）

- ✅ 差分小车正常驱动（轮子需嵌入地面约 6mm 才能产生接触力）
- ✅ 单线 LiDAR 输出 /scan（720 samples, 360°, LaserScan）
- ✅ Cartographer 建图 + map→odom TF 稳定（室内场景 < 5mm/10s）
- ✅ Nav2 在线导航成功到达目标（误差 < 0.3m）
- ✅ IMU 在室内场景稳定（test_world.world 空旷场景会导致姿态发散）
