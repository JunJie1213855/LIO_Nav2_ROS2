# 双目 + IMU VIO 定位验证 Pipeline 设计（2026-08-15）

## 目标

在现有项目下新增一套双目相机 + IMU 的 VIO 定位验证仿真：
Gazebo 双目小车输出左右图像 + IMU，VINS-Fusion（ROS2 移植版 VINS_ROS2）做视觉惯性里程计，
在 RViz 中观察轨迹/点云/特征点，验证定位效果。不接导航。

## 背景

- 项目已有 2D 差分小车 `diff_robot_2d.urdf`（底盘 + 单线 LiDAR + IMU + diff_drive）可复用底盘结构。
- `vins_run` 容器有已编译好的 VINS_ROS2（VINS-Fusion ROS2 移植），源码在宿主机 `/home/ros/ros_ws/VINS_ROS2`。
- VINS 有 pinhole 标定配置（vi_car：752×480，fx≈468.9）；Gazebo camera 插件也是 pinhole 模型，可精确对齐。
- 跨容器通信：lio_nav2 是 host 网络，vins_run 是 bridge 网络 → 需把 vins_run 重建为 host 网络。

## 架构（跨容器）

```
[lio_nav2 容器 — host 网络]
   ├─ Gazebo 室内场景 + 双目小车 (diff_robot_stereo.urdf)
   ├─ 左/右相机 → /cam0/image_raw /cam1/image_raw + camera_info
   ├─ IMU → /imu0
   ├─ diff_drive → /odom + odom→base_footprint TF
   └─ teleop_twist_keyboard → /cmd_vel（键盘遥控）
              │ DDS 跨容器（CycloneDDS，同 ROS_DOMAIN_ID，use_sim_time）
              ▼
[vins_run 容器 — 重建为 host 网络]
   ├─ VINS_ROS2 vins_node（已编译）
   ├─ 订阅 /cam0 /cam1 /imu0 → 输出 /vins_estimator/odometry + path + point_cloud
   └─ RViz 可视化轨迹/点云/特征点
```

## 文件清单

### 新建

| 文件 | 内容 |
|------|------|
| `src/get_urdf/model/diff_robot_stereo.urdf` | 双目小车（复用差分底盘 + 两个相机 + IMU） |
| `src/get_urdf/worlds/`（复用 indoor_2d.world） | 室内场景（有 Brick/Wood 纹理，提供视觉特征） |
| VINS 标定（宿主机 VINS_ROS2 config） | 新建 gazebo 标定：fx=fy=468.94，cx=376，cy=240，无畸变（匹配 Gazebo） |
| VINS 主配置 | 双目 + IMU，话题 /imu0 /cam0 /cam1，外参匹配 URDF |
| `scripts/stereo_vio_sim.sh` | lio_nav2 侧一键启动（Gazebo + 遥控） |
| `scripts/vins_run.sh` | vins_run 侧一键启动（VINS + RViz） |
| `docs/stereo_vio_run.md` | 运行文档 |

## 关键技术决策

1. **相机标定对齐**：Gazebo camera 无畸变、主点在图像中心（cx=376, cy=240），
   因此 VINS 标定文件用 `fx=fy=468.94, cx=376, cy=240, 畸变=0`，与 Gazebo 完全一致。
   horizontal_fov = 2·atan(752/(2·468.94)) ≈ 1.351 rad。
2. **相机安装**：双相机并排，基线 0.1m（body y 方向），光轴朝前（body x）。
   相机→body 旋转 `R=[[0,0,1],[-1,0,0],[0,-1,0]]`，平移 cam0=(0.15, +0.05, 0.2)、cam1=(0.15, -0.05, 0.2)。
3. **IMU 话题** `/imu0`（匹配 VINS 的 vi_car 约定），Gazebo IMU 插件 remap。
4. **跨容器**：vins_run 重建为 `--network host`，与 lio_nav2 直接互通；
   两容器 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、同 `ROS_DOMAIN_ID`、`use_sim_time=true` 订阅 /clock。
5. **VIO 初始化**：需小车运动（键盘遥控），运动时 VINS 完成初始化后输出里程计。

## 验证方式

启动两个容器脚本 → 键盘遥控小车在室内场景运动 → VINS 输出轨迹/点云 → RViz 观察轨迹与真实路径一致。
