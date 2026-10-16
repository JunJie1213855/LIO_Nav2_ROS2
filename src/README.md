# 源码目录管理

按功能域组织源码；分组目录不是 ROS 包，不放置 `package.xml`。
ROS 包名、可执行程序名和第三方算法仓库内部结构保持不变。

```text
src/
├── drivers/
│   └── livox_ros_driver2/
├── robot/
│   └── gld_robot_description/
├── simulation/
│   ├── get_urdf/
│   ├── livox_laser_simulation_RO2/
│   └── ign_sim_pointcloud_tool/
├── perception/
│   └── GroundSeg/efficient_online_segmentation/
├── localization/
│   ├── lio/
│   │   ├── FAST_LIO_ROBOAIRY/
│   │   ├── point_lio/
│   │   └── Super-LIO/
│   ├── adapters/
│   │   ├── lio_interface/
│   │   └── sensor_scan_generation/
│   └── relocalization/
│       ├── KISS-Matcher/
│       ├── global_relocalization_kiss_matcher/
│       ├── small_gicp_relocalization/
│       ├── global_small_gicp_relocalization/
│       ├── global_relocalization/
│       └── icp_registration/
├── planning/
│   ├── local/SCAN-Planner/
│   └── exploration/
│       ├── tare_planner/
│       └── far_planner/
├── bringup/
│   └── nav2_planner_bringup/
└── tools/
    ├── gui_teleop/
    └── pcd2pgm-master/
```

## 归属规则

| 目录 | 职责 |
| --- | --- |
| `drivers` | 真实硬件驱动 |
| `robot` | 实机模型、传感器安装外参 |
| `simulation` | 仿真环境、仿真传感器、仿真数据格式桥接 |
| `perception` | 地面分割等环境感知 |
| `localization/lio` | LIO 里程计后端 |
| `localization/adapters` | LIO 输出到标准里程计、坐标系和点云接口的适配 |
| `localization/relocalization` | 点云配准与全局重定位后端及其算法依赖 |
| `planning/local` | 局部规划算法 |
| `planning/exploration` | 自主探索算法 |
| `bringup` | 系统集成 launch、Nav2 和 SLAM 配置 |
| `tools` | 遥控、离线转换等辅助工具 |

新增功能包先选择功能域，不直接放在 `src/` 根目录。
多包算法仓库整体放入所属域，保留内部相对路径、许可证和说明文件。
例如 Super-LIO 的 `basic` 和 `super_lio` 两个包仍在同一算法目录内。
现有 `COLCON_IGNORE` 标记随原目录保留，备选后端的构建范围未改变。

## 迁移与构建

这轮只调整源码归属并同步路径引用；`nav2_planner_bringup` 暂不拆包，
地图、PCD 和配置随所属包一起移动。运行数据独立存放与启动入口统一留待后续调整。

旧 `build/` 中的 CMake 缓存记录了迁移前的绝对源码路径，首次重建需清理缓存：

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS="-j2" ./scripts/build.sh --cmake-clean-cache
source install/setup.bash
```

之后可继续使用 `./scripts/build.sh`。现有 `ros2 launch <包名> <launch文件>`
命令保持不变。资源文件优先通过 ROS 包索引定位，新增脚本避免硬编码源码目录。

包发现检查：

```bash
colcon list --base-paths src
```

本次迁移前后 colcon 发现的 35 个包名称和构建类型一致（包括普通 CMake 包
`kiss_matcher`），被 `COLCON_IGNORE` 排除的 SCAN 仿真包保持排除状态。
