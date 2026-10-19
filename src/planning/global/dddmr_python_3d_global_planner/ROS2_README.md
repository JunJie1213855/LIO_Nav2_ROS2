# DDDMR ROS2 包装节点

包含最新地图路径、网页/命令行使用方式、楼梯断连排查记录和 ROS2 演示参数的完整说明见
[docs/dddmr_global_nav.md](docs/dddmr_global_nav.md)。

ROS2 Humble：接收完整的静态全局点云地图，调用现有 `dddmr_py`，输出三维路径。
地图坐标系必须 z 轴向上。输入是完整地图，不是逐帧激光扫描；节点不负责建图或累积扫描。

```text
dddmr_python_3d_global_planner/   # ROS 包名：dddmr_ros2
  package.xml / setup.py / setup.cfg
  dddmr_ros2/                   # 节点和消息转换
  dddmr_py/dddmr_py/            # 独立 Python 规划核心
  launch/planner.launch.py
  config/planner.yaml
  resource/dddmr_ros2
  test/                        # 消息转换测试和话题联调
```

## 安装与构建

```bash
source /opt/ros/humble/setup.bash
cd /home/ros/ros_ws/pct_planner
colcon build --packages-select dddmr_ros2 --symlink-install
source install/setup.bash
ros2 launch dddmr_ros2 planner.launch.py
```

本目录本身就是 `ament_python` 功能包（ROS 包名 `dddmr_ros2`），构建时同时安装
`dddmr_ros2` 包装节点和 `dddmr_py` 规划核心，无需单独 pip 安装规划核心。
基础 ROS 依赖：rclpy、sensor_msgs_py、geometry_msgs、nav_msgs、tf2_ros、launch_ros。

修改地图话题：

```bash
ros2 launch dddmr_ros2 planner.launch.py map_topic:=/cloud_map path_topic:=/init_path
```

直接运行并设置起点、里程计与静态地图 QoS：

```bash
ros2 run dddmr_ros2 planner_node --ros-args \
  -p map_topic:=/cloud_map \
  -p start:='[0.0, 0.0, 0.0]' \
  -p odom_topic:=/Odometry \
  -p map_transient_local:=true
```

## 话题接口

| 方向 | 默认话题 | 类型 | 用途 |
|---|---|---|---|
| 输入 | `/global_points` | `sensor_msgs/msg/PointCloud2` | 含 x/y/z 的完整全局地图 |
| 输入 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz2 的 2D Goal Pose 或程序发布的三维目标 |
| 输入 | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz2 的 2D Pose Estimate，更新起点并重规划 |
| 输入 | 可选 `odom_topic` | `nav_msgs/msg/Odometry` | 更新下一次规划使用的起点，不按里程计频率重规划 |
| 输出 | `/init_path` | `nav_msgs/msg/Path` | 每个 PoseStamped 都带地图 frame_id、时间戳、三维位置和地面法向量对齐的四元数 |

没有起点消息时使用 `start` 参数（三维，位于地图坐标系）。
`/initialpose` 和里程计按最后收到的消息更新起点。
目标早于地图到达时会排队，建图完成后自动规划。
不同 frame 的起终点通过最新 TF 转到地图 frame；TF 缺失则输出空路径并报错，TF 就绪后重新发送目标。
输入 Pose 的姿态不约束终点航向，路径姿态由 DDDMR 计算。

发送三维目标示例（坐标需改成实际地图可通行位置）：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 12.0, y: 3.0, z: 3.0}, orientation: {w: 1.0}}}"
```

在 RViz2 中添加 Path，话题选 `/init_path`，Fixed Frame 使用地图 `header.frame_id`。
标准 2D 工具的 z 通常为 0；跨楼层目标请通过话题发布正确的 z。

## 地图更新、QoS 与失败行为

- 默认 `update_map: false`：只处理首张成功接收的地图，重启节点后可载入另一张。建图失败允许重收。
- `update_map: true`：每条地图都会触发重建，并对已有目标重规划。高频地图会使结果持续失效，建议上游低频发布完整地图。
- 地图订阅默认 Best Effort + Volatile，可接收常见可靠或尽力发送的数据流。
- 一次性、保留历史地图的发布器需要自身使用 Transient Local，并设置本节点 `map_transient_local: true`；Volatile 发布器不能匹配该模式。
- 路径输出为 Reliable + Transient Local（depth=1），晚加入的匹配订阅者可以收到最后结果。
- 新地图、新目标、新显式起点会清空旧路径；不可达或出错同样输出空 Path，避免沿用旧路径。
- 建图和搜索在单个后台 worker 执行，ROS 回调持续接收消息。新地图/目标使旧规划结果失效。
- `planner.*` 参数对应 `PlannerConfig` 字段，在启动时读取；话题点云直接建图，不使用 PCD 文件缓存。

完整默认配置见 `config/planner.yaml`。自定义参数文件：

```bash
ros2 launch dddmr_ros2 planner.launch.py params_file:=/absolute/path/planner.yaml
```

## 测试

```bash
source /opt/ros/humble/setup.bash
cd /home/ros/ros_ws/pct_planner
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD/src/dddmr_python_3d_global_planner:$PWD/src/dddmr_python_3d_global_planner/dddmr_py:$PYTHONPATH" \
  python3 -m pytest src/dddmr_python_3d_global_planner/test -q
```

构建并 source 工作空间后，可用独立 ROS_DOMAIN_ID 运行真实话题自检（合成平面地图，
测试目标先到、地图建图、路径输出及不可达目标清空路径）：

```bash
ROS_DOMAIN_ID=87 python3 src/dddmr_python_3d_global_planner/test/smoke_topics.py
```
