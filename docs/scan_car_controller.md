# SCAN-Planner 差速小车闭环控制器

`closed_loop_controller_car.cpp` 是独立节点，原 `closed_loop_controller.cpp` 保留。
差速控制器对 B-spline 进行空间采样，根据实测位置选择局部最近点和前视点，
通过纯追踪曲率生成 `linear.x`、`angular.z`；`linear.y` 始终为零。

## 编译与运行

已有工作空间依赖构建完成时，编译新增消息和控制器：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
MAKEFLAGS="-j2" colcon build --packages-select scan_planner_msgs scan_planner --executor sequential
source install/setup.bash
./scripts/nav2_scan_seg.sh 0
```

该脚本已选择 `controller_type:=car`。手动使用 LIO launch 时：

```bash
ros2 launch scan_planner scan_planner_lio_launch.py \
  controller_type:=car use_sim_time:=true \
  body_pose_topic:=/odom sensor_pose_topic:=/registered_odometry \
  cloud_topic:=/no_ground_cloud cloud_is_world:=false
```

`controller_type:=legacy` 使用原控制器。launch 每次只启动一个控制器，
默认仍为 `legacy`，其他已有入口的行为保持原样。

## 输入与输出

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `planning/bspline` | `scan_planner_msgs/Bspline` | 三次 B-spline，包含轨迹 ID 和起始时间 |
| `body_pose` | `nav_msgs/Odometry` | 车体位姿，launch 中映射到 `/odom` |
| `cmd_vel` | `geometry_msgs/Twist` | 前进速度和角速度 |
| `planning/execution_progress` | `scan_planner_msgs/ExecutionProgress` | 当前轨迹的实测投影进度、到达状态和跟踪丢失状态 |
| `planning/go2_execution_frozen` | `std_msgs/Bool` | 保留原有暂停反馈接口 |

差速模式下，launch 设置 `fsm.use_controller_progress=true`。
规划器以当前轨迹 ID 对应的进度检查重规划和轨迹剩余部分；
小车转向、减速或停止时，规划器不会仅因为原定时间已到就结束轨迹。
进度反馈缺失或超时时暂停推进，跟踪偏差过大时停车并请求重规划。
原控制器仍使用原有的计时及暂停逻辑。

## 参数

参数位于 `plan_manage/config/controllers.yaml` 的 `closed_loop_controller_car` 节点下。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `max_vx` / `max_vyaw` | 0.5 m/s / 1.0 rad/s | 最大前进速度和角速度 |
| `max_accel` / `max_decel` | 0.5 / 0.8 m/s² | 正常跟踪时的加减速限制；故障停车直接输出零速 |
| `max_yaw_accel` | 2.0 rad/s² | 角速度变化率限制 |
| `max_lateral_accel` | 0.4 m/s² | 曲率相关的速度上限 |
| `lookahead_min` / `lookahead_max` | 0.25 / 0.6 m | 空间前视距离范围 |
| `lookahead_time` | 0.6 s | 前视距离随指令速度增加的系数 |
| `rotate_enter` / `rotate_exit` | 0.7 / 0.25 rad | 大角度先转向，使用两个阈值避免反复启停 |
| `finish_dist` | 0.1 m | 末端位置到达容差；不要求末端朝向 |
| `max_tracking_error` | 0.8 m | 超限停车并请求重规划 |
| `progress_search_distance` | 1.0 m | 每次沿旧进度向前投影的搜索范围，避免跳到远处路径段 |
| `odom_timeout` | 0.5 s | 里程计消息及接收超时，超时停车 |
| `sample_dt` | 0.03 s | 对 B-spline 参数时间的采样间隔 |
| `odom_frame` | `odom` | 要求的位姿坐标系，规划路径必须与其对齐 |

控制器同时按转弯曲率、角速度上限、横向加速度和终点距离降低速度。
原地转向时不前进；常量 B-spline 作为急停保持处理，不会驱车追赶保持点。

## 验证

```bash
colcon test --packages-select scan_planner --ctest-args -R 'test_car_path_tracker|test_car_controller'
colcon test-result --test-result-base build/scan_planner --verbose
```

测试包含：侧向偏差纠正、45°/90° 路径、圆弧、原地转向、常量急停轨迹、
停车时进度不前移、里程计超时和非法轨迹。运动测试采用差速运动学；
Gazebo 中的轮胎滑移、接触和定位误差仍需结合实际场景验证。
