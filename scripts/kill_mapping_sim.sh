#!/usr/bin/env bash
# ============================================================
# 关闭 mapping_sim.sh 启动的所有终端和节点
# 用法: ./scripts/kill_mapping_sim.sh
# ============================================================

echo "=== 关闭仿真建图所有节点 ==="

# ── 方式1: 关闭 gnome-terminal 窗口 ──
WINDOW_TITLES=(
    "GUI控制"
    "FAST-LIO 里程计"
    "lio_interface"
    "Gazebo 仿真"
    "sensor_scan_generation"
    "3d点云转2d"
    "slam_toolbox 建图"
    "Nav2 导航"
)

for title in "${WINDOW_TITLES[@]}"; do
    if wmctrl -l 2>/dev/null | grep -q "$title"; then
        wmctrl -c "$title" 2>/dev/null && echo "  ✓ 关闭窗口: $title" || true
    fi
done

sleep 1

# ── 方式2: 按进程名杀 ──
PROCESS_NAMES=(
    "gzserver"
    "gzclient"
    "fastlio_mapping"
    "lio_interface_node"
    "sensor_scan_generation"
    "pointcloud_to_laserscan"
    "async_slam_toolbox_node"
    "gui_teleop_node"
    "map_server"
    "planner_server"
    "controller_server"
    "bt_navigator"
    "behavior_server"
    "waypoint_follower"
    "velocity_smoother"
    "nav2_smoother_server"
    "rviz2"
    "robot_state_publisher"
    "static_transform_publisher"
)

for proc in "${PROCESS_NAMES[@]}"; do
    if pgrep -f "$proc" > /dev/null 2>&1; then
        pkill -f "$proc" 2>/dev/null && echo "  ✓ 终止进程: $proc" || true
    fi
done

# ── 方式3: 按 launch 文件名强制清理 ──
LAUNCH_PATTERNS=(
    "mapping.launch.py"
    "lio_interface_launch.py"
    "get_urdf_launch.py"
    "sensor_scan_generation_launch.py"
    "pointcloud_to_laserscan_launch.py"
    "online_async_launch.py"
    "my_nav2_launch.py"
)

for pattern in "${LAUNCH_PATTERNS[@]}"; do
    PIDS=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [ -n "$PIDS" ]; then
        echo "$PIDS" | xargs kill -9 2>/dev/null && echo "  ✓ 强制终止: $pattern" || true
    fi
done

echo ""
echo "=== 全部节点已关闭 ==="

# 确认 Gazebo 已退出
if pgrep -f "gzserver\|gzclient" > /dev/null 2>&1; then
    echo "强制清理残留 Gazebo 进程..."
    killall -9 gzserver gzclient 2>/dev/null && echo "  ✓ Gazebo 已强制关闭" || true
fi

echo ""
echo "残留 ROS 2 节点检查:"
ros2 node list 2>/dev/null || echo "  (ROS 2 未 source 或已全部清理)"
