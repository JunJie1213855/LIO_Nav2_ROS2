#!/usr/bin/env bash
# ============================================================
# 关闭 mapping_real.sh 启动的所有终端和节点
# 用法: ./scripts/kill_mapping_real.sh
# ============================================================

echo "=== 关闭 Livox MID-360 实机建图所有节点 ==="

# ── 方式1: 关闭 gnome-terminal 窗口 ──
WINDOW_TITLES=(
    "Livox Fast-LIO 驱动"
    "FAST-LIO 里程计"
    "Fast-LIO lio_interface"
    "机器人描述"
    "sensor_scan_generation"
    "3d点云转2d"
    "slam_toolbox 建图"
)

for title in "${WINDOW_TITLES[@]}"; do
    if wmctrl -l 2>/dev/null | grep -q "$title"; then
        wmctrl -c "$title" 2>/dev/null && echo "  ✓ 关闭窗口: $title" || true
    fi
done

sleep 1

# ── 方式2: 按进程名杀 ──
PROCESS_NAMES=(
    "livox_ros_driver2"
    "fastlio_mapping"
    "lio_interface_node"
    "sensor_scan_generation"
    "pointcloud_to_laserscan"
    "async_slam_toolbox_node"
    "robot_state_publisher"
    "static_transform_publisher"
    "rviz2"
)

for proc in "${PROCESS_NAMES[@]}"; do
    if pgrep -f "$proc" > /dev/null 2>&1; then
        pkill -f "$proc" 2>/dev/null && echo "  ✓ 终止进程: $proc" || true
    fi
done

# ── 方式3: 按 launch 文件名强制清理 ──
LAUNCH_PATTERNS=(
    "fast_lio_msg_MID360"
    "mapping.launch.py"
    "fastlio_lio_interface"
    "gld_robot_description"
    "sensor_scan_generation"
    "pointcloud_to_laserscan"
    "online_async_launch"
)

for pattern in "${LAUNCH_PATTERNS[@]}"; do
    PIDS=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [ -n "$PIDS" ]; then
        echo "$PIDS" | xargs kill -9 2>/dev/null && echo "  ✓ 强制终止: $pattern" || true
    fi
done

echo ""
echo "=== 全部节点已关闭 ==="

echo ""
echo "残留 ROS 2 节点检查:"
ros2 node list 2>/dev/null || echo "  (ROS 2 未 source 或已全部清理)"
