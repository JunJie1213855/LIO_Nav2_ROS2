#!/usr/bin/env bash
# ============================================================
# 统一关闭脚本 — 自动识别并关闭仿真/实机/建图/导航所有节点
# 用法:
#   ./scripts/kill_all.sh            # 默认
#   ./scripts/kill_all.sh -f         # 强制 (跳过 ros2 检测，直接杀进程)
#   ./scripts/kill_all.sh --dry-run  # 只列出，不实际操作
# ============================================================

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

DRY_RUN=false
FORCE=false

for arg in "$@"; do
    case $arg in
        --dry-run) DRY_RUN=true ;;
        -f|--force) FORCE=true ;;
        -h|--help)
            echo "用法: $0 [--dry-run] [-f|--force]"
            echo ""
            echo "  统一关闭 3d_nav_ws 项目中仿真/实机/建图/导航的所有节点"
            echo ""
            echo "  选项:"
            echo "    --dry-run   只列出将要关闭的进程，不实际操作"
            echo "    -f, --force 强制模式，跳过 ros2 自动检测，直接按进程名杀"
            echo ""
            echo "  示例:"
            echo "    ./scripts/kill_all.sh            # 自动检测并清理"
            echo "    ./scripts/kill_all.sh --dry-run  # 预览"
            echo "    ./scripts/kill_all.sh -f         # 强力清理"
            exit 0
            ;;
    esac
done

# 尝试 source ROS 2（非强制模式下需要 ros2 node list）
if [ "$FORCE" = false ] && [ -f "install/setup.bash" ]; then
    source install/setup.bash 2>/dev/null || true
fi

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║         统一关闭脚本 — 3d_nav_ws 所有节点                    ║"
echo "╚══════════════════════════════════════════════════════════════╝"
[ "$DRY_RUN" = true ] && echo "  [DRY-RUN 模式，不执行实际操作]"
echo ""

# ═══════════════════════════════════════════════════════════════
# 阶段 1: 关闭 gnome-terminal 窗口
# ═══════════════════════════════════════════════════════════════
echo "── 阶段 1: 关闭 gnome-terminal 窗口 ──"

ALL_WINDOW_TITLES=(
    "GUI控制"
    "FAST-LIO 里程计"
    "lio_interface"
    "Gazebo 仿真"
    "sensor_scan_generation"
    "3d点云转2d"
    "slam_toolbox 建图"
    "Nav2 导航"
    "KISS + GICP 重定位"
    "small_gicp 重定位"
    "Livox Fast-LIO 驱动"
    "Livox Point-LIO 驱动"
    "Point-LIO 里程计"
    "Point-LIO lio_interface"
    "Fast-LIO lio_interface"
    "机器人描述"
    "Airy Z轴翻转修正"
    "地面+天花板滤波"
    "点云滤波"
    "点云滤波 180°"
    "点云格式转换器"
)

if command -v wmctrl &> /dev/null; then
    OPEN_WINDOWS=$(wmctrl -l 2>/dev/null || true)
    for title in "${ALL_WINDOW_TITLES[@]}"; do
        if echo "$OPEN_WINDOWS" | grep -q "$title"; then
            if [ "$DRY_RUN" = true ]; then
                echo "  [DRY-RUN] 将关闭窗口: $title"
            else
                wmctrl -c "$title" 2>/dev/null && echo "  ✓ 关闭窗口: $title" || true
            fi
        fi
    done
else
    echo "  (wmctrl 未安装，跳过窗口关闭)"
fi

sleep 1

# ═══════════════════════════════════════════════════════════════
# 阶段 2: 通过 ros2 node list 自动发现节点并终止其进程
# ═══════════════════════════════════════════════════════════════
echo ""
echo "── 阶段 2: 终止 ROS 2 节点 ──"

if [ "$FORCE" = false ] && command -v ros2 &> /dev/null; then
    ROS_NODES=$(ros2 node list 2>/dev/null || true)
    if [ -n "$ROS_NODES" ]; then
        echo "  发现以下 ROS 2 节点:"
        echo "$ROS_NODES" | while IFS= read -r node; do
            [ -z "$node" ] && continue
            echo "    - $node"
        done

        # 获取所有 ROS 相关进程的 PID 并终止
        ROS_PIDS=$(ps aux | grep -E "ros2|/ros/" | grep -v "grep\|kill_all" | awk '{print $2}' || true)
        for pid in $ROS_PIDS; do
            if [ -n "$pid" ] && [ "$pid" != "$$" ]; then
                if [ "$DRY_RUN" = true ]; then
                    echo "  [DRY-RUN] 将终止 PID=$pid"
                else
                    kill -9 "$pid" 2>/dev/null || true
                fi
            fi
        done
        [ "$DRY_RUN" = false ] && echo "  ✓ ROS 进程已清理"
    else
        echo "  (无 ROS 2 节点运行)"
    fi
else
    echo "  (跳过 ros2 自动检测)"
fi

sleep 1

# ═══════════════════════════════════════════════════════════════
# 阶段 3: 按进程名匹配
# ═══════════════════════════════════════════════════════════════
echo ""
echo "── 阶段 3: 按进程名终止 ──"

ALL_PROCESS_NAMES=(
    # Gazebo
    "gzserver" "gzclient" "gazebo"
    # Livox 驱动
    "livox_ros_driver2"
    # LIO
    "fastlio_mapping" "fast_lio" "point_lio"
    # 接口 & 传感器
    "lio_interface_node" "sensor_scan_generation"
    "pointcloud_to_laserscan" "ign_sim_pointcloud_tool"
    # 重定位
    "global_relocalization_kiss_matcher"
    "global_relocalization" "small_gicp_relocalization"
    # SLAM
    "async_slam_toolbox_node" "slam_toolbox"
    # Nav2
    "map_server" "planner_server" "controller_server"
    "bt_navigator" "behavior_server" "waypoint_follower"
    "velocity_smoother" "nav2_smoother_server" "lifecycle_manager"
    # 描述 & 状态
    "robot_state_publisher" "static_transform_publisher"
    "joint_state_publisher"
    # GUI & 控制
    "gui_teleop_node" "teleop_twist_keyboard" "rviz2"
    # 自定义脚本
    "ground_ceiling_filter.py" "airy_unflip.py" "ground_filter.py"
)

for proc in "${ALL_PROCESS_NAMES[@]}"; do
    PIDS=$(pgrep -f "$proc" 2>/dev/null || true)
    [ -z "$PIDS" ] && continue
    while IFS= read -r pid; do
        [ -z "$pid" ] && continue
        if [ "$DRY_RUN" = true ]; then
            echo "  [DRY-RUN] 将终止: $proc (PID=$pid)"
        else
            kill -9 "$pid" 2>/dev/null && echo "  ✓ 终止: $proc (PID=$pid)" || true
        fi
    done <<< "$PIDS"
done

sleep 1

# ═══════════════════════════════════════════════════════════════
# 阶段 4: 按 launch 文件匹配
# ═══════════════════════════════════════════════════════════════
echo ""
echo "── 阶段 4: 清理 launch 进程 ──"

ALL_LAUNCH_PATTERNS=(
    "get_urdf_launch.py"
    "sensor_scan_generation_launch.py"
    "pointcloud_to_laserscan_launch.py"
    "mapping.launch.py"
    "mapping_robosense_airy"
    "point_lio.launch.py"
    "fast_lio_msg_MID360"
    "point_lio_msg_MID360"
    "lio_interface_launch.py"
    "fastlio_lio_interface"
    "pointlio_lio_interface"
    "global_kiss_matcher_relocalization"
    "global_relocalization"
    "small_gicp_relocalization"
    "online_async_launch.py"
    "my_nav2_launch.py"
    "gld_robot_description"
)

for pattern in "${ALL_LAUNCH_PATTERNS[@]}"; do
    PIDS=$(pgrep -f "$pattern" 2>/dev/null || true)
    [ -z "$PIDS" ] && continue
    while IFS= read -r pid; do
        [ -z "$pid" ] && continue
        if [ "$DRY_RUN" = true ]; then
            echo "  [DRY-RUN] 将终止 launch: $pattern (PID=$pid)"
        else
            kill -9 "$pid" 2>/dev/null && echo "  ✓ 终止 launch: $pattern (PID=$pid)" || true
        fi
    done <<< "$PIDS"
done

# ═══════════════════════════════════════════════════════════════
# 阶段 5: 强制清理 Gazebo
# ═══════════════════════════════════════════════════════════════
echo ""
echo "── 阶段 5: 强制清理 Gazebo ──"

if pgrep -f "gzserver\|gzclient" > /dev/null 2>&1; then
    if [ "$DRY_RUN" = true ]; then
        echo "  [DRY-RUN] 将执行 killall -9 gzserver gzclient"
    else
        killall -9 gzserver gzclient 2>/dev/null && echo "  ✓ Gazebo 已强制关闭" || true
    fi
else
    echo "  (Gazebo 未运行)"
fi

# ═══════════════════════════════════════════════════════════════
# 最终检查
# ═══════════════════════════════════════════════════════════════
echo ""
echo "──────────────────────────────────────────────────────────────"

if [ "$DRY_RUN" = false ]; then
    REMAINING=$(ros2 node list 2>/dev/null | wc -l || echo "0")
    if [ "$REMAINING" -eq 0 ] 2>/dev/null; then
        echo "✓ 清理完成，无残留 ROS 2 节点"
    else
        echo "残留 $REMAINING 个 ROS 2 节点:"
        ros2 node list 2>/dev/null || true
        echo ""
        echo "可运行 ./scripts/kill_all.sh -f 强制清理"
    fi
else
    echo "DRY-RUN 完成，以上为将要执行的操作"
fi
