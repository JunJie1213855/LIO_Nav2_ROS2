#!/usr/bin/env bash
# ============================================================
# Docker 统一关闭脚本 — 关闭容器内所有 ROS 节点 / tmux 会话，
# 并可选停止或删除项目容器（宿主机上执行）
# 用法:
#   ./scripts/docker_shutdown.sh            # 只清理容器内节点，容器保持运行
#   ./scripts/docker_shutdown.sh --stop     # 清理节点后停止容器
#   ./scripts/docker_shutdown.sh --rm       # 清理节点后删除容器
#   ./scripts/docker_shutdown.sh --dry-run  # 只列出，不实际操作
# ============================================================

set -euo pipefail

# 本项目相关的容器名
CONTAINERS=(lio_nav2 lio_nav2_build)
# 本项目可能存在的 tmux 会话（mapping_sim_docker.sh 等创建）
TMUX_SESSIONS=(mapping_sim nav2_sim mapping_real nav2_real)

DRY_RUN=false
STOP=false
REMOVE=false

for arg in "$@"; do
    case $arg in
        --dry-run) DRY_RUN=true ;;
        --stop)    STOP=true ;;
        --rm)      REMOVE=true; STOP=true ;;
        -h|--help)
            echo "用法: $0 [--dry-run] [--stop] [--rm]"
            echo ""
            echo "  关闭 Docker 容器内仿真/建图/导航的所有节点"
            echo ""
            echo "  选项:"
            echo "    --dry-run  只列出将要执行的操作，不实际执行"
            echo "    --stop     清理节点后停止容器 (docker stop)"
            echo "    --rm       清理节点后删除容器 (docker rm -f，隐含 --stop)"
            echo ""
            echo "  示例:"
            echo "    $0            # 杀掉容器内所有节点，容器保留（下次启动快）"
            echo "    $0 --stop     # 今天不用了，停掉容器"
            echo "    $0 --rm       # 彻底删除容器（镜像和工作空间不受影响）"
            exit 0
            ;;
    esac
done

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║        Docker 统一关闭脚本 — 容器内节点 / 容器本体           ║"
echo "╚══════════════════════════════════════════════════════════════╝"
[ "$DRY_RUN" = true ] && echo "  [DRY-RUN 模式，不执行实际操作]"
echo ""

for c in "${CONTAINERS[@]}"; do
    # 容器不存在则跳过
    if ! docker inspect "$c" &>/dev/null; then
        continue
    fi

    RUNNING=$(docker inspect -f '{{.State.Running}}' "$c" 2>/dev/null || echo false)
    echo "══ 容器: $c (running=$RUNNING) ══"

    if [ "$RUNNING" = "true" ]; then
        # ── 阶段 1: 关闭 tmux 会话（等价于关闭 gnome-terminal 窗口）──
        echo "── 阶段 1: 关闭 tmux 会话 ──"
        OPEN_SESSIONS=$(docker exec "$c" tmux ls 2>/dev/null | cut -d: -f1 || true)
        if [ -n "$OPEN_SESSIONS" ]; then
            for s in "${TMUX_SESSIONS[@]}"; do
                if echo "$OPEN_SESSIONS" | grep -qx "$s"; then
                    if [ "$DRY_RUN" = true ]; then
                        echo "  [DRY-RUN] 将关闭 tmux 会话: $s"
                    else
                        docker exec "$c" tmux kill-session -t "$s" 2>/dev/null \
                            && echo "  ✓ 关闭 tmux 会话: $s" || true
                    fi
                fi
            done
        else
            echo "  (无 tmux 会话)"
        fi

        # ── 阶段 2: 复用容器内 kill_all.sh 清理 ROS 节点 ──
        echo "── 阶段 2: 清理容器内 ROS 节点 (kill_all.sh -f) ──"
        KILL_ARGS="-f"
        [ "$DRY_RUN" = true ] && KILL_ARGS="-f --dry-run"
        if docker exec "$c" test -x /ws/scripts/kill_all.sh 2>/dev/null; then
            docker exec "$c" bash -c \
                "source /opt/ros/humble/setup.bash 2>/dev/null; /ws/scripts/kill_all.sh $KILL_ARGS" \
                2>/dev/null | sed 's/^/  /' || true
        else
            # 兜底：工作空间未挂载到 /ws 时直接杀常见进程
            if [ "$DRY_RUN" = true ]; then
                echo "  [DRY-RUN] 将执行 killall -9 gzserver gzclient rviz2 ..."
            else
                docker exec "$c" bash -c \
                    "killall -9 gzserver gzclient rviz2 gui_teleop_node fastlio_mapping 2>/dev/null" || true
                echo "  ✓ 已按进程名清理"
            fi
        fi
    else
        echo "  (容器未运行，跳过节点清理)"
    fi

    # ── 阶段 3: 停止 / 删除容器 ──
    if [ "$REMOVE" = true ]; then
        echo "── 阶段 3: 删除容器 ──"
        if [ "$DRY_RUN" = true ]; then
            echo "  [DRY-RUN] 将执行 docker rm -f $c"
        else
            docker rm -f "$c" >/dev/null && echo "  ✓ 容器已删除: $c" || true
        fi
    elif [ "$STOP" = true ] && [ "$RUNNING" = "true" ]; then
        echo "── 阶段 3: 停止容器 ──"
        if [ "$DRY_RUN" = true ]; then
            echo "  [DRY-RUN] 将执行 docker stop $c"
        else
            docker stop -t 5 "$c" >/dev/null && echo "  ✓ 容器已停止: $c" || true
        fi
    fi
    echo ""
done

# ── 最终状态 ──
echo "──────────────────────────────────────────────────────────────"
echo "项目容器当前状态:"
FOUND=false
for c in "${CONTAINERS[@]}"; do
    if docker inspect "$c" &>/dev/null; then
        FOUND=true
        STATUS=$(docker inspect -f '{{.State.Status}}' "$c")
        echo "  - $c: $STATUS"
    fi
done
[ "$FOUND" = false ] && echo "  (无本项目容器)"
[ "$DRY_RUN" = true ] && echo "DRY-RUN 完成，以上为将要执行的操作"
exit 0
