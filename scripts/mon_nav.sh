#!/usr/bin/env bash
#
# mon_nav.sh — 监控 ros2 launch 启动的各个终端/节点的 CPU 与内存使用量
#
# 用法:
#   1. 正常启动导航脚本 (如 ./scripts/robosense_nav2_real_zlim.sh)
#   2. 然后在另一个终端运行本脚本:
#        ./scripts/mon_nav.sh
#
#   或者一步到位:
#        ./scripts/robosense_nav2_real_zlim.sh & sleep 3 && ./scripts/mon_nav.sh
#
# 显示:
#   每 2 秒刷新，按终端窗口分组显示每个节点组的 CPU% 和 RSS(MB)

set -euo pipefail

REFRESH_SEC="${1:-2}"                  # 刷新间隔（秒），默认 2

# --------------- ANSI 颜色 ---------------
RED='\033[0;31m'
GRN='\033[0;32m'
YEL='\033[1;33m'
CYA='\033[0;36m'
MAG='\033[0;35m'
RST='\033[0m'
BOLD='\033[1m'

# 终端标题 → 该终端下运行的 ROS 节点关键字（按实际命令匹配）
declare -A TERM_KEYWORDS
TERM_KEYWORDS["FAST-LIO"]="fastlio_mapping"
TERM_KEYWORDS["中间层"]="lio_interface_node|sensor_scan_generation|pointcloud_to_laserscan"
TERM_KEYWORDS["机器人描述"]="robot_state_publisher"
TERM_KEYWORDS["KISS.*GICP"]="global_kiss_matcher_relocalization"
TERM_KEYWORDS["Nav2"]="nav2"
TERM_KEYWORDS["robosense.*SDK"]="rslidar_sdk|rs_driver"
TERM_KEYWORDS["Gazebo"]="gzserver|gzclient"

# --------------- main ---------------
echo -e "${BOLD}${CYA}╔══════════════════════════════════════════════════════════════╗${RST}"
echo -e "${BOLD}${CYA}║  导航节点资源监控  (${REFRESH_SEC}s 刷新, Ctrl+C 退出)${RST}"
echo -e "${BOLD}${CYA}╚══════════════════════════════════════════════════════════════╝${RST}"

trap 'echo -e "\n${YEL}监控已停止${RST}"; rm -f /tmp/mon_nav_prev_*; exit 0' INT TERM

cpu_count=$(nproc)

while true; do
    clear

    echo -e "${BOLD}${CYA}━━━ $(date '+%H:%M:%S') ━━━ 系统总览 ━━━${RST}"

    # 系统内存总览
    mem_total=$(grep MemTotal /proc/meminfo | awk '{print $2}')
    mem_avail=$(grep MemAvailable /proc/meminfo | awk '{print $2}')
    mem_used_mb=$(( (mem_total - mem_avail) / 1024 ))
    mem_total_mb=$(( mem_total / 1024 ))
    mem_pct=$(( (mem_total - mem_avail) * 100 / mem_total ))
    load=$(uptime | awk -F'load average:' '{print $2}' | xargs)

    echo -e "  CPU 核数: ${cpu_count}   |   负载: ${load}"

    if [ "$mem_pct" -gt 90 ]; then
        echo -e "  内存: ${RED}${mem_used_mb}M / ${mem_total_mb}M (${mem_pct}%) ⚠️⚠️⚠️${RST}"
    elif [ "$mem_pct" -gt 70 ]; then
        echo -e "  内存: ${YEL}${mem_used_mb}M / ${mem_total_mb}M (${mem_pct}%) ⚠️${RST}"
    else
        echo -e "  内存: ${GRN}${mem_used_mb}M / ${mem_total_mb}M (${mem_pct}%)${RST}"
    fi

    echo ""
    echo -e "${BOLD}━━━ 按 gnome-terminal 窗口分组 ━━━${RST}"
    printf "  %-6s %-28s %7s %9s %s\n" "PID" "进程" "CPU%" "RSS(MB)" "命令"
    echo "  $(printf '─%.0s' {1..100})"

    total_cpu=0
    total_rss=0

    for term_title in "${!TERM_KEYWORDS[@]}"; do
        keyword="${TERM_KEYWORDS[$term_title]}"

        mapfile -t pids < <(pgrep -f "$keyword" 2>/dev/null || true)
        [ ${#pids[@]} -eq 0 ] && continue

        echo -e "  ${MAG}▸ ${term_title}${RST}"

        group_rss=0

        for pid in "${pids[@]}"; do
            [ ! -d "/proc/$pid" ] && continue

            stat_data=$(cat /proc/$pid/stat 2>/dev/null || true)
            [ -z "$stat_data" ] && continue

            rss_kb=$(awk '/VmRSS:/{print $2}' /proc/$pid/status 2>/dev/null || echo 0)
            rss_mb=$(( rss_kb / 1024 ))
            comm=$(cat /proc/$pid/comm 2>/dev/null || echo "?")
            cmdline=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | sed 's/.*install\/[^/]*\/lib\///' | cut -c1-55 || echo "?")

            # CPU% 计算
            utime=$(echo "$stat_data" | awk '{print $14}')
            stime=$(echo "$stat_data" | awk '{print $15}')
            cpu_pct="—"
            if [ -f "/tmp/mon_nav_prev_${pid}" ]; then
                read -r prev_utime prev_stime prev_total < "/tmp/mon_nav_prev_${pid}"
                delta_proc=$(( utime + stime - prev_utime - prev_stime ))
                delta_total=$(( $(awk 'NR==1{s=$1+$2+$3+$4+$5+$6+$7+$8; print s}' /proc/stat 2>/dev/null || echo 1) - prev_total ))
                if [ "$delta_total" -gt 0 ]; then
                    cpu_pct=$(awk "BEGIN {printf \"%.1f\", ${delta_proc}*100.0/${delta_total}*${cpu_count}}")
                fi
            fi
            total_now=$(awk 'NR==1{s=$1+$2+$3+$4+$5+$6+$7+$8; print s}' /proc/stat 2>/dev/null || echo 0)
            echo "$utime $stime $total_now" > "/tmp/mon_nav_prev_${pid}"

            # 颜色：高内存/CPU 标红
            if [ "$rss_mb" -gt 500 ]; then
                color_rss="${RED}${rss_mb}M${RST}"
            elif [ "$rss_mb" -gt 200 ]; then
                color_rss="${YEL}${rss_mb}M${RST}"
            else
                color_rss="${rss_mb}M"
            fi

            printf "  %-6s %-28s ${GRN}%6s${RST} %9s  %s\n" \
                "$pid" "${comm:0:28}" "${cpu_pct}%%" "$color_rss" "$cmdline"

            group_rss=$(( group_rss + rss_mb ))
        done

        if [ ${#pids[@]} -gt 1 ]; then
            echo -e "  ${CYA}      └─ 该组合计 RSS: ${group_rss} MB${RST}"
        fi
        total_rss=$(( total_rss + group_rss ))
    done

    # ---- 未被关键字覆盖的其他 ROS 进程 ----
    mapfile -t all_ros < <(pgrep -f "install/.*/lib/" 2>/dev/null || true)
    extra_pids=()
    for pid in "${all_ros[@]}"; do
        found=0
        for keyword in "${TERM_KEYWORDS[@]}"; do
            if cat /proc/$pid/cmdline 2>/dev/null | tr '\0' ' ' | grep -qE "$keyword"; then
                found=1; break
            fi
        done
        [ "$found" -eq 0 ] && extra_pids+=("$pid")
    done

    if [ ${#extra_pids[@]} -gt 0 ]; then
        echo ""
        echo -e "  ${YEL}▸ 其他 ROS 进程${RST}"
        for pid in "${extra_pids[@]}"; do
            [ ! -d "/proc/$pid" ] && continue
            rss_kb=$(awk '/VmRSS:/{print $2}' /proc/$pid/status 2>/dev/null || echo 0)
            rss_mb=$(( rss_kb / 1024 ))
            comm=$(cat /proc/$pid/comm 2>/dev/null || echo "?")
            cmdline=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | sed 's/.*install\/[^/]*\/lib\///' | cut -c1-55 || echo "?")
            printf "  %-6s %-28s %7s %8dM  %s\n" "$pid" "${comm:0:28}" "—" "$rss_mb" "$cmdline"
        done
    fi

    echo ""
    echo -e "${BOLD}━━━ 总计 ━━━${RST}"
    echo -e "  所有监控进程 RSS 合计: ${BOLD}${total_rss} MB${RST}"
    echo ""
    echo -e "${CYA}  按 Ctrl+C 退出${RST}"

    sleep "$REFRESH_SEC"
done
