#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

source install/setup.bash

MAP_PATH="${1:-$WORKSPACE_ROOT/src/planner/nav2_planner_bringup/map/test_map__2}"

ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH"

# 同步到 install 目录，供 launch 文件引用（原理同 save_pcd.sh）
BASE=$(basename "$MAP_PATH")
for f in "$MAP_PATH" "$MAP_PATH.yaml" "$MAP_PATH.pgm"; do
    [ -f "$f" ] || continue
    INSTALL_DIR="$WORKSPACE_ROOT/install/nav2_planner_bringup/share/nav2_planner_bringup/map"
    mkdir -p "$INSTALL_DIR"
    ln -sf "$f" "$INSTALL_DIR/$(basename "$f")"
done
echo "地图已保存到: $MAP_PATH"
