#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

MAP_PATH="${1:-$WORKSPACE_ROOT/src/localization/FAST_LIO_ROBOAIRY/PCD/robo_map.pcd}"
ros2 param set /fastlio_mapping map_file_path "$MAP_PATH"
ros2 service call /map_save std_srvs/srv/Trigger
