#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

ros2 run nav2_map_server map_saver_cli -f "$WORKSPACE_ROOT/src/me_nav2_bringup/map/test_map__2"
