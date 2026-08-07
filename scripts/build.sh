#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

colcon build --symlink-install \
             --cmake-args \
             -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_POLICY_VERSION_MINIMUM=3.5
