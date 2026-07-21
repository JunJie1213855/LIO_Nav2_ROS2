#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# USE_SYSTEM_TBB=ON: kiss_matcher_ros 会 add_subdirectory 编译 KISS-Matcher 核心，
# 该核心默认 FetchContent 从 GitHub 拉 oneTBB；make cppinstall 已把 oneTBB 装到系统，
# 打开此开关避免编译期联网（网络不通时 GnuTLS/SSL 报错）。
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DUSE_SYSTEM_TBB=ON
