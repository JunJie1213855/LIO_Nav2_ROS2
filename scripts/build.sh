#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

# 编译 kiss matcher
# cd $WORKSPACE_ROOT/src/registration/KISS-Matcher
# rm -rf cpp/kiss_matcher/build

# 用 -fPIC 重编（用系统 TBB，避免联网拉 oneTBB）
# cmake -B cpp/kiss_matcher/build cpp/kiss_matcher \
#  -DCMAKE_BUILD_TYPE=Release \
#  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
#  -DUSE_SYSTEM_TBB=ON
# 编译
# cmake --build cpp/kiss_matcher/build -j4
# sudo cmake --install cpp/kiss_matcher/build

# 回到工作空间
cd "$WORKSPACE_ROOT" || exit 1

MAKEFLAGS="-j4" colcon build --symlink-install \
             --executor sequential \
             --cmake-args \
             -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_POLICY_VERSION_MINIMUM=3.5
