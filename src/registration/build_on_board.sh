#!/bin/bash
# ===========================================================================
# 开发板一键构建脚本（纯 colcon）
#
# 使用方法:
#   1. 将 src/registration/ 目录整体拷贝到开发板的 workspace/src/ 下
#   2. cd src/registration && bash build_on_board.sh
#
# 前提: ROS 2 Humble 已安装且 source 过
# ===========================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$SCRIPT_DIR/../.."

echo "============================================"
echo " Step 1/3: 安装系统依赖"
echo "============================================"
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    libflann-dev \
    liblz4-dev \
    libtbb-dev \
    libgtsam-dev \
    libpcl-dev \
    libomp-dev

echo "============================================"
echo " Step 2/3: 离线安装 Eigen/TBB/ROBIN/KISS-Matcher/small_gicp"
echo "============================================"
bash "$SCRIPT_DIR/install_deps.sh"

echo "============================================"
echo " Step 3/3: colcon 编译全部重定位包"
echo "============================================"
cd "$WS_ROOT"
colcon build --symlink-install \
    --packages-select kiss_matcher_ros small_gicp_relocalization global_relocalization_kiss_matcher \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local

echo ""
echo "============================================"
echo " 完成！"
echo "============================================"
echo "source install/setup.bash"
