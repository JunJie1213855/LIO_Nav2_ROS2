#!/bin/bash
# ===========================================================================
# 离线安装所有依赖到系统（/usr/local）
#
# 使用方法:
#   cd src/registration && bash install_deps.sh
#
# 用本地 tarball 编译安装 Eigen3 / TBB / ROBIN / small_gicp
# 安装后 cmake find_package() 可直接找到，无需 FetchContent
# ===========================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
KISS_3RD="$SCRIPT_DIR/KISS-Matcher/cpp/kiss_matcher/3rdparty"
KISS_MATCHER_DIR="$SCRIPT_DIR/KISS-Matcher/cpp/kiss_matcher"
DEPS_DIR="$SCRIPT_DIR/offline_deps"
BUILD_DIR="/tmp/reg_deps_build"
JOBS=$(nproc)

echo "============================================"
echo " 离线安装 registration 依赖"
echo " 目标: /usr/local"
echo "============================================"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

# 清除从本机带过来的编译缓存（路径不对会导致 cmake 报错）
rm -rf "$KISS_MATCHER_DIR/build" "$KISS_MATCHER_DIR/build_fpic"

# ---- Eigen 3.4.0 (header-only) ----
echo ""
echo ">>> [1/5] Eigen 3.4.0"
tar xzf "$KISS_3RD/eigen/eigen-3.4.0.tar.gz" -C "$BUILD_DIR"
cd "$BUILD_DIR/eigen-3.4.0"
patch -p1 < "$KISS_3RD/eigen/eigen.patch" || true
cmake -B build . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DEIGEN_BUILD_DOC=OFF \
    -DEIGEN_BUILD_TESTING=OFF
sudo cmake --install build

# ---- oneTBB v2022.0.0 ----
echo ""
echo ">>> [2/5] oneTBB v2022.0.0"
tar xzf "$KISS_3RD/tbb/tbb-v2022.0.0.tar.gz" -C "$BUILD_DIR"
cd "$BUILD_DIR/oneTBB-2022.0.0"
cmake -B build . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DTBB_TEST=OFF \
    -DTBB_EXAMPLES=OFF \
    -DTBB_STRICT=OFF
cmake --build build -j$JOBS
sudo cmake --install build

# ---- ROBIN v1.2.7 ----
echo ""
echo ">>> [3/5] ROBIN v1.2.7"
tar xzf "$KISS_3RD/robin/robin-v1.2.7.tar.gz" -C "$BUILD_DIR"
cd "$BUILD_DIR/ROBIN-v.1.2.7"
cmake -B build . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DROBIN_OFFLINE_DEPS=$DEPS_DIR
cmake --build build -j$JOBS
sudo cmake --install build

# ---- KISS-Matcher core ----
echo ""
echo ">>> [4/5] KISS-Matcher core"
cmake -B "$KISS_MATCHER_DIR/build" "$KISS_MATCHER_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build "$KISS_MATCHER_DIR/build" -j$JOBS
sudo cmake --install "$KISS_MATCHER_DIR/build"

# ---- small_gicp ----
echo ""
echo ">>> [5/5] small_gicp"
tar xzf "$DEPS_DIR/small_gicp-master.tar.gz" -C "$BUILD_DIR"
cd "$BUILD_DIR/small_gicp-master"
cmake -B build . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j$JOBS
sudo cmake --install build

# 清理
rm -rf "$BUILD_DIR"

echo ""
echo "============================================"
echo " 全部依赖已安装到 /usr/local"
echo " Eigen3 / TBB / ROBIN / KISS-Matcher / small_gicp"
echo "============================================"
