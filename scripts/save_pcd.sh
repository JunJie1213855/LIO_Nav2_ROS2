#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

source install/setup.bash

PCD_DEST_DIR="$WORKSPACE_ROOT/src/planner/nav2_planner_bringup/pcd"

# FAST-LIO 节点名: /laserMapping（注意驼峰）
# 服务注册在节点命名空间下: /laserMapping/map_save
# map_save 服务触发后生成两个文件：
#   1. map_file_path  (YAML 配置，默认 "./test.pcd") → ikdtree 地图
#   2. PCD/dense_map.pcd (pcd_save_en=true 时) → 稠密累积点云，KISS-Matcher 用

echo "等待 FAST-LIO /map_save 服务就绪..."

# 等待服务出现（最多等 30 秒）
SERVICE=""
for i in $(seq 1 30); do
    for path in "/map_save" "/laserMapping/map_save"; do
        if ros2 service list 2>/dev/null | grep -q "$path"; then
            SERVICE="$path"
            break 2
        fi
    done
    sleep 1
    echo -n "."
done
echo ""

if [ -z "$SERVICE" ]; then
    echo "错误: FAST-LIO /map_save 服务未出现（等待 30 秒后超时）"
    exit 1
fi

echo "找到服务: $SERVICE"

# 确保目标目录存在
mkdir -p "$PCD_DEST_DIR"

# 触发保存
ros2 service call "$SERVICE" std_srvs/srv/Trigger

# --- 处理文件 1: ikdtree 地图 (test.pcd → robo_map.pcd) ---
IKDTREE_PCD="$WORKSPACE_ROOT/test.pcd"
if [ -f "$IKDTREE_PCD" ]; then
    mv "$IKDTREE_PCD" "$PCD_DEST_DIR/robo_map.pcd"
    echo "ikdtree 地图: $PCD_DEST_DIR/robo_map.pcd"
else
    echo "注意: 未找到 ikdtree 地图 ($IKDTREE_PCD)，可能 map_file_path 配置了其他路径"
fi

# --- 处理文件 2: 稠密累积点云 (PCD/dense_map.pcd → dense_map.pcd) ---
# FAST-LIO 中 ROOT_DIR 编译期固定，dense_map.pcd 保存在 PCD/dense_map.pcd
FAST_LIO_PCD_DIR="$WORKSPACE_ROOT/src/localization/FAST_LIO_ROBOAIRY/PCD"
DENSE_SRC="$FAST_LIO_PCD_DIR/dense_map.pcd"
if [ -f "$DENSE_SRC" ]; then
    cp "$DENSE_SRC" "$PCD_DEST_DIR/dense_map.pcd"
    echo "稠密点云: $PCD_DEST_DIR/dense_map.pcd"
else
    echo "注意: 未找到稠密点云 ($DENSE_SRC)，检查 FAST-LIO 配置中 pcd_save.pcd_save_en: true"
fi

# --- 同步到 install 目录 ---
# launch 用 get_package_share_directory 读取 install/ 路径，需要创建软链接
INSTALL_PCD_DIR="$WORKSPACE_ROOT/install/nav2_planner_bringup/share/nav2_planner_bringup/pcd"
mkdir -p "$INSTALL_PCD_DIR"

for pcd_file in robo_map.pcd dense_map.pcd; do
    src="$PCD_DEST_DIR/$pcd_file"
    if [ -f "$src" ]; then
        ln -sf "$src" "$INSTALL_PCD_DIR/$pcd_file"
        echo "安装链接: $INSTALL_PCD_DIR/$pcd_file"
    fi
done

echo ""
echo "===== 保存完成 ====="
echo "KISS-Matcher 加载: $PCD_DEST_DIR/dense_map.pcd"
echo "Nav2 导航启动:      ./scripts/nav2_sim_docker.sh"
