#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

source install/setup.bash

MAP_PATH="${1:-$WORKSPACE_ROOT/src/me_nav2_bringup/pcd/robo_map.pcd}"

# FAST-LIO 节点名是 /laser_mapping（非 /fastlio_mapping）
# 注意：map_file_path 只在 FAST-LIO 启动时读取一次，ros2 param set 运行时不生效
# 因此需要确保 config/mid360.yaml 中 pcd_save.pcd_save_en: True，
# 然后调用 /map_save service，PCD 会写到 YAML 中配置的默认路径

# 确保目标目录存在
mkdir -p "$(dirname "$MAP_PATH")"

# 触发保存
ros2 service call /map_save std_srvs/srv/Trigger

# FAST-LIO 默认配置 map_file_path: "./test.pcd"（相对于工作目录即 /ws/test.pcd）
DEFAULT_PCD="$WORKSPACE_ROOT/test.pcd"
if [ -f "$DEFAULT_PCD" ]; then
    mv "$DEFAULT_PCD" "$MAP_PATH"
    # 同步到 install 目录，供 launch 文件引用
    # （launch 用 get_package_share_directory 读取的是 install/ 路径，不会自动去 src/ 找）
    INSTALL_PCD="$WORKSPACE_ROOT/install/me_nav2_bringup/share/me_nav2_bringup/pcd/$(basename "$MAP_PATH")"
    mkdir -p "$(dirname "$INSTALL_PCD")"
    ln -sf "$MAP_PATH" "$INSTALL_PCD"
    echo "PCD 已保存到: $MAP_PATH"
    echo "安装目录:       $INSTALL_PCD"
else
    echo "错误: 未找到默认 PCD 文件 ($DEFAULT_PCD)，保存可能失败"
    exit 1
fi
