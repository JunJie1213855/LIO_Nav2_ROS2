# 更换 Gazebo 仿真地图

Gazebo 世界文件由 `get_urdf` 包的 launch 文件加载。以下三种方法任选。

## 1. 关键文件

```
src/get_urdf/
├── launch/
│   └── get_urdf_launch.py    ← 第 15 行指定 world 文件
├── worlds/
│   └── test_world.world      ← 当前使用的仿真地图
├── model/
│   └── simple_car.urdf       ← 机器人 URDF 模型
└── rviz/
    └── nav2_new.rviz
```

`get_urdf_launch.py` 第 15 行：

```python
world_file_path = os.path.join(pkg_share_path, 'worlds', 'test_world.world')
```

## 2. 方法一：使用现有 world 文件

```bash
# 查看所有可用地图
ls src/get_urdf/worlds/
```

修改 `src/get_urdf/launch/get_urdf_launch.py`：

```python
# 替换引号内的文件名即可
world_file_path = os.path.join(pkg_share_path, 'worlds', '你要的地图.world')
```

## 3. 方法二：自己创建新地图

在 `src/get_urdf/worlds/` 目录下新建 `.world` 文件：

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_map">

    <!-- 基础元素：光源 + 地面 -->
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- 障碍物示例：墙壁 -->
    <model name="wall_north">
      <static>true</static>
      <link name="link">
        <pose>5 0 0.5 0 0 0</pose>
        <visual name="visual">
          <geometry><box><size>0.2 10 1</size></box></geometry>
        </visual>
        <collision name="collision">
          <geometry><box><size>0.2 10 1</size></box></geometry>
        </collision>
      </link>
    </model>

    <!-- 障碍物示例：圆柱 -->
    <model name="pillar">
      <static>true</static>
      <link name="link">
        <pose>-3 2 0.5 0 0 0</pose>
        <visual name="visual">
          <geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry>
        </collision>
      </link>
    </model>

  </world>
</sdf>
```

常用几何体：

| 类型 | SDF 标签 | 示例参数 |
|------|---------|---------|
| 立方体 | `<box>` | `<size>长 宽 高</size>` |
| 圆柱 | `<cylinder>` | `<radius>半径</radius><length>高</length>` |
| 球 | `<sphere>` | `<radius>半径</radius>` |

## 4. 方法三：引用 Gazebo 官方模型

先下载模型库（只需一次）：

```bash
cd ~/.gazebo
git clone https://github.com/osrf/gazebo_models.git models
```

即可在 world 文件中引用：

```xml
<include><uri>model://bookshelf</uri><pose>3 0 0 0 0 0</pose></include>
<include><uri>model://cafe_table</uri><pose>0 4 0 0 0 0</pose></include>
<include><uri>model://construction_cone</uri><pose>1 -2 0 0 0 0</pose></include>
```

## 5. 重新部署

因为 `install(DIRECTORY ...)` 是复制而非 symlink，world 文件改动后需要重新部署：

```bash
cd ~/rosws/3d_nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select get_urdf
```

然后重启 tmux session 即可看到新地图。

## 6. 完整流程总结

```bash
# 1. 创建新 world 文件
vim src/get_urdf/worlds/my_map.world

# 2. 修改 launch 文件指向新地图
vim src/get_urdf/launch/get_urdf_launch.py
# 改第 15 行: 'test_world.world' → 'my_map.world'

# 3. 重新部署
colcon build --symlink-install --packages-select get_urdf
source install/setup.bash

# 4. 启动仿真
./scripts/nav2_sim_tmux.sh
```

## 7. 重要提示

- **换地图后必须重新建图**：新环境的 2D 地图（`.yaml`/`.pgm`）和 3D 点云（`.pcd`）与旧地图不匹配
- 使用 `mapping_sim.sh` 在新地图中建图
- 将保存的地图和点云更新到 `src/me_nav2_bringup/map/` 和 `src/me_nav2_bringup/pcd/`
- 更新 `my_nav2_launch.py` 中的 `map_yaml_file` 和重定位 launch 文件中的 `prior_pcd_file`
