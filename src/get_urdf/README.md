# get_urdf

**仿真环境启动包**，包含机器人 URDF、Gazebo 世界和 RViz 配置。

## 核心文件

| 文件 | 用途 |
|------|------|
| `launch/get_urdf_launch.py` | 启动 Gazebo + 生成机器人 + RViz |
| `model/simple_car.urdf` | 四轮滑移转向机器人 URDF |
| `worlds/test_world.world` | Gazebo 仿真世界 |
| `rviz/nav2_new.rviz` | 仿真模式 RViz 配置 |

## 启动流程

1. 启动 Gazebo + 加载 world
2. 发布 `/robot_description`
3. 延迟 3s 后生成 `simple_car`
4. 启动 RViz

## 换地图

修改 `get_urdf_launch.py`: `world_file_path` → 新 world 文件。详见 `docs/changemap.md`。
