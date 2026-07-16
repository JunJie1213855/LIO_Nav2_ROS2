# gld_robot_description

**实机 URDF 模型**，包含完整传感器外参。

## 与 get_urdf 的区别

| | get_urdf | gld_robot_description |
|--|----------|----------------------|
| 用途 | 仿真 | 实机 |
| 传感器 | 无 | RealSense D456/D405 + Orbbec Gemini |
| LiDAR 驱动 | Gazebo 插件 | livox_ros_driver2 |
| 启动脚本 | `mapping_sim.sh` / `nav2_sim.sh` | `mapping_real.sh` / `nav2_real.sh` |

## 启动

```bash
ros2 launch gld_robot_description gld_robot_description_launch.py
```
