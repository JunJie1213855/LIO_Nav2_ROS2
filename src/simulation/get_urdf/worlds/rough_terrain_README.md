# 大型螺旋坡道与障碍地形（Gazebo Classic 11）

`rough_terrain.world` 为 60 × 60 m 的地形，使用 257 × 257 高度采样生成静态网格（131072 个三角形）。
原点周围半径 2.5 m 保留平地。当前 launch 在此世界的默认出生点为螺旋入口 `(0, -23, 0.05)`，车头沿入口切线方向，偏航约 0.0645 rad（3.7°），朝向螺旋上坡方向。5 cm 初始离地间隙用于避免轮胎与地形穿插，启动后自然落地。其他世界保留原来的出生位置。

- 螺旋坡道绕原点旋转 1.5 圈，半径由 23 m 缩小至 9 m，地面由 0 m 平滑升至 3 m。
- 入口为 (0, -23)，终点为 (0, 9)。沿路径从外圈向内圈行进；白色虚线仅为预览辅助线。
- 路面宽 3.6 m，两侧各有 1.6 m 的土质过渡坡肩，属于连续土坡路堤。
- 轨道两侧有约 0.9 m 高的深灰围栏，中心距离路线中心 2.05 m，净通行宽度约 4 m；围栏随路面起伏升高，入口与终点不封口（纵向各退让 0.75 m）。
- 围栏下半部为连续实心矮墙，墙顶高于沿线地面约 0.45 m、厚 0.12 m，底部嵌入约 5 cm；上半部保留顶部横杆及约 2.5 m 间隔的立柱，总高约 0.9 m。所有部件合并为一个静态网格模型，显示与碰撞一致。围栏无纹理、不投射阴影，控制额外绘制开销。
- 路面中段叠加最高约 ±6 cm 的纵向起伏。
- (-22, 19)、(22, 19)、(-23, -8) 等处有小土坡；(-4, -5) 附近为连续凹凸路面。
- (20, -19) 附近有抬高地面上的浅凹坑。
- 共有 9 个圆柱、6 个直方体。3 个小圆柱靠近螺旋路边，其余分布在地面；均含碰撞体。
- 障碍物底部按其覆盖区域最高地形高度放置，并嵌入约 3 cm，保证不悬空。
- 网格间距约 23.4 cm；8 位高度图的高度量化间距约 1.57 cm。
- 高度图是单层地面，不包含上下交叉的高架道路；路堤两侧坡肩可能比行车方向更陡。

在已加载 ROS 工作空间的桌面终端中运行：

```bash
ros2 launch get_urdf get_urdf_launch.py world_path:=/home/ros/ros_ws/LIO_Nav2_ROS2/src/simulation/get_urdf/worlds/rough_terrain.world
```

只看地形：

```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH="/home/ros/ros_ws/LIO_Nav2_ROS2/src/simulation/get_urdf:${GAZEBO_RESOURCE_PATH}"
gazebo --verbose /home/ros/ros_ws/LIO_Nav2_ROS2/src/simulation/get_urdf/worlds/rough_terrain.world
```

修改同目录 `generate_rough_terrain.py`，然后重新生成：

```bash
PYTHONNOUSERSITE=1 python3 /home/ros/ros_ws/LIO_Nav2_ROS2/src/simulation/get_urdf/worlds/generate_rough_terrain.py
```

需要 numpy、scipy、Pillow、matplotlib。PYTHONNOUSERSITE 用于规避当前机器上用户目录与系统 matplotlib 的版本混用。
生成器同时输出世界文件、地形与围栏两个静态 STL 网格、高度图和预览图。运行时加载 `file://worlds/rough_terrain_mesh.stl` 与 `file://worlds/rough_terrain_fence.stl`，每个模型的碰撞和显示共用同一个网格。launch 自动设置包内资源搜索路径，不依赖源码绝对路径。
重新生成后运行 `colcon build --packages-select get_urdf`，将 STL 网格与世界文件一起安装，再 `source install/setup.bash`。分发或提交项目时，必须包含 STL 网格。
参照 test_world.world，地形使用 Gazebo/Grey 普通受光纯色材质，圆柱为 Gazebo/Orange、直方体为 Gazebo/Blue；无纹理、法线贴图、地形专用着色器及自发光。仅使用一盏太阳光，开启普通阴影。与 test_world 的地面一样，地形不投射阴影，但接收障碍物和机器人阴影；障碍物投影开启，避免将复杂地形加入投影绘制。坡面明暗来自普通光照。Gazebo 自身基础渲染仍需系统资源目录。相较之前 1025 × 1025 采样，地形三角形数减少约 94%；这不是整机内存或 CPU 实测降幅。小于约 23 cm 的地形细节会被简化。
三维预览将垂直比例放大 3 倍；俯视图颜色表示真实地面高度，橙色是圆柱、蓝色是直方体。预览并非 Gazebo 截图。
世界未添加无限地面，机器人应留在地形边界内。
