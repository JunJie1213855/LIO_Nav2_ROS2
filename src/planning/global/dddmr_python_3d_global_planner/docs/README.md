# dddmr_global_nav 文档

- [使用方法与注意事项](dddmr_global_nav.md)：安装、网页/命令行/Python API、ROS2、地图参数和故障排查。
- [先验地图、重定位与局部规划器接入](global_local_reloc.md)：接口适配、坐标与参考点、启动顺序、重规划和执行闭环。
- [ROS2 map.pcd 演示配置](map_ros2_example.yaml)：与当前 `dddmr_py/examples/config_map.yaml` 对齐的 ROS2 格式配置。
- [重定位接入参数示例](reloc_ros2_example.yaml)：完整先验地图话题、Odometry 起点和 `/init_path` 输出的接线模板。

当前实现的 Python 模块名是 `dddmr_py`，ROS2 包名是 `dddmr_ros2`，节点名是 `dddmr_planner`。
`dddmr_global_nav` 是本文档中的全局规划功能名称，并非当前可执行文件或 ROS2 包名。
