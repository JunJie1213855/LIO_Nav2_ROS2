# global_small_gicp_relocalization

基于 small_gicp 的全局重定位变体，通过 GICP 连续配准发布 `map → odom` TF。

## 与 small_gicp_relocalization 的区别

本包使用不同的初始化策略，在更大范围内搜索初始位姿。当前保留作为备选方案。

## 启动

```bash
ros2 launch global_small_gicp_relocalization global_small_gicp_relocalization_launch.py
```

## 注意

与 KISS-Matcher / small_gicp 二选一，不可同时运行。
