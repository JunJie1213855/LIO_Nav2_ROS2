# icp_registration

**一次性 ICP 配准节点**，仅在启动时执行一次 `map → odom` 发布，后续不再维护。

## 适用场景

已知机器人初始位姿大致在 (0,0,0) 附近，只需一次粗配准即可对齐先验地图。

## 与 KISS-Matcher 的区别

| | icp_registration | global_relocalization_kiss_matcher |
|--|------------------|-------------------------------------|
| 配准次数 | 仅启动时一次 | 持续跟踪 + 失败自动恢复 |
| 初始位姿 | 需要大致已知 | 无需 (全局搜索) |
| 适用性 | 简单场景 | 长距离导航 |

## 启动

```bash
ros2 launch icp_registration icp.launch.py
```

## 注意

只能二选一使用，**不可同时运行**多个发布 `map → odom` 的节点。
