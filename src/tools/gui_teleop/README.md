# gui_teleop

tkinter **GUI 遥控器**，Apple 风格界面，WASD 控制机器人，Shift 加速，空格急停。

## 控制

| 按键 | 动作 |
|------|------|
| W | 前进 |
| S | 后退 |
| A | 左转 |
| D | 右转 |
| Shift | 加速 |
| 空格 | 急停 |

## 启动

```bash
ros2 run gui_teleop gui_teleop_node
```

## 输出

`/cmd_vel` (Twist)

## 注意

与 Nav2 共享 `/cmd_vel`，导航模式下不要同时手动遥控。
