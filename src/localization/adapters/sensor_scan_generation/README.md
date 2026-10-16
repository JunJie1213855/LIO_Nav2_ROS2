# sensor_scan_generation

**标准化里程计发布节点** — 将 `lio_interface` 的 `odom → livox_frame` 换算为 `odom → base_footprint`，发布标准 `/odom` 话题和 TF。

---

## 1. 为什么需要这个节点？

`lio_interface` 输出的是 **LiDAR 在 odom 下的位姿** (`odom → livox_frame`)，但 Nav2 和 slam_toolbox 需要的是**底盘在 odom 下的位姿** (`odom → base_footprint`)。

LiDAR 和底盘不在同一个位置——它们在 URDF 中定义了一个固定的空间偏移。本节点负责用这个偏移做换算。

```
lio_interface 输出              本节点处理                    Nav2 消费
─────────────────              ──────────                   ──────────
odom → livox_frame      +      livox_frame → base_footprint    =    odom → base_footprint
     (LIO 动态)                    (URDF 静态)                     (最终需要的)
```

---

## 2. 数据流

```
┌──────────────────────────────────────────────────────────────────┐
│                     lio_interface (上游)                           │
│                                                                   │
│  /registered_odometry                                             │
│    frame_id     = "odom"           ← LIO 输出, 已对齐的标准位姿   │
│    child_frame  = "livox_frame"                                   │
│                                                                   │
│  /registered_scan                                                 │
│    frame_id     = "odom"           ← odom 系下的点云               │
└───────────────┬──────────────────────┬────────────────────────────┘
                │                      │
                ▼                      ▼
┌──────────────────────────────────────────────────────────────────┐
│                 sensor_scan_generation (本节点)                    │
│                                                                   │
│  输入:  odom → livox_frame  +  odom 系点云                       │
│                                                                   │
│  步骤:                                                            │
│    1. 从 TF 树读取 livox_frame → base_footprint (URDF 静态外参)  │
│    2. T_{odom}^{base_footprint} =                                 │
│         T_{odom}^{livox_frame} · T_{livox_frame}^{base_footprint} │
│    3. 点云逆变换: odom 系 → livox_frame 系                        │
│    4. 速度: 位移差分 / Δt                                        │
│                                                                   │
│  输出:                                                            │
│    ├─ /odom 话题                  (odom → base_footprint + 速度)  │
│    ├─ TF: odom → base_footprint   (动态 TF, 持续更新)             │
│    └─ /lidar_frame_pcd            (livox_frame 系点云)            │
└───────────────┬──────────────────────┬────────────────────────────┘
                │                      │
                ▼                      ▼
┌─────────────────────┐    ┌──────────────────────────┐
│  Nav2 / slam_toolbox│    │  KISS-Matcher 重定位      │
│  (消耗 /odom + TF)  │    │  pointcloud_to_laserscan  │
│                     │    │  (消耗 /lidar_frame_pcd)  │
└─────────────────────┘    └──────────────────────────┘
```

---

## 3. 数学公式推导

### 3.1 记号约定

| 符号 | 含义 |
|---|---|
| `T_A^B` | 坐标系 A 到 B 的刚体变换（= B 在 A 中的位姿） |
| `P^A` | 点 P 在坐标系 A 下的坐标 |
| `·` | 齐次变换：`R · p + t` |

---

### 3.2 位姿换算 — odom → base_footprint

**输入**（来自 lio_interface）：

```
/registered_odometry:
  frame_id      = "odom"
  child_frame   = "livox_frame"
  pose          = T_{odom}^{livox_frame}
```

**从 TF 树读取**（来自 URDF / robot_state_publisher）：

```
/tf_static:
  base_footprint → livox_frame   →   反向查询得到 T_{livox_frame}^{base_footprint}
```

**合成**：

```
T_{odom}^{base_footprint} = T_{odom}^{livox_frame} · T_{livox_frame}^{base_footprint}
```

展开为齐次形式：

```
┌               ┐   ┌                     ┐ ┌                             ┐
│ R_obf   t_obf │   │ R_olf   t_olf       │ │ R_lbf   t_lbf               │
│               │ = │                     │ │                             │
│ 0  0  0   1   │   │ 0  0  0     1       │ │ 0  0  0       1            │
└               ┘   └                     ┘ └                             ┘
  odom→base          odom→livox             livox→base

  其中:
    olf = odom_to_livox_frame     (LIO 给的)
    lbf = livox_to_base_footprint  (URDF 静态外参)
    obf = odom_to_base_footprint   (计算结果)
```

对应代码：

```cpp
// 解析 LIO 给的位置
tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

// 读取 URDF 静态外参
tf_lidar_to_base_footprint_ = getTransform("livox_frame", "base_footprint", stamp);

// 合成
tf_odom_to_base_footprint_ = tf_odom_to_lidar * tf_lidar_to_base_footprint_;
```

---

### 3.3 点云逆变换 — odom 系 → livox_frame 系

**为什么需要逆变换？**

`lio_interface` 把点云从 `camera_init` 转到了 `odom` 系统一处理，但下游重定位和 3D→2D 切片节点需要的是 **LiDAR 自身坐标系**下的点云（即传感器原始视角）。

**公式**：

```
P^{livox} = (T_{odom}^{livox_frame})^{-1} · P^{odom}
          = T_{livox_frame}^{odom} · P^{odom}
```

对应代码：

```cpp
// tf_odom_to_lidar = T_{odom}^{livox_frame}
// .inverse() → T_{livox_frame}^{odom}
pcl_ros::transformPointCloud("livox_frame", tf_odom_to_lidar.inverse(), *pcd_msg, out);
```

---

### 3.4 速度估算

LIO 不输出速度，本节点用**位姿差分**近似：

**线速度**：

```
v = (p_t − p_{t−1}) / Δt
```

其中 `p_t` 为当前帧 `odom → base_footprint` 的平移分量。

**角速度**：

```
q_diff = q_t · q_{t−1}^{-1}           ← 两个四元数的差
ω = axis(q_diff) · angle(q_diff) / Δt ← 轴角表示除以时间
```

对应代码：

```cpp
// 线速度
const auto linear_velocity = (transform.getOrigin() - previous_transform.getOrigin()) / dt;

// 角速度
const tf2::Quaternion q_diff = transform.getRotation() * previous_transform.getRotation().inverse();
const auto angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt;
```

---

### 3.5 完整变换链

```
┌─ 传感器原始数据 ──────────────────────┐
│  FAST-LIO: camera_init → body (IMU)   │
│            camera_init 系点云          │
└──────────────┬────────────────────────┘
               │
               ▼
┌─ lio_interface ───────────────────────┐
│                                        │
│  T_{odom}^{body} =                     │
│    T_{odom}^{camera_init}              │
│    · T_{camera_init}^{body}            │
│                                        │
│  P^{odom} = T_{odom}^{camera_init}     │
│              · P^{camera_init}         │
│                                        │
│  输出: odom → body (≈ livox_frame)    │
│        odom 系点云                     │
└──────────────┬────────────────────────┘
               │
               ▼
┌─ sensor_scan_generation ──────────────┐
│                                        │
│  T_{odom}^{base_footprint} =           │
│    T_{odom}^{livox_frame}              │
│    · T_{livox_frame}^{base_footprint}  │
│         ↑                              │
│      (从 URDF/TF 树读取)               │
│                                        │
│  速度: v = Δp/Δt,  ω = Δq/Δt         │
│                                        │
│  点云逆变换:                            │
│    P^{livox} = (T_{odom}^{livox_frame})^{-1} · P^{odom} │
└──────────────┬────────────────────────┘
               │
               ▼
    ┌──────────────────────┐
    │ /odom 话题            │──→ Nav2 controller/planner
    │ TF: odom→base_footprint│──→ Nav2 costmap 定位
    │ /lidar_frame_pcd      │──→ KISS-Matcher 重定位
    └──────────────────────┘        pointcloud_to_laserscan
```

---

## 4. TF 树中的位置

```
map ──(SLAM 发布)──→ odom ──(本节点发布)──→ base_footprint ──(URDF静态)──→ livox_frame
       ↑                  ↑                       ↑
   全局修正            里程计估计              机器人底盘
```

本节点发布的 `odom → base_footprint` 是连接里程计和机器人座的关键一环。没有它，map 帧无法触及 base_footprint，Nav2 的 costmap 无法定位。

---

## 5. 订阅与发布

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 订阅 | `/registered_odometry` | `nav_msgs/Odometry` | lio_interface 输出的对齐后位姿 |
| 订阅 | `/registered_scan` | `sensor_msgs/PointCloud2` | lio_interface 输出的 odom 系点云 |
| 发布 | `/odom` | `nav_msgs/Odometry` | 标准里程计, 供 Nav2 使用 |
| 发布 | TF: `odom → base_footprint` | `tf2_msgs/TFMessage` | 动态 TF, 供整个 TF 树查询 |
| 发布 | `/lidar_frame_pcd` | `sensor_msgs/PointCloud2` | livox_frame 系点云, 供重定位和 3D→2D 切片 |

---

## 6. 启动方式

```bash
# 仿真模式
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py

# 实机模式
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py use_sim_time:=False
```

### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `lidar_frame` | `livox_frame` | LiDAR 传感器的 TF 帧名 |
| `base_footprint_frame` | `base_footprint` | 机器人底盘投影帧名 |
| `chassis_frame` | `chassis` | 机器人机身帧名 |
| `use_sim_time` | `True` | 是否使用仿真时间 |

---

## 7. 已知局限

### 速度差分噪声

速度通过位姿差分估算，没有滤波器，高频抖动会放大。Nav2 的 `velocity_smoother` 在下游做平滑，一般够用。如果需要更精确的速度，可以在 LIO 内部直接输出（但 FAST-LIO 当前不实现）。

### 消息同步依赖

依赖 `message_filters::ApproximateTime` 同步位姿和点云。如果两路消息时间戳偏差过大（> 队列长度对应的窗口），会丢帧。正常情况下它们来自同一个 LIO 时刻，偏差极小。

### TF 依赖

初始化需要 `livox_frame → base_footprint` 的 TF 已发布。如果 `robot_state_publisher` 还没就绪，`getTransform` 会返回 identity（不阻塞），但位姿换算将不正确。

---

## 8. 与 lio_interface 的关系

| 步骤 | 节点 | 做什么 |
|---|---|---|
| ① | `lio_interface` | camera_init → odom 坐标系对齐 |
| ② | `sensor_scan_generation` | odom → base_footprint 换算 + 发布 `/odom` 和 TF |

两者必须同时运行。`mapping_sim.sh` / `nav2_sim.sh` 中按正确顺序启动。
