# lio_interface

**LIO 坐标系桥接节点** — 将 FAST-LIO / Point-LIO 内部里程计转换为 ROS 标准坐标系的中间层。

---

## 1. 为什么需要这个节点？

FAST-LIO 输出的是 **`camera_init` 坐标系**下的位姿，而 Nav2、slam_toolbox 等 ROS 导航栈期望使用 **`odom` 坐标系**。这两个坐标系原点不同，不能直接混用。

```
FAST-LIO 的世界: camera_init          ROS 标准的世界: odom
       ↓                                      ↓
  LIO 初始化时 IMU 的位置              机器人启动时 base_footprint 的位置
```

`lio_interface` 的作用：**在 camera_init 和 odom 之间建立一个固定的平移+旋转偏移量，把 LIO 的输出转换到 odom 下**，让下游导航栈可以正常工作。

---

## 2. 坐标系定义

| 坐标系 (frame_id) | 含义 | 来源 |
|---|---|---|
| `camera_init` | LIO 世界帧，初始化时 IMU 所在位置。固定不动。 | FAST-LIO 内部定义 |
| `body` | IMU 传感器本体坐标系。 | FAST-LIO 的 IMU 数据 |
| `livox_frame` | LiDAR 传感器安装位置（URDF 中的 link）。 | `robot_state_publisher` 发布 |
| `odom` | ROS 标准里程计坐标系，原点 = 启动时 base_footprint 位置。 | 本项目定义 |
| `base_footprint` | 机器人底盘地面投影。 | `robot_state_publisher` 发布 |

---

## 3. TF 树结构

```
odom ──(动态发布, 持续更新)──→ base_footprint
                                   │
                    (URDF 静态外参, /tf_static)
                                   ↓
                              livox_frame
```

- **静态 TF** (`/tf_static`): `base_footprint → chassis → livox_frame` — 由 `robot_state_publisher` 从 URDF 读取后发布，不变。
- **动态 TF** (`/tf`): `odom → base_footprint` — 由 `sensor_scan_generation` 根据本节点的输出计算并发布，每帧更新。

---

## 4. 数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                        FAST-LIO (上游)                           │
│                                                                  │
│  /Odometry                                                       │
│    frame_id     = "camera_init"    ← LIO 内部世界帧              │
│    child_frame  = "body"           ← IMU 本体的实时位姿          │
│                                                                  │
│  /cloud_registered                                              │
│    frame_id     = "camera_init"    ← LIO 世界帧下的点云          │
└───────────────┬──────────────────────┬──────────────────────────┘
                │                      │
                ▼                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                      lio_interface (本节点)                       │
│                                                                  │
│  ┌─ 初始化 (第一个 /Odometry 消息到达时, 只执行一次) ─┐         │
│  │                                                      │         │
│  │  从 TF 树读取: base_footprint → livox_frame (URDF)   │         │
│  │                                                      │         │
│  │  存储: tf_odom_to_lidar_odom_ = T_{bf}^{lf}        │         │
│  │         即 T_{odom}^{camera_init}  ← 固定偏移        │         │
│  │                                                      │         │
│  │  原理: 启动时 camera_init → body ≈ identity,         │         │
│  │        所以 odom 原点 ≈ base_footprint 此刻位置       │         │
│  └──────────────────────────────────────────────────────┘         │
│                                                                  │
│  ┌─ 每帧运行 ─────────────────────────────────────────────┐     │
│  │                                                         │     │
│  │  位姿转换:                                              │     │
│  │    T_{odom}^{body} = T_{odom}^{camera_init}            │     │
│  │                       × T_{camera_init}^{body}          │     │
│  │    → 发布到 /registered_odometry                       │     │
│  │      (frame_id="odom", child_frame_id="livox_frame")   │     │
│  │                                                         │     │
│  │  点云转换:                                              │     │
│  │    P_{odom} = T_{odom}^{camera_init} · P_{camera_init} │     │
│  │    → 发布到 /registered_scan                           │     │
│  └─────────────────────────────────────────────────────────┘     │
└───────────────┬──────────────────────┬──────────────────────────┘
                │                      │
                ▼                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                sensor_scan_generation (下游)                      │
│                                                                  │
│  输入: /registered_odometry + /registered_scan                  │
│                                                                  │
│  计算: odom → base_footprint                                    │
│    = T_{odom}^{livox_frame} × (T_{base_footprint}^{livox_frame})^{-1} │
│                                                                  │
│  发布: /odom 话题 + odom → base_footprint TF                    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 5. 数学公式推导

### 5.1 记号约定

| 符号 | 含义 |
|---|---|
| `T_A^B` | 坐标系 A 到坐标系 B 的刚体变换（4×4 齐次矩阵），即 B 在 A 中的位姿 |
| `P^A` | 点 P 在坐标系 A 下的坐标 |
| `p_i` | 点云中的第 i 个点 |
| `·` | 齐次变换：先旋转再平移 |

---

### 5.2 初始化 — 计算 odom → camera_init 对齐偏移

**目标**：让 `odom` 原点等于机器人启动时 `base_footprint` 的位置。

机器人启动时静止，LIO 刚完成初始化：

```
T_{camera_init}^{body} ≈ I    (robot hasn't moved)
```

我们希望 `T_{odom}^{base\_footprint} ≈ I`，即 odom 和 base_footprint 此刻重合：

```
T_{odom}^{body} · T_{body}^{base\_footprint} ≈ I

⇒  T_{odom}^{body} ≈ (T_{body}^{base\_footprint})^{-1}
                    = T_{base\_footprint}^{body}
```

由于 body 和 livox_frame 物理接近（IMU-LiDAR 外参 ≈ I）：

```
T_{base\_footprint}^{body} ≈ T_{base\_footprint}^{livox\_frame}
```

而 `T_{camera_init}^{body} ≈ I`，所以：

```
T_{odom}^{camera_init} = T_{odom}^{body} · (T_{camera_init}^{body})^{-1}
                       ≈ T_{odom}^{body} · I
                       ≈ T_{base\_footprint}^{livox\_frame}
```

**结论**：初始化时从 TF 树读取 `base_footprint → livox_frame` 的 URDF 静态外参，直接作为 `odom → camera_init` 的对齐偏移量：

```
T_{odom}^{camera_init} := T_{base\_footprint}^{livox\_frame}
```

对应代码：

```cpp
// 从 TF 树查询 URDF 中定义的 base_footprint → livox_frame
auto tf = tf_buffer_->lookupTransform("base_footprint", "livox_frame", ...);
tf2::fromMsg(tf.transform, tf_base_frame_to_lidar_frame);

// 存为 odom → camera_init 的固定偏移
tf_odom_to_lidar_odom_ = tf_base_frame_to_lidar_frame;
```

此后 `tf_odom_to_lidar_odom_` 不再改变。

---

### 5.3 位姿转换（每帧运行）

**输入**：FAST-LIO `/Odometry` 消息

```
消息内容:
  frame_id      = "camera_init"
  child_frame   = "body"
  pose          = T_{camera_init}^{body}    ← IMU 在 LIO 世界中的实时位姿
```

**输出**：`/registered_odometry` 消息

```
消息内容:
  frame_id      = "odom"
  child_frame   = "livox_frame"
  pose          = T_{odom}^{body}           ← 近似为 T_{odom}^{livox\_frame}
```

**推导**：

```
T_{odom}^{body} = T_{odom}^{camera_init} · T_{camera_init}^{body}
                   ↑                        ↑
              固定偏移量                  FAST-LIO 实时输出
              (初始化时算好)               (每帧更新)
```

由于 `T_{body}^{livox\_frame} ≈ I`（IMU-LiDAR 外参很小），有近似：

```
T_{odom}^{livox\_frame} ≈ T_{odom}^{body}
```

对应代码：

```cpp
// 从 FAST-LIO 消息读取 camera_init → body
tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar_frame);

// 合成 odom 下的位姿
tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar_frame;
//                 ↑                        ↑
//            T_{odom}^{camera_init}   T_{camera_init}^{body}

// 以 odom 为 frame_id 发布
out.header.frame_id = "odom";
out.child_frame_id = "livox_frame";
out.pose.pose = tf_odom_to_lidar;  // T_{odom}^{body} ≈ T_{odom}^{livox_frame}
```

---

### 5.4 点云转换（每帧运行）

**输入**：`/cloud_registered` 消息

```
消息内容:
  frame_id = "camera_init"
  points   = { p_i^{camera_init} }     ← 每个点坐标在 camera_init 系下
```

**输出**：`/registered_scan` 消息

```
消息内容:
  frame_id = "odom"
  points   = { p_i^{odom} }           ← 每个点坐标转换到 odom 系下
```

**推导**：

对于点云中的每个点 `p_i`，它在 `camera_init` 系下的坐标已知，需要转为 `odom` 系下的坐标。

刚体变换关系：

```
p_i^{odom} = T_{odom}^{camera_init} · p_i^{camera_init}
               ↑
          初始化时算好的固定偏移量
```

展开为齐次坐标形式：

```
┌         ┐   ┌                           ┐ ┌                   ┐
│ p_i^x   │   │ R_{odom}^{camera_init}  t │ │ p_i^x             │
│ p_i^y   │ = │                           │ │ p_i^y             │
│ p_i^z   │   │ 0    0    0            1  │ │ p_i^z             │
│    1    │odom└                           ┘ └    1              ┘camera_init
└         ┘
```

对应代码：

```cpp
// tf_odom_to_lidar_odom_ = T_{odom}^{camera_init}
// 输入 msg 的 frame_id = "camera_init"
// 输出 out 的 frame_id = "odom"
pcl_ros::transformPointCloud("odom", tf_odom_to_lidar_odom_, *msg, *out);
```

---

### 5.5 完整变换链总览

```
传感器原始数据 (camera_init 系)
        │
        ▼
┌─ lio_interface ─────────────────────────────────────────────────┐
│                                                                  │
│  点云:  P^{odom} = T_{odom}^{camera_init} · P^{camera_init}    │
│                                                                  │
│  位姿:  T_{odom}^{body} = T_{odom}^{camera_init}                │
│                              · T_{camera_init}^{body}            │
│                                                                  │
│  其中:  T_{odom}^{camera_init} := T_{base_footprint}^{livox_frame} │
│         (初始化时从 URDF 读取, 之后不变)                           │
└───────────────────┬──────────────────────────────────────────────┘
                    │
                    ▼  输出到 sensor_scan_generation
                    │
┌─ sensor_scan_generation ────────────────────────────────────────┐
│                                                                  │
│  T_{odom}^{base_footprint} =                                   │
│    T_{odom}^{livox_frame}                                       │
│      · T_{livox_frame}^{base_footprint}                         │
│                                                                  │
│  其中 T_{livox_frame}^{base_footprint} 也从 URDF/TF 树读取       │
└──────────────────┬───────────────────────────────────────────────┘
                   │
                   ▼
              发布 /odom 话题 + odom → base_footprint TF
                       │
                       ▼
                  Nav2 / slam_toolbox
```

---

## 6. 启动方式

| 模式 | 启动命令 | odometry_sub | cloud_topic | 适用场景 |
|---|---|---|---|---|---|
| FAST-LIO (默认) | `ros2 launch lio_interface lio_interface_launch.py` | `/Odometry` | `/cloud_registered` | 仿真 + 实机 FAST-LIO |
| Point-LIO | `ros2 launch lio_interface lio_interface_launch.py lio_type:=pointlio` | `/aft_mapped_to_init` | `/cloud_registered` | 实机 Point-LIO |
| **Super-LIO** | `ros2 launch lio_interface lio_interface_launch.py lio_type:=superlio` | `/lio/odom` | `/lio/cloud_world` | 实机 / 仿真 Super-LIO |

### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `lio_type` | `fastlio` | 一键选择 LIO 算法: `fastlio` / `pointlio` / `superlio` |
| `odometry_sub` | *(自动)* | 手动覆盖 LIO 里程计话题名 (留空则根据 lio_type 自动选择) |
| `cloud_topic` | *(自动)* | 手动覆盖 LIO 点云话题名 (留空则根据 lio_type 自动选择) |
| `use_sim_time` | `true` | 是否使用仿真时间 |

---

## 7. 已知局限

### IMU 帧 vs LiDAR 帧

FAST-LIO 输出的 `child_frame_id` 是 `"body"`（IMU 本体），但本节点把它当作 `"livox_frame"`（LiDAR）来发布。

```
当前:  T_{odom}^{body} = T_{odom}^{camera_init} × T_{camera_init}^{body}
期望:  T_{odom}^{livox_frame} = T_{odom}^{camera_init} × T_{camera_init}^{body} × T_{body}^{livox_frame}
                                                                        ↑ 缺失
```

因为当前 FAST-LIO 配置中 IMU-LiDAR 外参很小 (`extrinsic_T ≈ [0, 0, 0.28]`, `extrinsic_R ≈ I`)，这个误差在 2D 导航尺度下可忽略。如果未来 IMU 和 LiDAR 安装位置差得远，需要在此处补上。

### TF 依赖时序

初始化需要 `base_footprint → livox_frame` 的 TF 已经发布。如果 ROS 节点启动顺序不对（`robot_state_publisher` 还没加载好 URDF），初始化会失败并重试，直到 TF 就绪。

---

## 8. 与 sensor_scan_generation 的关系

这两个节点配合完成完整的坐标转换链：

| 步骤 | 节点 | 做什么 |
|---|---|---|
| ① | `lio_interface` | camera_init → odom 坐标系对齐 |
| ② | `sensor_scan_generation` | odom → base_footprint 换算 + 发布 `/odom` 话题和 TF |

两者必须同时运行，顺序不能乱。`mapping_sim.sh` / `nav2_sim.sh` 中它们按正确的顺序启动。
