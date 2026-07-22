# gld_robot_description

**实机 URDF 模型** — 定义机器人各部件之间的空间关系（静态 TF 外参），由 `robot_state_publisher` 解析后发布到 `/tf_static`。

---

## 1. 与 get_urdf 的区别

| | get_urdf | gld_robot_description |
|---|---|---|
| 场景 | Gazebo 仿真 | **实机** |
| 机器人模型 | simple_car（含轮子、Gazebo 驱动插件） | 纯 TF 骨架（无物理仿真） |
| 传感器 | Livox LiDAR（Gazebo 插件模拟） | Livox LiDAR（真实驱动 livox_ros_driver2） |
| 发布内容 | Gazebo 原生发布 TF + 传感器数据 | `robot_state_publisher` 只发布 `/tf_static` |

---

## 2. URDF 结构说明

### 2.1 Link（部件声明）

每个 `<link>` 代表机器人上的一个刚体部件：

| Link | 含义 | 是否必需 |
|---|---|---|
| `base_footprint` | 底盘在地面的投影。整个 TF 树的根节点。 | ✅ 必需 |
| `chassis` | 机器人机身刚体参照系。通常选在底盘重心。 | ✅ 必需 |
| `livox_frame` | Livox 激光雷达的安装位置。 | ✅ 必需（LiDAR） |
| `D456_1_link` | Realsense D456 深度相机 #1 | ❌ 可选 |
| `D456_2_link` | Realsense D456 深度相机 #2 | ❌ 可选 |
| `D405_1_link` | Realsense D405 短距深度相机 #1 | ❌ 可选 |
| `D405_2_link` | Realsense D405 短距深度相机 #2 | ❌ 可选 |
| `orbbec_gemini_1_link` | Orbbec Gemini 结构光相机 #1 | ❌ 可选 |
| `orbbec_gemini_2_link` | Orbbec Gemini 结构光相机 #2 | ❌ 可选 |
| `orbbec_gemini_3_link` | Orbbec Gemini 结构光相机 #3 | ❌ 可选 |

> **如果你只有 LiDAR + IMU**，只需要保留 `base_footprint`、`chassis`、`livox_frame` 三个 link，删除所有相机 link 即可。见下文的精简版 URDF。

### 2.2 Joint（关节连接）

每个 `<joint>` 定义父子 link 之间的空间关系（位置 + 姿态）：

```xml
<joint name="chassis_to_livox_frame" type="fixed">
  <parent link="chassis"/>              <!-- 父坐标系 -->
  <child link="livox_frame"/>           <!-- 子坐标系 -->
  <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>  <!-- 子在父中的位置和姿态 -->
</joint>
```

| 参数 | 含义 | 单位 |
|---|---|---|
| `xyz` | child 原点在 parent 坐标系中的位置 (x, y, z) | 米 |
| `rpy` | child 相对 parent 的旋转 (roll, pitch, yaw) | 弧度 |

比如 `xyz="0.15 0.0 0.45"` 表示传感器装在底盘前方 15cm、上方 45cm 的位置。

所有 joint 类型都是 `"fixed"`（刚性固定），因为传感器固连在机身上，不会相对运动。

**当前所有 `origin` 都是 `(0, 0, 0)`** — 这是占位值。实机部署前需要用 CAD 或卷尺测量后填入真实外参。

### 2.3 为什么没有 `<inertial>` 和 `<collision>`

仿真模型 (`simple_car.urdf`) 有惯性参数和碰撞体积，因为 Gazebo 需要它们做物理模拟。实机 URDF 只发布静态 TF，不需要这些。

---

## 3. TF 树

```
base_footprint                     ← TF 树根节点
  │
  └─ chassis                       ← 机身 (当前偏移 0,0,0)
       │
       ├─ livox_frame              ← LiDAR (当前偏移 0,0,0)
       ├─ D456_1_link              ← 可选相机
       ├─ D456_2_link              ← 可选相机
       ├─ D405_1_link              ← 可选相机
       ├─ D405_2_link              ← 可选相机
       ├─ orbbec_gemini_1_link     ← 可选相机
       ├─ orbbec_gemini_2_link     ← 可选相机
       └─ orbbec_gemini_3_link     ← 可选相机
```

---

## 4. 与 lio_interface 的关系

`lio_interface` 初始化时会查询这里定义的 TF：

```cpp
// lio_interface 初始化时执行
tf_buffer_->lookupTransform("base_footprint", "livox_frame", ...);
//                          ↑ 从本 URDF 的静态链获取  ↑
```

这个查询返回 `base_footprint → livox_frame` 的空间偏移，`lio_interface` 用它将 `camera_init` 对齐到 `odom`。

```
本 URDF 定义的静态 TF                 lio_interface 使用
─────────────────────                 ─────────────────
base_footprint → chassis              初始化时一次性读取:
  → livox_frame                        tf_odom_to_lidar_odom_ = T_{base_footprint}^{livox_frame}
```

**所以 `chassis_to_livox_frame` 的 `origin` 值直接影响整个里程计的对齐精度。** 实机部署时这个值必须准确测量。

---

## 5. 精简版 URDF（仅 LiDAR + IMU）

如果你的机器人只有一个 LiDAR + IMU，没有相机，建议用精简版：

```xml
<?xml version="1.0"?>
<robot name="gld_robot_description">

  <link name="base_footprint" />
  <link name="chassis" />
  <link name="livox_frame" />

  <joint name="base_to_chassis" type="fixed">
    <parent link="base_footprint"/>
    <child link="chassis"/>
    <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
  </joint>

  <joint name="chassis_to_livox_frame" type="fixed">
    <parent link="chassis"/>
    <child link="livox_frame"/>
    <origin xyz="0.15 0.0 0.45" rpy="0 0 0"/>   <!-- ← 实测后填入真实值 -->
  </joint>

</robot>
```

精简后 TF 树只有三个节点，和仿真 `simple_car.urdf` 结构一致：

```
base_footprint → chassis → livox_frame
```

---

## 6. 实机部署检查清单

| 步骤 | 做什么 |
|---|---|
| ① 测量外参 | CAD 或卷尺测量 `chassis → livox_frame` 的 xyz 和 rpy |
| ② 填入 URDF | 把测量值写入 `chassis_to_livox_frame` 的 `origin` |
| ③ 删除多余 link | 如果只有 LiDAR，删除所有相机 link |
| ④ 测量 `base_to_chassis` | 如果有离地高度，填入 `base_to_chassis` 的 z |
| ⑤ 编译 | `colcon build --packages-select gld_robot_description` |
| ⑥ 启动 | `ros2 launch gld_robot_description gld_robot_description_launch.py` |

---

## 7. 启动

```bash
ros2 launch gld_robot_description gld_robot_description_launch.py
```

只启动 `robot_state_publisher` + RViz2，不包含任何驱动或算法。
