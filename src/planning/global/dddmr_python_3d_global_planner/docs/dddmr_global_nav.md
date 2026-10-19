# dddmr_global_nav 使用方法与注意事项

本文按当前代码和 2026-10-18 的地图测试结果编写。

已有先验点云、重定位节点和局部规划器时，接入流程见
[global_local_reloc.md](global_local_reloc.md)。

## 1. 功能与名称

输入静态三维点云地图和三维起终点，输出地面机器人全局路径。处理流程：

```text
点云 → 体素降采样 → 法向量/可站立面提取 → 净空与障碍代价
     → 可通行邻接图 → 连通性检查/A* → 拉直、重采样、平滑 → 三维路径
```

| 名称 | 当前值 |
|---|---|
| 项目目录 | `/home/ros/ros_ws/pct_planner/src/dddmr_python_3d_global_planner` |
| Python 模块 | `dddmr_py` |
| ROS2 功能包 | `dddmr_ros2` |
| ROS2 可执行文件 | `planner_node` |
| ROS2 节点 | `dddmr_planner` |
| 默认输出话题 | `/init_path`，类型 `nav_msgs/msg/Path` |

本项目负责全局规划；定位、地图累积、实时动态避障、底盘控制和实际爬楼能力需要其他模块。
网页版的“规划成功”表示求得路径，不会控制实体机器人沿路径运动。

## 2. 当前目录结构

```text
dddmr_python_3d_global_planner/
├── package.xml / setup.py / setup.cfg    ROS2 ament_python 包描述
├── dddmr_py/
│   ├── dddmr_py/                        Python 规划核心、CLI、网页服务
│   ├── examples/config_example.yaml     通用示例参数
│   └── examples/config_map.yaml         map.pcd 演示参数
├── dddmr_ros2/                          ROS2 节点和消息转换
├── launch/planner.launch.py
├── config/planner.yaml                  ROS2 通用参数
├── scenepcd/                            map、building、luoxuan、demo_map 点云
├── docs/                                本文档及 ROS2 地图演示配置
└── test/                                ROS 消息转换与话题联调
```

**PCD 已放在 `scenepcd/`。** 从 `dddmr_py/` 运行时，应使用
`../scenepcd/map.pcd`；早期命令中的 `../map.pcd` 已不对应当前目录布局。

地图坐标单位使用米，z 轴向上。支持 PCD v0.7 的 ASCII、binary、binary_compressed。
点云需要包含有效的 x/y/z；程序不会根据文件名自动选择场景参数。

## 3. 安装与确认 Python 环境

独立 Python 使用 NumPy、SciPy；Numba 为可选加速依赖，PyYAML 用于完整 YAML 解析。
Python 版本要求不低于 3.9；本机 ROS2 Humble 使用 Python 3.10。

```bash
cd /home/ros/ros_ws/pct_planner/src/dddmr_python_3d_global_planner/dddmr_py
python3 -m pip install --user --no-build-isolation -e ".[fast,yaml]"
python3 -m dddmr_py --help
```

无法安装可选依赖时，可先安装基础版本：

```bash
python3 -m pip install --user --no-build-isolation -e .
```

`--no-build-isolation` 使用当前环境的构建工具，针对之前遇到的隔离构建环境缺少
`build_editable` hook 的问题。当前环境仍需具备支持可编辑安装的 setuptools（项目要求 >=64）。
`Defaulting to user installation` 只是用户目录安装提示，不是安装失败原因。

如果导入版本和预期不一致，检查：

```bash
python3 -m pip --version
python3 -c "import dddmr_py; print(dddmr_py.__file__)"
```

source ROS2 工作空间后，Python 可能优先导入 `install/` 下的构建副本。
修改源码后应重建工作空间，并重启服务或节点；单纯刷新浏览器不会重新加载 Python 代码。

## 4. 网页交互规划

### 4.1 启动 map.pcd 演示

```bash
cd /home/ros/ros_ws/pct_planner/src/dddmr_python_3d_global_planner/dddmr_py
python3 -m dddmr_py serve ../scenepcd/map.pcd \
  --start -5.46 7.45 0.06 \
  --port 8000 \
  --config examples/config_map.yaml
```

在本机浏览器打开 <http://localhost:8000>。初次启动会先处理地图，等终端提示服务已启动后再访问。
`0.0.0.0` 是监听地址，本机访问使用 `localhost` 或 `127.0.0.1`。
另一个电脑访问时使用运行服务的电脑 IP 和端口。

操作：

1. 左键拖动旋转，右键拖动或 Shift+拖动平移，滚轮缩放。
2. “设定起点”后单击地图，更新起点。
3. “设定目标点”后单击地图，触发规划。
4. 蓝色白边标记为起点，黄色白边标记为终点，均带文字标签。
5. 粉色路径宽度为 6 个 CSS 像素，带深色描边，缩放后仍保持屏幕粗细。
6. “重置视角”恢复默认相机位置。按 Ctrl+C 停止服务。

网页单击会选取鼠标附近的地面候选点。多层点云投影重叠时可能选到另一层，应旋转视角、
放大地图，并核对左侧显示的 XYZ，尤其是 z。局部点是绿色不代表它与当前起点连通。

路径和标记作为屏幕覆盖层显示，可能透过遮挡物可见；是否可通行应以规划器的地图判定为准。
网页展示会对子集点云抽样，不能从屏幕点间距直接推断真实地图是否断裂。

修改 YAML、Python 源码或地图后：**停止服务 → 重新启动 → Ctrl+Shift+R 强制刷新网页**。
已有的离线 HTML 是静态快照，需要重新生成。

### 4.2 从另一个终端提交目标

```bash
cd /home/ros/ros_ws/pct_planner/src/dddmr_python_3d_global_planner/dddmr_py
python3 -m dddmr_py goal 8.76 0.94 2.38 \
  --start -5.46 7.45 0.06 --port 8000 --out path.csv
```

通过 `goal` 命令省略 `--start` 时，服务使用内部当前位姿；成功规划后内部位姿会更新到路径终点。
网页每次请求显式提交面板中的起终点，因此连续点击时以页面起点为准。
从另一终端提交目标后，已经打开的网页不会自动轮询更新；刷新页面查看服务最新路径。

## 5. 离线命令行与 Python API

### 5.1 规划并导出

```bash
cd /home/ros/ros_ws/pct_planner/src/dddmr_python_3d_global_planner/dddmr_py
python3 -m dddmr_py plan ../scenepcd/map.pcd \
  --start -5.46 7.45 0.06 --goal 8.76 0.94 2.38 \
  --config examples/config_map.yaml \
  --out path.csv --json path.json --viz path.html
```

CSV 为 `x,y,z,yaw`，JSON 包含路径与规划统计；HTML 可用浏览器查看。
离线 HTML 不提供在线重新规划，改变起终点后需要重新执行命令。

查看地图统计或生成地图预览：

```bash
python3 -m dddmr_py info ../scenepcd/map.pcd \
  --config examples/config_map.yaml --viz map_debug.html
```

这里的 `info` 也会预处理地图，并非只读取 PCD 文件头。
不使用外部 PCD 的基础演示：

```bash
python3 -m dddmr_py plan demo --start 1 1 0 --goal 21 10 3 \
  --out demo_path.csv --viz demo_path.html
```

### 5.2 Python API

以下示例从项目的 `dddmr_py/` 目录执行：

```python
from dddmr_py import GlobalPlanner3D, PlannerConfig

cfg = PlannerConfig.from_yaml("examples/config_map.yaml")
planner = GlobalPlanner3D.from_pcd("../scenepcd/map.pcd", cfg, verbose=True)
path = planner.make_plan(start=(-5.46, 7.45, 0.06), goal=(8.76, 0.94, 2.38))
if path.success:
    path.to_csv("path.csv")
    print(path.length, path.poses)  # poses: N×4，[x,y,z,yaw]
else:
    print(path.message)
```

## 6. 地图参数与调参原则

独立 Python 模式使用扁平 YAML，参数名直接位于文件顶层。
CLI 参数会覆盖同名 YAML 参数，例如 `--robot-height 0.8`。
`connection_radius`、`max_roughness` 等没有独立 CLI 开关的参数，通过 YAML 设置。

| 参数 | 含义 | 当前 map.pcd 演示值 |
|---|---|---:|
| `robot_radius` | 圆柱碰撞模型外接半径，影响可穿过通道宽度 | 0.35 m |
| `robot_height` | 模型高度，影响净空和本体碰撞 | 0.80 m |
| `max_step` | 台阶/相邻高差相关约束，也参与净空查询下界 | 0.25 m |
| `max_slope_deg` | 局部表面及连接坡度相关约束 | 25° |
| `max_roughness` | 局部平面拟合粗糙度阈值 | 0.10 |
| `voxel_size` | 体素降采样尺寸，影响精度、速度和内存 | 0.10 m |
| `normal_k` | 法向量估计近邻数 | 16 |
| `connection_radius` | 图连接候选的三维距离上限 | 0.40 m |
| `max_neighbors` | 每个节点筛选的近邻数 | 12 |
| `inflation_radius` | 障碍代价膨胀范围 | 0.55 m |
| `snap_radius` | 起终点吸附搜索半径 | 1.50 m |
| `edge_collision_check` | 检查连接沿线碰撞 | true |
| `path_resolution` | 输出重采样间隔 | 0.10 m |

机器人尺寸是人为配置，**不会从 PCD 自动识别**。当前演示值并非实体机器人测量值。
真实机器人需要按尺寸和爬坡/跨阶能力设置参数。

关键区别：

- `connection_radius <= 0` 会自动使用 `2.6 * voxel_size`。分辨率 0.10 m 时只有 0.26 m。
- 连接距离是三维欧氏距离，台阶的水平间距和垂直高差共同决定是否超过半径。
- 增大半径只是扩大连接候选范围，仍须通过坡度、台阶、邻居筛选和沿线碰撞判定。
- `max_step` 不等价于“保证能爬这么高的所有楼梯”；法向量估计、净空和机器人半径也会影响结果。
- 改变地图分辨率会同时影响采样、法向量和自动连接范围，不能只视为速度开关。
- 膨胀半径通常应大于机器人半径；按实际通道宽度和机器人尺寸综合设置。

为不同地图维护独立配置，例如 `config_map.yaml`、`config_building.yaml`。
更换 PCD 不会自动加载对应文件，必须显式传 `--config`。

缓存可通过 `--cache-dir .dddmr_cache` 启用，用于复用同一地图的预处理结果。
若调整配置后结果与预期不一致，可先不传缓存目录进行对比，再重新生成缓存。
`--compat` 和 `--no-collision-check` 会关闭碰撞增强，只用于复现实验，不能用来证明路径物理可行。

## 7. ROS2 接入：地图话题 → /init_path

### 7.1 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/ros/ros_ws/pct_planner
colcon build --packages-select dddmr_ros2 --symlink-install
source install/setup.bash
ros2 pkg executables dddmr_ros2
```

功能包位于本项目根目录，构建时同时安装 `dddmr_py` 规划核心和 ROS 包装层。
ROS2 包依赖 rclpy、sensor_msgs_py、geometry_msgs、nav_msgs、tf2_ros 和 launch_ros 等。
修改源码或安装的数据文件后需要重新构建；修改外部传入的参数文件后重启节点即可。

### 7.2 启动

通用默认配置：

```bash
ros2 launch dddmr_ros2 planner.launch.py map_topic:=/global_points
```

**ROS2 默认参数与网页 map.pcd 演示参数不同。** 默认 `config/planner.yaml` 的高度是 1.20 m，
并未自动继承 `examples/config_map.yaml`。要复用地图演示参数，请使用随本文提供的 ROS2 格式配置：

```bash
ros2 launch dddmr_ros2 planner.launch.py \
  map_topic:=/global_points path_topic:=/init_path \
  params_file:=/home/ros/ros_ws/pct_planner/src/dddmr_python_3d_global_planner/docs/map_ros2_example.yaml
```

该配置仍然接收上游地图话题；节点本身不会因为参数文件名含 map 就读取 PCD，
也不提供 PCD 发布器。上游须将完整全局地图发布为 PointCloud2。
网页 `serve` 和 ROS2 节点是两个独立运行入口，启动网页服务不会同时发布 `/init_path`。

### 7.3 话题与起终点

| 方向 | 话题 | 消息类型 | 行为 |
|---|---|---|---|
| 输入 | `/global_points` | `sensor_msgs/msg/PointCloud2` | 完整全局点云，含 x/y/z 和非空 frame_id |
| 输入 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 发送三维目标，触发规划 |
| 输入 | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 更新起点并对现有目标重规划 |
| 输入 | `odom_topic` 参数指定 | `nav_msgs/msg/Odometry` | 更新下一次规划的起点，不按里程计频率重规划 |
| 输出 | `/init_path` | `nav_msgs/msg/Path` | 三维位置及地面法向量对齐的四元数姿态 |

没有起点消息时使用 `start` 参数；显式起点和里程计按最后收到的消息更新起点。
目标早于地图到达时会排队，建图完成后再规划。

发送这次已验证的目标（这里假设地图 frame_id 为 `map`，否则改成实际 frame）：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 8.76, y: 0.94, z: 2.38}, orientation: {w: 1.0}}}"
```

RViz2 添加 Path，话题选 `/init_path`，Fixed Frame 设置为地图 frame_id。
2D Goal Pose / 2D Pose Estimate 的 z 通常为 0；跨楼层时通过话题发送正确的三维坐标。
不同 frame 的起终点通过最新 TF 转到地图 frame。TF 不可用时会报错并输出空路径，
需要 TF 就绪后重新发送目标。输入终点姿态不约束最终航向。

### 7.4 地图更新、QoS、后台任务

- 输入必须是完整全局点云；节点不会累积逐帧激光点云。地图 z 轴须向上。
- 默认 `update_map: false` 仅处理首张接受的地图；建图失败时允许再次接收。
- `update_map: true` 每条地图触发重建，并对已有目标重规划。高频重发会持续使结果失效。
- 地图订阅默认 Best Effort + Volatile，可接收常见实时可靠/尽力发送数据流。
- 对一次性、保留历史地图的发布器，发布器和订阅端都需要 Transient Local；
  本节点通过 `map_transient_local: true` 设置。Volatile 发布器无法匹配该订阅模式。
- `/init_path` 为 Reliable + Transient Local，depth=1；匹配的晚加入订阅者可获取最后结果。
- 新目标、新显式起点或新地图会清空旧路径；失败同样发布空 Path。
- 建图与搜索在后台单 worker 串行执行；新目标或地图到达后旧规划结果会被丢弃。
- 规划参数在启动时读取，修改后重启。ROS2 点云输入不会使用 PCD 文件缓存。

检查接口：

```bash
ros2 topic info /global_points --verbose
ros2 topic info /init_path --verbose
ros2 topic echo /init_path --once
ros2 param get /dddmr_planner planner.robot_height
ros2 param get /dddmr_planner planner.connection_radius
```

ROS2 YAML 必须带节点和参数层级，例如：

```yaml
dddmr_planner:
  ros__parameters:
    start: [-5.46, 7.45, 0.06]
    planner:
      robot_radius: 0.35
      robot_height: 0.80
      connection_radius: 0.40
```

不要直接把扁平的 CLI YAML 作为 ROS2 参数文件，或把 ROS2 YAML 传给 CLI 的 `--config`。

## 8. map.pcd 已验证结果与边界

以下为当前地图演示配置下、起点 `(-5.46, 7.45, 0.06)` 的实测结果。
坐标单位为米，路径长度为近似值，代码、配置或地图改变后应重新验证。

| 终点 | 结果 | 路径长度 |
|---|---|---:|
| `(-5.36, 0.25, 1.01)` | 成功 | 7.58 m |
| `(6.26, -3.96, 1.56)` | 成功 | 22.96 m |
| `(8.76, 0.94, 2.38)` | 成功 | 30.04 m |
| `(-0.43, 3.57, 6.06)` | 不连通，仍未解决 | — |

排查中已经确认的不同原因：

1. 通用示例的 `max_step=0.12`、`max_roughness=0.05` 下，前段楼梯不连通；
   同时调整为 0.25 和 0.10 后，第一组点可达。该对比不单独证明其中哪个参数是唯一原因。
2. 机器人高度 1.20 m 时，某关键通道内约高出地面 1.0～1.2 m 的点云触发碰撞；
   改为演示高度 0.80 m 后，第二组点可达。
3. 二楼到三楼相关路径存在长度约 0.284 m、0.321 m 的必要图连接；
   原自动半径 0.26 m 会截断，显式改为 0.40 m 后第三组点可达，沿线碰撞检查保持开启。

这些结果不代表整栋楼任意两点都可达，也不代表对应实体机器人能爬这些楼梯。
当前 ROS2 联调只验证过合成平面地图的点云输入和路径输出；上述建筑路径是 Python 核心测试结果。

## 9. 常见现象与排查顺序

| 现象 | 含义或优先检查项 |
|---|---|
| 找不到 PCD 文件 | 确认当前工作目录，当前点云位于 `scenepcd/` |
| `build_editable` hook 报错 | 确认构建工具版本，使用本机 setuptools 的 `--no-build-isolation` 安装方式 |
| `No module named rclpy` | 先 source `/opt/ros/humble/setup.bash`，使用对应 Python |
| `Package 'dddmr_ros2' not found` | 先构建，再在当前终端 source 工作空间 `install/setup.bash` |
| 起点/终点附近没有可通行节点 | 坐标、楼层、机器人尺寸、候选点致命标记、吸附半径 |
| 起终点不连通 | 图的连通区域不同，需要检查中间通道，不能只看端点是绿色 |
| 规划耗时显示 0 ms | 连通性预检查可能直接失败，页面也可能将极短耗时取整；不代表未收到请求 |
| 建图耗时很长 | 地图规模、分辨率、法向量查询、首次 Numba 编译；建图时间不等于单次搜索时间 |
| 路径长度 0 m 且无粉线 | 先查看是否规划失败；失败时不会保留旧路径 |
| 修改参数无变化 | 重启服务/节点，确认加载的参数文件和 Python 模块路径，排除旧缓存或构建副本 |
| 目标点落在错误楼层 | 旋转/缩放视角，核对目标 z，或直接提交三维坐标 |
| 网页目标提交成功但 ROS 没路径 | 网页 HTTP 服务与 ROS2 节点是独立入口 |
| ROS2 收不到地图 | 话题名、消息类型、发布时机、QoS、ROS_DOMAIN_ID 和网络环境 |
| ROS2 有空 Path | 节点主动清理旧路径，查看起点/目标/TF/地图处理日志 |

建议固定一组起终点，先离线复现，再一次只调整一类因素，记录修改前后结果。
不要靠不断缩小机器人或关闭碰撞检查来掩盖地图/建图问题。

## 10. 自检与回归

独立规划核心测试：

```bash
cd /home/ros/ros_ws/pct_planner/src/dddmr_python_3d_global_planner/dddmr_py
python3 tests/test_planner.py
```

ROS2 消息转换测试（禁用本机不兼容的外部 pytest 插件自动加载）：

```bash
source /opt/ros/humble/setup.bash
cd /home/ros/ros_ws/pct_planner
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD/src/dddmr_python_3d_global_planner:$PWD/src/dddmr_python_3d_global_planner/dddmr_py:$PYTHONPATH" \
  python3 -m pytest src/dddmr_python_3d_global_planner/test -q
```

构建并 source 工作空间后的话题联调自检：

```bash
source /home/ros/ros_ws/pct_planner/install/setup.bash
cd /home/ros/ros_ws/pct_planner
ROS_DOMAIN_ID=87 python3 src/dddmr_python_3d_global_planner/test/smoke_topics.py
```

该自检发布合成平面地图，验证目标先到、后台建图、`/init_path` 输出和不可达目标清空路径。
测试时使用独立 ROS_DOMAIN_ID，避免与正在运行的业务节点混在一起。
