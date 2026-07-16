# FAST-LIO 性能分析指南

## 1. 内置分段计时（首选）

FAST-LIO 在 `laserMapping.cpp` 中已内置逐帧分段计时，无需外部工具。

### 1.1 开启

在配置文件中加一行：

```yaml
# src/localization/FAST_LIO_ROBOAIRY/config/robosenseAiry.yaml
runtime_pos_log_enable: true
```

### 1.2 计时覆盖的处理管线

```
timer_callback() @ 100Hz
    │
t0  ▼
    ├── p_imu->Process()              // IMU 前向传播 + 点云去畸变
    ├── lasermap_fov_segment()        // 局部地图滑动窗口
    │       └── ikdtree.Delete_Point_Boxes()  → kdtree_delete_time
    ├── downSizeFilterSurf.filter()   // 体素降采样
t1  ▼
    ├── 初始化 Nearest_Points
t2  ▼
    ├── kf.update_iterated_dyn_share_modified()   // IEKF 迭代更新
    │       └── h_share_model()
    │               ├── ikdtree.Nearest_Search()   → kdtree_search_time
    │               ├── esti_plane()               → match_time
    │               └── 雅可比矩阵 H 构造            → solve_time
    ├── publish_odometry()            // pub /Odometry + TF broadcast
t3  ▼
    ├── map_incremental()
    │       └── ikdtree.Add_Points()  → kdtree_incremental_time
t5  ▼
    ├── publish_frame_world()         // pcl::toROSMsg + pub->publish  ← 背压敏感点!
    ├── publish_frame_body()
    ├── publish_path()
    └── publish_effect_world()
```

### 1.3 终端实时输出

每帧一行：

```
[ mapping ]: time: IMU + Map + Input Downsample: 0.001234 ave match: 0.005678 ave solve: 0.000123  ave ICP: 0.008901  map incre: 0.002345 ave total: 0.012345 icp: 0.006789 construct H: 0.000123
```

| 字段 | 含义 | 对应变量 |
|------|------|----------|
| `IMU + Map + Input Downsample` | IMU 处理 + 地图 FOV + 降采样 | `t1 - t0` |
| `ave match` | 最近邻搜索 + 平面拟合（平均） | `match_time` |
| `ave solve` | 雅可比矩阵构造（平均） | `solve_time + solve_H_time` |
| `ave ICP` | IEKF 更新总耗时（平均） | `t_update_end - t_update_start` |
| `map incre` | KD-Tree 插入新点 | `kdtree_incremental_time` |
| `ave total` | 单帧总耗时（平均） | `t5 - t0` |
| `icp` | ICP 阶段（不含前后处理） | `t3 - t1` |
| `construct H` | 纯 H 矩阵构造时间（平均） | `solve_time` |

### 1.4 退出时 CSV 日志

文件路径：`Log/fast_lio_time_log.csv`

```csv
time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time
```

用 Python/pandas 快速分析：

```python
import pandas as pd
df = pd.read_csv("Log/fast_lio_time_log.csv")
print(df.describe())
df[["total time", "incremental time", "search time"]].plot()
```

---

## 2. perf 外部采样

内置计时确认瓶颈后，用 perf 深挖 CPU 微架构原因。

### 2.1 找到进程 PID

```bash
pgrep -f fastlio_mapping
```

### 2.2 perf stat — 宏观指标

```bash
PID=$(pgrep -f fastlio_mapping)
sudo perf stat -p $PID -- sleep 30
```

关键指标：

| 指标 | 含义 | 健康值 |
|------|------|--------|
| `instructions` / `cycles` (IPC) | 每周期执行指令数 | > 1.0 |
| `cache-misses` % | 缓存未命中率 | < 3% |
| `branch-misses` % | 分支预测错误率 | < 2% |
| `cpu-migrations` | 进程跨核心迁移次数 | 应接近 0 |

### 2.3 perf record — 采样调用栈

```bash
PID=$(pgrep -f fastlio_mapping)

# -F 99: 99Hz 采样 (不与 100Hz timer 共振)
# -g: 记录调用栈
sudo perf record -g -F 99 -p $PID -o /tmp/fast_lio.data -- sleep 30
```

### 2.4 perf report — 交互式查看

```bash
sudo perf report -i /tmp/fast_lio.data
```

快捷键：`/` 搜索，`Enter` 展开调用栈，`q` 退出。

### 2.5 火焰图

```bash
sudo perf script -i /tmp/fast_lio.data > /tmp/fast_lio.perf

git clone https://github.com/brendangregg/FlameGraph.git /tmp/FlameGraph
/tmp/FlameGraph/stackcollapse-perf.pl /tmp/fast_lio.perf | \
  /tmp/FlameGraph/flamegraph.pl > /tmp/fast_lio_flame.svg
```

用浏览器打开 SVG，宽度代表 CPU 占比。

### 2.6 perf top — 实时看热点

```bash
sudo perf top -p $(pgrep -f fastlio_mapping)
```

---

## 3. 背压检测

当怀疑 RELIABLE QoS 导致 publish 阻塞时，关注 `publish_frame_world()` 的耗时。

### 3.1 粗略判断

内置计时中，`total time` 减去所有显式阶段之和 = publish 耗时。若加载 `lio_interface` 后这个差值显著变大，即为背压。

### 3.2 perf probe 精确测量

```bash
sudo perf probe -x /path/to/fastlio_mapping \
  --add publish_start='publish_frame_world'
sudo perf probe -x /path/to/fastlio_mapping \
  --add publish_end='publish_frame_world%return'

sudo perf record -e probe:fastlio_mapping:publish_start \
                 -e probe:fastlio_mapping:publish_end \
                 -p $PID -- sleep 30

sudo perf script

# 清理
sudo perf probe -d publish_start
sudo perf probe -d publish_end
```

---

## 4. 快速诊断流程

```bash
# Step 1: 开启内置计时（改 yaml 后重启），观察各阶段耗时
#   total time > 30ms → 有瓶颈（100Hz 下每帧预算 10ms）

# Step 2: 定位瓶颈阶段
#   IMU + downsample 高 → 点云太大，调大 filter_size_surf
#   match/solve 高    → KD-Tree 搜索慢，调大 filter_size_map
#   map incre 高      → KD-Tree 插入慢，同上
#   publish 阶段高    → 背压，改 BEST_EFFORT QoS

# Step 3: 微架构分析
sudo perf stat -p $(pgrep -f fastlio_mapping) -- sleep 30

# Step 4: 火焰图定位具体函数
sudo perf record -g -F 99 -p $(pgrep -f fastlio_mapping) -- sleep 30
sudo perf report
```
