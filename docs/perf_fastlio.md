# FAST-LIO Perf 火焰图

用 `perf` 采样 FAST-LIO 的 CPU 调用栈，生成火焰图分析性能热点。

---

## 1. 环境准备

```bash
# 安装 perf（容器内）
docker exec lio_nav2 apt-get update
docker exec lio_nav2 apt-get install -y linux-tools-$(uname -r)

# 安装 FlameGraph 工具（容器内）
docker exec lio_nav2 git clone --depth 1 https://github.com/brendangregg/FlameGraph.git /tmp/FlameGraph

# 确认容器以 privileged 模式启动（perf 需要）
docker rm -f lio_nav2
docker run -d --name lio_nav2 --privileged \
  --network host --ipc host --cpus 8 \
  -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/dri:/dev/dri \
  -v /home/ros/ros_ws/LIO_Nav2_ROS2:/ws \
  -v /home/ros/dataset:/dataset \
  lio_nav2:humble sleep infinity

# 以 RelWithDebInfo 编译 FAST-LIO（带调试符号）
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash && cd /ws && \
  MAKEFLAGS='-j4' colcon build --symlink-install --executor sequential \
    --packages-select fast_lio fast_lio_robosense \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_SYSTEM_TBB=ON"
```

## 2. 启动建图

```bash
# 终端 1: 启动 FAST-LIO 建图
docker exec -it lio_nav2 /ws/scripts/robo_mapping_real_docker.sh
```

等 FAST-LIO 开始处理数据（`tmux attach -t robo_mapping` 查看，出现 `Initialize the map kdtree` 或有点云匹配日志即可）。

## 3. Perf 采样

```bash
# 终端 2: 采样 30 秒
docker exec lio_nav2 bash -c "
  PID=\$(pgrep fastlio_mapping | tail -1) && \
  echo \"PID=\$PID\" && \
  perf record -F 99 -g -p \$PID -o /tmp/perf.data -- sleep 30"

# 参数说明:
#   -F 99    每秒采样 99 次
#   -g       记录完整调用栈
#   -p PID   指定进程
#   -o FILE  输出文件
```

## 4. 生成火焰图

```bash
# 容器内生成
docker exec lio_nav2 bash -c "
  perf script -i /tmp/perf.data > /tmp/perf.out && \
  /tmp/FlameGraph/stackcollapse-perf.pl /tmp/perf.out > /tmp/perf.folded && \
  /tmp/FlameGraph/flamegraph.pl /tmp/perf.folded > /tmp/fastlio_flame.svg"

# 拷贝到宿主机
docker cp lio_nav2:/tmp/fastlio_flame.svg .
firefox fastlio_flame.svg
```

## 5. 查看文本报告（可选）

```bash
# 交互式
docker exec lio_nav2 perf report -i /tmp/perf.data

# 纯文本 Top 40
docker exec lio_nav2 perf report -i /tmp/perf.data --stdio | head -40
```

## 6. 火焰图阅读指南

| 特征 | 含义 |
|------|------|
| **横轴宽度** | 函数占 CPU 的比例，越宽越耗时 |
| **纵轴高度** | 调用栈深度，从下往上是 caller → callee |
| `ikdtree.Nearest_Search` | KD 树 5-最近邻搜索（通常最宽） |
| `esti_plane` | 平面拟合 + 点面残差 |
| `h_share_model` | IEKF 观测模型雅可比 |
| `downSizeFilterSurf.filter` | 体素降采样 |
| `sync_packages` | LiDAR/IMU 时间同步 |
| `imu forward propagation` | IMU 中值积分前向传播 |

## 7. 关闭

```bash
docker exec lio_nav2 tmux kill-session -t robo_mapping
```
