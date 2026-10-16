<div align="center">
    <h1>KISS-Matcher</h1>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
  <p><strong><em>保持简单，使其可扩展。</em></strong></p>
</div>

______________________________________________________________________

## KISS-Matcher 配准示例

1. 如果你只需要下载示例数据集，请使用 [download_datasets.py](https://github.com/kimdaebeom/KISS-Matcher/blob/main/python/utils/download_datasets.py)。

2. 然后，使用以下命令启动可视化：

```bash
ros2 launch kiss_matcher_ros visualizer_launch.py
```

## 配置

你可以在启动节点之前在 **`config/params.yaml`** 中自定义可视化参数。

### 配置示例

```yaml
registration_visualizer:
  ros__parameters:
    base_dir: "src/KISS-Matcher/cpp/examples/build/data/"

    # 指定源和目标 PCD 文件
    src_pcd_path: "Vel64/kitti_000540.pcd"
    tgt_pcd_path: "Vel64/kitti_001319.pcd"

    # 配准设置
    resolution: 0.2  # 体素网格分辨率
    moving_rate: 200.0  # 动画步数
    frame_rate: 30.0  # 动画帧率
    scale_factor: 1.0  # 点云缩放因子
```

### 参数说明

| 参数 | 描述 |
|------|------|
| `src_pcd_path` | 源 PCD 文件路径 |
| `tgt_pcd_path` | 目标 PCD 文件路径 |
| `resolution` | 体素网格降采样分辨率 |
| `moving_rate` | 过渡动画步数 |
| `frame_rate` | RViz 中动画的帧率 |
| `scale_factor` | 应用于点云的缩放因子 |

______________________________________________________________________

## 注意事项

- 确保你的 **PCD 文件** 存在于 `params.yaml` 中指定的正确目录下。
- 如果可视化无法启动，请验证 **PCL 和 ROS2 依赖** 是否已正确安装。
- 修改 **帧率（`frame_rate`）和动画步数（`moving_rate`）** 以获得更平滑的可视化效果。

现在，你可以在 ROS2 中可视化 **KISS-Matcher 配准结果** 了！🎉
