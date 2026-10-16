<div align="center">
    <h1>KISS-Matcher-SAM</h1>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
  <p><strong><em>LiDAR SLAM = 任意 LO/LIO + KISS-Matcher-SAM</em></strong></p>
</div>

______________________________________________________________________

## ROS2 KISS-Matcher-SAM

本仓库提供使用 KISS-Matcher 的 LiDAR SLAM 功能。如果你只想 **使用 ROS2 进行点云配准**，请参阅 [README_ROS2_REGISTRATION_CN.md](README_ROS2_REGISTRATION_CN.md)。

## :gear: 如何构建与运行

1. 将本仓库放入你的 colcon 工作空间，然后按如下方式构建：

```bash
cd ${YOUR_ROS2_WORKSPACE}/src
git clone https://github.com/MIT-SPARK/KISS-Matcher.git
cd ..
colcon build --packages-select kiss_matcher_ros
```

然后根据你的 shell 类型，使用 `source install/setup.bash` 或 `source install/setup.zsh` 来加载工作空间。

2. 运行以下命令：

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml
```

如果你想在自己的数据集上运行，请确保使用以下方式正确设置 `/cloud` 和 `/odom` 话题：

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml \
  odom_topic:=<YOUR_TOPIC> scan_topic:=<YOUR_TOPIC>
```

______________________________________________________________________

## 示例演示

要运行 `KISS-Matcher-SAM`，你需要一个外部的 LiDAR（惯性）里程计。我们使用 [SPARK-FAST-LIO](https://github.com/MIT-SPARK/spark-fast-lio) 作为示例。我们提供了 **两个开箱即用的 ROS2** 示例，使用预处理的 ROS2 bag 数据（因为原始数据仅以 ROS1 格式提供）。所有预处理的 ROS2 bag 文件可以在 [**这里**](https://www.dropbox.com/scl/fo/i56kucdzxpzq1mr5jula7/ALJpdqvOZT1hTaQXEePCvyI?rlkey=y5bvslyazf09erko7gl0aylll&st=dh91zyho&dl=0) 找到。

### MIT 校园 SLAM

1. 从我们的 [Dropbox](https://www.dropbox.com/scl/fo/i56kucdzxpzq1mr5jula7/ALJpdqvOZT1hTaQXEePCvyI?rlkey=y5bvslyazf09erko7gl0aylll&st=dh91zyho&dl=0) 下载 `10_14_acl_jackal` 和 `10_14_hathor`。

2. 使用以下命令构建并运行 `spark_fast_lio`：

**构建：**

```shell
cd ${YOUR_COLCON_WORKSPACE}/src
git clone https://github.com/MIT-SPARK/spark-fast-lio.git
colcon build --packages-up-to spark_fast_lio
```

**运行：**

```
ros2 launch spark_fast_lio mapping_mit_campus.launch.yaml scene_id:=acl_jackal
```

3. 按如下方式调整 `kiss_matcher_ros` 的参数（参见 [Issue #47](https://github.com/MIT-SPARK/KISS-Matcher/issues/47)）。这一步是必要的，因为扫描数据是使用 VLP-16 采集的，相对稀疏：

然后运行

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml
```

4. 在另一个终端中，播放 ROS 2 bag 文件：

```
ros2 bag play 10_14_acl_jackal
```

#### 建图定性结果

![](https://github.com/user-attachments/assets/bd24f054-9818-426c-814c-4422e438727a)

### 罗马斗兽场 SLAM

1. 从我们的 [Dropbox](https://www.dropbox.com/scl/fo/i56kucdzxpzq1mr5jula7/ALJpdqvOZT1hTaQXEePCvyI?rlkey=y5bvslyazf09erko7gl0aylll&st=dh91zyho&dl=0) 下载 `colosse_train0`。

2. 按照与上面相同的步骤运行 spark_fast_lio 和 kiss_matcher_ros。*此数据集无需调参*。

```
ros2 launch spark_fast_lio mapping_vbr_colosseo.launch.yaml
```

（在另一个终端中）

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml
```

3. 最后，播放 bag 文件：

```
ros2 bag play colosseo_train0
```

## 如何调参？

更多详情请参阅 [config/slam_config.yaml](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/ros/config/slam_config.yaml)。
