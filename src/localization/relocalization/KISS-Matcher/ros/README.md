<div align="center">
    <h1>KISS-Matcher-SAM</h1>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <br />
    <br />
  <br />
  <br />
  <p align="center"><img src="https://github.com/user-attachments/assets/026d775e-b8f5-4905-aede-b0a720282c0e" alt="KISS Matcher" width="80%"/></p>
  <p><strong><em>LiDAR SLAM = Any LO/LIO + KISS-Matcher-SAM</em></strong></p>
</div>

______________________________________________________________________

## ROS2 KISS-Matcher-SAM

This repository provides LiDAR SLAM using KISS-Matcher.
If you just want to **perform registration using ROS2**, see [README_ROS2_REGISTRATION.md](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/ros/README_ROS2_REGISTRATION.md).

## :gear: How To Build & RUN

1. Put this repository in your colcon workspace, and then build this repository as follows:

```bash
cd ${YOUR_ROS2_WORKSPACE}/src
git clone https://github.com/MIT-SPARK/KISS-Matcher.git
cd ..
colcon build --packages-select kiss_matcher_ros
```

And then source your workspace using `source install/setup.bash` or `source install/setup.zsh` depending on your shell.

2. Then, run the command below:

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml
```

If you want to run it on your own dataset, make sure to set the `/cloud` and `/odom` topics appropriately using:

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml \
  odom_topic:=<YOUR_TOPIC> scan_topic:=<YOUR_TOPIC>
```

______________________________________________________________________

## 🚀 Example demos

To run `KISS-Matcher-SAM`, you need to need an external LiDAR(-inertial) odometry. We use [SPARK-FAST-LIO](https://github.com/MIT-SPARK/spark-fast-lio) as an example.
We provide **two out-of-the-box ROS2** examples using pre-processed ROS2 bag data (because the original data are only available in ROS1).
All pre-processed ROS2 bag files can be found [**here**](https://www.dropbox.com/scl/fo/i56kucdzxpzq1mr5jula7/ALJpdqvOZT1hTaQXEePCvyI?rlkey=y5bvslyazf09erko7gl0aylll&st=dh91zyho&dl=0).

### 🇺🇸 SLAM on the MIT campus

1. Download `10_14_acl_jackal` and `10_14_hathor` from our [Dropbox](https://www.dropbox.com/scl/fo/i56kucdzxpzq1mr5jula7/ALJpdqvOZT1hTaQXEePCvyI?rlkey=y5bvslyazf09erko7gl0aylll&st=dh91zyho&dl=0).

1. Build and run `spark_fast_lio` with the following commands:

**To build:**

```shell
cd ${YOUR_COLCON_WORKSPACE}/src
git clone https://github.com/MIT-SPARK/spark-fast-lio.git
colcon build --packages-up-to spark_fast_lio
```

**To run:**

```
ros2 launch spark_fast_lio mapping_mit_campus.launch.yaml scene_id:=acl_jackal
```

3. Adjust the parameters of `kiss_matcher_ros` as below (see [Issue #47](https://github.com/MIT-SPARK/KISS-Matcher/issues/47).
   This is necessary because the scan was acquired using a VLP-16, which is relatively sparse:

![Image](https://github.com/user-attachments/assets/824d4686-a897-4261-9eef-9662f16622a4)

and run

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml
```

4. In another terminal, play the ROS 2 bag file:

```
ros2 bag play 10_14_acl_jackal
```

#### Qualitative mapping result

![](https://github.com/user-attachments/assets/bd24f054-9818-426c-814c-4422e438727a)

### 🏟️ SLAM on the Colosseum

1. Download `colosse_train0` from our [Dropbox](https://www.dropbox.com/scl/fo/i56kucdzxpzq1mr5jula7/ALJpdqvOZT1hTaQXEePCvyI?rlkey=y5bvslyazf09erko7gl0aylll&st=dh91zyho&dl=0).

1. Follow the same steps as above to run spark_fast_lio and kiss_matcher_ros.
   *No parameter tuning is needed for this dataset*.

```
ros2 launch spark_fast_lio mapping_vbr_colosseo.launch.yaml
```

(In another terminal)

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml
```

3. Finally, play the bag file:

```
ros2 bag play colosseo_train0
```

## How to Tune Parameters?

More details can be found in [config/slam_config.yaml](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/ros/config/slam_config.yaml).
