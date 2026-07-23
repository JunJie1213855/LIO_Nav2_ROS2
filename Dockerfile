# LIO_Nav2_ROS2 编译/运行环境镜像
# 构建:  docker build -t lio_nav2:humble .
# 编译:  docker run -it --network host -v $PWD:/ws lio_nav2:humble \
#          bash -c "source /opt/ros/humble/setup.bash && cd /ws && ./scripts/build.sh"
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# 基础工具 + 环境搭建.md 中列出的补装依赖 + Nav2/SLAM 相关 ROS 包
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget curl vim gnupg lsb-release software-properties-common \
    build-essential cmake ninja-build pkg-config \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    libtbb-dev libboost-all-dev qtbase5-dev qtbase5-private-dev python3-tk \
    libeigen3-dev libflann-dev libpcl-dev libomp-dev \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-octomap-server ros-humble-octomap-rviz-plugins \
    ros-humble-pointcloud-to-laserscan ros-humble-tf2-tools \
    ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-diagnostic-updater \
    ros-humble-gtsam ros-humble-tf-transformations \
    libapr1-dev libaprutil1-dev \
    ros-humble-ament-cmake-clang-format ros-humble-ament-cmake-clang-tidy \
    ros-humble-ament-cmake-black \
    tmux psmisc \
    && rm -rf /var/lib/apt/lists/*

# small_gicp（源码安装，small_gicp_relocalization / global_relocalization_kiss_matcher 依赖）
RUN git clone --depth 1 https://github.com/koide3/small_gicp.git /tmp/small_gicp \
    && cmake -S /tmp/small_gicp -B /tmp/small_gicp/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/small_gicp/build -j"$(nproc)" \
    && cmake --install /tmp/small_gicp/build \
    && ldconfig \
    && rm -rf /tmp/small_gicp

# KISS-Matcher C++ 库（kiss_matcher + ROBIN，FetchContent 需要联网）
# 只 COPY 该子目录，见 .dockerignore
COPY src/registration/KISS-Matcher /tmp/KISS-Matcher
RUN cd /tmp/KISS-Matcher && make cppinstall && ldconfig && rm -rf /tmp/KISS-Matcher

WORKDIR /ws
