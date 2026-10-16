# Go2 description for ROS 2 Humble

This package contains the Unitree Go2 Xacro model, RViz2 configuration and Gazebo Fortress integration used by SCAN-Planner.

Inspect the model and its 12 movable joints:

```bash
ros2 launch go2_description go2_rviz.launch.py
```

Start Gazebo Fortress with `gz_ros2_control`, the joint-state broadcaster, and the joint-trajectory controller:

```bash
ros2 launch go2_description go2_sim.launch.py
```

The physics launch publishes `/joint_states` and exposes the action `/joint_trajectory_controller/follow_joint_trajectory`. `ros_gz_bridge` bridges `/clock`, `/go2/imu`, and the four `/go2/*_foot_contact` topics.

The old Gazebo Classic and Unitree-specific visualization, force-injection and contact plugins are intentionally excluded.
