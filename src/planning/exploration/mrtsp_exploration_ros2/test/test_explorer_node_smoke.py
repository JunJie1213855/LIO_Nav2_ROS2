"""
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
import os

import rclpy

from mrtsp_exploration_ros2.explorer_node import MRTSPExplorerNode


def test_explorer_node_initializes(tmp_path):
    os.environ["ROS_LOG_DIR"] = str(tmp_path / "ros_logs")
    rclpy.init()
    node = MRTSPExplorerNode()
    try:
        assert node.map_topic == "/map"
        assert node.costmap_topic == "/global_costmap/costmap"
        assert node.local_costmap_topic == "/local_costmap/costmap"
        assert node.goal_preemption_on_frontier_opened is False
        assert node.goal_preemption_on_blocked_goal is False
        assert node.goal_preemption_enabled is False
        assert node.return_to_start_on_complete is False
    finally:
        node.destroy_node()
        rclpy.shutdown()
