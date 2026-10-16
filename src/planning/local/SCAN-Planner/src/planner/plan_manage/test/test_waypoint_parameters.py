"""Verify that the flat ROS 2 waypoint array is accepted at startup."""

import os

import launch
import launch_ros.actions
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[
            os.path.join(root, "config", "planner.yaml"),
            os.path.join(root, "config", "keypoints.example.yaml"),
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


class TestWaypointParameters:
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)
