"""Smoke-test that the migrated planner accepts its ROS 2 parameter file."""

import os

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml")
    )
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[config],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


class TestPlannerStartup:
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)
