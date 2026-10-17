"""Exercise the ROS controller with differential-drive motion and a simulated clock."""
import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from scan_planner_msgs.msg import Bspline, ExecutionProgress


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package="scan_planner", executable="closed_loop_controller_car",
        parameters=[{"use_sim_time": True}], output="screen",
        remappings=[("body_pose", "/car_test/odom"),
                    ("planning/bspline", "/car_test/bspline"),
                    ("cmd_vel", "/car_test/cmd_vel"),
                    ("planning/execution_progress", "/car_test/progress")],
    )
    return launch.LaunchDescription([controller, launch_testing.actions.ReadyToTest()])


class TestCarController(unittest.TestCase):
    def test_motion_and_watchdog(self):
        rclpy.init()
        node = rclpy.create_node("car_controller_test")
        self.command = Twist()
        self.feedback = None
        self.commands = []
        def command_callback(msg):
            self.command = msg
            self.commands.append(msg)
        def feedback_callback(msg):
            self.feedback = msg
        subscriptions = [
            node.create_subscription(Twist, "/car_test/cmd_vel", command_callback, 10),
            node.create_subscription(ExecutionProgress, "/car_test/progress", feedback_callback, 10),
        ]
        odom_pub = node.create_publisher(Odometry, "/car_test/odom", 10)
        spline_pub = node.create_publisher(Bspline, "/car_test/bspline", 10)
        clock_pub = node.create_publisher(Clock, "/clock", 10)
        ticks = 500
        def step(x, y, yaw, odom=True):
            nonlocal ticks
            ticks += 1
            clock = Clock()
            clock.clock.sec = ticks // 50
            clock.clock.nanosec = ticks % 50 * 20000000
            clock_pub.publish(clock)
            if odom:
                msg = Odometry()
                msg.header.stamp = clock.clock
                msg.header.frame_id = "odom"
                msg.child_frame_id = "base_footprint"
                msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
                msg.pose.pose.orientation.z = math.sin(yaw / 2)
                msg.pose.pose.orientation.w = math.cos(yaw / 2)
                odom_pub.publish(msg)
            deadline = time.monotonic() + 0.012
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.002)
            return clock.clock
        try:
            for _ in range(100):
                step(0.0, 0.0, 0.0)
                if spline_pub.get_subscription_count() and odom_pub.get_subscription_count():
                    break
            self.assertGreater(spline_pub.get_subscription_count(), 0)
            for traj_id, angle, initial_y, length in [
                    (1, 0.0, 0.3, 3.0), (2, math.pi / 4, 0.0, 3.0), (3, 0.0, 0.0, 0.3)]:
                x, y, yaw = 0.0, initial_y, 0.0
                spline = Bspline()
                spline.order, spline.traj_id = 3, traj_id
                spline.start_time = step(x, y, yaw)
                spline.knots = [float(i) for i in range(-3, 9)]
                spline.pos_pts = [Point(x=(i-1)*0.6*math.cos(angle),
                                        y=(i-1)*0.6*math.sin(angle), z=0.0) for i in range(8)]
                if length < 1.0:
                    # Short curved terminal segment with exactly zero endpoint velocity.
                    spline.knots = [0.0]*4 + [1.0]*4
                    spline.pos_pts = [Point(x=0.0, y=0.0), Point(x=0.1, y=0.05),
                                       Point(x=length, y=0.0), Point(x=length, y=0.0)]
                spline_pub.publish(spline)
                self.commands.clear()
                reached = False
                for _ in range(1500):
                    step(x, y, yaw)
                    if self.feedback is None or self.feedback.traj_id != traj_id:
                        continue
                    cmd = self.command
                    self.assertEqual(cmd.linear.y, 0.0)
                    self.assertGreaterEqual(cmd.linear.x, 0.0)
                    self.assertLessEqual(cmd.linear.x, 0.5 + 1e-6)
                    self.assertLessEqual(abs(cmd.angular.z), 1.0 + 1e-6)
                    self.assertFalse(self.feedback.tracking_lost)
                    x += 0.02 * cmd.linear.x * math.cos(yaw)
                    y += 0.02 * cmd.linear.x * math.sin(yaw)
                    yaw += 0.02 * cmd.angular.z
                    if self.feedback.reached_end:
                        reached = True
                        break
                self.assertTrue(reached, f"failed to reach angle={angle}, pose={(x,y,yaw)}")
                self.assertLess(math.hypot(x-length*math.cos(angle), y-length*math.sin(angle)), 0.12)
                self.assertEqual(self.command.linear.x, 0.0)

            # A fresh straight trajectory must move, then stop when odometry disappears.
            spline.traj_id = 4
            spline.start_time = step(0.0, 0.0, 0.0)
            spline.knots = [float(i) for i in range(-3, 9)]
            spline.pos_pts = [Point(x=(i-1)*0.6, y=0.0, z=0.0) for i in range(8)]
            spline_pub.publish(spline)
            for _ in range(25): step(0.0, 0.0, 0.0)
            self.assertGreater(self.command.linear.x, 0.0)
            before = self.feedback.time_from_start
            for _ in range(60): step(0.0, 0.0, 0.0, odom=False)
            self.assertEqual(self.command.linear.x, 0.0)
            self.assertEqual(self.command.angular.z, 0.0)
            self.assertAlmostEqual(self.feedback.time_from_start, before)

            # Malformed replacement trajectories must not leave an old command running.
            invalid = Bspline()
            invalid.traj_id = 5
            invalid.start_time = step(0.0, 0.0, 0.0)
            spline_pub.publish(invalid)
            for _ in range(10): step(0.0, 0.0, 0.0)
            self.assertEqual(self.command.linear.x, 0.0)
            self.assertEqual(self.command.angular.z, 0.0)
        finally:
            node.destroy_node()
            rclpy.shutdown()
