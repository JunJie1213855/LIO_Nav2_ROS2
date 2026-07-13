#!/usr/bin/env python3
"""
发送 Nav2 导航目标
用法:
  ros2 run me_nav2_bringup send_goal.py --ros-args -p x:=3.0 -p y:=-1.0 -p yaw:=0.0
"""
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import math


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value

        self.send_goal(x, y, yaw)

    def send_goal(self, x, y, yaw):
        self.get_logger().info('等待 Nav2 就绪...')
        self._action_client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = Time()  # 零时间戳 → Nav2 自动用最新 TF
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'发送目标: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_cb
        )
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            rclpy.shutdown()
            return
        self.get_logger().info('目标已接受')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ 到达目标!')
        else:
            self.get_logger().error(f'❌ 导航失败, status={status}')
        rclpy.shutdown()

    def feedback_cb(self, feedback):
        dist = feedback.feedback.distance_remaining
        self.get_logger().info(f'距目标: {dist:.2f}m', throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = Nav2GoalSender()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
