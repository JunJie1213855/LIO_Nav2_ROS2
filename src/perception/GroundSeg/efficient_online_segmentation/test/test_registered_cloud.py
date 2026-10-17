"""Real ROS tests for both input modes, coordinate transforms and field preservation."""
import math
import os
import subprocess
import tempfile
import time
from collections import Counter
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

CASES = [('world', True), ('world', False), ('sensor', True), ('sensor', False), ('cloud_only', False)]


@pytest.mark.parametrize('mode,sensor_pose', CASES)
def test_registered_cloud(monkeypatch, mode, sensor_pose):
    paired = mode != 'cloud_only'
    executable = os.environ['GROUND_SEPARATION_EXECUTABLE' if paired else 'GROUND_SEG_CLOUD_ONLY_EXECUTABLE']
    config = os.environ['GROUND_SEPARATION_CONFIG']
    domain = 50 + 5 * (os.getpid() % 28) + CASES.index((mode, sensor_pose))
    with tempfile.TemporaryDirectory(prefix='groundseg-test-') as temporary:
        monkeypatch.setenv('ROS_LOG_DIR', temporary)
        monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
        context = rclpy.context.Context()
        rclpy.init(context=context, domain_id=domain)
        node = rclpy.create_node('test_ground_seg', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        with open(Path(temporary) / 'node.log', 'w+') as log:
            process = subprocess.Popen([
                executable, '--ros-args', '--params-file', config,
                '-p', 'input_cloud_is_world:=' + str(mode == 'world').lower(),
                '-p', 'odometry_is_sensor_pose:=' + str(sensor_pose).lower(),
                '-p', 'sub_cloud_topic:=/test/cloud', '-p', 'odometry_topic:=/test/odom',
                '-p', 'pub_ground_cloud_topic:=/test/ground',
                '-p', 'pub_no_ground_cloud_topic:=/test/nonground',
                '-p', 'pub_odometry_topic:=/test/paired_odom',
            ], env=env, stdout=log, stderr=subprocess.STDOUT)
            received = {'ground': [], 'nonground': []}
            poses = []
            subscriptions = [node.create_subscription(
                PointCloud2, '/test/' + key, received[key].append, qos_profile_sensor_data)
                for key in received]
            pose_subscription = node.create_subscription(Odometry, '/test/paired_odom', poses.append, qos_profile_sensor_data)
            cloud_pub = node.create_publisher(PointCloud2, '/test/cloud', 10)
            pose_pub = node.create_publisher(Odometry, '/test/odom', 10)

            def wait(predicate, timeout=8.0, retry=None):
                deadline = time.monotonic() + timeout
                retry_at = time.monotonic() + 0.25
                while time.monotonic() < deadline:
                    if predicate():
                        return True
                    assert process.poll() is None, 'GroundSeg exited unexpectedly'
                    executor.spin_once(timeout_sec=0.03)
                    if retry is not None and time.monotonic() >= retry_at:
                        retry()
                        retry_at = time.monotonic() + 0.25
                return bool(predicate())

            def rows(cloud):
                return [tuple(float(v) for v in row) for row in point_cloud2.read_points(cloud)]

            try:
                assert wait(lambda: cloud_pub.get_subscription_count() > 0 and
                            (not paired or pose_pub.get_subscription_count() > 0) and
                            all(node.count_publishers(sub.topic_name) > 0 for sub in subscriptions) and
                            (not paired or node.count_publishers(pose_subscription.topic_name) > 0))
                if not paired:
                    assert pose_pub.get_subscription_count() == 0
                    assert node.count_publishers('/test/paired_odom') == 0
                for topic in ['/tf', '/tf_static']:
                    assert all(info.node_name != 'ground_separation' for info in node.get_publishers_info_by_topic(topic))
                yaw = 0.7
                c, s = math.cos(yaw), math.sin(yaw)
                points = []
                for sector in range(180):
                    angle = 2 * math.pi * (sector + 0.5) / 180
                    for radius, z in [(2.5 + 0.3*i, -0.25) for i in range(20)] + [(6.0, 0.4), (6.0, 0.8), (6.0, 1.2)]:
                        x, y = radius * math.cos(angle), radius * math.sin(angle)
                        xyz = (7 + c*x - s*y, -3 + s*x + c*y, 0.25 + z) if mode == 'world' else (x, y, z)
                        points.append((*xyz, 37.0, 0.012, len(points)))
                fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                          for i, name in enumerate(('x', 'y', 'z', 'intensity', 'time'))]
                fields.append(PointField(name='tag', offset=20, datatype=PointField.UINT16, count=1))
                header = Header(frame_id='odom' if mode == 'world' else 'livox_frame')
                header.stamp.sec, header.stamp.nanosec = 100, 123456789
                cloud = point_cloud2.create_cloud(header, fields, points)
                pose = Odometry()
                pose.header.frame_id = 'odom'
                pose.header.stamp.sec = 101  # Deliberate mismatch.
                pose.child_frame_id = 'livox_frame' if sensor_pose else 'base_footprint'
                pose.pose.pose.position.x, pose.pose.pose.position.y = 7.0, -3.0
                pose.pose.pose.position.z = 0.25 if sensor_pose else 0.0
                pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                pose.twist.twist.linear.x = 0.4
                pose.pose.covariance[0], pose.twist.covariance[0] = 0.02, 0.03
                cloud_pub.publish(cloud)
                if paired:
                    pose_pub.publish(pose)
                    assert not wait(lambda: bool(received['nonground']), timeout=0.5)
                    assert not poses
                    pose.header.stamp = header.stamp
                    pose_pub.publish(pose)
                # Discovery counts can precede delivery readiness for the first large
                # Best Effort sample. Reoffer the same pair only while no result exists.
                def retry_initial():
                    if not poses and not any(received.values()):
                        cloud_pub.publish(cloud)
                        if paired:
                            pose_pub.publish(pose)
                assert wait(lambda: all(received.values()) and (poses or not paired), retry=retry_initial)
                if paired:
                    assert poses[-1] == pose
                ground, nonground = received['ground'][-1], received['nonground'][-1]
                for output in [ground, nonground]:
                    assert output.header.frame_id == ('odom' if paired else 'livox_frame')
                    assert output.header.stamp == header.stamp
                    assert output.fields == fields
                    assert output.point_step == cloud.point_step
                    assert 0 < output.width < cloud.width
                actual = rows(ground) + rows(nonground)
                if mode != 'sensor':
                    assert Counter(actual) == Counter(rows(cloud))
                else:
                    by_tag = {p[-1]: p for p in actual}
                    assert len(by_tag) == cloud.width
                    for original in rows(cloud):
                        x, y, z = original[:3]
                        expected = (7+c*x-s*y, -3+s*x+c*y, 0.25+z)
                        result = by_tag[original[-1]]
                        assert all(math.isclose(a, b, abs_tol=2e-6) for a, b in zip(result[:3], expected))
                        assert result[3:] == original[3:]
                assert all(p[2] < (0.1 if paired else -0.15) for p in rows(ground))

                # Empty frames retain schema and timestamp; no old points leak through.
                header.stamp.sec = 102
                pose.header.stamp = header.stamp
                cloud_pub.publish(point_cloud2.create_cloud(header, fields, []))
                if paired:
                    pose_pub.publish(pose)
                assert wait(lambda: len(received['nonground']) == 2 and (not paired or len(poses) == 2))
                assert received['nonground'][-1].width == 0
                assert received['nonground'][-1].header.stamp == header.stamp
                if paired:
                    assert poses[-1] == pose

                # Malformed field layout is rejected without crashing or relaying a pose.
                header.stamp.sec = 103
                pose.header.stamp = header.stamp
                cloud.header = header
                cloud.fields[0].offset = cloud.point_step + 1
                cloud_pub.publish(cloud)
                if paired:
                    pose_pub.publish(pose)
                assert not wait(lambda: len(received['nonground']) > 2, timeout=0.3)
                assert len(poses) == (2 if paired else 0)
                cloud.fields[0].offset = 0
                if paired:
                    # Invalid pose/frame is also rejected as an entire group.
                    header.stamp.sec = 104
                    pose.header.stamp = header.stamp
                    pose.pose.pose.orientation.w = float('nan')
                    cloud.header = header
                    cloud_pub.publish(cloud)
                    pose_pub.publish(pose)
                    assert not wait(lambda: len(poses) > 2, timeout=0.3)
                    header.stamp.sec = 105
                    pose.header.stamp = header.stamp
                    pose.pose.pose.orientation.w = math.cos(yaw/2)
                    pose.header.frame_id = 'wrong_frame'
                    cloud.header = header
                    cloud_pub.publish(cloud)
                    pose_pub.publish(pose)
                    if mode == 'world':
                        assert not wait(lambda: len(poses) > 2, timeout=0.3)
                    else:
                        # For local clouds an arbitrary nonempty parent frame is legitimate.
                        assert wait(lambda: len(poses) == 3 and len(received['nonground']) == 3
                                    and len(received['ground']) == 3)
                        assert received['nonground'][-1].header.frame_id == 'wrong_frame'
                # A nonfinite coordinate is omitted; all remaining records survive.
                before = len(received['nonground'])
                header.stamp.sec = 106
                pose.header.stamp = header.stamp
                pose.header.frame_id = 'odom'
                invalid_points = points + [(float('nan'), 0.0, 0.0, 37.0, 0.012, 65000)]
                cloud_pub.publish(point_cloud2.create_cloud(header, fields, invalid_points))
                if paired:
                    pose_pub.publish(pose)
                assert wait(lambda: len(received['nonground']) == before + 1
                            and len(received['ground']) == before + 1
                            and (not paired or poses[-1].header.stamp.sec == 106))
                assert received['ground'][-1].width + received['nonground'][-1].width == len(points)
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read())
                raise
            finally:
                executor.shutdown()
                node.destroy_node()
                rclpy.shutdown(context=context)
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
