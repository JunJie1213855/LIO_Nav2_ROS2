import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from dddmr_ros2.node import PlannerNode

rclpy.init()
planner=PlannerNode(parameter_overrides=[
    Parameter('start',value=[0.5,0.5,0.0]),
    Parameter('planner.search_backend',value='python'),
    Parameter('planner.voxel_size',value=0.15),
])
client=Node('dddmr_smoke_client')
executor=SingleThreadedExecutor()
executor.add_node(planner);executor.add_node(client)
received=[]
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
sub=client.create_subscription(Path,'/init_path',received.append,qos)
map_pub=client.create_publisher(PointCloud2,'/global_points',1)
goal_pub=client.create_publisher(PoseStamped,'/goal_pose',10)
def spin_until(predicate,timeout=15):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        executor.spin_once(timeout_sec=0.05)
        if predicate():return
    raise AssertionError('Timed out waiting for ROS topics')
try:
    spin_until(lambda:map_pub.get_subscription_count()>0 and goal_pub.get_subscription_count()>0,5)
    goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=3.5;goal.pose.position.y=3.5;goal.pose.orientation.w=1.
    goal_pub.publish(goal)
    spin_until(lambda:planner._goal is not None)
    xy=np.arange(0,4.01,0.08);x,y=np.meshgrid(xy,xy)
    xyz=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)]).astype(np.float32)
    cloud=create_cloud_xyz32(Header(frame_id='map'),xyz)
    map_pub.publish(cloud)
    spin_until(lambda:any(m.poses for m in received),30)
    path=next(m for m in reversed(received) if m.poses)
    assert path.header.frame_id=='map'
    assert all(p.header==path.header for p in path.poses)
    assert np.linalg.norm([path.poses[-1].pose.position.x-3.5,path.poses[-1].pose.position.y-3.5])<0.4
    print('TOPIC_PASS:',len(path.poses),'poses from PointCloud2 -> /init_path')
    received.clear()
    goal.pose.position.x=1000.;goal_pub.publish(goal)
    spin_until(lambda:planner._future is None and not planner._needs_plan and received)
    assert all(not m.poses for m in received)
    print('FAILURE_PASS: unreachable target clears path')
finally:
    executor.remove_node(planner);executor.remove_node(client)
    planner.destroy_node();client.destroy_node();executor.shutdown();rclpy.try_shutdown()
