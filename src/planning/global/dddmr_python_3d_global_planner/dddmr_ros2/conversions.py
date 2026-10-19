"""Message conversion helpers; no running ROS graph required."""
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs_py.point_cloud2 import dtype_from_fields


def cloud_xyz(msg):
    """Decode XYZ, including organized clouds, row padding and big endian data."""
    dtype = dtype_from_fields(msg.fields, point_step=msg.point_step)
    if not {'x', 'y', 'z'}.issubset(dtype.names or ()):
        raise ValueError('PointCloud2 must contain x, y, z fields')
    if any(dtype.fields[k][0].kind != 'f' for k in ('x', 'y', 'z')):
        raise ValueError('PointCloud2 x/y/z must be FLOAT32 or FLOAT64')
    if msg.width == 0 or msg.height == 0:
        raise ValueError('Empty map')
    row_size = msg.width * msg.point_step
    if msg.row_step < row_size or len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Invalid PointCloud2 row_step or data length')
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step))
    xyz = np.column_stack([points[k].reshape(-1) for k in ('x', 'y', 'z')])
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz):
        raise ValueError('Map has no finite XYZ points')
    return np.ascontiguousarray(xyz, dtype=np.float32)


def transform_xyz(position, transform):
    """Apply a geometry_msgs/Transform (source -> target) to a position."""
    p = np.array([position.x, position.y, position.z], dtype=float)
    q = transform.rotation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quat)
    if not np.isfinite(quat).all() or norm < 1e-12:
        raise ValueError('Invalid TF quaternion')
    quat /= norm
    v = quat[:3]
    p = p + 2 * np.cross(v, np.cross(v, p) + quat[3] * p)
    t = transform.translation
    return p + np.array([t.x, t.y, t.z])


def path_message(result, frame_id, stamp):
    """Output positions and ground-aligned orientations in the map frame."""
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    if result is None or not result.success:
        return msg
    for point, quat in zip(result.points, result.quaternions()):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
        q = pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, quat)
        msg.poses.append(pose)
    return msg
