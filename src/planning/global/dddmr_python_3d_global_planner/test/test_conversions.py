import struct

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Transform
from sensor_msgs.msg import PointCloud2, PointField

from dddmr_py.planner import Path
from dddmr_ros2.conversions import cloud_xyz, path_message, transform_xyz


@pytest.mark.parametrize('big_endian', [False, True])
def test_organized_cloud_with_row_padding_and_nan(big_endian):
    cloud = PointCloud2(height=2, width=2, point_step=16, row_step=36,
                        is_bigendian=big_endian, is_dense=True)
    cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    points = [(1, 2, 3, 99), (4, 5, 6, 99), (7, 8, 9, 99), (float('nan'), 0, 0, 99)]
    fmt = ('>' if big_endian else '<') + 'ffff'
    data = b''
    for i in (0, 2):
        data += struct.pack(fmt, *points[i]) + struct.pack(fmt, *points[i + 1]) + b'\0' * 4
    cloud.data = data
    np.testing.assert_array_equal(cloud_xyz(cloud), [[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    assert bytes(cloud.data) == data  # Decoder must not mutate the incoming message.


def test_empty_and_missing_xyz_rejected():
    with pytest.raises(ValueError):
        cloud_xyz(PointCloud2())


def test_transform_rotation_and_translation():
    transform = Transform()
    transform.rotation.z = np.sin(np.pi / 4)
    transform.rotation.w = np.cos(np.pi / 4)
    transform.translation.x = 5.0
    np.testing.assert_allclose(transform_xyz(Point(x=1.0), transform), [5, 1, 0], atol=1e-7)


def test_path_preserves_xyz_header_and_normalized_orientation():
    result = Path(points=np.array([[0., 0., 1.], [1., 0., 2.]]),
                  yaw=np.zeros(2), normals=np.array([[0., 0., 1.], [0., 0., 1.]]))
    msg = path_message(result, 'cloud_map', Time(sec=123))
    assert msg.header.frame_id == 'cloud_map'
    assert len(msg.poses) == 2
    assert msg.poses[-1].pose.position.z == 2.
    for pose in msg.poses:
        assert pose.header == msg.header
        q = pose.pose.orientation
        assert np.isclose(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.)
    assert not path_message(None, 'cloud_map', Time()).poses
    result.success = False
    assert not path_message(result, 'cloud_map', Time()).poses
