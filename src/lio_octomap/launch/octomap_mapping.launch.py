import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# ── lio_type → (点云话题, map frame) 映射 ─────────────────────────────
#
#    octomap_server 订阅 PointCloud2 话题，同时通过 TF 查询传感器原点
#    做射线投射（ray tracing），标记射线路径上的体素为 free。
#    frame_id 决定八叉树地图绑定的坐标系。

_CLOUD_MAP = ('{"fastlio": "/cloud_registered",'
              ' "pointlio": "/cloud_registered",'
              ' "superlio": "/lio/cloud_world",'
              ' "unified": "/registered_scan"}')

_FRAME_MAP = ('{"fastlio": "camera_init",'
              ' "pointlio": "camera_init",'
              ' "superlio": "world",'
              ' "unified": "odom"}')


def generate_launch_description():

    # ── 启动参数 ─────────────────────────────────────────────────────

    lio_type_arg = DeclareLaunchArgument(
        'lio_type',
        default_value='fastlio',
        choices=['fastlio', 'pointlio', 'superlio', 'unified'],
        description='LIO 算法类型: fastlio | pointlio | superlio | unified'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.1',
        description='八叉树体素分辨率 (m), 越小越精细内存越大'
    )

    lio_type = LaunchConfiguration('lio_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')

    # ── 查表表达式 ───────────────────────────────────────────────────

    cloud_expr = PythonExpression([
        _CLOUD_MAP, "['", lio_type, "']"
    ])

    frame_expr = PythonExpression([
        _FRAME_MAP, "['", lio_type, "']"
    ])

    # ── octomap_server 节点 ──────────────────────────────────────────

    pkg_share = get_package_share_directory('lio_octomap')
    yaml_path = os.path.join(pkg_share, 'config', 'octomap.yaml')

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[yaml_path, {
            'use_sim_time': use_sim_time,
            'resolution': resolution,
            'frame_id': frame_expr,
        }],
        remappings=[
            ('/cloud_in', cloud_expr),        # octomap_server 固定订阅 /cloud_in
        ],
    )

    return LaunchDescription([
        lio_type_arg,
        use_sim_time_arg,
        resolution_arg,
        octomap_server_node,
    ])
