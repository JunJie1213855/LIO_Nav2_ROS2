import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# ── 各 LIO 算法的话题名映射 ──────────────────────────────────────────────
# lio_type 参数根据此表自动选 odometry_sub 和 cloud_topic。
# 添加新算法只需在这里加一行。

_ODOM_MAP = ('{"fastlio": "/Odometry",'
             ' "pointlio": "/aft_mapped_to_init",'
             ' "superlio": "/lio/odom"}')

_CLOUD_MAP = ('{"fastlio": "/cloud_registered",'
              ' "pointlio": "/cloud_registered",'
              ' "superlio": "/lio/cloud_world"}')


def generate_launch_description():

    # ── 启动参数 ─────────────────────────────────────────────────────────

    lio_type_arg = DeclareLaunchArgument(
        'lio_type',
        default_value='fastlio',
        choices=['fastlio', 'pointlio', 'superlio'],
        description='LIO 算法类型, 自动匹配对应的话题名'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )

    odom_sub_arg = DeclareLaunchArgument(
        'odometry_sub',
        default_value='',
        description='覆盖 LIO 里程计话题 (留空则根据 lio_type 自动选择)'
    )

    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='',
        description='覆盖 LIO 点云话题 (留空则根据 lio_type 自动选择)'
    )

    lio_type = LaunchConfiguration('lio_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_user = LaunchConfiguration('odometry_sub')
    cloud_user = LaunchConfiguration('cloud_topic')

    # ── PythonExpression: 如果用户没填就用 lio_type 预设 ─────────────────
    #
    # 表达式逻辑 (以 odom 为例):
    #   odom_user_value if odom_user_value != '' else ODOM_MAP[lio_type_value]
    #
    # PythonExpression 里 LaunchConfiguration 被替换为裸字符串值,
    # 所以需要用引号包裹:  "'" + LaunchConfig + "'"

    odom_expr = PythonExpression([
        "'", odom_user, "' if '", odom_user, "' != '' else ",
        _ODOM_MAP, "['", lio_type, "']"
    ])

    cloud_expr = PythonExpression([
        "'", cloud_user, "' if '", cloud_user, "' != '' else ",
        _CLOUD_MAP, "['", lio_type, "']"
    ])

    # ── 节点 ────────────────────────────────────────────────────────────

    lio_interface_node = Node(
        package='lio_interface',
        executable='lio_interface_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'odometry_sub': odom_expr,
        }],
        remappings=[
            ('/cloud_registered', cloud_expr),
        ],
    )

    return LaunchDescription([
        lio_type_arg,
        use_sim_time_arg,
        odom_sub_arg,
        cloud_topic_arg,
        lio_interface_node,
    ])
