"""
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

from typing import Sequence

from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .frontier_model import Frontier


def _marker_color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    color = ColorRGBA()
    color.r = float(r)
    color.g = float(g)
    color.b = float(b)
    color.a = float(a)
    return color


def build_frontier_markers(
    frontiers: Sequence[Frontier],
    frame_id: str,
    stamp,
    marker_scale: float,
) -> MarkerArray:
    markers = MarkerArray()

    points_marker = Marker()
    points_marker.header.frame_id = frame_id
    points_marker.header.stamp = stamp
    points_marker.ns = "mrtsp_frontiers"
    points_marker.id = 0
    points_marker.type = Marker.POINTS
    points_marker.action = Marker.ADD
    points_marker.scale.x = marker_scale
    points_marker.scale.y = marker_scale
    points_marker.color = _marker_color(0.15, 0.9, 0.2)

    for frontier in frontiers:
        point = Point()
        point.x = float(frontier.center_point[0])
        point.y = float(frontier.center_point[1])
        point.z = 0.0
        points_marker.points.append(point)

    markers.markers.append(points_marker)
    return markers


def build_selected_frontier_pose(
    frontier: Frontier,
    frame_id: str,
    stamp,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(frontier.center_point[0])
    pose.pose.position.y = float(frontier.center_point[1])
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose
