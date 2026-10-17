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

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mrtsp_exploration_ros2.explorer_node import MRTSPExplorerNode
from mrtsp_exploration_ros2.frontier_costs import RobotState
from mrtsp_exploration_ros2.frontier_model import Frontier


class _Time:
    def __init__(self, nanoseconds: int):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return _Time(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return None


def _frontier(x: float, y: float) -> Frontier:
    return Frontier(
        cells=((0, 0),),
        start_cell=(0, 0),
        centroid=(x, y),
        center_point=(x, y),
        center_cell=(0, 0),
        size=10,
    )


def _node() -> MRTSPExplorerNode:
    node = MRTSPExplorerNode.__new__(MRTSPExplorerNode)
    node.goal_preemption_enabled = True
    node.goal_preemption_on_frontier_opened = True
    node.goal_preemption_on_blocked_goal = True
    node.goal_preemption_min_interval_s = 0.0
    node.goal_preemption_skip_if_within_m = -1.0
    node.goal_in_progress = True
    node.goal_handle = object()
    node.global_frame = "map"
    node.active_frontier = _frontier(1.0, 1.0)
    node.active_goal_sent_time = _Time(0)
    node.cancel_request_in_progress = False
    node.replacement_candidate_frontier = None
    node.replacement_candidate_hits = 0
    node.replacement_required_hits = 2
    node.current_dispatch_id = 1
    node.dispatch_states = {}
    node._frontiers_equivalent = lambda first, second: first.center_point == second.center_point
    node._frontier_exists_in_set = lambda _frontier, _frontiers: False
    node._publish_debug_outputs = lambda *_args, **_kwargs: None
    node.get_clock = lambda: SimpleNamespace(now=lambda: _Time(5_000_000_000))
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_: None,
        warn=lambda *_: None,
        debug=lambda *_: None,
        error=lambda *_: None,
    )
    return node


def test_blocked_goal_replacement_dispatches_without_cancel(monkeypatch):
    node = _node()
    robot_state = RobotState(position=np.array([0.0, 0.0], dtype=float), yaw=0.0)
    replacement = _frontier(2.0, 2.0)
    cancel_reasons = []
    dispatch_calls = []

    node._active_goal_cost_status = lambda: "blocked"
    node._compute_frontier_context = lambda _state: (object(), [replacement], object())
    node._filter_frontiers_by_min_goal_distance = lambda frontiers, _state: list(frontiers)
    node._compute_frontier_order = lambda _frontiers, _map_adapter, _state: [0]
    node._build_goal_pose = lambda _state, _target: object()
    node._request_active_goal_cancel = lambda reason: cancel_reasons.append(reason)
    node._send_goal = (
        lambda _pose, frontier, goal_kind, dispatch_context="default": dispatch_calls.append(
            (frontier.center_point, goal_kind, dispatch_context)
        )
        or True
    )

    module = __import__("mrtsp_exploration_ros2.explorer_node", fromlist=["dummy"])
    monkeypatch.setattr(module, "build_selected_frontier_pose", lambda *_args, **_kwargs: None)

    node._consider_preempt_active_goal(robot_state)
    node._consider_preempt_active_goal(robot_state)

    assert cancel_reasons == []
    assert dispatch_calls == [((2.0, 2.0), "frontier", "replacement")]


def test_blocked_goal_without_replacement_uses_explicit_cancel(monkeypatch):
    node = _node()
    robot_state = RobotState(position=np.array([0.0, 0.0], dtype=float), yaw=0.0)
    cancel_reasons = []

    node._active_goal_cost_status = lambda: "blocked"
    node._compute_frontier_context = lambda _state: (object(), [], object())
    node._filter_frontiers_by_min_goal_distance = lambda frontiers, _state: list(frontiers)
    node._compute_frontier_order = lambda _frontiers, _map_adapter, _state: []
    node._request_active_goal_cancel = lambda reason: cancel_reasons.append(reason)

    module = __import__("mrtsp_exploration_ros2.explorer_node", fromlist=["dummy"])
    monkeypatch.setattr(module, "build_selected_frontier_pose", lambda *_args, **_kwargs: None)

    node._consider_preempt_active_goal(robot_state)

    assert len(cancel_reasons) == 1
    assert "no replacement frontier is available" in cancel_reasons[0]
