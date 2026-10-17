# mrtsp_exploration_ros2

ROS 2 Jazzy frontier exploration package implementing the method from:

- Liu, C., Zhang, D., Liu, W. et al. Enhancing autonomous exploration for robotics via real time map optimization and improved frontier costs. Sci Rep 15, 12261 (2025).
- DOI: [10.1038/s41598-025-97231-9](https://doi.org/10.1038/s41598-025-97231-9)

Check out main C++ exploration project for more details and features: 
- [frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2)

Benchmarks against other autonomous exploration algorithms: 
- [autonomous-exploration-demo-benchmark](https://github.com/mertgulerx/autonomous-exploration-demo-benchmark)

## Overview

This package provides a standalone ROS 2 explorer node that:

- subscribes to `/map` and `/global_costmap/costmap`
- optimizes the map in real time using bilateral filtering and dilation
- extracts frontiers using WFD on the optimized map
- computes frontier costs using the paper's equations
- builds an **MRTSP (Minimum Ratio Traveling Sales Person)** cost matrix
- dispatches the first frontier from a greedy MRTSP ordering to Nav2 using `NavigateToPose`

The package is intentionally independent and can be reused in any Nav2-based stack.

## Flowchart Diagram


```
           +-------------------------+
+----------| Map & Costmap Input     |
|          +-------------------------+
|                       |
|                       v
|          +-------------------------+
|          | Decision-Map            |
|          | Optimization            |
|          +-------------------------+
|                       |
|                       v
|          +-------------------------+
|          | WFD-Style               |
|          | Frontier Extraction     |
|          +-------------------------+
|                       |
|                       v
|         +---------------------------+
|         | Compute MRTSP Cost Matrix |
|         | & Greedy Ordering         |
|         +---------------------------+
|                       |
|                       |
|                       v
|          +-------------------------+
|          | Dispatch Goal via Nav2  |
|          +-------------------------+
|                       |
|                       v
|          +-------------------------+
|          | Monitor & Handle        |
|          | Preemption/Blocking     |
|          +-------------------------+
|                       |
|                       v
|                  .-----------.
|                /               \
|               /     Frontiers    \
|              /     Exhausted?     \
|             v                   v
|           No                   Yes
|             |                   |
|             |                   v
|             |          +---------------------------+
|             |          | Publish Completion Event  |
|             |          +---------------------------+
|             |
+-------------+
```

## Public ROS Interfaces

- Subscribes:
  - `map_topic` as `nav_msgs/OccupancyGrid`
  - `costmap_topic` as `nav_msgs/OccupancyGrid`
- Uses TF:
  - `global_frame -> robot_base_frame`
- Sends goals with:
  - `nav2_msgs/action/NavigateToPose`
- Optional debug publishers:
  - `/explore/frontiers` as `visualization_msgs/MarkerArray`
  - `/explore/selected_frontier` as `geometry_msgs/PoseStamped`
  - `/explore/optimized_map` as `nav_msgs/OccupancyGrid`

## Formula Interpretation

The paper contains visible typesetting errors in the bilateral filter equations. This package uses the academically corrected forms:

- Spatial kernel:
  - `G_sigma_s(||p-q||) = exp(-((i-x)^2 + (j-y)^2) / (2 sigma_s^2))`
- Range kernel:
  - `G_sigma_r(|I(p)-I(q)|) = exp(-((I(p)-I(q))^2) / (2 sigma_r^2))`
- Bilateral normalization:
  - `W_p = sum_q G_sigma_s G_sigma_r`
- Filtered value:
  - `I_bar(p) = 1/W_p * sum_q G_sigma_s G_sigma_r I(q)`

The path cost and MRTSP terms are implemented directly from the paper:

- `d(V_i, V_j) = max(d_m + d_u, d_n + d_v) - r_s`
- `P(V_i, V_j) = S`
- `M_i,j` and `t_lb(V_i, V_j)` match the corrected latex interpretation used during implementation.

## Parameters

Core parameters are in `config/params.yaml`.

Important ones:

- `map_topic`
- `costmap_topic`
- `local_costmap_topic`
- `navigate_to_pose_action_name`
- `global_frame`
- `robot_base_frame`
- `sigma_s`
- `sigma_r`
- `dilation_kernel_radius_cells`
- `sensor_effective_range_m`
- `weight_distance_wd`
- `weight_gain_ws`
- `max_linear_speed_vmax`
- `max_angular_speed_wmax`
- `occ_threshold`
- `min_frontier_size_cells`
- `publish_debug_topics`
- `goal_preemption_on_frontier_opened`
- `goal_preemption_on_blocked_goal`
- `goal_preemption_min_interval_s`
- `goal_preemption_skip_if_within_m`
- `return_to_start_on_complete`

`goal_preemption_on_frontier_opened` and `goal_preemption_on_blocked_goal` are intentionally
disabled by default so the package keeps paper-like behavior unless the integrator explicitly opts
into online frontier revalidation. When enabled, the node can independently cancel the active goal
if the target frontier has already opened or if the goal point becomes blocked in the available
global/local costmaps.

When a replacement frontier is available, preemption is sent as a replacement
`NavigateToPose` goal (without a mandatory cancel-then-resend round trip). Explicit
cancel is reserved for cases where the active goal becomes invalid and no safe
replacement frontier exists.

When no frontiers remain, the node now announces exploration completion once and stops replanning.
`return_to_start_on_complete` is disabled by default for package neutrality, but can be enabled by
integrators that want the robot to navigate back to its recorded start pose after exploration ends.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mrtsp_exploration_ros2
source install/setup.bash
```

## Run

```bash
ros2 launch mrtsp_exploration_ros2 explore.launch.py
```

Or with an explicit params file:

```bash
ros2 launch mrtsp_exploration_ros2 explore.launch.py \
  params_file:=/path/to/params.yaml
```

## Package Structure

- `mrtsp_exploration_ros2/occupancy_grid.py`: occupancy grid helpers
- `mrtsp_exploration_ros2/map_optimizer.py`: bilateral filtering and dilation
- `mrtsp_exploration_ros2/wfd_frontier.py`: WFD frontier extraction
- `mrtsp_exploration_ros2/frontier_model.py`: frontier dataclasses
- `mrtsp_exploration_ros2/frontier_costs.py`: path/gain/MRTSP cost equations
- `mrtsp_exploration_ros2/mrtsp_solver.py`: greedy frontier ordering
- `mrtsp_exploration_ros2/explorer_node.py`: ROS 2 node and Nav2 action client
- `mrtsp_exploration_ros2/visualization.py`: debug marker generation

## Algorithm and Mathematics

### Frontier Definition

The package follows the classical frontier idea: a frontier is an unknown cell that borders free space.

The eligibility test implemented in the search layer can be summarized as:

```text
frontier(p) is true if:
  map(p) = unknown
  and there exists q in N(p) such that map(q) = free
  and there does not exist q in N(p) blocked by the active costmap policy
```

`N(p)` is the local neighborhood around cell `p`. The implementation uses an 8-connected neighborhood plus the center cell for local scans.

### WFD-Style Two-Level Search

The core search keeps the WFD idea from [Frontier Based Exploration for Autonomous Robot](https://arxiv.org/abs/1806.03581). The code uses two BFS layers:

1. A map-space BFS expands through reachable map cells.
2. When a frontier cell is found, a frontier BFS grows that connected frontier cluster.

The search flow is:

```text
1. Project the robot pose into map coordinates.
2. If the robot starts in unknown or occupied space, recover the nearest free seed.
3. Expand a BFS over reachable map cells.
4. When a frontier cell is detected, grow the full connected frontier cluster.
5. Reject tiny clusters.
6. Build one frontier candidate from that cluster.
7. Repeat until the reachable map area is exhausted.
```

This preserves the core WFD idea while adapting it to a ROS 2 exploration stack that also reasons over global and local costmaps.

### Frontier Candidate Geometry

Each frontier cluster is converted into a `FrontierCandidate` with:

- a centroid used for ranking and equivalence checks
- a center point used as the MRTSP dispatch baseline
- a start world point used by the MRTSP path-cost calculation
- an optional reachable goal point used by nearest-style navigation dispatch
- a cluster size used as the information-gain proxy

The centroid is computed directly from frontier cells:

```text
c_x = (1 / N) * sum(x_i)
c_y = (1 / N) * sum(y_i)
```

In `nearest` mode, the node searches free and unblocked neighbors around frontier cells and selects a reachable navigation point close to the centroid:

```text
choose g that minimizes ||g - c||^2
subject to:
  g is free
  g is not blocked in the global costmap
  g is not blocked in the local costmap
  and optionally ||g - r||^2 >= d_min^2
```

Where:

- `c` is the frontier centroid
- `g` is the candidate goal point
- `r` is the robot position
- `d_min` is the active minimum-distance gate

If no goal candidate satisfies the distance constraint, the implementation falls back to the best unconstrained reachable point.

### Decision-Map Optimization

The decision-map stage is inspired by the 2025 paper and implemented directly in this package. The purpose is to reduce invalid frontiers caused by sparse sensing while preserving occupied structure.

The occupancy grid is first mapped into a paper-style image:

- occupied = `0`
- unknown = `205`
- free = `255`

Let `I(p)` be the paper-image value at cell `p`. The bilateral filter uses a spatial-domain Gaussian and a range-domain Gaussian:

```text
G_s(p, q) = exp(-||p - q||^2 / (2 * sigma_s^2))
G_r(p, q) = exp(-(I(p) - I(q))^2 / (2 * sigma_r^2))
```

The normalization term and filtered value are:

```text
W(p) = sum_{q in S} G_s(p, q) * G_r(p, q)
I_bar(p) = (1 / W(p)) * sum_{q in S} G_s(p, q) * G_r(p, q) * I(q)
```

Where `S` is the local filter support around `p`.

After filtering, the package thresholds the image back into a frontier-decision image:

- cells above the free/unknown midpoint become free
- cells below that threshold become unknown
- occupied cells from the raw image remain occupied

Finally, the package applies circular free-space dilation with radius `dilation_kernel_radius_cells` over the thresholded result. This expands filtered free support while keeping occupied cells fixed.

In practice, this stage:

- reduces invalid frontiers caused by sparse sensing
- preserves occupied cells
- keeps narrow traversable passages when the optimization parameters are tuned conservatively

### Frontier Cost Model

In `mrtsp` mode, the package uses frontier size as the information-gain proxy:

```text
P(V_i, V_j) = size(V_j)
```

The path-cost term follows the candidate geometry used in the implementation:

```text
d(V_i, V_j) = max(d_m + d_u, d_n + d_v) - r_s
```

Where:

- `d_m` is the distance from the source frontier center point to the target frontier center point
- `d_n` is the distance from the source frontier center point to the target frontier centroid
- `d_u` is the distance from the target frontier center point to the target start world point
- `d_v` is the distance from the target frontier centroid to the target start world point
- `r_s` is `sensor_effective_range_m`

The implementation uses frontier cluster size as the gain term. The path-cost term can become negative when a candidate frontier is effectively already within sensing range. That behavior is intentional because it biases ordering toward frontiers that can expose area efficiently with less added travel.

### MRTSP Ordering

Once frontier path cost and information gain are available, the package builds a directed MRTSP-style cost matrix over the robot start node and all frontier candidates.

For frontier-to-frontier transitions, the heuristic is:

```text
M(i, j) = (weight_distance_wd * d(V_i, V_j)) / (weight_gain_ws * P(V_i, V_j))
```

For start-to-frontier transitions, the package adds a lower-bound start term derived from robot translation and heading limits:

```text
M(0, j) =
  (weight_distance_wd * d(V_0, V_j)) / (weight_gain_ws * P(V_0, V_j))
  + t_lb(j)
```

With:

```text
t_lb(j) = min(
  L(robot, V_j) / max_linear_speed_vmax,
  |delta_yaw(robot, V_j)| / max_angular_speed_wmax
)
```

This package uses a greedy traversal over that MRTSP-style matrix. It is therefore a practical ROS 2 implementation of the paper's decision model, not an exhaustive MRTSP solver.


