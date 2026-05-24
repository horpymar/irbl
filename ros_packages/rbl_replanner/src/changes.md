# Implemented Changes

This file summarizes the final implemented code changes that remain in the current program.

## Replanner Grid, Inflation, and Clearance

### Corrected local grid dimensions

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::RBLReplanner()`.
- Location: `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`, `ReplannerParams`.
- The replanner now uses the existing `map_width` and `map_height` parameters only. The earlier attempted `map_length` usage was removed because `ReplannerParams` does not define `map_length`.
- The local A* grid is built as:
  - `_X_ = ceil(params.map_width / params.replanner_vox_size)`
  - `_Y_ = ceil(params.map_width / params.replanner_vox_size)`
  - `_Z_ = ceil(params.map_height / params.replanner_vox_size)`
- This keeps the map API consistent with the actual parameter structure.

### Conservative inflation coefficient

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::RBLReplanner()`.
- The inflation coefficient was changed to:
  ```cpp
  inflation_coeff_ = std::ceil(inflation_ / params.replanner_vox_size);
  ```
- The previous `-1` was removed so the hard inflated occupancy used by A* is not less conservative than the controller safety scale.

### Radial obstacle inflation

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::fillAndInflateGrid()`.
- The inflated occupancy grid now uses spherical/radial inflation instead of cubic inflation.
- A voxel is inflated only if:
  ```cpp
  dx * dx + dy * dy + dz * dz <= inflation_coeff_ * inflation_coeff_
  ```
- Boundary handling was also corrected so valid grid index `0` is not discarded.
- Null or missing cloud/grid input is handled safely by returning early.

### Line-by-line clearance transform

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::calculateClearanceGrid()`.
- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::calculate1dSquaredDistance()`.
- Location: `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`, declarations of `calculateClearanceGrid()` and `calculate1dSquaredDistance()`.
- Clearance is now computed with a 3D Felzenszwalb-Huttenlocher squared distance transform applied line-by-line:
  - X pass
  - Y pass
  - Z pass
- Occupied cells are initialized to `0`, free cells to a large `INF`, and the final stored clearance is the integer square root of the squared distance.
- The helper signature is now:
  ```cpp
  void calculate1dSquaredDistance(int* f, int* d, int n);
  ```
- This replaces the older stride/vector variant and keeps the transform explicit for each axis.

## Local Subgoal Selection

### Ray-based local subgoal

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::initializationPlan()`.
- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::localSubgoalOnMapBoundary()`.
- Location: `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`, declaration of `localSubgoalOnMapBoundary()`.
- The replanner no longer independently clamps each goal coordinate to the local map. Instead, it projects a local subgoal along the ray from the local start cell toward the global goal cell.
- This preserves the global goal direction and avoids artificial diagonal/corner targets caused by independent axis clamping.

### Subgoal limited by reliable RBL radius

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::localSubgoalOnMapBoundary()`.
- Location: `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`, `ReplannerParams::rbl_radius`.
- The local subgoal is capped by the controller reliable region:
  ```cpp
  effective_radius_cells = params_.rbl_radius / params_.replanner_vox_size;
  ```
- The final subgoal parameter is selected from:
  - map boundary intersection
  - RBL reliable radius limit
  - real goal, if it is inside the local map
- This prevents A* from committing to local targets that the RBL controller cannot reliably follow.

## A*/RBL Controller Consensus

### Shared controller safety parameters

- Location: `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`, `ReplannerParams`.
- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `RBLController::RBLController()`.
- The controller now passes RBL-related parameters into the replanner:
  ```cpp
  replanner_params.rbl_radius         = params.radius;
  replanner_params.rbl_lookahead      = params.path_lookahead_distance;
  replanner_params.ciri_inflation     = params.encumbrance + params.voxel_size;
  replanner_params.heading_weight     = 0.0;
  replanner_params.outside_rbl_weight = 5.0;
  replanner_params.min_clearance      = params.voxel_size;
  replanner_params.max_flight_z       = std::min(3.6, params.z_max - 0.4);
  ```
- This makes A* more conservative in the same regions where the controller is reliable.

### Hard clearance gate in A*

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::AStarPlan()`.
- Location: `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`, `ReplannerParams::min_clearance`.
- A* rejects cells whose clearance is below the configured minimum:
  ```cpp
  if (params_.min_clearance > 0.0 && clearance < params_.min_clearance) {
    continue;
  }
  ```
- This prevents A* from planning through cells that are too close to inflated obstacles.

### Soft penalty outside reliable RBL radius

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::AStarPlan()`.
- Location: `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`, `ReplannerParams::outside_rbl_weight`.
- A* still may search outside the reliable RBL radius, but nodes outside that radius become more expensive:
  ```cpp
  outside_rbl = max(0.0, dist_from_agent - params_.rbl_radius);
  rbl_penalty = params_.outside_rbl_weight * outside_rbl * outside_rbl;
  ```
- This keeps far-away nodes usable, but makes paths inside the controller reliable region preferred.

### Clearance-based diversity cost

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::AStarPlan()`.
- A* retains and uses the clearance safety cost:
  ```cpp
  safety_penalty = params_.weight_safety / (clearance + params_.eps);
  ```
- This makes narrow passages more expensive than wider passages, helping the planner prefer routes with better clearance when the path length is comparable.

## Planner Height Limitation

### Maximum A* flight height

- Location: `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`, `ReplannerParams::max_flight_z`.
- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `RBLController::RBLController()`.
- The controller sets:
  ```cpp
  replanner_params.max_flight_z = std::min(3.6, params.z_max - 0.4);
  ```
- With the current `z_max = 4.0`, A* is capped at `3.6 m`.

### Height gate in A*

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::AStarPlan()`.
- A* skips any generated neighbor whose world Z would exceed `params_.max_flight_z`.
- This prevents the planner from producing paths that would require flight at or above `4 m`.

### Height-aware local subgoal

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::localSubgoalOnMapBoundary()`.
- The local subgoal Z bound is also capped by `max_flight_z`.
- This prevents the local target itself from being placed above the allowed flight ceiling.

## Virtual Obstacles and Recovery Memory

### Replanner virtual obstacle interface

- Location: `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`, `ReplannerVirtualObstacle`.
- Location: `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`, `RBLReplanner::setVirtualObstacles()`.
- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::setVirtualObstacles()`.
- A virtual obstacle is represented by:
  ```cpp
  Eigen::Vector3d center;
  double radius;
  double weight;
  ```
- Virtual obstacles are not written into the real occupancy grid or point cloud. They are a temporary soft cost layer.

### Virtual obstacle penalty in A*

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::virtualObstaclePenalty()`.
- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::AStarPlan()`.
- A* adds the virtual obstacle cost into `tentative_g`:
  ```cpp
  tentative_g = ... + virtual_obstacle_penalty;
  ```
- The penalty is radial and quadratic:
  ```cpp
  penalty += obstacle.weight * normalized * normalized;
  ```
- This makes repeated stuck regions more expensive but still passable if no other path exists.

### Replanning when virtual obstacles change

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::setVirtualObstacles()`.
- When the virtual obstacle list changes, `goal_changed_` is set to `true`.
- This forces the replanner to recompute the path after recovery memory changes.

### Controller-side recovery state

- Location: `ros_packages/rbl_controller_core/include/rbl_controller_core/rbl_controller.h`, recovery state members.
- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, recovery functions.
- The controller now stores:
  - `RecoveryMode`
  - breadcrumb history
  - virtual obstacle memory
  - active recovery goal
  - stuck/progress timers
  - path acceptance state
- This state is only active when `params_.replanner == true`.

### Stuck detection

- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `RBLController::updateRecoveryState()`.
- Stuck detection uses progress-based conditions:
  - active goal exists
  - not close to final goal
  - grace period after goal change has passed
  - path exists and no pending replan is waiting
  - no meaningful movement/progress for several seconds
  - velocity is low
- This reduces false positives during normal replanning, first-plan waiting, or near-goal behavior.

### Breadcrumb-based escape

- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `appendRecoveryBreadcrumb()`.
- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `selectEscapeBreadcrumb()`.
- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `enterRecoveryEscape()`.
- Breadcrumbs are appended when the UAV moves far enough from the last stored breadcrumb.
- On stuck, the controller selects a previous breadcrumb behind the UAV and temporarily uses it as the planning goal.
- The real user goal remains stored in `goal_`; the recovery target is returned by `activePlanningGoal()` only while in `ESCAPE_BACKTRACK`.

### Recovery fallback reference

- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `RBLController::recoveryFallbackRef()`.
- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `RBLController::getNextRef()`.
- If A* has no path while the controller is in `ESCAPE_BACKTRACK`, the controller no longer commands a hard hold at the current position.
- It outputs a short reference toward the selected breadcrumb:
  ```cpp
  reference = agent_pos_ + step * to_escape.normalized();
  ```
- This prevents recovery from turning into a dead hover while the escape path is being replanned.

### Failed-path virtual obstacle chain

- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `addFailedPathVirtualObstacles()`.
- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `addVirtualObstacleMemory()`.
- After repeated stuck in the same area, the controller adds a chain of soft virtual obstacles along recent breadcrumbs before the stuck point.
- This penalizes the failed approach, not only the final stuck point.
- The escape endpoint is intentionally left mostly unpenalized so the UAV can still leave the trap.
- Repeated failures increase radius/weight up to configured caps.

## Path Stability and Anti-Flip Behavior

### Monotonic path progress index

- Location: `ros_packages/rbl_controller_core/include/rbl_controller_core/rbl_controller.h`, `path_progress_index_`.
- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `RBLController::determineWaypointFixedDistance()`.
- The waypoint selector no longer searches the entire path for the closest point on every control cycle.
- It searches from `path_progress_index_` forward:
  ```cpp
  search_begin = min(path_progress_index_, path.size() - 1);
  search_end = min(path.size(), search_begin + 35);
  ```
- The index only moves forward.
- This prevents U-turn paths from switching to a nearby but topologically wrong branch behind the UAV.

### Replanner path acceptance hysteresis

- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `shouldAcceptReplannerPath()`.
- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `acceptReplannerPath()`.
- Periodic replanner results are rejected if the new path initially points almost opposite to the currently executed waypoint while the current segment is still clear.
- Forced changes are still accepted when:
  - current path is empty
  - planning goal changed
  - recovery mode changed
  - current waypoint segment is blocked
- This reduces flip-flopping between two opposite local paths.

## Adaptive Lookahead

### Segment visibility check

- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `RBLController::isWaypointSegmentClear()`.
- The controller checks whether the straight segment from the UAV to the selected waypoint intersects the replanner inflated map.
- This detects the case where a lookahead point lies across an obstacle, such as on the opposite side of a U-turn.

### Adaptive lookahead distance

- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `RBLController::determineWaypointFixedDistance()`.
- The controller starts with the configured `path_lookahead_distance`.
- If the direct segment to the candidate waypoint is blocked, the lookahead is repeatedly reduced:
  ```cpp
  lookahead *= 0.7;
  ```
- The minimum lookahead is:
  ```cpp
  min(max_lookahead, max(0.9, 3.0 * params_.voxel_size))
  ```
- This keeps long lookahead on open straight paths, but prevents the controller from trying to cut through obstacles during U-turns.

## Goal Handling and Recovery Reset

### Goal generation and stale replanner protection

- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `RBLController::setGoal()`.
- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, async replanner handling in `getNextRef()`.
- Each user goal increments `goal_generation_`.
- Async replanner results are accepted only if their generation matches the current goal generation.
- This prevents stale paths from old goals from replacing current paths.

### Recovery reset on user goal change

- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`, `RBLController::resetRecoveryForNewGoal()`.
- On a new user goal:
  - breadcrumbs are cleared
  - virtual obstacles are cleared
  - recovery mode returns to `NORMAL`
  - progress/stuck timers are reset
  - path progress state is reset
- This prevents recovery memory from an old navigation goal from influencing a new command.

## Logging and Diagnostics

### Replanner timing and A* counters

- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::plan()`.
- Location: `ros_packages/rbl_replanner/src/replanner.cpp`, `RBLReplanner::AStarPlan()`.
- The replanner logs timing for initialization, inflation, clearance, A*, and conversion to world path.
- A* logs counts for:
  - expanded nodes
  - generated nodes
  - out-of-bounds skips
  - occupied skips
  - clearance skips
  - altitude skips
  - closed skips
  - stale open entries

### Controller recovery and lookahead logs

- Location: `ros_packages/rbl_controller_core/src/rbl_controller.cpp`.
- The controller now logs:
  - rejected opposite path updates
  - waiting for replanner path
  - reduced adaptive lookahead
  - blocked minimum lookahead fallback
  - stuck detection
  - recovery escape start/finish
  - failed-path virtual obstacle chain creation

