# RBL Replanner Algorithm Changes

This note summarizes the algorithmic and computation changes made to the RBL replanner/controller code.

## 1. Clearance Map Computation

The replanner uses an inflated voxel grid for obstacle avoidance. A clearance value is then computed for each voxel and used as a cost term in A*.

### Previous behavior

Clearance was computed using a priority-queue expansion over the whole local grid. This produced correct distances, but it was unnecessarily expensive because every grid cell was processed through a Dijkstra-like propagation.

### New behavior

Clearance is now computed with a separable 3D Euclidean distance transform:

1. Occupied/inflated obstacle voxels are initialized with distance `0`.
2. Free voxels are initialized with a large value.
3. A one-dimensional squared-distance transform is applied along:
   - X,
   - Y,
   - Z.
4. The final stored clearance is `floor(sqrt(distance_squared))` in grid cells.

This keeps the same meaning for the planner: each free voxel has an approximate distance to the nearest inflated obstacle, but the computation is linear in the number of voxels.

## 2. A* Open-List Fix

The A* implementation previously allowed the same voxel to be inserted into the open queue many times before it was closed.

### Previous behavior

For each expanded voxel:

1. Neighbor nodes were allocated.
2. They were pushed into the open queue.
3. Duplicate or worse paths to the same voxel were only discarded later, after popping from the queue.

This caused large amounts of duplicate work.

### New behavior

The replanner now keeps a `best_g_score` value for every voxel.

For every neighbor:

```text
tentative_g = current_g + motion_cost + safety_penalty + deviation_penalty
```

The neighbor is pushed into the open queue only if:

```text
tentative_g < best_g_score[neighbor]
```

Otherwise, the candidate is skipped immediately.

This makes the implementation closer to standard A*: each voxel tracks the cheapest known way to reach it, and worse duplicate paths are not expanded.

## 3. Safety Cost Is Part Of The Path Cost

The replanner uses obstacle clearance to bias A* away from obstacles:

```text
safety_penalty = weight_safety / (clearance + eps)
```

### Previous behavior

The safety and deviation penalties were added only to the node priority:

```text
f = g + h + penalties
```

but `g` itself did not accumulate the penalties.

### New behavior

The penalties are now included in `g`:

```text
g = parent_g + motion_cost + safety_penalty + deviation_penalty
f = g + h
```

This means the full path cost reflects obstacle clearance, not only the immediate queue ordering.

## 4. Path Smoothing Disabled

The replanner previously smoothed the raw A* path before returning it to the controller.

This was removed from the active planning path.

### Reason

The smoothing step could shortcut across several A* nodes and erase obstacle-avoidance structure from the voxel path. In practice, this made the controller receive a path that could be much more direct than the actual safe A* route.

### New behavior

The replanner returns the raw A* path converted to world coordinates.

This preserves the local bends and obstacle-avoidance decisions made by A*.

## 5. Controller Uses A Direct Path Lookahead

The controller does not follow the A* path directly. It computes a local waypoint from the path and uses that waypoint as the destination for the centroid computation.

### Previous behavior

The controller:

1. Found the closest A* path point to the drone.
2. Looked only at the next path point.
3. Built a direction from the drone to that next point.
4. Slowly moved the previous waypoint toward a radius-based target in that direction.

This made the A* path influence weak, especially when the path bent around obstacles.

### New behavior

The active waypoint is now a direct lookahead point on the A* path:

1. Find the path point closest to the drone.
2. Starting from that point, walk forward along path segments.
3. Find the first intersection between the path and a sphere around the drone.
4. Use that intersection as the waypoint.

The lookahead radius is clamped by distance to the final goal, so the waypoint does not overshoot the goal near the end.
The lookahead distance is configured separately from the local planning radius using:

```yaml
rbl_controller:
  path_lookahead_distance: 3.0
```

This is closer to a pure-pursuit path-following strategy and makes the controller follow the actual A* path geometry more directly.

## 6. Algorithmic Effect

Together, these changes make the replanner/controller behavior more consistent:

- A* computes a path using obstacle clearance as part of the accumulated path cost.
- The clearance field is computed efficiently using a distance transform.
- Duplicate A* queue entries are suppressed using per-voxel best costs.
- The raw A* path is preserved instead of being smoothed away.
- The controller chooses a waypoint directly from the A* path instead of filtering toward a target based only on the next point.

The result is that the replanner has a stronger and more direct effect on the controller trajectory while keeping the computation practical for repeated replanning.
