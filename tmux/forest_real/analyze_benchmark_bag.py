#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import numpy as np


def default_analysis_dir(bag_path):
    bag_path = Path(bag_path)
    if bag_path.parent.name == "benchmark_bags":
        return bag_path.parent.parent / "bag_analysis" / bag_path.name
    return bag_path.parent / "bag_analysis" / bag_path.name


def is_bag_dir(path):
    path = Path(path)
    return path.is_dir() and (path / "metadata.yaml").exists() and (
        any(path.glob("*.mcap")) or any(path.glob("*.db3"))
    )


def find_bags(path):
    path = Path(path)
    if is_bag_dir(path):
        return [path]
    if not path.is_dir():
        raise RuntimeError(f"Input path is neither a rosbag2 directory nor a directory of bags: {path}")
    bags = [child for child in path.iterdir() if is_bag_dir(child)]
    return sorted(bags, key=lambda p: p.name)


def bag_mode(bag_path):
    name = Path(bag_path).name
    if name.startswith("no_replanner"):
        return "no_replanner"
    if name.startswith("astar"):
        return "astar"
    return "unknown"


def import_rosbag_tools():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    return rosbag2_py, deserialize_message, get_message, point_cloud2


def make_reader(bag_path):
    rosbag2_py, _, _, _ = import_rosbag_tools()
    storage_id = "mcap" if any(Path(bag_path).glob("*.mcap")) else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    return reader, topic_types


def read_bag(bag_path, uav_name, default_goal):
    _, deserialize_message, get_message, point_cloud2 = import_rosbag_tools()
    reader, topic_types = make_reader(bag_path)

    odom_topic = f"/{uav_name}/estimation_manager/odom_main"
    map_topic = "/uav2/losos_server/current_submap_pc"
    path_topic = f"/{uav_name}/rbl_controller/path"

    odom = []
    events = []
    goals = []
    first_cloud = None
    path_msgs = 0
    topic_counts = {}

    msg_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        topic_counts[topic] = topic_counts.get(topic, 0) + 1

        if topic not in msg_types:
            continue

        if topic == odom_topic:
            msg = deserialize_message(data, msg_types[topic])
            p = msg.pose.pose.position
            v = msg.twist.twist.linear
            odom.append((timestamp_ns * 1e-9, np.array([p.x, p.y, p.z]), np.array([v.x, v.y, v.z])))

        elif topic == "/benchmark/event":
            msg = deserialize_message(data, msg_types[topic])
            events.append((timestamp_ns * 1e-9, msg.data))

        elif topic == "/benchmark/goal":
            msg = deserialize_message(data, msg_types[topic])
            if len(msg.data) >= 3:
                goals.append((timestamp_ns * 1e-9, np.array(msg.data[:3], dtype=float)))

        elif topic == map_topic and first_cloud is None:
            msg = deserialize_message(data, msg_types[topic])
            points = []
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([float(point[0]), float(point[1]), float(point[2])])
            first_cloud = np.asarray(points, dtype=float)

        elif topic == path_topic:
            path_msgs += 1

    goal = goals[-1][1] if goals else np.asarray(default_goal, dtype=float)
    return {
        "odom": odom,
        "events": events,
        "goals": goals,
        "goal": goal,
        "cloud": first_cloud,
        "path_msgs": path_msgs,
        "topic_counts": topic_counts,
        "topic_types": topic_types,
    }


def nearest_distances(query_points, obstacle_points):
    if obstacle_points is None or len(obstacle_points) == 0 or len(query_points) == 0:
        return np.array([])

    try:
        from scipy.spatial import cKDTree

        tree = cKDTree(obstacle_points)
        distances, _ = tree.query(query_points, k=1)
        return distances
    except Exception:
        distances = []
        chunk = 512
        for i in range(0, len(query_points), chunk):
            q = query_points[i : i + chunk]
            diff = q[:, None, :] - obstacle_points[None, :, :]
            distances.extend(np.sqrt(np.sum(diff * diff, axis=2)).min(axis=1))
        return np.asarray(distances)


def path_length(points):
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def fmt(value, unit=""):
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "n/a"
    return f"{value:.3f}{unit}"


def make_plots(args, times, positions, goal, cloud, clearance):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = default_analysis_dir(args.bag)
    output_dir.mkdir(parents=True, exist_ok=True)

    t_rel = times - times[0]

    fig, ax = plt.subplots(figsize=(10, 7))
    if cloud is not None and len(cloud) > 0:
        cloud_xy = cloud
        if args.plot_obstacle_stride > 1:
            cloud_xy = cloud_xy[:: args.plot_obstacle_stride]
        ax.scatter(cloud_xy[:, 0], cloud_xy[:, 1], s=3, c=cloud_xy[:, 2], cmap="viridis", alpha=0.65, linewidths=0)
    ax.plot(positions[:, 0], positions[:, 1], color="tab:red", linewidth=2.0, label="UAV trajectory")
    ax.scatter(positions[0, 0], positions[0, 1], color="tab:green", s=60, label="start", zorder=3)
    ax.scatter(goal[0], goal[1], color="tab:blue", s=80, marker="*", label="goal", zorder=3)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY trajectory")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    trajectory_path = output_dir / "trajectory_xy.png"
    fig.savefig(trajectory_path, dpi=200)
    plt.close(fig)

    if len(clearance):
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(t_rel, clearance, label="clearance minus encumbrance", linewidth=1.7)
        ax.axhline(0.0, color="tab:red", linestyle="--", linewidth=1.2, label="zero clearance")
        ax.set_xlabel("time since goal_set [s]")
        ax.set_ylabel("clearance [m]")
        ax.set_title("Clearance over time")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        fig.tight_layout()
        clearance_path = output_dir / "clearance.png"
        fig.savefig(clearance_path, dpi=200)
        plt.close(fig)
    else:
        clearance_path = None

    return output_dir, trajectory_path, clearance_path


def analyze(args, print_details=True):
    bag = read_bag(args.bag, args.uav_name, args.goal)
    odom = bag["odom"]

    if not odom:
        raise RuntimeError(f"No odometry messages found for /{args.uav_name}/estimation_manager/odom_main")

    event_goal_set = [t for t, event in bag["events"] if event == "goal_set"]
    event_goal_reached = [t for t, event in bag["events"] if event == "goal_reached"]

    start_t = event_goal_set[0] if event_goal_set else (bag["goals"][0][0] if bag["goals"] else odom[0][0])
    end_t = event_goal_reached[0] if event_goal_reached else min(odom[-1][0], start_t + args.timeout)

    odom_window = [(t, p, v) for t, p, v in odom if start_t <= t <= end_t]
    if not odom_window:
        raise RuntimeError("No odometry samples in selected benchmark window")

    times = np.asarray([sample[0] for sample in odom_window])
    positions = np.asarray([sample[1] for sample in odom_window])
    velocities = np.asarray([sample[2] for sample in odom_window])
    goal = bag["goal"]

    cloud = bag["cloud"]
    if cloud is not None:
        finite_mask = np.isfinite(cloud).all(axis=1)
        cloud = cloud[finite_mask]
        if args.obstacle_z_min is not None:
            cloud = cloud[cloud[:, 2] >= args.obstacle_z_min]
        if args.obstacle_voxel > 0 and len(cloud) > 0:
            vox = np.floor(cloud / args.obstacle_voxel).astype(np.int64)
            _, keep = np.unique(vox, axis=0, return_index=True)
            cloud = cloud[np.sort(keep)]

    nearest = nearest_distances(positions, cloud)
    clearance = nearest - args.encumbrance if len(nearest) else np.array([])
    dist_to_goal = np.linalg.norm(positions - goal, axis=1)
    speeds = np.linalg.norm(velocities, axis=1)

    reached_by_event = bool(event_goal_reached)
    reached_by_distance = bool(np.min(dist_to_goal) <= args.goal_tolerance)
    success = reached_by_event or reached_by_distance

    duration = times[-1] - times[0]
    result = {
        "bag": args.bag,
        "mode": bag_mode(args.bag),
        "goal": goal,
        "events": bag["events"],
        "start_time": start_t,
        "end_time": end_t,
        "duration": float(duration),
        "success": success,
        "reached_by_event": reached_by_event,
        "reached_by_distance": reached_by_distance,
        "odom_samples": len(positions),
        "start_position": positions[0],
        "end_position": positions[-1],
        "path_length": path_length(positions),
        "min_dist_to_goal": float(np.min(dist_to_goal)),
        "final_dist_to_goal": float(dist_to_goal[-1]),
        "mean_speed": float(np.mean(speeds)),
        "max_speed": float(np.max(speeds)),
        "map_cloud_points_used": 0 if cloud is None else len(cloud),
        "nearest_obstacle_distance_min": float(np.min(nearest)) if len(nearest) else math.nan,
        "nearest_obstacle_distance_mean": float(np.mean(nearest)) if len(nearest) else math.nan,
        "clearance_min_minus_encumbrance": float(np.min(clearance)) if len(clearance) else math.nan,
        "clearance_mean_minus_encumbrance": float(np.mean(clearance)) if len(clearance) else math.nan,
        "path_messages": bag["path_msgs"],
        "topic_counts": bag["topic_counts"],
    }

    if print_details:
        print("Benchmark bag summary")
        print("=====================")
        print(f"bag: {args.bag}")
        print(f"uav: {args.uav_name}")
        print(f"goal: [{goal[0]:.3f}, {goal[1]:.3f}, {goal[2]:.3f}]")
        print(f"events: {bag['events'] if bag['events'] else 'none'}")
        print(f"start_time: {start_t:.3f}")
        print(f"end_time: {end_t:.3f}")
        print(f"duration: {fmt(duration, ' s')}")
        print(f"success: {success} (event={reached_by_event}, distance={reached_by_distance})")
        print()
        print("Trajectory")
        print("----------")
        print(f"odom_samples: {len(positions)}")
        print(f"start_position: [{positions[0,0]:.3f}, {positions[0,1]:.3f}, {positions[0,2]:.3f}]")
        print(f"end_position: [{positions[-1,0]:.3f}, {positions[-1,1]:.3f}, {positions[-1,2]:.3f}]")
        print(f"path_length: {fmt(path_length(positions), ' m')}")
        print(f"min_dist_to_goal: {fmt(float(np.min(dist_to_goal)), ' m')}")
        print(f"final_dist_to_goal: {fmt(float(dist_to_goal[-1]), ' m')}")
        print(f"mean_speed: {fmt(float(np.mean(speeds)), ' m/s')}")
        print(f"max_speed: {fmt(float(np.max(speeds)), ' m/s')}")
        print()
        print("Obstacles")
        print("---------")
        print(f"map_cloud_points_used: {0 if cloud is None else len(cloud)}")
        print(f"nearest_obstacle_distance_min: {fmt(float(np.min(nearest)), ' m') if len(nearest) else 'n/a'}")
        print(f"nearest_obstacle_distance_mean: {fmt(float(np.mean(nearest)), ' m') if len(nearest) else 'n/a'}")
        print(f"clearance_min_minus_encumbrance: {fmt(float(np.min(clearance)), ' m') if len(clearance) else 'n/a'}")
        print(f"clearance_mean_minus_encumbrance: {fmt(float(np.mean(clearance)), ' m') if len(clearance) else 'n/a'}")
        print()
        print("Recorded topics")
        print("---------------")
        print(f"path_messages: {bag['path_msgs']}")
        for topic in sorted(bag["topic_counts"]):
            print(f"{topic}: {bag['topic_counts'][topic]}")

    if args.plots:
        output_dir, trajectory_path, clearance_path = make_plots(
            args, times, positions, goal, cloud, clearance
        )
        result["plot_output_dir"] = output_dir
        result["trajectory_plot"] = trajectory_path
        result["clearance_plot"] = clearance_path
        if print_details:
            print()
            print("Plots")
            print("-----")
            print(f"output_dir: {output_dir}")
            print(f"trajectory_xy: {trajectory_path}")
            if clearance_path:
                print(f"clearance: {clearance_path}")

    return result


def print_batch_summary(results):
    groups = {}
    for result in results:
        groups.setdefault(result["mode"], []).append(result)

    print("Benchmark batch summary")
    print("=======================")
    print(f"bags_analyzed: {len(results)}")
    print()

    for mode in ("no_replanner", "astar", "unknown"):
        runs = groups.get(mode, [])
        if not runs:
            continue

        durations = np.asarray([run["duration"] for run in runs], dtype=float)
        successes = sum(1 for run in runs if run["success"])

        print(mode)
        print("-" * len(mode))
        for run in runs:
            print(
                f"{Path(run['bag']).name}: duration={fmt(run['duration'], ' s')}, "
                f"success={run['success']}, path_length={fmt(run['path_length'], ' m')}, "
                f"min_clearance={fmt(run['clearance_min_minus_encumbrance'], ' m')}"
            )
        print(
            f"average_duration: {fmt(float(np.mean(durations)), ' s')} "
            f"(std={fmt(float(np.std(durations)), ' s')}, n={len(runs)}, success={successes}/{len(runs)})"
        )
        print()


def main():
    parser = argparse.ArgumentParser(description="Analyze one RBL benchmark rosbag2 bag or a directory of benchmark bags.")
    parser.add_argument("bag", type=Path, help="Path to rosbag2 directory or benchmark_bags directory")
    parser.add_argument("--uav-name", default="uav1")
    parser.add_argument("--goal", nargs=3, type=float, default=[90.0, 0.0, 2.0])
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.3)
    parser.add_argument("--encumbrance", type=float, default=0.6)
    parser.add_argument("--obstacle-z-min", type=float, default=0.2)
    parser.add_argument("--obstacle-voxel", type=float, default=0.2)
    parser.add_argument("--plots", action="store_true", help="Write PNG plots under bag_analysis/")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--plot-obstacle-stride", type=int, default=2)
    args = parser.parse_args()

    bags = find_bags(args.bag)
    if len(bags) == 1 and is_bag_dir(args.bag):
        analyze(args)
        return

    results = []
    for bag in bags:
        bag_args = argparse.Namespace(**vars(args))
        bag_args.bag = bag
        if args.output_dir is not None:
            bag_args.output_dir = args.output_dir / bag.name
        try:
            results.append(analyze(bag_args, print_details=False))
        except Exception as exc:
            print(f"{bag.name}: failed: {exc}")

    print_batch_summary(results)
    if args.plots:
        analysis_root = args.output_dir if args.output_dir is not None else args.bag.parent / "bag_analysis"
        print(f"plots_root: {analysis_root}")


if __name__ == "__main__":
    main()
