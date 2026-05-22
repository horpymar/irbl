#!/usr/bin/env python3

import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BenchmarkRecorder(Node):
    def __init__(self):
        super().__init__("benchmark_recorder")

        self.uav_name = os.environ.get("UAV_NAME", "uav1")
        self.mode = os.environ.get("BENCHMARK_MODE")
        self.config = os.environ.get("RBL_CONTROLLER_CONFIG", "./config/rbl_controller.yaml")
        if not self.mode:
            try:
                with open(self.config, "r", encoding="utf-8") as config_file:
                    self.mode = "astar" if "replanner: true" in config_file.read() else "no_replanner"
            except OSError:
                self.mode = "unknown"

        self.timeout_s = float(os.environ.get("BENCHMARK_TIMEOUT", "240"))
        self.storage = os.environ.get("BENCHMARK_STORAGE", "mcap")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.out_dir = os.environ.get("BENCHMARK_BAG_DIR", f"./benchmark_bags/{self.mode}_{stamp}")
        self.done_file = os.environ.get("BENCHMARK_DONE_FILE", os.path.join(self.out_dir, "benchmark_done.txt"))
        os.makedirs(os.path.dirname(self.out_dir), exist_ok=True)

        self.proc = None
        self.finished = False
        self.start_wall = time.monotonic()

        self.create_subscription(String, "/benchmark/event", self.event_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.start_recording()

    def start_recording(self):
        topics = [
            "/clock",
            "/tf",
            "/tf_static",
            "/benchmark/event",
            "/benchmark/goal",
            f"/{self.uav_name}/estimation_manager/odom_main",
            f"/{self.uav_name}/rbl_controller/path",
            f"/{self.uav_name}/rbl_controller/replanner_waypoint",
            f"/{self.uav_name}/rbl_controller/target",
            f"/{self.uav_name}/rbl_controller/centroid",
            f"/{self.uav_name}/rbl_controller/actively_sensed_A",
            f"/{self.uav_name}/rbl_controller/local_obstacles",
            "/uav2/losos_server/current_submap_pc",
        ]
        cmd = ["ros2", "bag", "record", "-s", self.storage, "-o", self.out_dir] + topics
        self.get_logger().info(f"Recording benchmark bag to {self.out_dir}")
        self.proc = subprocess.Popen(cmd)

    def event_callback(self, msg):
        self.get_logger().info(f"Benchmark event: {msg.data}")
        if msg.data == "goal_reached":
            self.stop_recording("goal_reached")

    def timer_callback(self):
        if time.monotonic() - self.start_wall >= self.timeout_s:
            self.stop_recording("timeout")

    def stop_recording(self, reason):
        if self.finished:
            return
        self.finished = True
        self.get_logger().info(f"Stopping benchmark recording: {reason}")
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
        try:
            os.makedirs(os.path.dirname(self.done_file), exist_ok=True)
            with open(self.done_file, "w", encoding="utf-8") as done_file:
                done_file.write(f"{reason}\n")
        except OSError as exc:
            self.get_logger().warn(f"Failed to write benchmark done file: {exc}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = BenchmarkRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_recording("interrupted")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
