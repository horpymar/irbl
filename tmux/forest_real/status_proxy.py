#!/usr/bin/env python3

import math
import os
import shutil
import time

import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Quaternion
from mrs_msgs.msg import (
    ControlManagerDiagnostics,
    EstimationDiagnostics,
    HwApiStatus,
    TrackerCommand,
    UavStatus,
    UavStatusShort,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, MagneticField, NavSatFix


def quat_to_yaw(quaternion: Quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RateTracker:
    def __init__(self):
        self.last_time = None
        self.rate = 0.0

    def tick(self, stamp: float) -> None:
        if self.last_time is not None:
            delta = stamp - self.last_time
            if delta > 0.0:
                self.rate = 1.0 / delta
        self.last_time = stamp


class StatusProxy(Node):
    def __init__(self):
        uav_name = os.environ.get('UAV_NAME', 'uav1')
        super().__init__('uav_status_proxy', namespace=uav_name)

        self.uav_name = uav_name
        self.uav_type = os.environ.get('UAV_TYPE', 'x500')
        self.sim_start_wall = time.time()

        self.control_diag = None
        self.estimation_diag = None
        self.hw_api_status = None
        self.odom = None
        self.tracker_cmd = None
        self.battery = None
        self.gnss = None
        self.mag = None

        self.control_rate = RateTracker()
        self.estimation_rate = RateTracker()
        self.hw_api_status_rate = RateTracker()
        self.odom_rate = RateTracker()
        self.tracker_cmd_rate = RateTracker()
        self.battery_rate = RateTracker()
        self.gnss_rate = RateTracker()
        self.mag_rate = RateTracker()

        self.full_pub = self.create_publisher(UavStatus, 'uav_status_acquisition/uav_status', 10)
        self.short_pub = self.create_publisher(UavStatusShort, 'uav_status_acquisition/uav_status_short', 10)

        self.create_subscription(ControlManagerDiagnostics, 'control_manager/diagnostics', self._control_diag_cb, 10)
        self.create_subscription(EstimationDiagnostics, 'estimation_manager/diagnostics', self._estimation_diag_cb, 10)
        self.create_subscription(HwApiStatus, 'hw_api/status', self._hw_api_status_cb, 10)
        self.create_subscription(Odometry, 'estimation_manager/odom_main', self._odom_cb, 10)
        self.create_subscription(TrackerCommand, 'control_manager/tracker_cmd', self._tracker_cmd_cb, 10)
        self.create_subscription(BatteryState, 'hw_api/battery_state', self._battery_cb, 10)
        self.create_subscription(NavSatFix, 'hw_api/gnss', self._gnss_cb, 10)
        self.create_subscription(MagneticField, 'hw_api/magnetic_field', self._mag_cb, 10)

        self.create_timer(0.25, self._publish_status)

    def _stamp_seconds(self, stamp: RosTime) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _control_diag_cb(self, msg: ControlManagerDiagnostics) -> None:
        self.control_diag = msg
        self.control_rate.tick(self._stamp_seconds(msg.stamp))

    def _estimation_diag_cb(self, msg: EstimationDiagnostics) -> None:
        self.estimation_diag = msg
        self.estimation_rate.tick(self._stamp_seconds(msg.header.stamp))

    def _hw_api_status_cb(self, msg: HwApiStatus) -> None:
        self.hw_api_status = msg
        self.hw_api_status_rate.tick(self._stamp_seconds(msg.stamp))

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self.odom_rate.tick(self._stamp_seconds(msg.header.stamp))

    def _tracker_cmd_cb(self, msg: TrackerCommand) -> None:
        self.tracker_cmd = msg
        self.tracker_cmd_rate.tick(self._stamp_seconds(msg.header.stamp))

    def _battery_cb(self, msg: BatteryState) -> None:
        self.battery = msg
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            self.battery_rate.tick(self._stamp_seconds(msg.header.stamp))

    def _gnss_cb(self, msg: NavSatFix) -> None:
        self.gnss = msg
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            self.gnss_rate.tick(self._stamp_seconds(msg.header.stamp))

    def _mag_cb(self, msg: MagneticField) -> None:
        self.mag = msg
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            self.mag_rate.tick(self._stamp_seconds(msg.header.stamp))

    def _color(self, available: bool) -> int:
        return 1 if available else 0

    def _memory_stats(self):
        total_kb = 0.0
        available_kb = 0.0
        try:
            with open('/proc/meminfo', 'r', encoding='utf-8') as meminfo:
                for line in meminfo:
                    if line.startswith('MemTotal:'):
                        total_kb = float(line.split()[1])
                    elif line.startswith('MemAvailable:'):
                        available_kb = float(line.split()[1])
        except OSError:
            pass
        return available_kb / 1024.0 / 1024.0, total_kb / 1024.0 / 1024.0

    def _cpu_ghz(self) -> float:
        try:
            with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq', 'r', encoding='utf-8') as cpu_freq:
                return float(cpu_freq.read().strip()) / 1e6
        except OSError:
            return 0.0

    def _publish_status(self) -> None:
        msg = UavStatus()
        short_msg = UavStatusShort()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.uav_name = self.uav_name
        msg.uav_type = self.uav_type
        msg.uav_mass = '-'

        if self.control_diag is not None:
            msg.control_manager_diag_hz = float(self.control_rate.rate)
            msg.control_manager_diag_color = self._color(True)
            msg.controllers = [self.control_diag.active_controller] + [name for name in self.control_diag.available_controllers if name != self.control_diag.active_controller]
            msg.trackers = [self.control_diag.active_tracker] + [name for name in self.control_diag.available_trackers if name != self.control_diag.active_tracker]
            msg.gains = []
            msg.constraints = []
            msg.null_tracker = self.control_diag.active_tracker.lower().startswith('null')
            msg.flying_normally = self.control_diag.flying_normally
            msg.have_goal = self.control_diag.tracker_status.have_goal
            msg.tracking_trajectory = self.control_diag.tracker_status.tracking_trajectory
            msg.callbacks_enabled = self.control_diag.tracker_status.callbacks_enabled
        else:
            msg.controllers = []
            msg.trackers = []
            msg.gains = []
            msg.constraints = []
            msg.control_manager_diag_color = self._color(False)

        msg.secs_flown = int(max(0.0, time.time() - self.sim_start_wall))

        if self.estimation_diag is not None:
            msg.odom_hz = float(self.odom_rate.rate or self.estimation_rate.rate)
            msg.odom_color = self._color(True)
            msg.odom_frame = self.estimation_diag.header.frame_id
            msg.odom_estimators = list(self.estimation_diag.running_state_estimators)
            msg.horizontal_estimator = self.estimation_diag.estimator_horizontal
            msg.vertical_estimator = self.estimation_diag.estimator_vertical
            msg.heading_estimator = self.estimation_diag.estimator_heading
            msg.agl_estimator = self.estimation_diag.estimator_agl_height
            msg.max_flight_z = float(self.estimation_diag.max_flight_z)
        else:
            msg.odom_color = self._color(False)
            msg.odom_estimators = []

        if self.odom is not None:
            msg.odom_x = float(self.odom.pose.pose.position.x)
            msg.odom_y = float(self.odom.pose.pose.position.y)
            msg.odom_z = float(self.odom.pose.pose.position.z)
            msg.odom_hdg = float(quat_to_yaw(self.odom.pose.pose.orientation))
            short_msg.odom_x = msg.odom_x
            short_msg.odom_y = msg.odom_y
            short_msg.odom_z = msg.odom_z
            short_msg.odom_hdg = msg.odom_hdg

        if self.tracker_cmd is not None:
            msg.cmd_x = float(self.tracker_cmd.position.x)
            msg.cmd_y = float(self.tracker_cmd.position.y)
            msg.cmd_z = float(self.tracker_cmd.position.z)
            msg.cmd_hdg = float(getattr(self.tracker_cmd, 'heading', 0.0))
            short_msg.cmd_x = msg.cmd_x
            short_msg.cmd_y = msg.cmd_y
            short_msg.cmd_z = msg.cmd_z
            short_msg.cmd_hdg = msg.cmd_hdg

        free_ram, total_ram = self._memory_stats()
        msg.free_ram = free_ram
        msg.total_ram = total_ram
        msg.cpu_ghz = self._cpu_ghz()
        msg.cpu_load = 0.0
        msg.cpu_load_total = 0.0
        msg.cpu_temperature = 0.0
        try:
            msg.free_hdd = int(shutil.disk_usage('/').free / (1024 ** 3))
        except OSError:
            msg.free_hdd = 0

        if self.hw_api_status is not None:
            msg.hw_api_hz = float(self.hw_api_status_rate.rate)
            msg.hw_api_state_hz = float(self.hw_api_status_rate.rate)
            msg.hw_api_cmd_hz = float(self.tracker_cmd_rate.rate)
            msg.hw_api_color = self._color(True)
            msg.hw_api_mode = self.hw_api_status.mode
            msg.hw_api_armed = self.hw_api_status.armed
            msg.rc_mode = not self.hw_api_status.offboard
        else:
            msg.hw_api_color = self._color(False)

        if self.battery is not None:
            msg.hw_api_battery_hz = float(self.battery_rate.rate)
            msg.battery_volt = float(self.battery.voltage)
            msg.battery_curr = float(self.battery.current)

        if self.gnss is not None:
            msg.hw_api_gnss_ok = self.gnss.status.status >= 0
            msg.hw_api_gnss_fix_type = 3 if self.gnss.status.status >= 0 else 0
            msg.hw_api_gnss_status_hz = float(self.gnss_rate.rate)
            if len(self.gnss.position_covariance) > 0:
                msg.hw_api_gnss_pos_acc = float(self.gnss.position_covariance[0] ** 0.5)

        if self.mag is not None:
            msg.mag_norm_hz = float(self.mag_rate.rate)
            msg.mag_norm = float(math.sqrt(
                self.mag.magnetic_field.x ** 2 +
                self.mag.magnetic_field.y ** 2 +
                self.mag.magnetic_field.z ** 2
            ))

        msg.custom_topics = []
        msg.custom_string_outputs = []
        msg.node_cpu_loads.node_names = []
        msg.node_cpu_loads.cpu_loads = []
        msg.collision_avoidance_enabled = False
        msg.avoiding_collision = False
        msg.automatic_start_can_takeoff = False
        msg.num_other_uavs = 0
        msg.mass_estimate = 0.0
        msg.mass_set = 0.0
        msg.thrust = 0.0

        short_msg.odom_hz = msg.odom_hz
        short_msg.odom_color = msg.odom_color

        self.full_pub.publish(msg)
        self.short_pub.publish(short_msg)


def main() -> None:
    rclpy.init()
    node = StatusProxy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()