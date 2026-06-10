#!/usr/bin/env python3
"""
sim_interaction_gate.py

Publishes /interaction_active when the robot has reached a human-interaction
pose in simulation.

Inputs:
  /human_tracking  Float32MultiArray [detected, centre_offset, -1.0]
  /scan            LaserScan

Output:
  /interaction_active Bool

This is intentionally non-invasive:
  - behaviour.cpp still owns /cmd_vel
  - this node only decides when the laptop mimicry camera is allowed to publish
    arm pose names
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan


class SimInteractionGate(Node):
    def __init__(self):
        super().__init__("sim_interaction_gate")

        self.declare_parameter("human_tracking_topic", "/human_tracking")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("interaction_topic", "/interaction_active")

        self.declare_parameter("camera_horizontal_fov_deg", 60.0)
        self.declare_parameter("lidar_window_deg", 10.0)
        self.declare_parameter("target_distance_m", 1.65)
        self.declare_parameter("target_tolerance_m", 0.25)
        self.declare_parameter("centred_offset_threshold", 0.20)

        self.declare_parameter("good_cycles_required", 5)
        self.declare_parameter("release_timeout_sec", 1.0)
        self.declare_parameter("active_range_slack_m", 0.45)

        self.human_tracking_topic = self.get_parameter("human_tracking_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.interaction_topic = self.get_parameter("interaction_topic").value

        self.camera_fov_rad = math.radians(float(self.get_parameter("camera_horizontal_fov_deg").value))
        self.lidar_window_rad = math.radians(float(self.get_parameter("lidar_window_deg").value))
        self.target_distance_m = float(self.get_parameter("target_distance_m").value)
        self.target_tolerance_m = float(self.get_parameter("target_tolerance_m").value)
        self.centred_offset_threshold = float(self.get_parameter("centred_offset_threshold").value)

        self.good_cycles_required = int(self.get_parameter("good_cycles_required").value)
        self.release_timeout_sec = float(self.get_parameter("release_timeout_sec").value)
        self.active_range_slack_m = float(self.get_parameter("active_range_slack_m").value)

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub_tracking = self.create_subscription(
            Float32MultiArray,
            self.human_tracking_topic,
            self.tracking_callback,
            10,
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            sensor_qos,
        )

        self.pub_active = self.create_publisher(Bool, self.interaction_topic, 10)

        self.latest_scan = None
        self.detected = False
        self.offset = 0.0
        self.last_tracking_time = 0.0

        self.active = False
        self.good_count = 0
        self.last_good_time = 0.0

        self.timer = self.create_timer(0.1, self.evaluate)

        self.get_logger().info(
            f"sim_interaction_gate started: target={self.target_distance_m:.2f} m, "
            f"topic={self.interaction_topic}"
        )

    @staticmethod
    def normalise_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def tracking_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        self.detected = msg.data[0] > 0.5
        self.offset = float(msg.data[1])
        self.last_tracking_time = time.monotonic()

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def get_lidar_range_near_human(self):
        if self.latest_scan is None:
            return math.inf

        scan = self.latest_scan
        if not scan.ranges or scan.angle_increment == 0.0:
            return math.inf

        # Same sign convention as behaviour.cpp:
        # centre_offset < 0 means human is left, so bearing is positive.
        human_bearing = -self.offset * self.camera_fov_rad

        min_range = math.inf

        for i, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value <= max(0.05, scan.range_min):
                continue
            if scan.range_max > 0.0 and value > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            diff = self.normalise_angle(angle - human_bearing)

            if abs(diff) <= self.lidar_window_rad:
                min_range = min(min_range, value)

        return min_range

    def evaluate(self):
        now = time.monotonic()
        tracking_fresh = (now - self.last_tracking_time) <= 1.0
        range_m = self.get_lidar_range_near_human()

        centred = abs(self.offset) <= self.centred_offset_threshold
        at_distance = (
            math.isfinite(range_m) and
            abs(range_m - self.target_distance_m) <= self.target_tolerance_m
        )

        good = self.detected and tracking_fresh and centred and at_distance

        if good:
            self.good_count += 1
            self.last_good_time = now
        else:
            self.good_count = 0

        if not self.active:
            if self.good_count >= self.good_cycles_required:
                self.active = True
                self.get_logger().warn(
                    f"Interaction active: human centred, LiDAR range={range_m:.2f} m"
                )
        else:
            still_reasonable = (
                self.detected and
                tracking_fresh and
                math.isfinite(range_m) and
                abs(range_m - self.target_distance_m) <= self.active_range_slack_m
            )

            if not still_reasonable and (now - self.last_good_time) > self.release_timeout_sec:
                self.active = False
                self.get_logger().warn("Interaction inactive: human/range no longer valid")

        self.pub_active.publish(Bool(data=self.active))

        self.get_logger().info(
            f"interaction_active={self.active} detected={self.detected} "
            f"offset={self.offset:.2f} range={range_m:.2f} good_count={self.good_count}",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimInteractionGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
