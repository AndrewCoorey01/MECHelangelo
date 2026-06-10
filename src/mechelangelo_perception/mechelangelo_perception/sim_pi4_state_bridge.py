#!/usr/bin/env python3
"""
sim_pi4_state_bridge.py

Simulation bridge that mimics the physical Pi 5 pi4_bridge.py behaviour, but
uses ROS String JSON state topics instead of HTTP.

Why this exists
---------------
Physical path:
  Pi 4 Flask /state
      -> pi4_bridge.py polls HTTP
      -> /human_tracking, /human_detected, /arm/right_pose, /arm/left_pose

Simulation path:
  sim_pi4_state_camera.py tracking role publishes /sim_pi4/tracking_state
  sim_pi4_state_camera.py mimicry role publishes /sim_pi4/mimicry_state
      -> this bridge combines them
      -> /human_tracking, /human_detected, /arm/right_pose, /arm/left_pose

This keeps the ROS behaviour node using the same interface in sim and physical.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32MultiArray


TURN_OFFSET = 0.25
OFFSET_SMOOTHING_ALPHA = 0.35


class SimPi4StateBridge(Node):
    def __init__(self):
        super().__init__("sim_pi4_state_bridge")

        self.declare_parameter("tracking_state_topic", "/sim_pi4/tracking_state")
        self.declare_parameter("mimicry_state_topic", "/sim_pi4/mimicry_state")
        self.declare_parameter("state_timeout_sec", 1.0)

        self.tracking_state_topic = self.get_parameter("tracking_state_topic").value
        self.mimicry_state_topic = self.get_parameter("mimicry_state_topic").value
        self.state_timeout_sec = float(self.get_parameter("state_timeout_sec").value)

        # Output topics matching physical bridge.
        self.pub_right = self.create_publisher(String, "/arm/right_pose", 10)
        self.pub_left = self.create_publisher(String, "/arm/left_pose", 10)
        self.pub_turn = self.create_publisher(String, "/arm/turn", 10)
        self.pub_locked = self.create_publisher(Bool, "/arm/locked", 10)

        self.pub_human_detected = self.create_publisher(Bool, "/human_detected", 10)
        self.pub_human_tracking = self.create_publisher(Float32MultiArray, "/human_tracking", 10)

        self.sub_tracking = self.create_subscription(
            String,
            self.tracking_state_topic,
            self.tracking_state_callback,
            10,
        )

        self.sub_mimicry = self.create_subscription(
            String,
            self.mimicry_state_topic,
            self.mimicry_state_callback,
            10,
        )

        self.tracking_state = {}
        self.mimicry_state = {}
        self.last_tracking_rx = 0.0
        self.last_mimicry_rx = 0.0

        self.last_locked = False
        self.filtered_centre_offset = 0.0

        self.timer = self.create_timer(0.1, self.publish_combined_state)

        self.get_logger().info(
            f"sim_pi4_state_bridge started: tracking={self.tracking_state_topic}, "
            f"mimicry={self.mimicry_state_topic}"
        )

    def _parse_json_state(self, msg: String):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("JSON state was not an object")
            return data
        except Exception as exc:
            self.get_logger().warn(f"Invalid state JSON: {exc}")
            return None

    def tracking_state_callback(self, msg: String):
        data = self._parse_json_state(msg)
        if data is None:
            return
        self.tracking_state = data
        self.last_tracking_rx = time.monotonic()

    def mimicry_state_callback(self, msg: String):
        data = self._parse_json_state(msg)
        if data is None:
            return
        self.mimicry_state = data
        self.last_mimicry_rx = time.monotonic()

    def turn_cmd_to_offset(self, turn_cmd):
        turn_cmd = str(turn_cmd or "STOP").strip().upper()

        if turn_cmd == "TURN_LEFT":
            return -TURN_OFFSET

        if turn_cmd == "TURN_RIGHT":
            return TURN_OFFSET

        return 0.0

    def publish_human_tracking(self, locked, centre_offset):
        msg = Float32MultiArray()
        msg.data = [
            1.0 if locked else 0.0,
            float(centre_offset),
            -1.0,  # sim camera has no trusted depth; behaviour must use LiDAR
        ]
        self.pub_human_tracking.publish(msg)

    def publish_combined_state(self):
        now = time.monotonic()
        tracking_fresh = (now - self.last_tracking_rx) <= self.state_timeout_sec
        mimicry_fresh = (now - self.last_mimicry_rx) <= self.state_timeout_sec

        if tracking_fresh:
            locked = bool(self.tracking_state.get("locked", False))
            turn_cmd = str(self.tracking_state.get("turn_cmd") or "STOP").strip().upper()
        else:
            locked = False
            turn_cmd = "STOP"

        raw_offset = self.turn_cmd_to_offset(turn_cmd)
        if not locked:
            raw_offset = 0.0

        self.filtered_centre_offset = (
            (1.0 - OFFSET_SMOOTHING_ALPHA) * self.filtered_centre_offset
            + OFFSET_SMOOTHING_ALPHA * raw_offset
        )

        if mimicry_fresh:
            right_pose = self.mimicry_state.get("right_confirmed") or ""
            left_pose = self.mimicry_state.get("left_confirmed") or ""
        else:
            right_pose = ""
            left_pose = ""

        # Original arm/debug topics.
        self.pub_right.publish(String(data=str(right_pose)))
        self.pub_left.publish(String(data=str(left_pose)))
        self.pub_turn.publish(String(data=turn_cmd))
        self.pub_locked.publish(Bool(data=locked))

        # Detection edge trigger.
        if locked != self.last_locked:
            self.pub_human_detected.publish(Bool(data=locked))
            if locked:
                self.get_logger().warn("Sim human lock acquired — interrupting autonomous behaviour")
            else:
                self.get_logger().warn("Sim human lock lost — behaviour will return to autonomous search")

        # Continuous tracking heartbeat for behaviour.
        self.publish_human_tracking(
            locked=locked,
            centre_offset=self.filtered_centre_offset,
        )

        self.last_locked = locked


def main(args=None):
    rclpy.init(args=args)
    node = SimPi4StateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
