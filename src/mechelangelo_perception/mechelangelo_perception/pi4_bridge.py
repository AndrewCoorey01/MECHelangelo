#!/usr/bin/env python3
"""
pi4_bridge.py — ROS 2 bridge node for Pi 5.

This keeps the Pi 4 camera code unchanged.

Pi 4 /state provides:
  right_confirmed
  left_confirmed
  turn_cmd        TURN_LEFT / TURN_RIGHT / STOP
  locked          true / false

This bridge publishes:
  /arm/right_pose       std_msgs/String
  /arm/left_pose        std_msgs/String
  /arm/turn             std_msgs/String
  /arm/locked           std_msgs/Bool

  /human_detected       std_msgs/Bool
  /human_tracking       std_msgs/Float32MultiArray
                         [detected, centre_offset, distance_m]

Important:
  The Pi camera has no depth.
  Therefore distance_m is always -1.0.
  Behaviour must use LiDAR for human distance.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32MultiArray

import requests


# ── Config ────────────────────────────────────────────────────────

PI4_URL = "http://172.20.10.3:5000/state"
POLL_HZ = 10.0
REQUEST_TIMEOUT_SEC = 0.30

# Pi 4 only sends TURN_LEFT / TURN_RIGHT / STOP.
# Convert that into the centre_offset format used by behaviour.
#
# Behaviour convention:
#   centre_offset < 0  = human left of image centre
#   centre_offset > 0  = human right of image centre
#   centre_offset == 0 = human centred
TURN_OFFSET = 0.25

# Smooths offset changes so the robot does not snap between left/stop/right.
# Higher = faster response, lower = smoother.
OFFSET_SMOOTHING_ALPHA = 0.35


class Pi4Bridge(Node):
    def __init__(self):
        super().__init__("pi4_bridge")

        # Original arm/debug topics.
        self.pub_right = self.create_publisher(
            String,
            "/arm/right_pose",
            10
        )

        self.pub_left = self.create_publisher(
            String,
            "/arm/left_pose",
            10
        )

        self.pub_turn = self.create_publisher(
            String,
            "/arm/turn",
            10
        )

        self.pub_locked = self.create_publisher(
            Bool,
            "/arm/locked",
            10
        )

        # Behaviour topics.
        self.pub_human_detected = self.create_publisher(
            Bool,
            "/human_detected",
            10
        )

        self.pub_human_tracking = self.create_publisher(
            Float32MultiArray,
            "/human_tracking",
            10
        )

        self.last_locked = False
        self.filtered_centre_offset = 0.0

        self.create_timer(1.0 / POLL_HZ, self.poll)

        self.get_logger().info(
            f"pi4_bridge started — polling {PI4_URL} at {POLL_HZ:.1f} Hz"
        )

    def turn_cmd_to_offset(self, turn_cmd):
        """
        Convert Pi 4 turn command into behaviour centre_offset.

        Pi 4:
          TURN_LEFT  = human is left of centre
          TURN_RIGHT = human is right of centre

        Behaviour:
          negative offset = human left
          positive offset = human right
        """
        turn_cmd = str(turn_cmd or "STOP").strip().upper()

        if turn_cmd == "TURN_LEFT":
            return -TURN_OFFSET

        if turn_cmd == "TURN_RIGHT":
            return TURN_OFFSET

        return 0.0

    def publish_human_tracking(self, locked, centre_offset):
        """
        Publish the continuous tracking message used by behaviour.

        Format:
          data[0] = detected
          data[1] = centre_offset
          data[2] = distance_m

        distance_m = -1.0 because the Pi camera has no depth.
        Behaviour must use LiDAR for distance.
        """
        msg = Float32MultiArray()
        msg.data = [
            1.0 if locked else 0.0,
            float(centre_offset),
            -1.0,
        ]
        self.pub_human_tracking.publish(msg)

    def publish_safe_lost_state(self):
        """
        Called if the Pi 4 cannot be reached.
        This tells behaviour the human is lost.
        """
        self.pub_locked.publish(Bool(data=False))
        self.pub_turn.publish(String(data="STOP"))

        self.filtered_centre_offset = 0.0

        self.publish_human_tracking(
            locked=False,
            centre_offset=0.0
        )

        if self.last_locked:
            self.pub_human_detected.publish(Bool(data=False))

        self.last_locked = False

    def poll(self):
        try:
            response = requests.get(
                PI4_URL,
                timeout=REQUEST_TIMEOUT_SEC
            )
            response.raise_for_status()
            data = response.json()

            if not isinstance(data, dict):
                raise ValueError("Pi 4 /state response was not a JSON object")

        except requests.exceptions.ConnectionError:
            self.get_logger().warn(
                "Cannot reach Pi 4 — check ethernet cable and IP address",
                throttle_duration_sec=5.0
            )
            self.publish_safe_lost_state()
            return

        except requests.exceptions.Timeout:
            self.get_logger().warn(
                "Pi 4 poll timed out",
                throttle_duration_sec=2.0
            )
            self.publish_safe_lost_state()
            return

        except Exception as e:
            self.get_logger().warn(
                f"Poll error: {e}",
                throttle_duration_sec=2.0
            )
            self.publish_safe_lost_state()
            return

        # These keys match your current Pi 4 /state endpoint.
        right_pose = data.get("right_confirmed") or ""
        left_pose = data.get("left_confirmed") or ""
        turn_cmd = str(data.get("turn_cmd") or "STOP").strip().upper()
        locked = bool(data.get("locked", False))

        # Publish original arm/debug topics.
        self.pub_right.publish(String(data=right_pose))
        self.pub_left.publish(String(data=left_pose))
        self.pub_turn.publish(String(data=turn_cmd))
        self.pub_locked.publish(Bool(data=locked))

        # Convert Pi 4 left/right/stop command into behaviour centre offset.
        raw_offset = self.turn_cmd_to_offset(turn_cmd)

        if not locked:
            raw_offset = 0.0

        self.filtered_centre_offset = (
            (1.0 - OFFSET_SMOOTHING_ALPHA) * self.filtered_centre_offset
            + OFFSET_SMOOTHING_ALPHA * raw_offset
        )

        # Only publish /human_detected when lock changes.
        # /human_tracking is published continuously below.
        if locked != self.last_locked:
            self.pub_human_detected.publish(Bool(data=locked))

            if locked:
                self.get_logger().warn(
                    "Human lock acquired — interrupting autonomous behaviour"
                )
            else:
                self.get_logger().warn(
                    "Human lock lost — behaviour will return to autonomous search"
                )

        # Continuous message used by behaviour.
        self.publish_human_tracking(
            locked=locked,
            centre_offset=self.filtered_centre_offset
        )

        self.last_locked = locked


def main():
    rclpy.init()

    node = Pi4Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()