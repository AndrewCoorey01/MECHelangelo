#!/usr/bin/env python3
"""
simple_camera_human_approach.py

A deliberately minimal physical-robot human approach controller.

It does NOT subscribe to LaserScan and does NOT use the large behaviour node.

Inputs
------
/human_tracking  std_msgs/msg/Float32MultiArray
    data[0] = detected (1.0 or 0.0)
    data[1] = centre_offset
    data[2] = ignored

Outputs
-------
/cmd_vel             geometry_msgs/msg/Twist
/interaction_active  std_msgs/msg/Bool
/voice_prompt        std_msgs/msg/String

Control sequence
----------------
1. Stop if the camera has no human lock.
2. Turn in place until the person is centred.
3. Drive straight for a calibrated amount of actual forward-motion time.
4. Stop and publish interaction_active=true.

This is intended for a controlled demonstration in a clear area. It has no
LiDAR obstacle avoidance. Keep direct access to the motor power switch.
"""

import math
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String


class State(Enum):
    WAITING = auto()
    ALIGNING = auto()
    APPROACHING = auto()
    INTERACTION = auto()


class SimpleCameraHumanApproach(Node):
    def __init__(self) -> None:
        super().__init__("simple_camera_human_approach")

        # All important values can be changed from the command line without
        # editing this file.
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("turn_speed_radps", 0.45)
        self.declare_parameter("forward_speed_mps", 0.10)
        self.declare_parameter("approach_seconds", 12.0)
        self.declare_parameter("centre_threshold", 0.08)
        self.declare_parameter("recenter_threshold", 0.12)
        self.declare_parameter("centred_samples_required", 5)
        self.declare_parameter("human_message_timeout_sec", 0.75)
        self.declare_parameter("reset_progress_after_lost_sec", 5.0)

        # Current bridge convention after the earlier steering correction:
        # negative offset means the camera asks for a physical right turn.
        # With the current base, positive angular.z physically turns right.
        #
        # Therefore:
        #     angular.z = -sign(offset) * turn_speed
        #
        # If the robot turns the wrong way, run with:
        #     -p steering_sign:=1.0
        self.declare_parameter("steering_sign", -1.0)

        self.control_hz = float(
            self.get_parameter("control_hz").value
        )
        self.turn_speed = float(
            self.get_parameter("turn_speed_radps").value
        )
        self.forward_speed = float(
            self.get_parameter("forward_speed_mps").value
        )
        self.approach_seconds = float(
            self.get_parameter("approach_seconds").value
        )
        self.centre_threshold = float(
            self.get_parameter("centre_threshold").value
        )
        self.recenter_threshold = float(
            self.get_parameter("recenter_threshold").value
        )
        self.centred_samples_required = int(
            self.get_parameter("centred_samples_required").value
        )
        self.human_message_timeout = float(
            self.get_parameter("human_message_timeout_sec").value
        )
        self.reset_progress_after_lost = float(
            self.get_parameter("reset_progress_after_lost_sec").value
        )
        self.steering_sign = float(
            self.get_parameter("steering_sign").value
        )

        if self.control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if self.turn_speed <= 0.0:
            raise ValueError("turn_speed_radps must be positive")
        if self.forward_speed <= 0.0:
            raise ValueError("forward_speed_mps must be positive")
        if self.approach_seconds <= 0.0:
            raise ValueError("approach_seconds must be positive")

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )
        self.interaction_pub = self.create_publisher(
            Bool,
            "/interaction_active",
            10,
        )
        self.voice_pub = self.create_publisher(
            String,
            "/voice_prompt",
            10,
        )

        self.create_subscription(
            Float32MultiArray,
            "/human_tracking",
            self.human_callback,
            10,
        )

        self.state = State.WAITING
        self.human_detected = False
        self.centre_offset = 0.0
        self.last_human_message_time = None
        self.lost_since = None

        self.centred_samples = 0
        self.forward_motion_seconds = 0.0
        self.interaction_announced = False

        self.last_control_time = self.get_clock().now()
        self.last_status_log_time = self.get_clock().now()

        self.create_timer(
            1.0 / self.control_hz,
            self.control_loop,
        )

        estimated_distance = (
            self.forward_speed * self.approach_seconds
        )

        self.get_logger().warn(
            "SIMPLE CAMERA-ONLY APPROACH ACTIVE. "
            "LiDAR and the main behaviour node are intentionally ignored."
        )
        self.get_logger().info(
            "Settings: turn=%.2f rad/s forward=%.2f m/s "
            "forward_time=%.1f s estimated_travel=%.2f m "
            "steering_sign=%+.1f",
            self.turn_speed,
            self.forward_speed,
            self.approach_seconds,
            estimated_distance,
            self.steering_sign,
        )

    def human_callback(
        self,
        msg: Float32MultiArray,
    ) -> None:
        if len(msg.data) < 2:
            self.get_logger().error(
                "/human_tracking needs at least "
                "[detected, centre_offset]"
            )
            return

        self.human_detected = float(msg.data[0]) >= 0.5
        self.centre_offset = float(msg.data[1])
        self.last_human_message_time = self.get_clock().now()

        if self.human_detected:
            self.lost_since = None

    def publish_cmd(
        self,
        linear_x: float = 0.0,
        angular_z: float = 0.0,
    ) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def publish_stop(self) -> None:
        self.publish_cmd(0.0, 0.0)

    def human_message_is_fresh(self, now) -> bool:
        if self.last_human_message_time is None:
            return False

        age = (
            now - self.last_human_message_time
        ).nanoseconds / 1e9

        return age <= self.human_message_timeout

    def enter_interaction(self) -> None:
        self.state = State.INTERACTION
        self.publish_stop()
        self.interaction_pub.publish(Bool(data=True))

        if not self.interaction_announced:
            self.voice_pub.publish(
                String(
                    data=(
                        "Hello. I have reached the interaction position."
                    )
                )
            )
            self.get_logger().warn(
                "INTERACTION REACHED after %.2f s of actual "
                "centred forward motion.",
                self.forward_motion_seconds,
            )
            self.interaction_announced = True

    def status_log(
        self,
        now,
        message: str,
        *args,
    ) -> None:
        elapsed = (
            now - self.last_status_log_time
        ).nanoseconds / 1e9

        if elapsed >= 0.5:
            self.get_logger().info(message, *args)
            self.last_status_log_time = now

    def control_loop(self) -> None:
        now = self.get_clock().now()
        dt = (
            now - self.last_control_time
        ).nanoseconds / 1e9
        self.last_control_time = now
        dt = max(0.0, min(dt, 0.25))

        if self.state == State.INTERACTION:
            self.publish_stop()
            self.interaction_pub.publish(Bool(data=True))
            return

        self.interaction_pub.publish(Bool(data=False))

        fresh = self.human_message_is_fresh(now)

        if not fresh or not self.human_detected:
            self.publish_stop()
            self.centred_samples = 0

            if self.lost_since is None:
                self.lost_since = now

            lost_duration = (
                now - self.lost_since
            ).nanoseconds / 1e9

            if lost_duration >= self.reset_progress_after_lost:
                if self.forward_motion_seconds > 0.0:
                    self.get_logger().warn(
                        "Human lost for %.1f s. Resetting approach timer.",
                        lost_duration,
                    )
                self.forward_motion_seconds = 0.0
                self.state = State.WAITING
            else:
                self.state = State.WAITING

            self.status_log(
                now,
                "WAITING: camera lock unavailable. "
                "progress=%.2f/%.2f s",
                self.forward_motion_seconds,
                self.approach_seconds,
            )
            return

        offset_abs = abs(self.centre_offset)

        # Hysteresis prevents rapid switching between turn and forward motion.
        threshold = (
            self.recenter_threshold
            if self.state == State.APPROACHING
            else self.centre_threshold
        )

        if offset_abs > threshold:
            self.state = State.ALIGNING
            self.centred_samples = 0

            turn_direction = math.copysign(
                1.0,
                self.centre_offset,
            )
            angular_z = (
                self.steering_sign *
                turn_direction *
                self.turn_speed
            )

            self.publish_cmd(
                linear_x=0.0,
                angular_z=angular_z,
            )

            self.status_log(
                now,
                "ALIGNING: offset=%+.3f cmd angular=%+.2f rad/s "
                "progress=%.2f/%.2f s",
                self.centre_offset,
                angular_z,
                self.forward_motion_seconds,
                self.approach_seconds,
            )
            return

        self.centred_samples += 1

        if self.centred_samples < self.centred_samples_required:
            self.state = State.ALIGNING
            self.publish_stop()

            self.status_log(
                now,
                "CENTRE CONFIRM: offset=%+.3f sample=%d/%d",
                self.centre_offset,
                self.centred_samples,
                self.centred_samples_required,
            )
            return

        self.state = State.APPROACHING
        self.publish_cmd(
            linear_x=self.forward_speed,
            angular_z=0.0,
        )
        self.forward_motion_seconds += dt

        self.status_log(
            now,
            "APPROACHING: offset=%+.3f cmd=%.2f m/s "
            "progress=%.2f/%.2f s",
            self.centre_offset,
            self.forward_speed,
            self.forward_motion_seconds,
            self.approach_seconds,
        )

        if self.forward_motion_seconds >= self.approach_seconds:
            self.enter_interaction()

    def emergency_stop(self) -> None:
        for _ in range(3):
            self.publish_stop()
        self.interaction_pub.publish(Bool(data=False))


def main() -> None:
    rclpy.init()
    node = SimpleCameraHumanApproach()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
