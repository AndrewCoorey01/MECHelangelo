#!/usr/bin/env python3
"""
sim_named_arm_pose_bridge.py

MECHelangelo simulation bridge for the 4-servo arm model.

This version assumes joint3 is ACTIVE in the SDF/URDF and publishes 8 joints:

  left_joint1, left_joint2, left_joint3, left_hand_joint,
  right_joint1, right_joint2, right_joint3, right_hand_joint

It converts the real robot LX-16A physical servo angles into Gazebo joint radians
using the project convention:

  physical servo centre 120 deg == sim joint 0 rad
  physical servo range 0..240 deg == sim joint range -2.0944..+2.0944 rad

The right-arm servo map is copied from the physical camera code.
The pasted physical camera code still had an empty LEFT_POSE_MAP, so this bridge
uses the right map as a mirrored left-arm fallback until calibrated left values
are added below.
"""

import json
import math
from typing import Dict, List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_hand_joint",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_hand_joint",
]

# Real robot right-arm servo pose map copied from the physical camera code.
# Servo 2 salute is below hardware min in the real code; move_servos() clamps it to 0 deg.
RIGHT_SERVO_POSE_MAP: Dict[str, Dict[int, float]] = {
    "arm_down":     {1: 120.0, 2: 30.0, 3: 120.0,  4: 120.0},
    "straight_arm": {1: 121.4, 2: 115.0, 3: 129.4, 4: 118.6},
    "arm_90_up":    {1: 120.0,  2: 120.0, 3: 120.0, 4: 215.0},
    "arm_90_out":   {1: 120.0,  2: 120.0, 3: 215.0, 4: 120.0},
    "salute":       {1: 169.0,  2: 183.0,  3: 6.0, 4: 158.0},
    "handshake":    {1: 120.0, 2: 35.0,  3: 45.0,  4: 120.0},
    "arm_crossed":  {1: 203.5, 2: 131.5, 3: 25.0, 4: 120.0},
}

# Add real calibrated left-arm values here when available.
# If this stays empty, the bridge uses RIGHT_SERVO_POSE_MAP as a mirrored fallback.
LEFT_SERVO_POSE_MAP: Dict[str, Dict[int, float]] = {
    # Example format:
    # "arm_down": {1: 121.0, 2: 115.0, 3: 36.5, 4: 117.8},
}

VALID_POSES = sorted(set(RIGHT_SERVO_POSE_MAP.keys()) | set(LEFT_SERVO_POSE_MAP.keys()))


def normalise_pose_name(value: str) -> str:
    return value.strip().lower().replace(" ", "_").replace("-", "_")


def clamp_servo_deg(angle_deg: float) -> float:
    """Match the physical move_servos() safety clamp."""
    return max(0.0, min(240.0, float(angle_deg)))


def servo_deg_to_joint_rad(angle_deg: float, centre_deg: float = 120.0, sign: float = 1.0) -> float:
    """Convert physical LX-16A degrees to sim joint radians."""
    return sign * math.radians(clamp_servo_deg(angle_deg) - centre_deg)


class SimNamedArmPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("sim_named_arm_pose_bridge")

        self.declare_parameter("right_pose_topic", "/arm/right_pose")
        self.declare_parameter("left_pose_topic", "/arm/left_pose")
        self.declare_parameter("trajectory_topic", "/set_joint_trajectory")
        self.declare_parameter("move_time_sec", 2.0)
        self.declare_parameter("mimicry_state_topic", "")
        self.declare_parameter("suppress_repeated_pose", True)

        # Servo ID to sim joint mapping. With joint3 active, this should match the real 4-servo arm.
        self.declare_parameter("left_joint1_servo_id", 1)
        self.declare_parameter("left_joint2_servo_id", 2)
        self.declare_parameter("left_joint3_servo_id", 3)
        self.declare_parameter("left_hand_servo_id", 4)

        self.declare_parameter("right_joint1_servo_id", 1)
        self.declare_parameter("right_joint2_servo_id", 2)
        self.declare_parameter("right_joint3_servo_id", 3)
        self.declare_parameter("right_hand_servo_id", 4)

        # Per-joint sign flips for final calibration without editing the SDF/URDF again.
        self.declare_parameter("left_joint1_sign", -1.0)
        self.declare_parameter("left_joint2_sign", 1.0)
        self.declare_parameter("left_joint3_sign", 1.0)
        self.declare_parameter("left_hand_sign", 1.0)

        self.declare_parameter("right_joint1_sign", 1.0)
        self.declare_parameter("right_joint2_sign", 1.0)
        self.declare_parameter("right_joint3_sign", 1.0)
        self.declare_parameter("right_hand_sign", 1.0)

        # If true, use the right physical map for left poses when LEFT_SERVO_POSE_MAP is empty.
        # This is only a mirrored fallback. Replace LEFT_SERVO_POSE_MAP when calibrated left values exist.
        self.declare_parameter("use_right_map_as_left_fallback", True)

        self.right_pose_topic = self.get_parameter("right_pose_topic").value
        self.left_pose_topic = self.get_parameter("left_pose_topic").value
        self.trajectory_topic = self.get_parameter("trajectory_topic").value
        self.move_time_sec = float(self.get_parameter("move_time_sec").value)
        self.suppress_repeated_pose = bool(self.get_parameter("suppress_repeated_pose").value)
        self.use_right_map_as_left_fallback = bool(self.get_parameter("use_right_map_as_left_fallback").value)
        mimicry_state_topic = self.get_parameter("mimicry_state_topic").value

        self.left_ids = [
            int(self.get_parameter("left_joint1_servo_id").value),
            int(self.get_parameter("left_joint2_servo_id").value),
            int(self.get_parameter("left_joint3_servo_id").value),
            int(self.get_parameter("left_hand_servo_id").value),
        ]
        self.right_ids = [
            int(self.get_parameter("right_joint1_servo_id").value),
            int(self.get_parameter("right_joint2_servo_id").value),
            int(self.get_parameter("right_joint3_servo_id").value),
            int(self.get_parameter("right_hand_servo_id").value),
        ]
        self.left_signs = [
            float(self.get_parameter("left_joint1_sign").value),
            float(self.get_parameter("left_joint2_sign").value),
            float(self.get_parameter("left_joint3_sign").value),
            float(self.get_parameter("left_hand_sign").value),
        ]
        self.right_signs = [
            float(self.get_parameter("right_joint1_sign").value),
            float(self.get_parameter("right_joint2_sign").value),
            float(self.get_parameter("right_joint3_sign").value),
            float(self.get_parameter("right_hand_sign").value),
        ]

        self.current_left = [0.0, 0.0, 0.0, 0.0]
        self.current_right = [0.0, 0.0, 0.0, 0.0]
        self.last_left_pose: Optional[str] = None
        self.last_right_pose: Optional[str] = None

        self.trajectory_pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)
        self.create_subscription(String, self.right_pose_topic, self.right_pose_cb, 10)
        self.create_subscription(String, self.left_pose_topic, self.left_pose_cb, 10)

        if mimicry_state_topic:
            self.create_subscription(String, mimicry_state_topic, self.mimicry_state_cb, 10)

        self.get_logger().info(
            f"Sim arm bridge ready: {self.left_pose_topic} + {self.right_pose_topic} -> {self.trajectory_topic}"
        )
        self.get_logger().info(f"Publishing 8 active joints: {', '.join(JOINT_NAMES)}")
        self.get_logger().info(
            "RIGHT mapping: "
            f"joint1<-servo{self.right_ids[0]} sign={self.right_signs[0]}, "
            f"joint2<-servo{self.right_ids[1]} sign={self.right_signs[1]}, "
            f"joint3<-servo{self.right_ids[2]} sign={self.right_signs[2]}, "
            f"hand<-servo{self.right_ids[3]} sign={self.right_signs[3]}"
        )
        self.get_logger().info(
            "LEFT mapping: "
            f"joint1<-servo{self.left_ids[0]} sign={self.left_signs[0]}, "
            f"joint2<-servo{self.left_ids[1]} sign={self.left_signs[1]}, "
            f"joint3<-servo{self.left_ids[2]} sign={self.left_signs[2]}, "
            f"hand<-servo{self.left_ids[3]} sign={self.left_signs[3]}"
        )
        if not LEFT_SERVO_POSE_MAP and self.use_right_map_as_left_fallback:
            self.get_logger().warn(
                "LEFT_SERVO_POSE_MAP is empty. Using mirrored RIGHT_SERVO_POSE_MAP fallback for left poses. "
                "Replace LEFT_SERVO_POSE_MAP with calibrated left-arm values when available."
            )

        self.publish_trajectory("startup_neutral")

    def right_pose_cb(self, msg: String) -> None:
        self.set_pose("right", msg.data)

    def left_pose_cb(self, msg: String) -> None:
        self.set_pose("left", msg.data)

    def mimicry_state_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        right = data.get("right_confirmed") or data.get("right_pose") or data.get("right")
        left = data.get("left_confirmed") or data.get("left_pose") or data.get("left")

        if right:
            self.set_pose("right", str(right))
        if left:
            self.set_pose("left", str(left))

    def pose_map_for_side(self, side: str) -> Dict[str, Dict[int, float]]:
        if side == "right":
            return RIGHT_SERVO_POSE_MAP
        if LEFT_SERVO_POSE_MAP:
            return LEFT_SERVO_POSE_MAP
        if self.use_right_map_as_left_fallback:
            return RIGHT_SERVO_POSE_MAP
        return LEFT_SERVO_POSE_MAP

    def pose_to_joints(self, side: str, pose: str) -> Optional[List[float]]:
        pose_map = self.pose_map_for_side(side)
        servo_map = pose_map.get(pose)
        if not servo_map:
            return None

        servo_ids = self.right_ids if side == "right" else self.left_ids
        signs = self.right_signs if side == "right" else self.left_signs

        try:
            joints = [
                servo_deg_to_joint_rad(servo_map[servo_ids[0]], sign=signs[0]),
                servo_deg_to_joint_rad(servo_map[servo_ids[1]], sign=signs[1]),
                servo_deg_to_joint_rad(servo_map[servo_ids[2]], sign=signs[2]),
                servo_deg_to_joint_rad(servo_map[servo_ids[3]], sign=signs[3]),
            ]
        except KeyError as exc:
            self.get_logger().error(f"Servo ID {exc} not available in {side} pose '{pose}': {servo_map}")
            return None

        source = "real-right-map"
        if side == "left" and LEFT_SERVO_POSE_MAP:
            source = "real-left-map"
        elif side == "left":
            source = "mirrored-right-fallback"

        self.get_logger().info(
            f"{side.upper()} physical pose {pose} ({source}): "
            f"servo={servo_map} -> sim joints={[round(v, 3) for v in joints]}"
        )
        return joints

    def set_pose(self, side: str, raw_pose: str) -> None:
        pose = normalise_pose_name(raw_pose)
        if not pose or pose in ("none", "null", "---", "__none__"):
            return

        if side == "right":
            joints = self.pose_to_joints("right", pose)
            if joints is None:
                self.get_logger().warn(f"Ignoring unknown RIGHT pose '{raw_pose}'. Valid: {VALID_POSES}")
                return
            if self.suppress_repeated_pose and pose == self.last_right_pose:
                return
            self.current_right = joints
            self.last_right_pose = pose

        elif side == "left":
            joints = self.pose_to_joints("left", pose)
            if joints is None:
                self.get_logger().warn(f"Ignoring unknown LEFT pose '{raw_pose}'. Valid: {VALID_POSES}")
                return
            if self.suppress_repeated_pose and pose == self.last_left_pose:
                return
            self.current_left = joints
            self.last_left_pose = pose

        else:
            return

        self.get_logger().info(f"{side.upper()} pose confirmed: {pose} -> publishing full 8-joint trajectory")
        self.publish_trajectory(f"{side}:{pose}")

    def publish_trajectory(self, reason: str) -> None:
        msg = JointTrajectory()
        # Keep stamp zero to match the manual command that Gazebo executes immediately.
        msg.header.frame_id = "base_link"
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = list(self.current_left) + list(self.current_right)
        point.time_from_start = Duration(seconds=self.move_time_sec).to_msg()
        msg.points.append(point)

        self.trajectory_pub.publish(msg)
        self.get_logger().info(f"Published {reason}: positions={[round(v, 3) for v in point.positions]}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimNamedArmPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()