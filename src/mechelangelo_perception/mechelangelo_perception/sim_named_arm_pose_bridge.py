#!/usr/bin/env python3
"""
sim_named_arm_pose_bridge.py

Stable joint3-fixed simulation bridge for MECHelangelo.

This matches the stable SDF/URDF where joint3 is fixed and the Gazebo arm plugin
accepts six active joints on /set_joint_trajectory:

  left_joint1, left_joint2, left_hand_joint,
  right_joint1, right_joint2, right_hand_joint

Important: the JointTrajectory header stamp is intentionally left at zero.
This matches the manual ros2 topic pub command that Gazebo executed correctly.
"""

import json
from typing import Dict, List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "left_joint1",
    "left_joint2",
    "left_hand_joint",
    "right_joint1",
    "right_joint2",
    "right_hand_joint",
]

LEFT_POSES: Dict[str, List[float]] = {
    "arm_down":     [0.00, -1.20, 0.00],
    "straight_arm": [0.00,  0.00, 0.00],
    "arm_90_up":    [0.00,  1.10, 0.35],
    "arm_90_out":   [1.10,  0.00, 0.00],
    "salute":       [0.55,  1.15, 1.00],
    "handshake":    [0.35,  0.30, 0.00],
    "arm_crossed":  [-0.80, 0.60, 0.45],
}

RIGHT_POSES: Dict[str, List[float]] = {
    "arm_down":     [0.00, -1.20, 0.00],
    "straight_arm": [0.00,  0.00, 0.00],
    "arm_90_up":    [0.00,  1.10, 0.35],
    "arm_90_out":   [-1.10, 0.00, 0.00],
    "salute":       [-0.55, 1.15, 1.00],
    "handshake":    [-0.35, 0.30, 0.00],
    "arm_crossed":  [0.80,  0.60, 0.45],
}

VALID_POSES = sorted(set(LEFT_POSES.keys()) | set(RIGHT_POSES.keys()))


def normalise_pose_name(value: str) -> str:
    return value.strip().lower().replace(" ", "_").replace("-", "_")


class SimNamedArmPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("sim_named_arm_pose_bridge")

        self.declare_parameter("right_pose_topic", "/arm/right_pose")
        self.declare_parameter("left_pose_topic", "/arm/left_pose")
        self.declare_parameter("trajectory_topic", "/set_joint_trajectory")
        self.declare_parameter("move_time_sec", 2.0)
        self.declare_parameter("mimicry_state_topic", "")
        self.declare_parameter("suppress_repeated_pose", True)

        self.right_pose_topic = self.get_parameter("right_pose_topic").value
        self.left_pose_topic = self.get_parameter("left_pose_topic").value
        self.trajectory_topic = self.get_parameter("trajectory_topic").value
        self.move_time_sec = float(self.get_parameter("move_time_sec").value)
        self.suppress_repeated_pose = bool(self.get_parameter("suppress_repeated_pose").value)
        mimicry_state_topic = self.get_parameter("mimicry_state_topic").value

        self.current_left = [0.0, 0.0, 0.0]
        self.current_right = [0.0, 0.0, 0.0]
        self.last_left_pose: Optional[str] = None
        self.last_right_pose: Optional[str] = None

        self.trajectory_pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)
        self.create_subscription(String, self.right_pose_topic, self.right_pose_cb, 10)
        self.create_subscription(String, self.left_pose_topic, self.left_pose_cb, 10)

        if mimicry_state_topic:
            self.create_subscription(String, mimicry_state_topic, self.mimicry_state_cb, 10)

        self.get_logger().info(
            f"Sim arm bridge ready: {self.left_pose_topic} + {self.right_pose_topic} -> "
            f"{self.trajectory_topic} | publishing 6 active joints | valid poses={', '.join(VALID_POSES)}"
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

    def set_pose(self, side: str, raw_pose: str) -> None:
        pose = normalise_pose_name(raw_pose)
        if not pose or pose in ("none", "null", "---", "__none__"):
            return

        if side == "left":
            pose_map = LEFT_POSES
            if pose not in pose_map:
                self.get_logger().warn(f"Ignoring unknown LEFT pose '{raw_pose}'. Valid: {VALID_POSES}")
                return
            if self.suppress_repeated_pose and pose == self.last_left_pose:
                return
            self.current_left = pose_map[pose]
            self.last_left_pose = pose

        elif side == "right":
            pose_map = RIGHT_POSES
            if pose not in pose_map:
                self.get_logger().warn(f"Ignoring unknown RIGHT pose '{raw_pose}'. Valid: {VALID_POSES}")
                return
            if self.suppress_repeated_pose and pose == self.last_right_pose:
                return
            self.current_right = pose_map[pose]
            self.last_right_pose = pose

        else:
            self.get_logger().warn(f"Ignoring invalid side '{side}'")
            return

        self.get_logger().info(f"{side.upper()} pose confirmed: {pose} -> publishing full 6-joint trajectory")
        self.publish_trajectory(f"{side}:{pose}")

    def publish_trajectory(self, reason: str) -> None:
        msg = JointTrajectory()
        # Keep stamp zero so gazebo_ros_joint_pose_trajectory executes immediately.
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