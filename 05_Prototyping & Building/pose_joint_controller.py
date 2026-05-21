#!/usr/bin/env python3
"""
pose_joint_controller.py

Subscribes to /arm_pose (Float32MultiArray from mqtt_bridge) and drives
joint2 (left shoulder) and joint4 (left elbow) by publishing to /joint_states.

/arm_pose layout:
  [0] l_shoulder  (degrees, 0-180)
  [1] l_elbow     (degrees, 0-180)
  [2] l_z_fwd
  [3] reserved
  [4] r_shoulder
  [5] r_elbow
  [6] r_z_fwd
  [7] reserved

Joint limits (from URDF):
  joint2 (left shoulder): -3.142 to 0.0   rad
  joint4 (left elbow):    -1.57  to 1.57  rad

Mapping:
  shoulder: 0 deg -> -pi  (arm down),  180 deg -> 0 (arm up)
  elbow:    0 deg -> -pi/2 (folded),   90 deg -> 0 (neutral), 180 deg -> +pi/2
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# Threshold: ignore updates where angle changed less than this (degrees)
DEADBAND_DEG = 1.0


def shoulder_deg_to_rad(deg: float) -> float:
    """Map 0-180 deg -> -pi to 0 rad. Clamps to URDF limits."""
    rad = (deg / 180.0) * math.pi - math.pi
    return max(-math.pi, min(0.0, rad))


def elbow_deg_to_rad(deg: float) -> float:
    """Map 0-180 deg -> -pi/2 to +pi/2 rad. Clamps to URDF limits."""
    rad = (deg - 90.0) * (math.pi / 180.0)
    return max(-1.57, min(1.57, rad))


class PoseJointController(Node):
    def __init__(self):
        super().__init__('pose_joint_controller')

        self._last_shoulder_deg = None
        self._last_elbow_deg    = None

        # Start joints at neutral positions
        self._shoulder_rad = -math.pi / 2.0  # midpoint of -pi to 0
        self._elbow_rad    = 0.0              # midpoint of -1.57 to 1.57

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/arm_pose',
            self._arm_pose_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Publish at 30hz to keep rviz updated even when no new pose data
        self.timer = self.create_timer(1.0 / 30.0, self._publish_joint_states)

        self.get_logger().info(
            'pose_joint_controller started\n'
            '  Subscribing: /arm_pose\n'
            '  Publishing:  /joint_states\n'
            '  Controlling: joint2 (shoulder), joint4 (elbow)'
        )

    def _arm_pose_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 2:
            self.get_logger().warn('Received /arm_pose with fewer than 2 elements')
            return

        shoulder_deg = data[0]
        elbow_deg    = data[1]

        # -1.0 sentinel means no data this frame
        if shoulder_deg < 0 and elbow_deg < 0:
            return

        # Shoulder
        if shoulder_deg >= 0:
            if (self._last_shoulder_deg is None or
                    abs(shoulder_deg - self._last_shoulder_deg) >= DEADBAND_DEG):
                self._shoulder_rad = shoulder_deg_to_rad(shoulder_deg)
                self._last_shoulder_deg = shoulder_deg

        # Elbow
        if elbow_deg >= 0:
            if (self._last_elbow_deg is None or
                    abs(elbow_deg - self._last_elbow_deg) >= DEADBAND_DEG):
                self._elbow_rad = elbow_deg_to_rad(elbow_deg)
                self._last_elbow_deg = elbow_deg

        self.get_logger().info(
            f'shoulder: {shoulder_deg:>6.1f}deg -> {self._shoulder_rad:+.3f} rad  |  '
            f'elbow: {elbow_deg:>6.1f}deg -> {self._elbow_rad:+.3f} rad'
        )

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = ['joint1', 'joint2', 'joint3', 'joint4']
        msg.position = [0.0, self._shoulder_rad, 0.0, self._elbow_rad]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseJointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()