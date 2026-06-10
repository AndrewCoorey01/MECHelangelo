#!/usr/bin/env python3
"""
arm_pose_bridge.py — Bridges /arm_pose to Gazebo joint positions.

Subscribes to /arm_pose (Float32MultiArray, 8 values):
  [l_shoulder, l_elbow, l_z_fwd, pad,
   r_shoulder, r_elbow, r_z_fwd, pad]

  All angle values are in degrees (MediaPipe convention).
  A value of -1.0 means that landmark was not detected — the joint
  holds the rest pose instead.

On each message, calls /gazebo/set_model_configuration to teleport
the arm joints directly.  Works because arm links have gravity
disabled in the SDF, so no controller is needed.

Joint mapping
─────────────
  l_z_fwd    → left_joint1  (shoulder Z rotation: arm swings forward/back)
  l_shoulder → left_joint2  (shoulder Y pitch:    arm raises/lowers)
  l_elbow    → left_joint3  (elbow X pitch:       elbow bends)
  (fixed)    → left_joint4  (elbow Y roll:        not tracked, held at 0)

Right arm mirrors left with the Z-swing direction negated.

Angle convention (MediaPipe BlazePose)
───────────────────────────────────────
  shoulder:  0°  = arm hanging at side
             90° = arm horizontal
            >90° = arm raised (clamped to horizontal — joint2 max is 0 rad)

  elbow:    180° = straight arm
             90° = 90° bend
              0° = fully bent (clamped)

  z_fwd:    depth delta in arbitrary units from the perception node.
            Scaled by Z_FWD_GAIN (rad per unit) — tune this constant
            if the forward/backward swing looks wrong.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import SetModelConfiguration

# ── Tuning constants ──────────────────────────────────────────────────────────

# Name of the robot model as spawned in Gazebo (matches -entity in launch file)
GAZEBO_MODEL_NAME = 'mechelangelo'

# Gain applied to l_z_fwd / r_z_fwd before converting to joint1 radians.
# Increase if the arm barely swings; decrease if it overshoots.
Z_FWD_GAIN = 1.0  # rad per unit

# Rest pose used when a landmark is not detected (value == -1.0)
REST_SHOULDER_DEG = 0.0    # arm hanging straight down
REST_ELBOW_DEG    = 160.0  # slight elbow bend — looks natural
REST_Z_FWD        = 0.0    # no forward swing

# ─────────────────────────────────────────────────────────────────────────────

# Ordered list of joints passed to SetModelConfiguration.
# Must match the index order of _compute_positions() output.
ARM_JOINTS = [
    'left_joint1',   # shoulder Z rotation
    'left_joint2',   # shoulder Y pitch
    'left_joint3',   # elbow X pitch
    'left_joint4',   # elbow Y roll (fixed at 0)
    'right_joint1',
    'right_joint2',
    'right_joint3',
    'right_joint4',
]


def _deg_to_rad(deg: float) -> float:
    return math.radians(deg)


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _resolve(value: float, rest: float) -> float:
    """Return rest if value is the 'not detected' sentinel, otherwise value."""
    return rest if value < 0.0 else value


def _arm_joints(shoulder_deg: float, elbow_deg: float, z_fwd: float,
                mirror_z: bool = False) -> list:
    """
    Convert MediaPipe arm angles to four URDF joint positions (radians).

    joint1 (shoulder Z): z_fwd * Z_FWD_GAIN, negated for right arm
    joint2 (shoulder Y): (shoulder_deg - 90) * π/180  →  0 = horizontal
    joint3 (elbow X):    -(180 - elbow_deg) * π/180   →  0 = straight
    joint4 (elbow Y):    0.0  (roll not tracked)
    """
    z_sign = -1.0 if mirror_z else 1.0

    j1 = _clamp(z_sign * z_fwd * Z_FWD_GAIN, -math.pi, math.pi)
    j2 = _clamp(_deg_to_rad(shoulder_deg - 90.0),  -math.pi, 0.0)
    j3 = _clamp(_deg_to_rad(-(180.0 - elbow_deg)), -math.pi, math.pi)
    j4 = 0.0

    return [j1, j2, j3, j4]


class ArmPoseBridge(Node):

    def __init__(self):
        super().__init__('arm_pose_bridge')

        self._pending = False

        self._client = self.create_client(
            SetModelConfiguration,
            '/gazebo/set_model_configuration',
        )

        self._sub = self.create_subscription(
            Float32MultiArray,
            '/arm_pose',
            self._on_arm_pose,
            10,
        )

        self.get_logger().info(
            'ArmPoseBridge: waiting for /gazebo/set_model_configuration ...')
        self._client.wait_for_service()
        self.get_logger().info('ArmPoseBridge: Gazebo joint service ready.')

    def _on_arm_pose(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 8:
            self.get_logger().warn(
                'arm_pose_bridge: expected 8 values, got %d — ignoring.',
                len(msg.data))
            return

        if self._pending:
            return  # drop frame while last service call is in flight

        l_shoulder = _resolve(float(msg.data[0]), REST_SHOULDER_DEG)
        l_elbow    = _resolve(float(msg.data[1]), REST_ELBOW_DEG)
        l_z_fwd    = _resolve(float(msg.data[2]), REST_Z_FWD)
        r_shoulder = _resolve(float(msg.data[4]), REST_SHOULDER_DEG)
        r_elbow    = _resolve(float(msg.data[5]), REST_ELBOW_DEG)
        r_z_fwd    = _resolve(float(msg.data[6]), REST_Z_FWD)

        left  = _arm_joints(l_shoulder, l_elbow, l_z_fwd, mirror_z=False)
        right = _arm_joints(r_shoulder, r_elbow, r_z_fwd, mirror_z=True)

        req = SetModelConfiguration.Request()
        req.model_name      = GAZEBO_MODEL_NAME
        req.urdf_param_name = ''
        req.joint_names     = ARM_JOINTS
        req.joint_positions = left + right

        self._pending = True
        future = self._client.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        self._pending = False
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn(
                    'ArmPoseBridge: set_model_configuration failed: %s',
                    result.status_message)
        except Exception as exc:
            self.get_logger().error(
                'ArmPoseBridge: service call exception: %s', str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = ArmPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
