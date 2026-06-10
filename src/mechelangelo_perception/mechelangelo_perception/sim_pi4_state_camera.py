#!/usr/bin/env python3
"""
sim_pi4_state_camera.py

Simulation version of the physical Pi 4 camera program.

Purpose
-------
This node lets the simulation use the same camera-state interface as the
physical Pi 4 code.

It has two roles:

1) tracking
   - Camera source: usually Gazebo robot camera.
   - Detects and locks onto the sim human.
   - Publishes a JSON state message shaped like the physical Pi 4 /state:
       {
         "right_confirmed": null,
         "left_confirmed": null,
         "turn_cmd": "TURN_LEFT" / "TURN_RIGHT" / "STOP",
         "locked": true / false
       }
   - This state is consumed by sim_pi4_state_bridge.py, which publishes
     /human_tracking for the behaviour node.

2) mimicry
   - Camera source: usually laptop/USB camera.
   - Uses the same KNN-style pose-name classification structure as the
     physical Pi 4 camera code.
   - Does not publish arm poses until /interaction_active is true.
   - Publishes confirmed pose names in the same state shape:
       {
         "right_confirmed": "salute",
         "left_confirmed": "arm_down",
         "turn_cmd": "STOP",
         "locked": true
       }

Debug stream
------------
Each instance also exposes:
  http://localhost:<flask-port>/
  http://localhost:<flask-port>/state
  http://localhost:<flask-port>/video
  http://localhost:<flask-port>/knn/status

Training
--------
For mimicry mode, the KNN files must exist. Defaults:
  /home/pi/pose_knn_right.json
  /home/pi/pose_knn_left.json

You can override them with:
  --right-knn-file /path/to/pose_knn_right.json
  --left-knn-file  /path/to/pose_knn_left.json
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from collections import Counter, defaultdict, deque
from typing import Dict, Optional, Tuple

os.environ["OMP_NUM_THREADS"] = "2"

import cv2
import mediapipe as mp
import numpy as np
from flask import Flask, Response, jsonify

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

cv2.setNumThreads(2)

# -----------------------------------------------------------------------------
# Constants copied/kept compatible with the physical Pi 4 camera logic
# -----------------------------------------------------------------------------

VISIBILITY_FLOOR = 0.50
LOCK_STEAL_MARGIN = 0.20
GRACE_PERIOD_SEC = 5.0
ANCHOR_DRIFT_LIMIT = 0.25
KEY_LM = [11, 12, 13, 14, 15, 16, 23, 24]

CENTER_DEAD_ZONE = 0.08
POSE_COOLDOWN_SEC = 2.0

FEATURE_NAMES = [
    "shoulder_angle",
    "elbow_angle",
    "z_fwd",
    "z_rear",
    "wrist_rel_y",
    "wrist_rel_x",
    "elbow_rel_y",
    "elbow_rel_x",
    "forearm_angle_2d",
    "upper_arm_angle_2d",
    "elbow_from_centre",
]

FEATURE_WEIGHTS = np.array([
    1.5,
    1.5,
    0.5,
    0.5,
    2.0,
    2.0,
    2.5,
    2.0,
    2.0,
    3.0,
    4.0,
], dtype=float)

ANGLE_SCALE = 90.0
Z_SCALE = 0.25
SEGMENT_ANGLE_SCALE = 90.0

# Pose names supported by the current physical camera training setup.
DEFAULT_POSES = [
    "arm_down",
    "straight_arm",
    "arm_90_up",
    "arm_90_out",
    "salute",
    "handshake",
    "arm_crossed",
]

# -----------------------------------------------------------------------------
# Flask/debug globals
# -----------------------------------------------------------------------------

app = Flask(__name__)
frame_lock = threading.Lock()
output_frame = None

# These are updated by the active node instance.
node_instance = None


# -----------------------------------------------------------------------------
# Utility functions
# -----------------------------------------------------------------------------

def _norm_angle(deg: Optional[float]) -> Optional[float]:
    return (deg - 90.0) / ANGLE_SCALE if deg is not None else None


def _norm_z(z: Optional[float]) -> Optional[float]:
    return z / Z_SCALE if z is not None else None


def _norm_seg(deg: Optional[float]) -> Optional[float]:
    return deg / SEGMENT_ANGLE_SCALE if deg is not None else None


def _calc_angle_3d(a, b, c) -> float:
    ba = np.subtract(a, b)
    bc = np.subtract(c, b)
    denom = np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6
    cosang = np.dot(ba, bc) / denom
    return float(np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0))))


def _safe_float(value, default=-1.0) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except Exception:
        return default


def _draw_text(frame, text, x, y, color=(255, 255, 255), scale=0.45, thick=1):
    cv2.putText(frame, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)


# -----------------------------------------------------------------------------
# KNN classifier compatible with the physical Pi 4 feature layout
# -----------------------------------------------------------------------------

class ArmKNN:
    MIN_EXAMPLES = 5
    K = 9
    CONF_FLOOR = 0.60
    VOTE_WINDOW = 10
    VOTE_MAJORITY = 0.60

    def __init__(self, side: str, save_file: str):
        assert side in ("left", "right")
        self.side = side
        self.save_file = save_file
        self.examples = []
        self.pose_counts = defaultdict(int)
        self._vote_buf = deque(maxlen=self.VOTE_WINDOW)
        self.candidate = None
        self.confidence = 0.0
        self._load()

    def _raw_vals(self, angles, z_deltas, wrist, extra):
        if self.side == "left":
            return [
                angles.get("L shoulder"),
                angles.get("L elbow"),
                z_deltas.get("L"),
                z_deltas.get("L elbow wrist"),
                wrist.get("L rel_y"),
                wrist.get("L rel_x"),
                extra.get("L elbow_rel_y"),
                extra.get("L elbow_rel_x"),
                extra.get("L forearm_angle"),
                extra.get("L upper_arm_angle"),
                extra.get("L elbow_from_centre"),
            ]

        return [
            angles.get("R shoulder"),
            angles.get("R elbow"),
            z_deltas.get("R"),
            z_deltas.get("R elbow wrist"),
            wrist.get("R rel_y"),
            wrist.get("R rel_x"),
            extra.get("R elbow_rel_y"),
            extra.get("R elbow_rel_x"),
            extra.get("R forearm_angle"),
            extra.get("R upper_arm_angle"),
            extra.get("R elbow_from_centre"),
        ]

    def _to_norm_vector(self, angles, z_deltas, wrist, extra):
        raw = self._raw_vals(angles, z_deltas, wrist, extra)
        norms = [
            _norm_angle(raw[0]),
            _norm_angle(raw[1]),
            _norm_z(raw[2]),
            _norm_z(raw[3]),
            raw[4],
            raw[5],
            raw[6],
            raw[7],
            _norm_seg(raw[8]),
            _norm_seg(raw[9]),
            raw[10],
        ]

        vec = np.zeros(11, dtype=float)
        mask = np.zeros(11, dtype=float)

        for i, value in enumerate(norms):
            if value is not None:
                vec[i] = float(value)
                mask[i] = 1.0

        return vec, mask

    def record(self, pose_name, angles, z_deltas, wrist, extra):
        vec, mask = self._to_norm_vector(angles, z_deltas, wrist, extra)

        if int(mask.sum()) < 4:
            print(f"[KNN-{self.side}] Only {int(mask.sum())} features visible — skipping")
            return False

        self.examples.append((vec, mask, pose_name))
        self.pose_counts[pose_name] += 1
        self._save()
        print(f"[KNN-{self.side}] Recorded '{pose_name}' ({self.pose_counts[pose_name]} examples)")
        return True

    def clear_pose(self, pose_name):
        self.examples = [(v, m, p) for v, m, p in self.examples if p != pose_name]
        self.pose_counts[pose_name] = 0
        self._save()

    def clear_all(self):
        self.examples = []
        self.pose_counts = defaultdict(int)
        self._save()

    def _dist(self, va, ma, vb, mb):
        common = ma * mb * FEATURE_WEIGHTS
        diff = (va - vb) * common
        return float(np.sqrt(np.dot(diff, diff)))

    def classify(self, angles, z_deltas, wrist, extra):
        if len(self.examples) < self.MIN_EXAMPLES:
            return None, 0.0

        qv, qm = self._to_norm_vector(angles, z_deltas, wrist, extra)

        if qm.sum() < 3:
            return None, 0.0

        dists = sorted([
            (self._dist(qv, qm, ev, em), ep)
            for ev, em, ep in self.examples
        ])

        votes = Counter(pose_name for _, pose_name in dists[:self.K])
        best = votes.most_common(1)[0][0]
        confidence = votes[best] / self.K

        if confidence < self.CONF_FLOOR:
            return None, confidence

        return best, confidence

    def push_vote(self, angles, z_deltas, wrist, extra):
        raw_pose, raw_confidence = self.classify(angles, z_deltas, wrist, extra)
        self.candidate = raw_pose
        self.confidence = raw_confidence

        self._vote_buf.append(raw_pose if raw_pose else "__none__")

        if len(self._vote_buf) < self.VOTE_WINDOW // 2:
            return None

        counts = Counter(self._vote_buf)
        total = len(self._vote_buf)

        best = max(
            (pose_name for pose_name in counts if pose_name != "__none__"),
            key=lambda pose_name: counts[pose_name],
            default=None,
        )

        if best and counts[best] / total >= self.VOTE_MAJORITY:
            return best

        return None

    def status(self):
        return {
            "side": self.side,
            "file": self.save_file,
            "example_count": len(self.examples),
            "pose_counts": dict(self.pose_counts),
            "candidate": self.candidate,
            "confidence": self.confidence,
        }

    def _save(self):
        try:
            os.makedirs(os.path.dirname(self.save_file), exist_ok=True)
            with open(self.save_file, "w") as f:
                json.dump([
                    {
                        "vec": vec.tolist(),
                        "mask": mask.tolist(),
                        "pose": pose,
                    }
                    for vec, mask, pose in self.examples
                ], f)
        except Exception as exc:
            print(f"[KNN-{self.side}] Save failed: {exc}")

    def _load(self):
        if not os.path.exists(self.save_file):
            print(f"[KNN-{self.side}] No training data at {self.save_file}")
            return

        try:
            with open(self.save_file) as f:
                data = json.load(f)

            if data and len(data[0].get("vec", [])) != 11:
                print(
                    f"[KNN-{self.side}] Training file has {len(data[0].get('vec', []))} features, "
                    "but this node expects 11. Not loading old file."
                )
                return

            self.examples = [
                (np.array(item["vec"], dtype=float),
                 np.array(item["mask"], dtype=float),
                 item["pose"])
                for item in data
            ]

            for _, _, pose in self.examples:
                self.pose_counts[pose] += 1

            print(
                f"[KNN-{self.side}] Loaded {len(self.examples)} examples "
                f"across {len(self.pose_counts)} poses"
            )
        except Exception as exc:
            print(f"[KNN-{self.side}] Load failed: {exc}")


# -----------------------------------------------------------------------------
# Locking and turn command state
# -----------------------------------------------------------------------------

class RobotCommand:
    LEFT = "TURN_LEFT"
    RIGHT = "TURN_RIGHT"
    STOP = "STOP"

    def __init__(self):
        self.lateral = self.STOP

    def update(self, shoulder_mid_x_norm: float):
        offset = shoulder_mid_x_norm - 0.5
        if abs(offset) <= CENTER_DEAD_ZONE:
            self.lateral = self.STOP
        elif offset < 0.0:
            self.lateral = self.LEFT
        else:
            self.lateral = self.RIGHT


class LockState:
    def __init__(self):
        self.locked = False
        self.score = 0.0
        self.anchor_x = None
        self.anchor_y = None
        self.lost_since = None

    def _score(self, lm, w, h):
        shoulder_y = (lm[11].y + lm[12].y) * 0.5 * h
        hip_y = (lm[23].y + lm[24].y) * 0.5 * h
        torso_height = abs(hip_y - shoulder_y)
        visibility = sum(lm[i].visibility for i in KEY_LM) / len(KEY_LM)
        return torso_height * visibility

    def _visibility(self, lm):
        return sum(lm[i].visibility for i in KEY_LM) / len(KEY_LM)

    def _shoulder_mid(self, lm):
        return (lm[11].x + lm[12].x) * 0.5, (lm[11].y + lm[12].y) * 0.5

    def _acquire(self, score, x, y):
        self.locked = True
        self.score = score
        self.anchor_x = x
        self.anchor_y = y
        self.lost_since = None

    def _release(self):
        self.locked = False
        self.score = 0.0
        self.anchor_x = None
        self.anchor_y = None
        self.lost_since = None

    def update_with_detection(self, lm, w, h):
        now = time.monotonic()
        score = self._score(lm, w, h)
        visibility = self._visibility(lm)
        x, y = self._shoulder_mid(lm)

        if not self.locked:
            if visibility >= VISIBILITY_FLOOR:
                self._acquire(score, x, y)
                return True
            return False

        same_region = (
            abs(x - self.anchor_x) < ANCHOR_DRIFT_LIMIT and
            abs(y - self.anchor_y) < ANCHOR_DRIFT_LIMIT
        )

        if same_region:
            self.anchor_x = 0.9 * self.anchor_x + 0.1 * x
            self.anchor_y = 0.9 * self.anchor_y + 0.1 * y
            self.score = 0.8 * self.score + 0.2 * score

            if visibility >= VISIBILITY_FLOOR:
                self.lost_since = None
                return True

            if self.lost_since is None:
                self.lost_since = now
            if now - self.lost_since >= GRACE_PERIOD_SEC:
                self._release()
            return False

        if score > self.score * (1.0 + LOCK_STEAL_MARGIN):
            self._acquire(score, x, y)
            return True

        if self.lost_since is None:
            self.lost_since = now
        if now - self.lost_since >= GRACE_PERIOD_SEC:
            self._acquire(score, x, y)
            return True

        return False

    def update_no_detection(self):
        if not self.locked:
            return False

        now = time.monotonic()
        if self.lost_since is None:
            self.lost_since = now

        if now - self.lost_since >= GRACE_PERIOD_SEC:
            self._release()
            return False

        return True

    @property
    def grace_active(self):
        return self.locked and self.lost_since is not None


# -----------------------------------------------------------------------------
# ROS camera node
# -----------------------------------------------------------------------------

class SimPi4StateCamera(Node):
    def __init__(self, parsed_args):
        super().__init__("sim_pi4_state_camera")

        self.args = parsed_args
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frame_lock = threading.Lock()

        self.state_pub = self.create_publisher(String, self.args.state_topic, 10)
        self.active_sub = self.create_subscription(
            Bool,
            self.args.activation_topic,
            self.activation_callback,
            10,
        )

        self.interaction_active = False
        self.robot_cmd = RobotCommand()
        self.lock_state = LockState()

        self.right_knn = ArmKNN("right", self.args.right_knn_file)
        self.left_knn = ArmKNN("left", self.args.left_knn_file)

        self.last_right_pose = None
        self.last_right_time = 0.0
        self.last_left_pose = None
        self.last_left_time = 0.0

        self.last_state = {
            "right_confirmed": None,
            "left_confirmed": None,
            "turn_cmd": "STOP",
            "locked": False,
            "interaction_active": False,
            "source": self.args.role,
        }

        self.mp_pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1 if self.args.role == "mimicry" else 0,
            smooth_landmarks=True,
            enable_segmentation=False,
            smooth_segmentation=False,
            min_detection_confidence=self.args.min_detection_confidence,
            min_tracking_confidence=self.args.min_tracking_confidence,
        )
        self.mp_draw = mp.solutions.drawing_utils

        self.frame_count = 0
        self.cap = None

        if self.args.camera == "sim":
            self.image_sub = self.create_subscription(
                Image,
                self.args.image_topic,
                self.image_callback,
                10,
            )
            self.get_logger().info(f"Subscribing to sim camera: {self.args.image_topic}")
        elif self.args.camera == "usb":
            self.cap = cv2.VideoCapture(self.args.device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
            if not self.cap.isOpened():
                raise RuntimeError(f"Could not open USB camera device {self.args.device}")
            self.get_logger().info(f"Using USB camera device {self.args.device}")
        else:
            raise ValueError(f"Unsupported camera source: {self.args.camera}")

        self.timer = self.create_timer(1.0 / self.args.process_hz, self.process_once)
        self.state_timer = self.create_timer(0.2, self.publish_state)

        self.get_logger().info(
            f"sim_pi4_state_camera started: role={self.args.role}, "
            f"camera={self.args.camera}, state_topic={self.args.state_topic}, "
            f"activation_topic={self.args.activation_topic}"
        )

    def activation_callback(self, msg: Bool):
        self.interaction_active = bool(msg.data)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Image conversion failed: {exc}")
            return

        with self.latest_frame_lock:
            self.latest_frame = frame

    def get_frame(self):
        if self.args.camera == "sim":
            with self.latest_frame_lock:
                if self.latest_frame is None:
                    return None
                return self.latest_frame.copy()

        ok, frame = self.cap.read()
        if not ok:
            return None
        return frame

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps(self.last_state)))

    def _set_state(self, *, right=None, left=None, turn="STOP", locked=False):
        self.last_state = {
            "right_confirmed": right,
            "left_confirmed": left,
            "turn_cmd": turn,
            "locked": bool(locked),
            "interaction_active": bool(self.interaction_active),
            "source": self.args.role,
            "stamp": time.time(),
        }
        self.publish_state()

    def _extract_features(self, lm, w, h):
        angles = {k: None for k in ("R shoulder", "R elbow", "L shoulder", "L elbow")}
        z_deltas = {k: None for k in ("R", "R elbow wrist", "L", "L elbow wrist")}
        wrist = {k: None for k in ("R rel_y", "R rel_x", "L rel_y", "L rel_x")}
        extra = {k: None for k in (
            "R elbow_rel_y",
            "R elbow_rel_x",
            "R forearm_angle",
            "R upper_arm_angle",
            "R elbow_from_centre",
            "L elbow_rel_y",
            "L elbow_rel_x",
            "L forearm_angle",
            "L upper_arm_angle",
            "L elbow_from_centre",
        )}

        def vis(i):
            return lm[i].visibility > VISIBILITY_FLOOR

        def pt3(i):
            return (lm[i].x * w, lm[i].y * h, lm[i].z * w)

        shoulder_mid_x = (lm[11].x + lm[12].x) * 0.5

        # Right arm
        if vis(24) and vis(12) and vis(14):
            angles["R shoulder"] = _calc_angle_3d(pt3(24), pt3(12), pt3(14))
        elif vis(11) and vis(12) and vis(14):
            angles["R shoulder"] = _calc_angle_3d(pt3(11), pt3(12), pt3(14))

        if vis(12) and vis(14) and vis(16):
            angles["R elbow"] = _calc_angle_3d(pt3(12), pt3(14), pt3(16))

        if vis(12) and vis(14):
            z_deltas["R"] = lm[14].z - lm[12].z
            extra["R elbow_rel_y"] = lm[14].y - lm[12].y
            extra["R elbow_rel_x"] = lm[14].x - lm[12].x
            extra["R upper_arm_angle"] = math.degrees(
                math.atan2(lm[14].y - lm[12].y, lm[14].x - lm[12].x)
            )

        if vis(14) and vis(16):
            z_deltas["R elbow wrist"] = lm[16].z - lm[14].z
            extra["R forearm_angle"] = math.degrees(
                math.atan2(lm[16].y - lm[14].y, lm[16].x - lm[14].x)
            )

        if vis(16) and vis(12):
            wrist["R rel_y"] = lm[16].y - lm[12].y
            wrist["R rel_x"] = lm[16].x - lm[12].x

        if vis(14):
            extra["R elbow_from_centre"] = lm[14].x - shoulder_mid_x

        # Left arm
        if vis(23) and vis(11) and vis(13):
            angles["L shoulder"] = _calc_angle_3d(pt3(23), pt3(11), pt3(13))
        elif vis(12) and vis(11) and vis(13):
            angles["L shoulder"] = _calc_angle_3d(pt3(12), pt3(11), pt3(13))

        if vis(11) and vis(13) and vis(15):
            angles["L elbow"] = _calc_angle_3d(pt3(11), pt3(13), pt3(15))

        if vis(11) and vis(13):
            z_deltas["L"] = lm[13].z - lm[11].z
            extra["L elbow_rel_y"] = lm[13].y - lm[11].y
            extra["L elbow_rel_x"] = lm[13].x - lm[11].x
            extra["L upper_arm_angle"] = math.degrees(
                math.atan2(lm[13].y - lm[11].y, lm[13].x - lm[11].x)
            )

        if vis(13) and vis(15):
            z_deltas["L elbow wrist"] = lm[15].z - lm[13].z
            extra["L forearm_angle"] = math.degrees(
                math.atan2(lm[15].y - lm[13].y, lm[15].x - lm[13].x)
            )

        if vis(15) and vis(11):
            wrist["L rel_y"] = lm[15].y - lm[11].y
            wrist["L rel_x"] = lm[15].x - lm[11].x

        if vis(13):
            extra["L elbow_from_centre"] = lm[13].x - shoulder_mid_x

        return angles, z_deltas, wrist, extra

    def _draw_common_overlay(self, frame, lm, locked, use_detection):
        h, w = frame.shape[:2]
        color = (0, 255, 0) if use_detection else (80, 80, 80)
        self.mp_draw.draw_landmarks(
            frame,
            self.results.pose_landmarks,
            mp.solutions.pose.POSE_CONNECTIONS,
            landmark_drawing_spec=self.mp_draw.DrawingSpec(color=color, thickness=1, circle_radius=2),
            connection_drawing_spec=self.mp_draw.DrawingSpec(color=color, thickness=1),
        )

        shoulder_mid_x = (lm[11].x + lm[12].x) * 0.5
        cx = w // 2
        px = int(shoulder_mid_x * w)
        cv2.line(frame, (cx, 0), (cx, h), (80, 80, 80), 1)
        cv2.line(frame, (px, 0), (px, h), (0, 200, 255), 1)

        status = "LOCK" if locked else "SEEK"
        if self.args.role == "mimicry" and not self.interaction_active:
            status = "WAIT_INTERACTION"
        _draw_text(frame, f"{self.args.role.upper()} {status}", 10, 24, (0, 255, 255), 0.55, 2)
        _draw_text(frame, f"state: {self.args.state_topic}", 10, 46, (200, 200, 200), 0.38, 1)

    def process_once(self):
        global output_frame

        frame = self.get_frame()
        if frame is None:
            return

        frame = cv2.resize(frame, (self.args.width, self.args.height))
        self.frame_count += 1

        # Only run MediaPipe every second frame to keep simulation responsive.
        if self.frame_count % 2 != 0:
            with frame_lock:
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
                if ok:
                    output_frame = buf.tobytes()
            return

        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.mp_pose.process(rgb)

        if not self.results.pose_landmarks:
            self.lock_state.update_no_detection()
            self._set_state(
                right=self.last_right_pose if self.args.role == "mimicry" and self.interaction_active else None,
                left=self.last_left_pose if self.args.role == "mimicry" and self.interaction_active else None,
                turn="STOP",
                locked=False,
            )
            _draw_text(frame, "No person detected", 20, 30, (0, 0, 255), 0.7, 2)
        else:
            lm = self.results.pose_landmarks.landmark
            use_detection = self.lock_state.update_with_detection(lm, w, h)
            self._draw_common_overlay(frame, lm, self.lock_state.locked, use_detection)

            shoulder_mid_x = (lm[11].x + lm[12].x) * 0.5
            self.robot_cmd.update(shoulder_mid_x)

            if self.args.role == "tracking":
                # Match the physical Pi 4 /state shape.
                self._set_state(
                    right=None,
                    left=None,
                    turn=self.robot_cmd.lateral if self.lock_state.locked else "STOP",
                    locked=self.lock_state.locked,
                )
                _draw_text(frame, f"turn_cmd: {self.robot_cmd.lateral}", 10, h - 18, (0, 255, 128), 0.5, 1)

            elif self.args.role == "mimicry":
                # The laptop camera should only drive arms after the robot has arrived.
                if self.interaction_active and use_detection and not self.lock_state.grace_active:
                    angles, z_deltas, wrist, extra = self._extract_features(lm, w, h)
                    now = time.monotonic()

                    right_vote = self.right_knn.push_vote(angles, z_deltas, wrist, extra)
                    left_vote = self.left_knn.push_vote(angles, z_deltas, wrist, extra)

                    if right_vote and right_vote != self.last_right_pose and (now - self.last_right_time) >= POSE_COOLDOWN_SEC:
                        self.last_right_pose = right_vote
                        self.last_right_time = now
                        self.get_logger().info(f"Right pose confirmed: {right_vote}")

                    if left_vote and left_vote != self.last_left_pose and (now - self.last_left_time) >= POSE_COOLDOWN_SEC:
                        self.last_left_pose = left_vote
                        self.last_left_time = now
                        self.get_logger().info(f"Left pose confirmed: {left_vote}")

                    self._set_state(
                        right=self.last_right_pose,
                        left=self.last_left_pose,
                        turn="STOP",
                        locked=self.lock_state.locked,
                    )

                    _draw_text(frame, f"R: {self.last_right_pose or '---'}", 10, h - 38, (255, 200, 0), 0.5, 1)
                    _draw_text(frame, f"L: {self.last_left_pose or '---'}", 10, h - 18, (255, 128, 0), 0.5, 1)
                else:
                    # Do not move arms before the robot is at the interaction distance.
                    self._set_state(
                        right=None,
                        left=None,
                        turn="STOP",
                        locked=False,
                    )
                    _draw_text(frame, "Waiting for /interaction_active before publishing arm poses",
                               10, h - 18, (0, 165, 255), 0.4, 1)

        with frame_lock:
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if ok:
                output_frame = buf.tobytes()

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


# -----------------------------------------------------------------------------
# Flask routes
# -----------------------------------------------------------------------------

@app.route("/")
def index():
    port = getattr(node_instance.args, "flask_port", 0) if node_instance else 0
    return (
        '<html><body style="background:#111;color:white;text-align:center">'
        f"<h2>sim_pi4_state_camera</h2>"
        f"<p>role: {node_instance.args.role if node_instance else 'unknown'}</p>"
        f'<img src="/video"><br><br>'
        f'<a style="color:cyan" href="/state">/state</a> | '
        f'<a style="color:cyan" href="/knn/status">/knn/status</a>'
        f"<p>port {port}</p>"
        "</body></html>"
    )


@app.route("/video")
def video():
    def generate_stream():
        while True:
            with frame_lock:
                frame = output_frame
            if frame is None:
                time.sleep(0.1)
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.1)

    return Response(generate_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/state")
def state_route():
    if node_instance is None:
        return jsonify({})
    return jsonify(node_instance.last_state)


@app.route("/knn/status")
def knn_status():
    if node_instance is None:
        return jsonify({})
    return jsonify({
        "right": node_instance.right_knn.status(),
        "left": node_instance.left_knn.status(),
        "interaction_active": node_instance.interaction_active,
        "last_right_pose": node_instance.last_right_pose,
        "last_left_pose": node_instance.last_left_pose,
    })


@app.route("/record/right/<pose_name>")
def record_right(pose_name):
    if node_instance is None or node_instance.args.role != "mimicry":
        return jsonify({"ok": False, "error": "not in mimicry mode"})
    if not hasattr(node_instance, "results") or not node_instance.results.pose_landmarks:
        return jsonify({"ok": False, "error": "no landmarks"})
    lm = node_instance.results.pose_landmarks.landmark
    # Use configured dimensions; this is only for training convenience.
    angles, z_deltas, wrist, extra = node_instance._extract_features(lm, node_instance.args.width, node_instance.args.height)
    ok = node_instance.right_knn.record(pose_name, angles, z_deltas, wrist, extra)
    return jsonify({"ok": ok, "pose": pose_name})


@app.route("/record/left/<pose_name>")
def record_left(pose_name):
    if node_instance is None or node_instance.args.role != "mimicry":
        return jsonify({"ok": False, "error": "not in mimicry mode"})
    if not hasattr(node_instance, "results") or not node_instance.results.pose_landmarks:
        return jsonify({"ok": False, "error": "no landmarks"})
    lm = node_instance.results.pose_landmarks.landmark
    angles, z_deltas, wrist, extra = node_instance._extract_features(lm, node_instance.args.width, node_instance.args.height)
    ok = node_instance.left_knn.record(pose_name, angles, z_deltas, wrist, extra)
    return jsonify({"ok": ok, "pose": pose_name})


@app.route("/poses")
def poses_route():
    return jsonify(DEFAULT_POSES)


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description="Simulation Pi 4 state camera")
    parser.add_argument("--role", choices=["tracking", "mimicry"], required=True)
    parser.add_argument("--camera", choices=["sim", "usb"], required=True)

    parser.add_argument("--image-topic", default="/mechelangelo/camera/image_raw")
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)

    parser.add_argument("--state-topic", default="/sim_pi4/state")
    parser.add_argument("--activation-topic", default="/interaction_active")

    parser.add_argument("--right-knn-file", default="/home/pi/pose_knn_right.json")
    parser.add_argument("--left-knn-file", default="/home/pi/pose_knn_left.json")

    parser.add_argument("--process-hz", type=float, default=15.0)
    parser.add_argument("--min-detection-confidence", type=float, default=0.55)
    parser.add_argument("--min-tracking-confidence", type=float, default=0.55)

    parser.add_argument("--flask-host", default="0.0.0.0")
    parser.add_argument("--flask-port", type=int, default=5010)

    return parser.parse_args(remove_ros_args(sys.argv)[1:])


def main(args=None):
    global node_instance

    parsed_args = parse_args()

    rclpy.init(args=args)
    node_instance = SimPi4StateCamera(parsed_args)

    flask_thread = threading.Thread(
        target=lambda: app.run(
            host=parsed_args.flask_host,
            port=parsed_args.flask_port,
            threaded=True,
            use_reloader=False,
        ),
        daemon=True,
    )
    flask_thread.start()

    try:
        rclpy.spin(node_instance)
    except KeyboardInterrupt:
        pass
    finally:
        node_instance.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
