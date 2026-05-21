#!/usr/bin/env python3
"""
MECHelangelo human pose tracking node.

Supports three camera sources from terminal arguments:

Simulation camera:
    python3 pose_tracking_human_ros_merged.py --camera sim

Raspberry Pi Camera:
    python3 pose_tracking_human_ros_merged.py --camera pi

USB / laptop webcam:
    python3 pose_tracking_human_ros_merged.py --camera usb --device 0

All modes publish the same ROS output:
    /human_tracking  std_msgs/msg/Float32MultiArray

Message format:
    data[0] = detected flag, 1.0 if locked/valid human, else 0.0
    data[1] = centre offset, negative = human left of centre, positive = right
    data[2] = estimated distance in metres, or -1.0 if unavailable
"""

import argparse
import json
import os
import threading
import time
from collections import deque

# Limit CPU threads before importing/using OpenCV heavily.
os.environ["OMP_NUM_THREADS"] = "2"

import cv2
import mediapipe as mp
import numpy as np
from flask import Flask, Response
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

cv2.setNumThreads(2)

# ── Flask/debug stream globals ─────────────────────────────────────
app = Flask(__name__)
frame_lock = threading.Lock()
output_frame = None
last_angles = {}
last_z_deltas = {}

# ── Default ROS topics ─────────────────────────────────────────────
DEFAULT_CAMERA_TOPIC = "/mechelangelo/camera/image_raw"
DEFAULT_TRACKING_TOPIC = "/human_tracking"

tracking_node = None
camera_provider = None
args = None

# ── MQTT ───────────────────────────────────────────────────────────
BROKER_IP = "192.168.4.65"
BROKER_PORT = 1883
MQTT_TOPIC = "arm/angles"
MQTT_CMD_TOPIC = "robot/commands"
MQTT_RETRY_SEC = 5

mqtt_connected = False
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)


def _on_connect(client, userdata, flags, reason_code, properties):
    global mqtt_connected
    if reason_code == 0:
        mqtt_connected = True
        print("[MQTT] Connected to broker")
    else:
        mqtt_connected = False
        print(f"[MQTT] Connection refused, reason code: {reason_code}")


def _on_disconnect(client, userdata, flags, reason_code, properties):
    global mqtt_connected
    mqtt_connected = False
    print(f"[MQTT] Disconnected (rc={reason_code}) — will retry")


mqtt_client.on_connect = _on_connect
mqtt_client.on_disconnect = _on_disconnect


def mqtt_connect_loop():
    while True:
        if not mqtt_connected:
            try:
                mqtt_client.connect(BROKER_IP, BROKER_PORT, keepalive=60)
                mqtt_client.loop_start()
            except OSError as e:
                print(f"[MQTT] Cannot reach broker ({e}) — retrying in {MQTT_RETRY_SEC}s")
                time.sleep(MQTT_RETRY_SEC)
                continue
        time.sleep(MQTT_RETRY_SEC)


def mqtt_publish(payload: str, topic: str = MQTT_TOPIC):
    if mqtt_connected:
        try:
            mqtt_client.publish(topic, payload)
        except Exception as e:
            print(f"[MQTT] Publish failed: {e}")


threading.Thread(target=mqtt_connect_loop, daemon=True).start()

# ── MediaPipe ──────────────────────────────────────────────────────
mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils

# ── Colours ────────────────────────────────────────────────────────
COL_SHOULDER = (0, 255, 255)
COL_ELBOW = (255, 255, 0)
COL_Z = (180, 255, 180)
COL_NA = (120, 120, 120)
COL_RED = (0, 0, 255)
COL_LOCK_BOX = (0, 255, 0)
COL_GRACE = (0, 165, 255)
COL_CMD_ACT = (0, 255, 128)
FONT = cv2.FONT_HERSHEY_SIMPLEX

# ── Lock-on tuning ─────────────────────────────────────────────────
VISIBILITY_FLOOR = 0.50
LOCK_STEAL_MARGIN = 0.20
GRACE_PERIOD_SEC = 5
ANCHOR_DRIFT_LIMIT = 0.25
KEY_LM = [11, 12, 13, 14, 15, 16, 23, 24]

# ── Distance estimation constants ──────────────────────────────────
SHOULDER_WIDTH_M = 0.42
FOCAL_LENGTH_PX = 280.0

# Robot behaviour thresholds, also used for debug display.
DIST_TOO_CLOSE = 1.0
DIST_TARGET = 1.5
DIST_IDEAL_NEAR = 1.4
DIST_IDEAL_FAR = 2.2
DIST_TOO_FAR = 2.8

CENTER_DEAD_ZONE = 0.08
DIST_SMOOTH_N = 6


# ── Argument parsing ───────────────────────────────────────────────
def parse_args():
    parser = argparse.ArgumentParser(description="MECHelangelo human pose tracking node")
    parser.add_argument(
        "--camera",
        choices=["sim", "pi", "usb"],
        default="sim",
        help="Camera source: sim uses ROS image topic, pi uses Picamera2, usb uses OpenCV VideoCapture.",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_CAMERA_TOPIC,
        help="ROS image topic to subscribe to when using --camera sim.",
    )
    parser.add_argument(
        "--tracking-topic",
        default=DEFAULT_TRACKING_TOPIC,
        help="ROS topic used to publish Float32MultiArray human tracking data.",
    )
    parser.add_argument(
        "--device",
        type=int,
        default=0,
        help="Camera device index when using --camera usb.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=320,
        help="Frame width.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=240,
        help="Frame height.",
    )
    parser.add_argument(
        "--flask-host",
        default="0.0.0.0",
        help="Flask debug stream host.",
    )
    parser.add_argument(
        "--flask-port",
        type=int,
        default=5000,
        help="Flask debug stream port.",
    )
    return parser.parse_args()


# ── Camera providers ───────────────────────────────────────────────
class HumanTrackingNode(Node):
    def __init__(self, camera_source: str, camera_topic: str, tracking_topic: str, width: int, height: int):
        super().__init__("human_pose_tracking")
        self.camera_source = camera_source
        self.camera_topic = camera_topic
        self.tracking_topic = tracking_topic
        self.width = width
        self.height = height

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frame_lock = threading.Lock()

        if self.camera_source == "sim":
            self.image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.image_callback,
                10,
            )
            self.get_logger().info(f"Subscribed to simulation camera: {self.camera_topic}")

        self.tracking_pub = self.create_publisher(
            Float32MultiArray,
            self.tracking_topic,
            10,
        )
        self.get_logger().info(f"Publishing human tracking: {self.tracking_topic}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert camera image: {e}")
            return

        frame = cv2.resize(frame, (self.width, self.height))
        with self.latest_frame_lock:
            self.latest_frame = frame

    def get_latest_frame(self):
        with self.latest_frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def publish_tracking(self, detected: bool, centre_offset: float, distance_m):
        msg = Float32MultiArray()
        msg.data = [
            1.0 if detected else 0.0,
            float(centre_offset),
            float(distance_m) if distance_m is not None else -1.0,
        ]
        self.tracking_pub.publish(msg)


class PiCameraProvider:
    def __init__(self, width: int, height: int):
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (width, height)},
            buffer_count=2,
        )
        self.picam2.configure(config)
        self.picam2.start()
        print(f"[CAMERA] Using Raspberry Pi camera at {width}x{height}")

    def get_frame(self):
        raw = self.picam2.capture_array()
        if raw is None:
            return None
        if len(raw.shape) == 3 and raw.shape[2] == 4:
            return cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
        if len(raw.shape) == 3 and raw.shape[2] == 3:
            return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        return raw

    def close(self):
        try:
            self.picam2.stop()
        except Exception:
            pass


class USBCameraProvider:
    def __init__(self, device: int, width: int, height: int):
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open USB camera device {device}")
        print(f"[CAMERA] Using USB camera device {device} at {width}x{height}")

    def get_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        return frame

    def close(self):
        self.cap.release()


# ── Command state ─────────────────────────────────────────────────
class RobotCommand:
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT = "TURN_LEFT"
    RIGHT = "TURN_RIGHT"
    STOP = "STOP"

    def __init__(self):
        self.lateral = self.STOP
        self.fwd_back = self.STOP
        self.distance = None
        self._dist_buf = deque(maxlen=DIST_SMOOTH_N)
        self._last_publish = 0.0
        self._PUBLISH_HZ = 5

    def push_distance(self, raw_m: float):
        self._dist_buf.append(raw_m)
        self.distance = float(np.median(self._dist_buf))
        return self.distance

    def update(self, shoulder_mid_x_norm: float):
        offset = shoulder_mid_x_norm - 0.5

        if abs(offset) <= CENTER_DEAD_ZONE:
            self.lateral = self.STOP
        elif offset < 0:
            self.lateral = self.LEFT
        else:
            self.lateral = self.RIGHT

        if self.distance is None:
            self.fwd_back = self.STOP
        elif self.distance < DIST_TARGET:
            self.fwd_back = self.BACKWARD
        elif self.distance > DIST_TARGET:
            self.fwd_back = self.FORWARD
        else:
            self.fwd_back = self.STOP

        now = time.monotonic()
        if now - self._last_publish >= 1.0 / self._PUBLISH_HZ:
            self._publish()
            self._last_publish = now

    def _publish(self):
        payload = json.dumps({
            "lateral": self.lateral,
            "fwd_back": self.fwd_back,
            "distance_m": round(self.distance, 2) if self.distance is not None else None,
        })
        mqtt_publish(payload, MQTT_CMD_TOPIC)

    def summary_lines(self):
        dist_str = f"{self.distance:.2f}m" if self.distance is not None else "---"
        return [
            ("DIST", dist_str, False),
            ("TURN", self.lateral, self.lateral != self.STOP),
            ("MOVE", self.fwd_back, self.fwd_back != self.STOP),
        ]


robot_cmd = RobotCommand()


# ── Lock-on state machine ──────────────────────────────────────────
class LockState:
    def __init__(self):
        self.locked = False
        self.score = 0.0
        self.anchor_x = None
        self.anchor_y = None
        self.lost_since = None

    def _compute_score(self, lm, w, h):
        shoulder_y = (lm[11].y + lm[12].y) / 2.0 * h
        hip_y = (lm[23].y + lm[24].y) / 2.0 * h
        torso_h = abs(hip_y - shoulder_y)
        mean_vis = sum(lm[i].visibility for i in KEY_LM) / len(KEY_LM)
        return torso_h * mean_vis

    def _mean_vis(self, lm):
        return sum(lm[i].visibility for i in KEY_LM) / len(KEY_LM)

    def _shoulder_mid(self, lm):
        return (lm[11].x + lm[12].x) / 2.0, (lm[11].y + lm[12].y) / 2.0

    def _acquire(self, score, ax, ay):
        self.locked = True
        self.score = score
        self.anchor_x = ax
        self.anchor_y = ay
        self.lost_since = None

    def _release(self):
        self.locked = False
        self.score = 0.0
        self.anchor_x = None
        self.anchor_y = None
        self.lost_since = None

    def update_with_detection(self, lm, w, h):
        now = time.monotonic()
        score = self._compute_score(lm, w, h)
        mean_vis = self._mean_vis(lm)
        ax, ay = self._shoulder_mid(lm)

        if not self.locked:
            if mean_vis >= VISIBILITY_FLOOR:
                self._acquire(score, ax, ay)
                return True
            return False

        dx = abs(ax - self.anchor_x)
        dy = abs(ay - self.anchor_y)
        same_region = dx < ANCHOR_DRIFT_LIMIT and dy < ANCHOR_DRIFT_LIMIT

        if same_region:
            self.anchor_x = 0.9 * self.anchor_x + 0.1 * ax
            self.anchor_y = 0.9 * self.anchor_y + 0.1 * ay
            self.score = 0.8 * self.score + 0.2 * score

            if mean_vis >= VISIBILITY_FLOOR:
                self.lost_since = None
                return True

            if self.lost_since is None:
                self.lost_since = now
            if (now - self.lost_since) >= GRACE_PERIOD_SEC:
                self._release()
                return False
            return False

        if score > self.score * (1.0 + LOCK_STEAL_MARGIN):
            self._acquire(score, ax, ay)
            return True

        if self.lost_since is None:
            self.lost_since = now
        if (now - self.lost_since) >= GRACE_PERIOD_SEC:
            self._acquire(score, ax, ay)
            return True
        return False

    def update_no_detection(self):
        if not self.locked:
            return False
        now = time.monotonic()
        if self.lost_since is None:
            self.lost_since = now
        if (now - self.lost_since) >= GRACE_PERIOD_SEC:
            self._release()
            return False
        return True

    @property
    def grace_active(self):
        return self.locked and self.lost_since is not None

    def grace_elapsed_fraction(self):
        if self.lost_since is None:
            return 0.0
        return min(1.0, (time.monotonic() - self.lost_since) / GRACE_PERIOD_SEC)


lock_state = LockState()


# ── Geometry helpers ───────────────────────────────────────────────
def calc_angle_3d(a, b, c):
    ba = np.subtract(a, b)
    bc = np.subtract(c, b)
    cosang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
    return np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))


def estimate_distance(lm, w, vis_fn):
    if not (vis_fn(11) and vis_fn(12)):
        return None

    sx1 = lm[11].x * w
    sx2 = lm[12].x * w
    shoulder_px = abs(sx2 - sx1)

    if shoulder_px < 10:
        return None

    return (SHOULDER_WIDTH_M * FOCAL_LENGTH_PX) / shoulder_px


# ── OSD drawing helpers ────────────────────────────────────────────
def draw_lock_box(frame, lm, w, h):
    xs = [lm[i].x * w for i in range(len(lm)) if lm[i].visibility > 0.3]
    ys = [lm[i].y * h for i in range(len(lm)) if lm[i].visibility > 0.3]
    if not xs:
        return

    pad = 10
    x1 = max(0, int(min(xs)) - pad)
    y1 = max(0, int(min(ys)) - pad)
    x2 = min(w, int(max(xs)) + pad)
    y2 = min(h, int(max(ys)) + pad)
    color = COL_GRACE if lock_state.grace_active else COL_LOCK_BOX

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    label = "GRACE" if lock_state.grace_active else "LOCKED"
    cv2.putText(frame, label, (x1 + 2, max(y1 - 4, 10)), FONT, 0.38, color, 1)

    if lock_state.grace_active:
        bar_w = x2 - x1
        filled = int(bar_w * (1.0 - lock_state.grace_elapsed_fraction()))
        cv2.rectangle(frame, (x1, y1 - 3), (x1 + filled, y1), COL_GRACE, -1)


def draw_centre_guide(frame, w, h, shoulder_mid_x_norm):
    cx = w // 2
    cv2.line(frame, (cx, 0), (cx, h - 52), (80, 80, 80), 1)

    person_x = int(shoulder_mid_x_norm * w)
    cv2.line(frame, (person_x, 0), (person_x, h - 52), (0, 200, 255), 1)

    offset_px = person_x - cx
    dead_px = int(CENTER_DEAD_ZONE * w)
    if abs(offset_px) > dead_px:
        cv2.arrowedLine(
            frame,
            (cx, h // 2 - 52),
            (person_x, h // 2 - 52),
            COL_CMD_ACT,
            2,
            tipLength=0.3,
        )


def draw_distance_gauge(frame, w, h, dist_m):
    gauge_x = 4
    gauge_w = 6
    gauge_top = 10
    gauge_bot = h - 60

    cv2.rectangle(frame, (gauge_x, gauge_top), (gauge_x + gauge_w, gauge_bot), (50, 50, 50), -1)

    if dist_m is None or dist_m <= 0:
        return

    display_min, display_max = 0.5, 4.0
    frac = 1.0 - (min(max(dist_m, display_min), display_max) - display_min) / (display_max - display_min)
    fill_y = gauge_bot - int(frac * (gauge_bot - gauge_top))

    if dist_m < DIST_TOO_CLOSE or dist_m > DIST_TOO_FAR:
        col = COL_RED
    elif DIST_IDEAL_NEAR <= dist_m <= DIST_IDEAL_FAR:
        col = (0, 220, 0)
    else:
        col = (0, 180, 255)

    cv2.rectangle(frame, (gauge_x, fill_y), (gauge_x + gauge_w, gauge_bot), col, -1)

    for threshold in [DIST_TOO_CLOSE, DIST_TARGET, DIST_IDEAL_NEAR, DIST_IDEAL_FAR, DIST_TOO_FAR]:
        t_frac = 1.0 - (min(max(threshold, display_min), display_max) - display_min) / (display_max - display_min)
        t_y = gauge_bot - int(t_frac * (gauge_bot - gauge_top))
        cv2.line(frame, (gauge_x - 2, t_y), (gauge_x + gauge_w + 2, t_y), (200, 200, 200), 1)


def draw_command_panel(frame, w, h):
    panel_x, panel_y = 16, 8
    line_h = 16

    for i, (label, value, active) in enumerate(robot_cmd.summary_lines()):
        y = panel_y + i * line_h
        col_val = COL_CMD_ACT if active else COL_NA
        cv2.putText(frame, f"{label}:", (panel_x, y + 12), FONT, 0.34, (160, 160, 160), 1)
        cv2.putText(frame, value, (panel_x + 42, y + 12), FONT, 0.40, col_val, 1)


def draw_info_bar(frame, angles, z_deltas):
    h, w = frame.shape[:2]
    bar_h = 52

    if lock_state.grace_active:
        bar_color = (0, 60, 80)
    elif lock_state.locked:
        bar_color = (20, 50, 20)
    else:
        bar_color = (50, 20, 20)

    cv2.rectangle(frame, (0, h - bar_h), (w, h), bar_color, -1)
    cv2.line(frame, (0, h - bar_h + 26), (w, h - bar_h + 26), (60, 60, 60), 1)

    slot_w = w // 4
    angle_labels = [
        ("L.Shldr", angles.get("L shoulder"), COL_SHOULDER),
        ("L.Elbw", angles.get("L elbow"), COL_ELBOW),
        ("R.Shldr", angles.get("R shoulder"), COL_SHOULDER),
        ("R.Elbw", angles.get("R elbow"), COL_ELBOW),
    ]

    for i, (label, value, color) in enumerate(angle_labels):
        x = i * slot_w + 4
        cv2.putText(frame, label, (x, h - bar_h + 11), FONT, 0.32, color, 1)
        text = f"{value:.0f}d" if value is not None else "---"
        cv2.putText(frame, text, (x, h - bar_h + 23), FONT, 0.35, color if value is not None else COL_NA, 1)

    z_labels = [
        ("L.Zf", z_deltas.get("L")),
        ("L.Zr", z_deltas.get("L elbow wrist")),
        ("R.Zf", z_deltas.get("R")),
        ("R.Zr", z_deltas.get("R elbow wrist")),
    ]

    for i, (label, value) in enumerate(z_labels):
        x = i * slot_w + 4
        cv2.putText(frame, label, (x, h - bar_h + 37), FONT, 0.28, COL_Z, 1)
        if value is not None:
            arrow = "f" if value < -0.02 else ("b" if value > 0.02 else "0")
            cv2.putText(frame, f"{value:.2f}{arrow}", (x, h - 4), FONT, 0.28, COL_Z, 1)
        else:
            cv2.putText(frame, "---", (x, h - 4), FONT, 0.28, COL_NA, 1)

    mqtt_col = (0, 255, 0) if mqtt_connected else (0, 0, 255)
    cv2.circle(frame, (w - 8, h - 8), 5, mqtt_col, -1)

    if lock_state.grace_active:
        status, scol = "GRACE", COL_GRACE
    elif lock_state.locked:
        status, scol = "LOCK", COL_LOCK_BOX
    else:
        status, scol = "SEEK", COL_RED

    cv2.putText(frame, status, (w - 56, 16), FONT, 0.5, scol, 2)


# ── Frame acquisition ──────────────────────────────────────────────
def get_current_frame():
    if args.camera == "sim":
        return tracking_node.get_latest_frame()
    return camera_provider.get_frame()


# ── Main processing loop ───────────────────────────────────────────
def process_frames():
    global output_frame, last_angles, last_z_deltas, tracking_node

    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=0,
        smooth_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    frame_count = 0
    results = None
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 55]
    last_no_frame_log = 0.0

    while True:
        if tracking_node is None:
            time.sleep(0.05)
            continue

        frame = get_current_frame()

        if frame is None:
            now = time.monotonic()
            if now - last_no_frame_log > 2.0:
                if args.camera == "sim":
                    print(f"[VISION] Waiting for frames on {args.topic}")
                else:
                    print(f"[VISION] Waiting for frames from {args.camera} camera")
                last_no_frame_log = now
            time.sleep(0.05)
            continue

        frame = cv2.resize(frame, (args.width, args.height))
        frame_count += 1
        h, w = frame.shape[:2]

        run_mp = frame_count % 2 == 0

        if run_mp:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb)

            angles = {k: None for k in ("L shoulder", "L elbow", "R shoulder", "R elbow")}
            z_deltas = {k: None for k in ("L", "L elbow wrist", "R", "R elbow wrist")}
            publish_angles = False
            published_tracking = False

            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark
                use_detection = lock_state.update_with_detection(lm, w, h)

                skel_col = (0, 200, 255) if (use_detection and not lock_state.grace_active) else (80, 80, 80)

                mp_draw.draw_landmarks(
                    frame,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_draw.DrawingSpec(
                        color=(0, 255, 0) if use_detection else (60, 60, 60),
                        thickness=1,
                        circle_radius=2,
                    ),
                    connection_drawing_spec=mp_draw.DrawingSpec(color=skel_col, thickness=1),
                )

                if lock_state.locked:
                    draw_lock_box(frame, lm, w, h)

                if use_detection and not lock_state.grace_active:

                    def pt3d(i):
                        return (lm[i].x * w, lm[i].y * h, lm[i].z * w)

                    def pt2d(i):
                        return (int(lm[i].x * w), int(lm[i].y * h))

                    def vis(i):
                        return lm[i].visibility > VISIBILITY_FLOOR

                    l_hip = pt3d(23)
                    l_shoulder = pt3d(11)
                    l_elbow = pt3d(13)
                    l_wrist = pt3d(15)
                    r_hip = pt3d(24)
                    r_shoulder = pt3d(12)
                    r_elbow = pt3d(14)
                    r_wrist = pt3d(16)

                    if vis(23) and vis(11) and vis(13):
                        angles["L shoulder"] = calc_angle_3d(l_hip, l_shoulder, l_elbow)
                    elif vis(12) and vis(11) and vis(13):
                        angles["L shoulder"] = calc_angle_3d(r_shoulder, l_shoulder, l_elbow)

                    if vis(24) and vis(12) and vis(14):
                        angles["R shoulder"] = calc_angle_3d(r_hip, r_shoulder, r_elbow)
                    elif vis(11) and vis(12) and vis(14):
                        angles["R shoulder"] = calc_angle_3d(l_shoulder, r_shoulder, r_elbow)

                    if vis(11) and vis(13) and vis(15):
                        angles["L elbow"] = calc_angle_3d(l_shoulder, l_elbow, l_wrist)
                    if vis(12) and vis(14) and vis(16):
                        angles["R elbow"] = calc_angle_3d(r_shoulder, r_elbow, r_wrist)

                    if vis(11) and vis(13):
                        z_deltas["L"] = lm[13].z - lm[11].z
                    if vis(13) and vis(15):
                        z_deltas["L elbow wrist"] = lm[15].z - lm[13].z
                    if vis(12) and vis(14):
                        z_deltas["R"] = lm[14].z - lm[12].z
                    if vis(14) and vis(16):
                        z_deltas["R elbow wrist"] = lm[16].z - lm[14].z

                    raw_dist = estimate_distance(lm, w, vis)
                    if raw_dist is not None:
                        robot_cmd.push_distance(raw_dist)

                    shoulder_mid_x = (lm[11].x + lm[12].x) / 2.0
                    centre_offset = shoulder_mid_x - 0.5

                    robot_cmd.update(shoulder_mid_x)

                    tracking_node.publish_tracking(
                        True,
                        centre_offset,
                        robot_cmd.distance if robot_cmd.distance is not None else -1.0,
                    )
                    published_tracking = True

                    draw_centre_guide(frame, w, h, shoulder_mid_x)
                    draw_distance_gauge(frame, w, h, robot_cmd.distance)

                    for idx, key, col in [
                        (11, "L shoulder", COL_SHOULDER),
                        (13, "L elbow", COL_ELBOW),
                        (12, "R shoulder", COL_SHOULDER),
                        (14, "R elbow", COL_ELBOW),
                    ]:
                        if angles[key] is not None:
                            x, y = pt2d(idx)
                            cv2.putText(frame, f"{angles[key]:.0f}", (x + 5, y - 5), FONT, 0.38, col, 1)

                    if z_deltas["L"] is not None:
                        x, y = pt2d(13)
                        cv2.putText(frame, f"z:{z_deltas['L']:.2f}", (x + 5, y + 12), FONT, 0.3, COL_Z, 1)
                    if z_deltas["R"] is not None:
                        x, y = pt2d(14)
                        cv2.putText(frame, f"z:{z_deltas['R']:.2f}", (x + 5, y + 12), FONT, 0.3, COL_Z, 1)

                    publish_angles = True

                elif lock_state.grace_active:
                    tracking_node.publish_tracking(False, 0.0, -1.0)
                    published_tracking = True
                    cv2.putText(frame, "Searching...", (20, 30), FONT, 0.6, COL_GRACE, 1)

                if not published_tracking and not lock_state.locked:
                    tracking_node.publish_tracking(False, 0.0, -1.0)

                if publish_angles:
                    payload = json.dumps({
                        "l_shoulder": round(angles["L shoulder"], 1) if angles["L shoulder"] is not None else None,
                        "l_elbow": round(angles["L elbow"], 1) if angles["L elbow"] is not None else None,
                        "r_shoulder": round(angles["R shoulder"], 1) if angles["R shoulder"] is not None else None,
                        "r_elbow": round(angles["R elbow"], 1) if angles["R elbow"] is not None else None,
                        "l_z_fwd": round(z_deltas["L"], 3) if z_deltas["L"] is not None else None,
                        "r_z_fwd": round(z_deltas["R"], 3) if z_deltas["R"] is not None else None,
                        "distance_m": round(robot_cmd.distance, 2) if robot_cmd.distance is not None else None,
                    })
                    mqtt_publish(payload)

                last_angles = angles
                last_z_deltas = z_deltas

            else:
                in_grace = lock_state.update_no_detection()
                tracking_node.publish_tracking(False, 0.0, -1.0)

                if not in_grace:
                    cv2.putText(frame, "No person detected", (20, 30), FONT, 0.7, COL_RED, 2)
                else:
                    cv2.putText(frame, "Searching...", (20, 30), FONT, 0.6, COL_GRACE, 1)

        else:
            if last_angles and results is not None and results.pose_landmarks and lock_state.locked and not lock_state.grace_active:
                lm = results.pose_landmarks.landmark
                shoulder_mid_x = (lm[11].x + lm[12].x) / 2.0
                draw_centre_guide(frame, w, h, shoulder_mid_x)
                draw_distance_gauge(frame, w, h, robot_cmd.distance)

                for idx, key, col in [
                    (11, "L shoulder", COL_SHOULDER),
                    (13, "L elbow", COL_ELBOW),
                    (12, "R shoulder", COL_SHOULDER),
                    (14, "R elbow", COL_ELBOW),
                ]:
                    if last_angles.get(key) is not None:
                        x = int(lm[idx].x * w)
                        y = int(lm[idx].y * h)
                        cv2.putText(frame, f"{last_angles[key]:.0f}", (x + 5, y - 5), FONT, 0.38, col, 1)

        draw_command_panel(frame, w, h)
        draw_info_bar(frame, last_angles, last_z_deltas)

        ok, buffer = cv2.imencode(".jpg", frame, encode_params)
        if ok:
            with frame_lock:
                output_frame = buffer.tobytes()

        time.sleep(0.03)


# ── Flask routes ───────────────────────────────────────────────────
def generate_stream():
    while True:
        with frame_lock:
            frame = output_frame

        if frame is None:
            time.sleep(0.1)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )


@app.route("/")
def index():
    camera_text = args.camera if args is not None else "unknown"
    topic_text = args.topic if args is not None else "unknown"
    tracking_text = args.tracking_topic if args is not None else "unknown"
    return (
        '<html><body style="background:#111;color:white;text-align:center">'
        "<h2>MECHelangelo Pose Stream</h2>"
        f"<p>Camera source: {camera_text}</p>"
        f"<p>Camera topic/device: {topic_text}</p>"
        f"<p>Tracking topic: {tracking_text}</p>"
        '<img src="/video">'
        "</body></html>"
    )


@app.route("/video")
def video():
    return Response(generate_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")


# ── Main ───────────────────────────────────────────────────────────
def main():
    global tracking_node, camera_provider, args

    args = parse_args()

    if args.camera == "pi":
        camera_provider = PiCameraProvider(args.width, args.height)
    elif args.camera == "usb":
        camera_provider = USBCameraProvider(args.device, args.width, args.height)

    rclpy.init()
    tracking_node = HumanTrackingNode(
        camera_source=args.camera,
        camera_topic=args.topic,
        tracking_topic=args.tracking_topic,
        width=args.width,
        height=args.height,
    )

    ros_thread = threading.Thread(target=rclpy.spin, args=(tracking_node,), daemon=True)
    ros_thread.start()

    vision_thread = threading.Thread(target=process_frames, daemon=True)
    vision_thread.start()

    print("Open debug stream in browser:")
    print(f"http://127.0.0.1:{args.flask_port}")
    print(f"Camera source: {args.camera}")
    if args.camera == "sim":
        print(f"Subscribing to camera topic: {args.topic}")
    elif args.camera == "usb":
        print(f"Using USB camera device: {args.device}")
    else:
        print("Using Raspberry Pi Camera through Picamera2")
    print(f"Publishing tracking topic: {args.tracking_topic}")

    try:
        app.run(host=args.flask_host, port=args.flask_port, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        if camera_provider is not None:
            camera_provider.close()

        if tracking_node is not None:
            tracking_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()