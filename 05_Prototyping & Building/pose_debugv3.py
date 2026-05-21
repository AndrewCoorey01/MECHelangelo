import cv2
import mediapipe as mp
import numpy as np
from picamera2 import Picamera2
from flask import Flask, Response
import threading
import time
import paho.mqtt.client as mqtt
import json
import os
from collections import deque

# ── Limit CPU threads before anything else ────────────────────────
cv2.setNumThreads(2)
os.environ['OMP_NUM_THREADS'] = '2'

app = Flask(__name__)
frame_lock = threading.Lock()
output_frame = None
last_angles = {}
last_z_deltas = {}

# ── MQTT ──────────────────────────────────────────────────────────
BROKER_IP = '192.168.4.65'
BROKER_PORT = 1883
MQTT_TOPIC = 'arm/angles'
MQTT_CMD_TOPIC = 'robot/commands' # NEW: separate topic for movement commands
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

# ── MediaPipe ─────────────────────────────────────────────────────
mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils

# ── Camera ────────────────────────────────────────────────────────
picam2 = Picamera2()
config = picam2.create_video_configuration(
 main={"size": (320, 240)},
 buffer_count=2
)
picam2.configure(config)
picam2.start()

# ── Colours ───────────────────────────────────────────────────────
COL_SHOULDER = (0, 255, 255)
COL_ELBOW = (255, 255, 0)
COL_Z = (180, 255, 180)
COL_NA = (120, 120, 120)
COL_RED = (0, 0, 255)
COL_LOCK_BOX = (0, 255, 0)
COL_LOCK_TEXT = (0, 255, 0)
COL_GRACE = (0, 165, 255)
COL_DIST = (255, 180, 0) # orange — distance readout
COL_CMD = (255, 255, 255) # white — command text
COL_CMD_ACT = (0, 255, 128) # bright green — active command
FONT = cv2.FONT_HERSHEY_SIMPLEX

# ── Lock-on tuning ────────────────────────────────────────────────
VISIBILITY_FLOOR = 0.50
LOCK_STEAL_MARGIN = 0.20
GRACE_PERIOD_SEC = 5
ANCHOR_DRIFT_LIMIT = 0.25
KEY_LM = [11, 12, 13, 14, 15, 16, 23, 24]

# ── Distance estimation constants ─────────────────────────────────
# Average human shoulder width in metres.
# Adjust FOCAL_LENGTH_PX once: measure shoulder_px at a known distance,
# then FOCAL_LENGTH_PX = shoulder_px * known_distance_m / SHOULDER_WIDTH_M
# Default value calibrated for a ~60-degree horizontal FOV at 320 px wide.
SHOULDER_WIDTH_M = 0.42 # real-world shoulder width (metres)
FOCAL_LENGTH_PX = 280.0 # tune this for your specific lens

# Robot behaviour thresholds (metres)
DIST_TOO_CLOSE = 1.0 # back up if closer than this
DIST_IDEAL_NEAR = 1.4 # comfortable following zone
DIST_IDEAL_FAR = 2.2 # comfortable following zone
DIST_TOO_FAR = 2.8 # move forward if further than this

# Horizontal centering thresholds (fraction of frame width from centre)
CENTER_DEAD_ZONE = 0.08 # within ±8% of frame width — do nothing
TURN_THRESHOLD = 0.20 # beyond ±20% — stronger turn command

# Smoothing window for distance (frames)
DIST_SMOOTH_N = 6


# ── Command state ─────────────────────────────────────────────────
class RobotCommand:
 """
 Decouples command generation from rendering.
 Holds the current recommended commands and publishes them via MQTT.
 """
 FORWARD = "FORWARD"
 BACKWARD = "BACKWARD"
 LEFT = "TURN_LEFT"
 RIGHT = "TURN_RIGHT"
 STOP = "STOP"

 def __init__(self):
 self.lateral = self.STOP # LEFT / RIGHT / STOP
 self.fwd_back = self.STOP # FORWARD / BACKWARD / STOP
 self.distance = None # metres, smoothed
 self._dist_buf = deque(maxlen=DIST_SMOOTH_N)
 self._last_publish = 0.0
 self._PUBLISH_HZ = 5 # publish command at most 5×/sec

 def push_distance(self, raw_m: float):
 """Add a raw distance sample; returns smoothed value."""
 self._dist_buf.append(raw_m)
 self.distance = float(np.median(self._dist_buf))
 return self.distance

 def update(self, shoulder_mid_x_norm: float):
 """
 shoulder_mid_x_norm : 0..1, horizontal position of shoulder midpoint
 0 = left edge, 1 = right edge, 0.5 = centre
 """
 # ── Lateral command (turn to centre person) ────────────────
 offset = shoulder_mid_x_norm - 0.5 # negative = person left of centre
 if abs(offset) <= CENTER_DEAD_ZONE:
 self.lateral = self.STOP
 elif offset < 0:
 # Person is left of centre — robot turns left
 self.lateral = self.LEFT
 else:
 # Person is right of centre — robot turns right
 self.lateral = self.RIGHT

 # ── Forward/backward command ───────────────────────────────
 if self.distance is None:
 self.fwd_back = self.STOP
 elif self.distance < DIST_TOO_CLOSE:
 self.fwd_back = self.BACKWARD
 elif self.distance > DIST_TOO_FAR:
 self.fwd_back = self.FORWARD
 else:
 self.fwd_back = self.STOP

 # ── Publish at limited rate ────────────────────────────────
 now = time.monotonic()
 if now - self._last_publish >= 1.0 / self._PUBLISH_HZ:
 self._publish()
 self._last_publish = now

 def _publish(self):
 payload = json.dumps({
 'lateral': self.lateral,
 'fwd_back': self.fwd_back,
 'distance_m': round(self.distance, 2) if self.distance else None,
 })
 mqtt_publish(payload, MQTT_CMD_TOPIC)

 def summary_lines(self):
 """Return (label, value, is_active) tuples for the OSD."""
 dist_str = f"{self.distance:.2f}m" if self.distance is not None else "---"
 return [
 ("DIST", dist_str, False),
 ("TURN", self.lateral, self.lateral != self.STOP),
 ("MOVE", self.fwd_back, self.fwd_back != self.STOP),
 ]


robot_cmd = RobotCommand()


# ── Lock-on state machine ─────────────────────────────────────────
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
 same_region = (dx < ANCHOR_DRIFT_LIMIT and dy < ANCHOR_DRIFT_LIMIT)

 if same_region:
 self.anchor_x = 0.9 * self.anchor_x + 0.1 * ax
 self.anchor_y = 0.9 * self.anchor_y + 0.1 * ay
 self.score = 0.8 * self.score + 0.2 * score
 if mean_vis >= VISIBILITY_FLOOR:
 self.lost_since = None
 return True
 else:
 if self.lost_since is None:
 self.lost_since = now
 if (now - self.lost_since) >= GRACE_PERIOD_SEC:
 self._release()
 return False
 return False
 else:
 if score > self.score * (1.0 + LOCK_STEAL_MARGIN):
 self._acquire(score, ax, ay)
 return True
 else:
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


# ── Geometry helpers ──────────────────────────────────────────────
def calc_angle_3d(a, b, c):
 ba = np.subtract(a, b)
 bc = np.subtract(c, b)
 cosang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
 return np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))


def estimate_distance(lm, w, vis_fn) -> float | None:
 """
 Similar-triangles distance estimate using shoulder width.

 Returns distance in metres, or None if shoulders not reliably visible.

 Formula: D = (SHOULDER_WIDTH_M * FOCAL_LENGTH_PX) / shoulder_px_width

 Accuracy notes:
 - ±15–20 % when person faces camera squarely
 - Degrades when person is turned > ~45° (shoulders foreshorten)
 - FOV / focal-length calibration is the dominant error source;
 tune FOCAL_LENGTH_PX against a tape measure once and it stays good.
 """
 if not (vis_fn(11) and vis_fn(12)):
 return None

 sx1 = lm[11].x * w
 sx2 = lm[12].x * w
 shoulder_px = abs(sx2 - sx1)

 if shoulder_px < 10: # too small to trust
 return None

 return (SHOULDER_WIDTH_M * FOCAL_LENGTH_PX) / shoulder_px


# ── OSD drawing helpers ───────────────────────────────────────────
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
 """
 Draw a vertical centre line and a marker showing where the person is.
 The gap between them makes offset immediately legible.
 """
 cx = w // 2
 # Faint centre line
 cv2.line(frame, (cx, 0), (cx, h - 52), (80, 80, 80), 1)

 # Shoulder midpoint marker
 person_x = int(shoulder_mid_x_norm * w)
 cv2.line(frame, (person_x, 0), (person_x, h - 52), (0, 200, 255), 1)

 # Arrow from centre to person if outside dead zone
 offset_px = person_x - cx
 dead_px = int(CENTER_DEAD_ZONE * w)
 if abs(offset_px) > dead_px:
 tip_x = cx + offset_px
 cv2.arrowedLine(frame, (cx, h // 2 - 52), (tip_x, h // 2 - 52),
 COL_CMD_ACT, 2, tipLength=0.3)


def draw_distance_gauge(frame, w, h, dist_m):
 """
 A simple coloured rectangle on the left edge that fills proportionally
 to how close the person is. Green = ideal zone, red = too close/far.
 """
 gauge_x = 4
 gauge_w = 6
 gauge_top = 10
 gauge_bot = h - 60

 # Background
 cv2.rectangle(frame, (gauge_x, gauge_top), (gauge_x + gauge_w, gauge_bot),
 (50, 50, 50), -1)

 if dist_m is None:
 return

 # Clamp to display range 0.5–4 m
 display_min, display_max = 0.5, 4.0
 frac = 1.0 - (min(max(dist_m, display_min), display_max) - display_min) \
 / (display_max - display_min)
 fill_y = gauge_bot - int(frac * (gauge_bot - gauge_top))

 if dist_m < DIST_TOO_CLOSE or dist_m > DIST_TOO_FAR:
 col = COL_RED
 elif DIST_IDEAL_NEAR <= dist_m <= DIST_IDEAL_FAR:
 col = (0, 220, 0)
 else:
 col = (0, 180, 255)

 cv2.rectangle(frame, (gauge_x, fill_y), (gauge_x + gauge_w, gauge_bot), col, -1)

 # Tick lines for thresholds
 for threshold in [DIST_TOO_CLOSE, DIST_IDEAL_NEAR, DIST_IDEAL_FAR, DIST_TOO_FAR]:
 t_frac = 1.0 - (min(max(threshold, display_min), display_max) - display_min) \
 / (display_max - display_min)
 t_y = gauge_bot - int(t_frac * (gauge_bot - gauge_top))
 cv2.line(frame, (gauge_x - 2, t_y), (gauge_x + gauge_w + 2, t_y), (200, 200, 200), 1)


def draw_command_panel(frame, w, h):
 """
 Draw a small command readout panel in the top-left corner.
 Shows DIST / TURN / MOVE with active commands highlighted.
 """
 panel_x, panel_y = 16, 8
 line_h = 16

 for i, (label, value, active) in enumerate(robot_cmd.summary_lines()):
 y = panel_y + i * line_h
 col_val = COL_CMD_ACT if active else COL_NA
 cv2.putText(frame, f"{label}:", (panel_x, y + 12),
 FONT, 0.34, (160, 160, 160), 1)
 cv2.putText(frame, value, (panel_x + 36, y + 12),
 FONT, 0.40, col_val, 1)


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
 ('L.Shldr', angles.get('L shoulder'), COL_SHOULDER),
 ('L.Elbw', angles.get('L elbow'), COL_ELBOW),
 ('R.Shldr', angles.get('R shoulder'), COL_SHOULDER),
 ('R.Elbw', angles.get('R elbow'), COL_ELBOW),
 ]
 for i, (label, value, color) in enumerate(angle_labels):
 x = i * slot_w + 4
 cv2.putText(frame, label, (x, h - bar_h + 11), FONT, 0.32, color, 1)
 if value is not None:
 cv2.putText(frame, f"{value:.0f}d", (x, h - bar_h + 23), FONT, 0.35, color, 1)
 else:
 cv2.putText(frame, "---", (x, h - bar_h + 23), FONT, 0.35, COL_NA, 1)

 z_labels = [
 ('L.Zf', z_deltas.get('L')),
 ('L.Zr', z_deltas.get('L elbow wrist')),
 ('R.Zf', z_deltas.get('R')),
 ('R.Zr', z_deltas.get('R elbow wrist')),
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


# ── Main processing loop ──────────────────────────────────────────
def process_frames():
 global output_frame, last_angles, last_z_deltas

 pose = mp_pose.Pose(
 static_image_mode=False,
 model_complexity=0,
 smooth_landmarks=True,
 min_detection_confidence=0.5,
 min_tracking_confidence=0.5
 )

 frame_count = 0
 results = None
 encode_params = [cv2.IMWRITE_JPEG_QUALITY, 55]

 while True:
 raw = picam2.capture_array()
 if raw is None:
 continue

 frame_count += 1
 frame = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
 h, w = frame.shape[:2]

 run_mp = (frame_count % 2 == 0)

 if run_mp:
 rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
 results = pose.process(rgb)

 angles = {k: None for k in ('L shoulder', 'L elbow', 'R shoulder', 'R elbow')}
 z_deltas = {k: None for k in ('L', 'L elbow wrist', 'R', 'R elbow wrist')}
 publish = False

 if results.pose_landmarks:
 lm = results.pose_landmarks.landmark
 use_detection = lock_state.update_with_detection(lm, w, h)

 skel_col = (0, 200, 255) if (use_detection and not lock_state.grace_active) \
 else (80, 80, 80)
 mp_draw.draw_landmarks(
 frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
 landmark_drawing_spec=mp_draw.DrawingSpec(
 color=(0, 255, 0) if use_detection else (60, 60, 60),
 thickness=1, circle_radius=2),
 connection_drawing_spec=mp_draw.DrawingSpec(
 color=skel_col, thickness=1)
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

 l_hip = pt3d(23); l_shoulder = pt3d(11)
 l_elbow = pt3d(13); l_wrist = pt3d(15)
 r_hip = pt3d(24); r_shoulder = pt3d(12)
 r_elbow = pt3d(14); r_wrist = pt3d(16)

 # ── Angles ────────────────────────────────────
 if vis(23) and vis(11) and vis(13):
 angles['L shoulder'] = calc_angle_3d(l_hip, l_shoulder, l_elbow)
 elif vis(12) and vis(11) and vis(13):
 angles['L shoulder'] = calc_angle_3d(r_shoulder, l_shoulder, l_elbow)

 if vis(24) and vis(12) and vis(14):
 angles['R shoulder'] = calc_angle_3d(r_hip, r_shoulder, r_elbow)
 elif vis(11) and vis(12) and vis(14):
 angles['R shoulder'] = calc_angle_3d(l_shoulder, r_shoulder, r_elbow)

 if vis(11) and vis(13) and vis(15):
 angles['L elbow'] = calc_angle_3d(l_shoulder, l_elbow, l_wrist)
 if vis(12) and vis(14) and vis(16):
 angles['R elbow'] = calc_angle_3d(r_shoulder, r_elbow, r_wrist)

 # ── Z-deltas ──────────────────────────────────
 if vis(11) and vis(13):
 z_deltas['L'] = lm[13].z - lm[11].z
 if vis(13) and vis(15):
 z_deltas['L elbow wrist'] = lm[15].z - lm[13].z
 if vis(12) and vis(14):
 z_deltas['R'] = lm[14].z - lm[12].z
 if vis(14) and vis(16):
 z_deltas['R elbow wrist'] = lm[16].z - lm[14].z

 # ── Distance estimation ───────────────────────
 raw_dist = estimate_distance(lm, w, vis)
 if raw_dist is not None:
 robot_cmd.push_distance(raw_dist)

 # ── Shoulder midpoint (normalised 0–1) ────────
 shoulder_mid_x = (lm[11].x + lm[12].x) / 2.0

 # ── Update robot commands ─────────────────────
 robot_cmd.update(shoulder_mid_x)

 # ── OSD overlays ──────────────────────────────
 draw_centre_guide(frame, w, h, shoulder_mid_x)
 draw_distance_gauge(frame, w, h, robot_cmd.distance)

 for idx, key, col in [
 (11, 'L shoulder', COL_SHOULDER),
 (13, 'L elbow', COL_ELBOW),
 (12, 'R shoulder', COL_SHOULDER),
 (14, 'R elbow', COL_ELBOW),
 ]:
 if angles[key] is not None:
 x, y = pt2d(idx)
 cv2.putText(frame, f"{angles[key]:.0f}",
 (x + 5, y - 5), FONT, 0.38, col, 1)

 if z_deltas['L'] is not None:
 x, y = pt2d(13)
 cv2.putText(frame, f"z:{z_deltas['L']:.2f}",
 (x + 5, y + 12), FONT, 0.3, COL_Z, 1)
 if z_deltas['R'] is not None:
 x, y = pt2d(14)
 cv2.putText(frame, f"z:{z_deltas['R']:.2f}",
 (x + 5, y + 12), FONT, 0.3, COL_Z, 1)

 publish = True

 else:
 in_grace = lock_state.update_no_detection()
 if not in_grace:
 cv2.putText(frame, "No person detected",
 (20, 30), FONT, 0.7, COL_RED, 2)
 else:
 cv2.putText(frame, "Searching...",
 (20, 30), FONT, 0.6, COL_GRACE, 1)

 if publish:
 payload = json.dumps({
 'l_shoulder': round(angles['L shoulder'], 1) if angles['L shoulder'] else None,
 'l_elbow': round(angles['L elbow'], 1) if angles['L elbow'] else None,
 'r_shoulder': round(angles['R shoulder'], 1) if angles['R shoulder'] else None,
 'r_elbow': round(angles['R elbow'], 1) if angles['R elbow'] else None,
 'l_z_fwd': round(z_deltas['L'], 3) if z_deltas['L'] else None,
 'r_z_fwd': round(z_deltas['R'], 3) if z_deltas['R'] else None,
 'distance_m': round(robot_cmd.distance, 2) if robot_cmd.distance else None,
 })
 mqtt_publish(payload)

 last_angles = angles
 last_z_deltas = z_deltas

 else:
 # Off-frame: redraw last angle labels without rerunning MediaPipe
 if last_angles and results is not None \
 and results.pose_landmarks and lock_state.locked \
 and not lock_state.grace_active:
 lm = results.pose_landmarks.landmark

 # Redraw distance gauge and guide with cached values
 shoulder_mid_x = (lm[11].x + lm[12].x) / 2.0
 draw_centre_guide(frame, w, h, shoulder_mid_x)
 draw_distance_gauge(frame, w, h, robot_cmd.distance)

 for idx, key, col in [
 (11, 'L shoulder', COL_SHOULDER),
 (13, 'L elbow', COL_ELBOW),
 (12, 'R shoulder', COL_SHOULDER),
 (14, 'R elbow', COL_ELBOW),
 ]:
 if last_angles.get(key) is not None:
 x = int(lm[idx].x * w)
 y = int(lm[idx].y * h)
 cv2.putText(frame, f"{last_angles[key]:.0f}",
 (x + 5, y - 5), FONT, 0.38, col, 1)

 # Always draw command panel (top-left) and info bar (bottom)
 draw_command_panel(frame, w, h)
 draw_info_bar(frame, last_angles, last_z_deltas)

 _, buffer = cv2.imencode(".jpg", frame, encode_params)
 with frame_lock:
 output_frame = buffer.tobytes()

 time.sleep(0.05)


# ── Flask routes ──────────────────────────────────────────────────
def generate_stream():
 while True:
 with frame_lock:
 frame = output_frame
 if frame is None:
 time.sleep(0.1)
 continue
 yield (b'--frame\r\n'
 b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route("/")
def index():
 return (
 '<html><body style="background:#111;color:white;text-align:center">'
 '<h2>Pose Stream</h2>'
 '<img src="/video">'
 '</body></html>'
 )

@app.route("/video")
def video():
 return Response(generate_stream(),
 mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
 t = threading.Thread(target=process_frames, daemon=True)
 t.start()
 print("Open on laptop:")
 print("http://192.168.4.58:5000")
 app.run(host="0.0.0.0", port=5000, threaded=True)