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

# ── Limit CPU threads before anything else ────────────────────────
cv2.setNumThreads(2)
os.environ['OMP_NUM_THREADS'] = '2'

app = Flask(__name__)
frame_lock = threading.Lock()
output_frame = None
last_angles = {}
last_z_deltas = {}

# ── MQTT ──────────────────────────────────────────────────────────
BROKER_IP   = '192.168.4.62'
BROKER_PORT = 1883
MQTT_TOPIC  = 'arm/angles'

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.connect(BROKER_IP, BROKER_PORT, keepalive=60)
mqtt_client.loop_start()

# ── MediaPipe ─────────────────────────────────────────────────────
mp_pose = mp.solutions.pose
mp_draw  = mp.solutions.drawing_utils

# ── Camera ────────────────────────────────────────────────────────
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (320, 240)},
    buffer_count=2
)
picam2.configure(config)
picam2.start()

# ── Pre-allocate reusable buffers ─────────────────────────────────
_frame_rgb = np.empty((240, 320, 3), dtype=np.uint8)
_frame_bgr = np.empty((240, 320, 3), dtype=np.uint8)

# ── Colours (defined once) ────────────────────────────────────────
COL_SHOULDER = (0,   255, 255)
COL_ELBOW    = (255, 255,   0)
COL_Z        = (180, 255, 180)
COL_NA       = (120, 120, 120)
COL_RED      = (0,   0,   255)
FONT         = cv2.FONT_HERSHEY_SIMPLEX

def calc_angle_3d(a, b, c):
    ba = np.subtract(a, b)
    bc = np.subtract(c, b)
    cosang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
    return np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))

def draw_info_bar(frame, angles, z_deltas):
    h, w = frame.shape[:2]
    bar_h = 52
    cv2.rectangle(frame, (0, h - bar_h), (w, h), (30, 30, 30), -1)
    cv2.line(frame, (0, h - bar_h + 26), (w, h - bar_h + 26), (60, 60, 60), 1)

    slot_w = w // 4

    angle_labels = [
        ('L.Shldr', angles.get('L shoulder'), COL_SHOULDER),
        ('L.Elbw',  angles.get('L elbow'),    COL_ELBOW),
        ('R.Shldr', angles.get('R shoulder'), COL_SHOULDER),
        ('R.Elbw',  angles.get('R elbow'),    COL_ELBOW),
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
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 55]

    while True:
        raw = picam2.capture_array()
        if raw is None:
            continue

        frame_count += 1

        # Convert YUV420 to BGR
        frame = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
        h, w = frame.shape[:2]

        # Run MediaPipe every other frame to halve CPU load
        if frame_count % 2 == 0:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb)

            angles = {k: None for k in ('L shoulder','L elbow','R shoulder','R elbow')}
            z_deltas = {k: None for k in ('L','L elbow wrist','R','R elbow wrist')}

            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark

                mp_draw.draw_landmarks(
                    frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_draw.DrawingSpec(
                        color=(0,255,0), thickness=1, circle_radius=2),
                    connection_drawing_spec=mp_draw.DrawingSpec(
                        color=(0,200,255), thickness=1)
                )

                def pt3d(i):
                    return (lm[i].x * w, lm[i].y * h, lm[i].z * w)

                def pt2d(i):
                    return (int(lm[i].x * w), int(lm[i].y * h))

                def vis(i):
                    return lm[i].visibility > 0.5

                l_hip      = pt3d(23)
                l_shoulder = pt3d(11)
                l_elbow    = pt3d(13)
                l_wrist    = pt3d(15)
                r_hip      = pt3d(24)
                r_shoulder = pt3d(12)
                r_elbow    = pt3d(14)
                r_wrist    = pt3d(16)

                # Shoulder angles
                if vis(23) and vis(11) and vis(13):
                    angles['L shoulder'] = calc_angle_3d(l_hip, l_shoulder, l_elbow)
                elif vis(12) and vis(11) and vis(13):
                    angles['L shoulder'] = calc_angle_3d(r_shoulder, l_shoulder, l_elbow)

                if vis(24) and vis(12) and vis(14):
                    angles['R shoulder'] = calc_angle_3d(r_hip, r_shoulder, r_elbow)
                elif vis(11) and vis(12) and vis(14):
                    angles['R shoulder'] = calc_angle_3d(l_shoulder, r_shoulder, r_elbow)

                # Elbow angles
                if vis(11) and vis(13) and vis(15):
                    angles['L elbow'] = calc_angle_3d(l_shoulder, l_elbow, l_wrist)

                if vis(12) and vis(14) and vis(16):
                    angles['R elbow'] = calc_angle_3d(r_shoulder, r_elbow, r_wrist)

                # Z deltas
                if vis(11) and vis(13):
                    z_deltas['L'] = lm[13].z - lm[11].z
                if vis(13) and vis(15):
                    z_deltas['L elbow wrist'] = lm[15].z - lm[13].z
                if vis(12) and vis(14):
                    z_deltas['R'] = lm[14].z - lm[12].z
                if vis(14) and vis(16):
                    z_deltas['R elbow wrist'] = lm[16].z - lm[14].z

                # Draw angles next to joints
                for idx, key, col in [
                    (11, 'L shoulder', COL_SHOULDER),
                    (13, 'L elbow',    COL_ELBOW),
                    (12, 'R shoulder', COL_SHOULDER),
                    (14, 'R elbow',    COL_ELBOW),
                ]:
                    if angles[key] is not None:
                        x, y = pt2d(idx)
                        cv2.putText(frame, f"{angles[key]:.0f}",
                                    (x + 5, y - 5), FONT, 0.38, col, 1)

                # Draw Z next to elbows
                if z_deltas['L'] is not None:
                    x, y = pt2d(13)
                    cv2.putText(frame, f"z:{z_deltas['L']:.2f}",
                                (x + 5, y + 12), FONT, 0.3, COL_Z, 1)
                if z_deltas['R'] is not None:
                    x, y = pt2d(14)
                    cv2.putText(frame, f"z:{z_deltas['R']:.2f}",
                                (x + 5, y + 12), FONT, 0.3, COL_Z, 1)

                # Publish MQTT only when angles changed meaningfully
                payload = json.dumps({
                    'l_shoulder': round(angles['L shoulder'], 1) if angles['L shoulder'] else None,
                    'l_elbow':    round(angles['L elbow'],    1) if angles['L elbow']    else None,
                    'r_shoulder': round(angles['R shoulder'], 1) if angles['R shoulder'] else None,
                    'r_elbow':    round(angles['R elbow'],    1) if angles['R elbow']    else None,
                    'l_z_fwd':    round(z_deltas['L'],        3) if z_deltas['L']        else None,
                    'r_z_fwd':    round(z_deltas['R'],        3) if z_deltas['R']        else None,
                })
                mqtt_client.publish(MQTT_TOPIC, payload)

            else:
                cv2.putText(frame, "No person detected",
                            (20, 30), FONT, 0.7, COL_RED, 2)

            # Cache for odd frames
            last_angles   = angles
            last_z_deltas = z_deltas

        else:
            # Odd frame — reuse last results, skip MediaPipe entirely
            if last_angles:
                for idx, key, col in [
                    (11, 'L shoulder', COL_SHOULDER),
                    (13, 'L elbow',    COL_ELBOW),
                    (12, 'R shoulder', COL_SHOULDER),
                    (14, 'R elbow',    COL_ELBOW),
                ]:
                    if last_angles.get(key) is not None:
                        lm = results.pose_landmarks.landmark if results.pose_landmarks else None
                        if lm:
                            x = int(lm[idx].x * w)
                            y = int(lm[idx].y * h)
                            cv2.putText(frame, f"{last_angles[key]:.0f}",
                                        (x + 5, y - 5), FONT, 0.38, col, 1)

        draw_info_bar(frame, last_angles, last_z_deltas)

        _, buffer = cv2.imencode(".jpg", frame, encode_params)
        with frame_lock:
            output_frame = buffer.tobytes()

        time.sleep(0.05)

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