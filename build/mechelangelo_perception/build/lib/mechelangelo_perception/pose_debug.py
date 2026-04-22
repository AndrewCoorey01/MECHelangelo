import cv2
import mediapipe as mp
import numpy as np
from picamera2 import Picamera2
from flask import Flask, Response
import threading
import time
import paho.mqtt.client as mqtt
import json

app = Flask(__name__)
frame_lock = threading.Lock()
output_frame = None

# ── MQTT setup ────────────────────────────────────────────────────
BROKER_IP   = '192.168.4.62'  # your laptop's IP on hotspot
BROKER_PORT = 1883
MQTT_TOPIC  = 'arm/angles'

mqtt_client = mqtt.Client()
mqtt_client.connect(BROKER_IP, BROKER_PORT, keepalive=60)
mqtt_client.loop_start()

mp_pose = mp.solutions.pose
mp_draw  = mp.solutions.drawing_utils

picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (320, 240)},
    buffer_count=3
)
picam2.configure(config)
picam2.start()

def calc_angle_3d(a, b, c):
    """Calculate joint angle using full 3D coordinates."""
    a = np.array([a[0], a[1], a[2]])
    b = np.array([b[0], b[1], b[2]])
    c = np.array([c[0], c[1], c[2]])
    ba = a - b
    bc = c - b
    cosang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
    return np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))

def draw_info_bar(frame, angles, z_deltas):
    """Draw a dark bar at the bottom showing all angles and Z deltas."""
    h, w = frame.shape[:2]
    bar_h = 52

    # Dark background bar
    cv2.rectangle(frame, (0, h - bar_h), (w, h), (30, 30, 30), -1)

    # Dividing line between angle row and Z row
    cv2.line(frame, (0, h - bar_h + 26), (w, h - bar_h + 26),
             (60, 60, 60), 1)

    SHOULDER_COLOR = (0,   255, 255)
    ELBOW_COLOR    = (255, 255,   0)
    Z_COLOR        = (180, 255, 180)
    NA_COLOR       = (120, 120, 120)

    slot_w = w // 4

    # ── Row 1: joint angles ───────────────────────────────────────
    angle_labels = [
        ('L.Shldr', angles.get('L shoulder'), SHOULDER_COLOR),
        ('L.Elbw',  angles.get('L elbow'),    ELBOW_COLOR),
        ('R.Shldr', angles.get('R shoulder'), SHOULDER_COLOR),
        ('R.Elbw',  angles.get('R elbow'),    ELBOW_COLOR),
    ]

    for i, (label, value, color) in enumerate(angle_labels):
        x = i * slot_w + 4
        y_label = h - bar_h + 11
        y_value = h - bar_h + 23

        cv2.putText(frame, label,
                    (x, y_label),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, color, 1)

        if value is not None:
            val_text = f"{value:.0f}d"
        else:
            val_text = "---"
            color = NA_COLOR

        cv2.putText(frame, val_text,
                    (x, y_value),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

    # ── Row 2: Z depth deltas ─────────────────────────────────────
    # Z delta = elbow Z minus shoulder Z
    # Negative = elbow closer to camera = arm reaching forward
    z_labels = [
        ('L.Z fwd', z_deltas.get('L')),
        ('L.Z rel', z_deltas.get('L elbow wrist')),
        ('R.Z fwd', z_deltas.get('R')),
        ('R.Z rel', z_deltas.get('R elbow wrist')),
    ]

    for i, (label, value) in enumerate(z_labels):
        x = i * slot_w + 4
        y_label = h - bar_h + 37
        y_value = h - 4

        cv2.putText(frame, label,
                    (x, y_label),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, Z_COLOR, 1)

        if value is not None:
            # Show direction arrow based on sign
            arrow = "fwd" if value < -0.02 else ("bk" if value > 0.02 else "~0")
            val_text = f"{value:.2f}{arrow}"
        else:
            val_text = "---"

        cv2.putText(frame, val_text,
                    (x, y_value),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, Z_COLOR, 1)

def process_frames():
    global output_frame

    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=0,
        smooth_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    while True:
        frame = picam2.capture_array()
        if frame is None:
            continue

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        h, w = frame.shape[:2]
        results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        angles = {
            'L shoulder': None,
            'L elbow':    None,
            'R shoulder': None,
            'R elbow':    None,
        }

        z_deltas = {
            'L':              None,   # shoulder -> elbow (forward reach L)
            'L elbow wrist':  None,   # elbow -> wrist (forearm depth L)
            'R':              None,   # shoulder -> elbow (forward reach R)
            'R elbow wrist':  None,   # elbow -> wrist (forearm depth R)
        }

        if results.pose_landmarks:
            lm = results.pose_landmarks.landmark

            mp_draw.draw_landmarks(
                frame,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS
            )

            def pt2d(i):
                """2D pixel coordinates for drawing."""
                return (lm[i].x * w, lm[i].y * h)

            def pt3d(i):
                """3D coordinates — Z scaled to same units as X/Y."""
                return (lm[i].x * w, lm[i].y * h, lm[i].z * w)

            def visible(i):
                return lm[i].visibility > 0.5

            # 2D points for drawing labels on screen
            l_shoulder_2d = pt2d(11)
            l_elbow_2d    = pt2d(13)
            l_wrist_2d    = pt2d(15)
            r_shoulder_2d = pt2d(12)
            r_elbow_2d    = pt2d(14)
            r_wrist_2d    = pt2d(16)

            # 3D points for angle calculation
            l_hip_3d      = pt3d(23)
            l_shoulder_3d = pt3d(11)
            l_elbow_3d    = pt3d(13)
            l_wrist_3d    = pt3d(15)
            r_hip_3d      = pt3d(24)
            r_shoulder_3d = pt3d(12)
            r_elbow_3d    = pt3d(14)
            r_wrist_3d    = pt3d(16)

            # ── Shoulder angles (3D) ──────────────────────────────
            # Prefer hip reference, fall back to opposite shoulder
            if visible(23) and visible(11) and visible(13):
                angles['L shoulder'] = calc_angle_3d(
                    l_hip_3d, l_shoulder_3d, l_elbow_3d)
            elif visible(12) and visible(11) and visible(13):
                angles['L shoulder'] = calc_angle_3d(
                    r_shoulder_3d, l_shoulder_3d, l_elbow_3d)

            if visible(24) and visible(12) and visible(14):
                angles['R shoulder'] = calc_angle_3d(
                    r_hip_3d, r_shoulder_3d, r_elbow_3d)
            elif visible(11) and visible(12) and visible(14):
                angles['R shoulder'] = calc_angle_3d(
                    l_shoulder_3d, r_shoulder_3d, r_elbow_3d)

            # ── Elbow angles (3D) ─────────────────────────────────
            if visible(11) and visible(13) and visible(15):
                angles['L elbow'] = calc_angle_3d(
                    l_shoulder_3d, l_elbow_3d, l_wrist_3d)

            if visible(12) and visible(14) and visible(16):
                angles['R elbow'] = calc_angle_3d(
                    r_shoulder_3d, r_elbow_3d, r_wrist_3d)

            # ── Z depth deltas ────────────────────────────────────
            # Negative = that joint is closer to camera than reference
            if visible(11) and visible(13):
                z_deltas['L'] = lm[13].z - lm[11].z

            if visible(13) and visible(15):
                z_deltas['L elbow wrist'] = lm[15].z - lm[13].z

            if visible(12) and visible(14):
                z_deltas['R'] = lm[14].z - lm[12].z

            if visible(14) and visible(16):
                z_deltas['R elbow wrist'] = lm[16].z - lm[14].z

            # ── Draw angle numbers next to joints ─────────────────
            SHOULDER_COLOR = (0,   255, 255)
            ELBOW_COLOR    = (255, 255,   0)
            Z_COLOR        = (180, 255, 180)

            if angles['L shoulder'] is not None:
                x, y = int(l_shoulder_2d[0]), int(l_shoulder_2d[1])
                cv2.putText(frame, f"{angles['L shoulder']:.0f}",
                            (x + 6, y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, SHOULDER_COLOR, 1)

            if angles['L elbow'] is not None:
                x, y = int(l_elbow_2d[0]), int(l_elbow_2d[1])
                cv2.putText(frame, f"{angles['L elbow']:.0f}",
                            (x + 6, y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, ELBOW_COLOR, 1)

            if angles['R shoulder'] is not None:
                x, y = int(r_shoulder_2d[0]), int(r_shoulder_2d[1])
                cv2.putText(frame, f"{angles['R shoulder']:.0f}",
                            (x + 6, y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, SHOULDER_COLOR, 1)

            if angles['R elbow'] is not None:
                x, y = int(r_elbow_2d[0]), int(r_elbow_2d[1])
                cv2.putText(frame, f"{angles['R elbow']:.0f}",
                            (x + 6, y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, ELBOW_COLOR, 1)

            # Draw Z delta next to elbow (shows forward reach)
            if z_deltas['L'] is not None:
                x, y = int(l_elbow_2d[0]), int(l_elbow_2d[1])
                cv2.putText(frame, f"z:{z_deltas['L']:.2f}",
                            (x + 6, y + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, Z_COLOR, 1)

            if z_deltas['R'] is not None:
                x, y = int(r_elbow_2d[0]), int(r_elbow_2d[1])
                cv2.putText(frame, f"z:{z_deltas['R']:.2f}",
                            (x + 6, y + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, Z_COLOR, 1)

            # Terminal output
            angle_str = "  ".join(
                f"{k}: {v:5.1f}" if v is not None else f"{k}: ---"
                for k, v in angles.items()
            )
            z_str = "  ".join(
                f"{k}: {v:+.2f}" if v is not None else f"{k}: ---"
                for k, v in z_deltas.items()
            )
            print(f"{angle_str}  |  {z_str}")
            # Publish to MQTT
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
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)

        draw_info_bar(frame, angles, z_deltas)

        ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ret:
            continue

        with frame_lock:
            output_frame = buffer.tobytes()
        time.sleep(0.1)

def generate_stream():
    while True:
        with frame_lock:
            frame = output_frame
        if frame is None:
            time.sleep(0.15)
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               frame + b'\r\n')

@app.route("/")
def index():
    return """
    <html>
    <body style="background:#111;color:white;text-align:center">
    <h2>Pose Stream</h2>
    <img src="/video">
    </body>
    </html>
    """

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