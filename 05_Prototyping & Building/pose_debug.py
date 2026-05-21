import cv2
import mediapipe as mp
import numpy as np
from picamera2 import Picamera2
from flask import Flask, Response
import threading

app = Flask(__name__)
output_frame = None
frame_lock = threading.Lock()

mp_pose = mp.solutions.pose
mp_draw  = mp.solutions.drawing_utils

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

def calc_angle(a, b, c):
    a, b, c = np.array(a[:2]), np.array(b[:2]), np.array(c[:2])
    ba = a - b
    bc = c - b
    cosang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
    return np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))

def draw_angle(frame, point, angle, color=(255, 255, 0)):
    x, y = int(point[0]), int(point[1])
    cv2.putText(frame, f"{angle:.0f}deg",
                (x + 10, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

def draw_point(frame, point, color=(0, 255, 0)):
    cv2.circle(frame, (int(point[0]), int(point[1])), 5, color, -1)

def draw_line(frame, p1, p2, color=(0, 200, 255)):
    cv2.line(frame,
             (int(p1[0]), int(p1[1])),
             (int(p2[0]), int(p2[1])),
             color, 2)

def process_frames():
    global output_frame

    with mp_pose.Pose(
        static_image_mode=False,
        model_complexity=0,
        smooth_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    ) as pose:

        while True:
            frame = picam2.capture_array()
            h, w  = frame.shape[:2]
            display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            results = pose.process(frame)

            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark

                mp_draw.draw_landmarks(
                    display,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_draw.DrawingSpec(
                        color=(0, 255, 0), thickness=2, circle_radius=3),
                    connection_drawing_spec=mp_draw.DrawingSpec(
                        color=(0, 200, 255), thickness=2)
                )

                def lm_pt(idx):
                    return (lm[idx].x * w, lm[idx].y * h)

                l_shoulder = lm_pt(11)
                l_elbow    = lm_pt(13)
                l_wrist    = lm_pt(15)
                l_hip      = lm_pt(23)
                r_shoulder = lm_pt(12)
                r_elbow    = lm_pt(14)
                r_wrist    = lm_pt(16)
                r_hip      = lm_pt(24)

                l_elbow_ang    = calc_angle(l_shoulder, l_elbow, l_wrist)
                r_elbow_ang    = calc_angle(r_shoulder, r_elbow, r_wrist)
                l_shoulder_ang = calc_angle(l_hip, l_shoulder, l_elbow)
                r_shoulder_ang = calc_angle(r_hip, r_shoulder, r_elbow)

                draw_angle(display, l_elbow,    l_elbow_ang,    (255, 255,   0))
                draw_angle(display, r_elbow,    r_elbow_ang,    (255, 255,   0))
                draw_angle(display, l_shoulder, l_shoulder_ang, (  0, 255, 255))
                draw_angle(display, r_shoulder, r_shoulder_ang, (  0, 255, 255))

                print(
                    f"L_shoulder: {l_shoulder_ang:5.1f}  "
                    f"R_shoulder: {r_shoulder_ang:5.1f}  "
                    f"L_elbow: {l_elbow_ang:5.1f}  "
                    f"R_elbow: {r_elbow_ang:5.1f}"
                )

            else:
                cv2.putText(display, "No person detected",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 0, 255), 2)

            _, buffer = cv2.imencode('.jpg', display,
                                     [cv2.IMWRITE_JPEG_QUALITY, 70])
            with frame_lock:
                output_frame = buffer.tobytes()

def generate_stream():
    while True:
        with frame_lock:
            if output_frame is None:
                continue
            frame = output_frame
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return '''
    <html><body style="background:#111;margin:0;display:flex;
    flex-direction:column;align-items:center;padding:20px;
    font-family:sans-serif;color:white">
    <h2 style="margin-bottom:12px">Pose Debug Feed</h2>
    <img src="/feed" style="max-width:100%;border:2px solid #444;border-radius:8px">
    </body></html>
    '''

@app.route('/feed')
def feed():
    return Response(generate_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    t = threading.Thread(target=process_frames, daemon=True)
    t.start()
    print("Stream ready -- open http://192.168.4.58:5000 on your laptop")
    app.run(host='0.0.0.0', port=5000, threaded=True)