#!/usr/bin/env python3
"""
Camera pose detection with KNN classifier — BOTH ARMS.
Feature vector per arm (11 features):

  0  shoulder_angle    3D angle: hip->shoulder->elbow
  1  elbow_angle       3D angle: shoulder->elbow->wrist
  2  z_fwd             elbow.z - shoulder.z  (depth, noisy)
  3  z_rear            wrist.z - elbow.z     (depth, noisy)
  4  wrist_rel_y       wrist.y - shoulder.y  (neg = wrist above shoulder)
  5  wrist_rel_x       wrist.x - shoulder.x  (arm reach sideways)
  6  elbow_rel_y         elbow.y - shoulder.y  (neg = elbow above shoulder)
  7  elbow_rel_x         elbow.x - shoulder.x  (elbow reach sideways)
  8  forearm_angle_2d    atan2(wrist.y-elbow.y, wrist.x-elbow.x) in degrees
  9  upper_arm_angle_2d  atan2(elbow.y-shoulder.y, elbow.x-shoulder.x) in degrees
 10  elbow_from_centre   elbow.x - shoulder_mid_x

Old feature-count .json files are detected and cleared automatically.
Re-record all poses after any feature update.
"""

import cv2
import mediapipe as mp
import numpy as np
from picamera2 import Picamera2
from flask import Flask, Response, jsonify
import threading
import time
import os
import json
import math
from collections import deque, defaultdict, Counter

# ── Servo imports ─────────────────────────────────────────────────
from pylx16a.lx16a import LX16A, ServoTimeoutError

cv2.setNumThreads(2)
os.environ['OMP_NUM_THREADS'] = '2'

app           = Flask(__name__)
frame_lock    = threading.Lock()
output_frame  = None
last_angles   = {}
last_z_deltas = {}
last_wrist    = {}
last_extra    = {}

mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils

picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (640, 480), "format": "XRGB8888"},
    controls={
        "AwbEnable":          True,
        "AeEnable":           True,
        "NoiseReductionMode": 2,
        "Sharpness":          2.0,
        "Contrast":           1.1,
    },
    buffer_count=2,
)
picam2.configure(config)
picam2.start()
time.sleep(1.0)

# ── Colour palette ────────────────────────────────────────────────
COL_SHOULDER  = (0,   255, 255)
COL_ELBOW     = (255, 255,   0)
COL_WRIST     = (200, 180, 255)
COL_EXTRA     = (180, 255, 180)
COL_NA        = (120, 120, 120)
COL_RED       = (0,   0,   255)
COL_LOCK_BOX  = (0,   255,   0)
COL_GRACE     = (0,   165, 255)
COL_CMD_ACT   = (0,   255, 128)
COL_POSE      = (255, 200,   0)
COL_POSE_LOCK = (0,   255, 128)
COL_LEFT_ARM  = (255, 128,   0)   # orange tint for left arm overlays
FONT          = cv2.FONT_HERSHEY_SIMPLEX

# ── Detection & tracking constants ────────────────────────────────
VISIBILITY_FLOOR   = 0.50
LOCK_STEAL_MARGIN  = 0.20
GRACE_PERIOD_SEC   = 5
ANCHOR_DRIFT_LIMIT = 0.25
KEY_LM = [11, 12, 13, 14, 15, 16, 23, 24]

SHOULDER_WIDTH_M = 0.42
FOCAL_LENGTH_PX  = 280.0
DIST_TOO_CLOSE   = 1.0
DIST_IDEAL_NEAR  = 1.4
DIST_IDEAL_FAR   = 2.2
DIST_TOO_FAR     = 2.8
CENTER_DEAD_ZONE = 0.08
DIST_SMOOTH_N    = 6

POSE_COOLDOWN_SEC = 2.7   # seconds blocked after firing (covers move_time + 0.2s buffer)
STABILITY_N       = 5     # consecutive matching KNN votes required before firing

# ── Servo setup ───────────────────────────────────────────────────
RIGHT_SERVO_PORT = "/dev/ttyACM0"
LEFT_SERVO_PORT  = "/dev/ttyACM1"
SERVO_TIMEOUT    = 0.5
_servo_lock      = threading.Lock()

def _init_servos():
    """Probe servos 1-4 on both ports. Non-fatal if hardware absent."""
    for port, label in [(RIGHT_SERVO_PORT, "R"), (LEFT_SERVO_PORT, "L")]:
        try:
            LX16A.initialize(port, SERVO_TIMEOUT)
            found = []
            for sid in range(1, 5):
                try:
                    time.sleep(0.05)
                    LX16A(sid)
                    found.append(sid)
                    print(f"[SERVO-{label}] Found servo {sid}")
                except ServoTimeoutError:
                    pass
                except Exception as e:
                    print(f"[SERVO-{label}] Servo {sid} error: {e}")
            print(f"[SERVO-{label}] Ready on {port} — IDs: {found}")
        except Exception as e:
            print(f"[SERVO-{label}] Init failed on {port} (running without): {e}")

def _move_on_port(port, label, servo_angles, time_ms):
    """Re-initialise the given port then move. Runs under _servo_lock."""
    try:
        LX16A.initialize(port, SERVO_TIMEOUT)
    except Exception as e:
        print(f"[SERVO-{label}] Re-init failed: {e}"); return
    for sid, angle in servo_angles.items():
        try:
            safe_angle = max(0.0, min(240.0, float(angle)))
            LX16A(sid).move(safe_angle, time_ms)
            print(f"[SERVO-{label}] Servo {sid} → {safe_angle:.1f}° over {time_ms}ms")
        except ServoTimeoutError:
            print(f"[SERVO-{label}] Timeout on servo {sid}")
        except Exception as e:
            print(f"[SERVO-{label}] Error on servo {sid}: {e}")

def move_right_servos(servo_angles: dict, time_ms: int):
    """Move right arm servos (ttyACM0) in a background thread."""
    def _move():
        with _servo_lock:
            _move_on_port(RIGHT_SERVO_PORT, "R", servo_angles, time_ms)
    threading.Thread(target=_move, daemon=True).start()

def move_left_servos(servo_angles: dict, time_ms: int):
    """Move left arm servos (ttyACM1) in a background thread."""
    def _move():
        with _servo_lock:
            _move_on_port(LEFT_SERVO_PORT, "L", servo_angles, time_ms)
    threading.Thread(target=_move, daemon=True).start()

_init_servos()

# ── Right arm pose map — ttyACM0, servos 1-4 ─────────────────────
RIGHT_POSE_MAP = {
    'arm_down':    {'servo_angles': {1: 121.0, 2: 115.0, 3: 36.5,  4: 117.8}, 'time_ms': 2500},
    'straight_arm':{'servo_angles': {1: 121.4, 2: 115.0, 3: 129.4, 4: 118.6}, 'time_ms': 2500},
    'arm_90_up':   {'servo_angles': {1: 30.5,  2: 115.0, 3: 127.4, 4: 118.3}, 'time_ms': 2500},
    'arm_90_out':  {'servo_angles': {1: 30.5,  2: 115.0, 3: 35.8,  4: 117.8}, 'time_ms': 2500},
    'salute':      {'servo_angles': {1: 84.0,  2: 0.0,   3: 191.5, 4: 160.5}, 'time_ms': 2500},
    'handshake':   {'servo_angles': {1: 124.3, 2: 18.5,  3: 38.9,  4: 115.9}, 'time_ms': 2500},
    'arm_crossed': {'servo_angles': {1: 178.0, 2: 42.0,  3: 41.0,  4: 148.0}, 'time_ms': 2500},
}

# ── Left arm pose map — ttyACM1, servos 5-8 ──────────────────────
# Placeholder angles — replace with calibrated left arm values.
LEFT_POSE_MAP = {
    'arm_down':    {'servo_angles': {5: 120.0, 6: 120.0, 7: 40.0,  8: 110.0}, 'time_ms': 2500},
    'straight_arm':{'servo_angles': {5: 120.0, 6: 120.0, 7: 135.0, 8: 110.0}, 'time_ms': 2500},
    'arm_90_up':   {'servo_angles': {5: 215.0, 6: 117.5, 7: 130.0, 8: 110.0}, 'time_ms': 2500},
    'arm_90_out':  {'servo_angles': {5: 215.0, 6: 117.5, 7: 40.0,  8: 110.0}, 'time_ms': 2500},
    'salute':      {'servo_angles': {5: 172.0, 6: 0.0,   7: 190.0, 8: 76.5},  'time_ms': 2500},
    'handshake':   {'servo_angles': {5: 120.0, 6: 30.0,  7: 40.0,  8: 110.0}, 'time_ms': 2500},
    'arm_crossed': {'servo_angles': {5: 80.0,  6: 14.0,  7: 52.5,  8: 63.0},  'time_ms': 2500},
}

# Park both arms at rest on startup
move_right_servos(RIGHT_POSE_MAP['arm_down']['servo_angles'], RIGHT_POSE_MAP['arm_down']['time_ms'])
move_left_servos(LEFT_POSE_MAP['arm_down']['servo_angles'], LEFT_POSE_MAP['arm_down']['time_ms'])
print("[SERVO] Startup — arms → arm_down")

# ── Normalisation ─────────────────────────────────────────────────
ANGLE_SCALE         = 90.0
Z_SCALE             = 0.25
SEGMENT_ANGLE_SCALE = 90.0

FEATURE_NAMES = [
    'shoulder_angle', 'elbow_angle',
    'z_fwd', 'z_rear',
    'wrist_rel_y', 'wrist_rel_x',
    'elbow_rel_y', 'elbow_rel_x',
    'forearm_angle_2d', 'upper_arm_angle_2d',
    'elbow_from_centre',
]
FEATURE_WEIGHTS = np.array([
    1.5,  # shoulder_angle
    1.5,  # elbow_angle
    0.5,  # z_fwd               — noisy
    0.5,  # z_rear              — noisy
    2.0,  # wrist_rel_y
    2.0,  # wrist_rel_x
    2.5,  # elbow_rel_y         — salute raises elbow
    2.0,  # elbow_rel_x
    2.0,  # forearm_angle_2d
    3.0,  # upper_arm_angle_2d
    4.0,  # elbow_from_centre   — KEY: arm_down elbow near body, arm_90_out far out
], dtype=float)


def _norm_angle(deg):
    return (deg - 90.0) / ANGLE_SCALE if deg is not None else None

def _norm_z(z):
    return z / Z_SCALE if z is not None else None

def _norm_seg(deg):
    return deg / SEGMENT_ANGLE_SCALE if deg is not None else None


# ── KNN Classifier ────────────────────────────────────────────────
class ArmKNN:
    MIN_EXAMPLES  = 5
    K             = 9
    CONF_FLOOR    = 0.60
    VOTE_WINDOW   = 10
    VOTE_MAJORITY = 0.60

    def __init__(self, side, save_file=None):
        assert side in ('left', 'right')
        self.side        = side
        self.save_file   = save_file or f'/home/pi/pose_knn_{side}.json'
        self.examples    = []
        self.pose_counts = defaultdict(int)
        self._vote_buf   = deque(maxlen=self.VOTE_WINDOW)
        self.candidate   = None
        self.confidence  = 0.0
        self._load()

    def _raw_vals(self, angles, z_deltas, wrist, extra):
        if self.side == 'left':
            return [
                angles.get('L shoulder'),
                angles.get('L elbow'),
                z_deltas.get('L'),
                z_deltas.get('L elbow wrist'),
                wrist.get('L rel_y'),
                wrist.get('L rel_x'),
                extra.get('L elbow_rel_y'),
                extra.get('L elbow_rel_x'),
                extra.get('L forearm_angle'),
                extra.get('L upper_arm_angle'),
                extra.get('L elbow_from_centre'),
            ]
        else:
            return [
                angles.get('R shoulder'),
                angles.get('R elbow'),
                z_deltas.get('R'),
                z_deltas.get('R elbow wrist'),
                wrist.get('R rel_y'),
                wrist.get('R rel_x'),
                extra.get('R elbow_rel_y'),
                extra.get('R elbow_rel_x'),
                extra.get('R forearm_angle'),
                extra.get('R upper_arm_angle'),
                extra.get('R elbow_from_centre'),
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
        vec  = np.zeros(11, dtype=float)
        mask = np.zeros(11, dtype=float)
        for i, v in enumerate(norms):
            if v is not None:
                vec[i] = v; mask[i] = 1.0
        return vec, mask

    def record(self, pose_name, angles, z_deltas, wrist, extra):
        raw       = self._raw_vals(angles, z_deltas, wrist, extra)
        vec, mask = self._to_norm_vector(angles, z_deltas, wrist, extra)
        if int(mask.sum()) < 4:
            print(f"[KNN-{self.side}] Only {int(mask.sum())} features visible — skipping")
            return False
        self.examples.append((vec, mask, pose_name))
        self.pose_counts[pose_name] += 1
        self._save()
        fa = raw[8]; ua = raw[9]; ey = raw[6]; ec = raw[10]
        print(f"[KNN-{self.side}] Recorded '{pose_name}' ({self.pose_counts[pose_name]} ex)  "
              f"forearm={fa:.1f}° upper_arm={ua:.1f}° elbow_y={ey:.3f} elbow_ctr={ec:.3f}")
        return True

    def clear_pose(self, pose_name):
        self.examples = [(v,m,p) for v,m,p in self.examples if p != pose_name]
        self.pose_counts[pose_name] = 0
        self._save()
        print(f"[KNN-{self.side}] Cleared '{pose_name}'")

    def clear_all(self):
        self.examples = []; self.pose_counts = defaultdict(int)
        self._save(); print(f"[KNN-{self.side}] Cleared all")

    def _dist(self, va, ma, vb, mb):
        c = ma * mb * FEATURE_WEIGHTS
        d = (va - vb) * c
        return float(np.sqrt(np.dot(d, d)))

    def classify(self, angles, z_deltas, wrist, extra):
        if len(self.examples) < self.MIN_EXAMPLES: return None, 0.0
        qv, qm = self._to_norm_vector(angles, z_deltas, wrist, extra)
        if qm.sum() < 3: return None, 0.0
        dists = sorted([(self._dist(qv,qm,ev,em), ep) for ev,em,ep in self.examples])
        votes = Counter(p for _,p in dists[:self.K])
        best  = votes.most_common(1)[0][0]
        conf  = votes[best] / self.K
        return (best, conf) if conf >= self.CONF_FLOOR else (None, conf)

    def push_vote(self, angles, z_deltas, wrist, extra):
        raw_pose, raw_conf = self.classify(angles, z_deltas, wrist, extra)
        self.candidate = raw_pose; self.confidence = raw_conf
        self._vote_buf.append(raw_pose if raw_pose else '__none__')
        if len(self._vote_buf) < self.VOTE_WINDOW // 2: return None
        counts = Counter(self._vote_buf); total = len(self._vote_buf)
        best = max((p for p in counts if p != '__none__'),
                   key=lambda p: counts[p], default=None)
        return best if best and counts[best]/total >= self.VOTE_MAJORITY else None

    def diagnose(self):
        if len(self.examples) < self.MIN_EXAMPLES:
            print(f"[KNN-{self.side}] Not enough examples."); return
        poses = sorted(set(p for _,_,p in self.examples))
        idx   = {p:i for i,p in enumerate(poses)}
        conf  = np.zeros((len(poses),len(poses)),dtype=int)
        correct = 0
        for lo in range(len(self.examples)):
            tp = self.examples[lo][2]
            other = [e for i,e in enumerate(self.examples) if i!=lo]
            if len(other) < self.MIN_EXAMPLES: continue
            qv,qm = self.examples[lo][0], self.examples[lo][1]
            dists = sorted([(self._dist(qv,qm,ev,em),ep) for ev,em,ep in other])
            pred  = Counter(p for _,p in dists[:self.K]).most_common(1)[0][0]
            conf[idx[tp]][idx[pred]] += 1
            if pred==tp: correct += 1
        acc = correct/len(self.examples)*100
        print(f"\n{'='*72}")
        print(f"[KNN-{self.side.upper()}] LOO-CV  {correct}/{len(self.examples)}  ({acc:.1f}%)")
        print(f"{'='*72}")
        print(f"{'':>15}", end='')
        for p in poses: print(f"{p[:9]:>10}", end='')
        print("  <- predicted")
        print(f"{'':>15}" + "-"*10*len(poses))
        for i,p in enumerate(poses):
            print(f"{p[:15]:>15}", end='')
            for j in range(len(poses)):
                v=conf[i][j]
                print(f"{'['+str(v)+']':>10}" if i==j else f"{'!!'+str(v) if v else '0':>10}", end='')
            rt=conf[i].sum()
            print(f"  {conf[i][i]/rt*100 if rt else 0:.0f}%  ({self.pose_counts[p]} ex)")
        print()
        print("Feature spread per pose:")
        print(f"  {'pose':>15}  " + " ".join(f"{n[:8]:>10}" for n in FEATURE_NAMES))
        for p in poses:
            vecs  = np.array([v for v,_,ep in self.examples if ep==p])
            masks = np.array([m for _,m,ep in self.examples if ep==p])
            row   = f"  {p:>15}"
            for fi in range(10):
                valid = vecs[masks[:,fi]>0,fi]
                row  += f"  {valid.mean():+.2f}" if len(valid) else "   ---"
            print(row)
        print()
        print("Overlap warnings (sep < spread):")
        warned = False
        for i,p1 in enumerate(poses):
            for p2 in poses[i+1:]:
                v1s=np.array([v for v,_,ep in self.examples if ep==p1])
                m1s=np.array([m for _,m,ep in self.examples if ep==p1])
                v2s=np.array([v for v,_,ep in self.examples if ep==p2])
                m2s=np.array([m for _,m,ep in self.examples if ep==p2])
                for fi,fn in enumerate(FEATURE_NAMES):
                    v1=v1s[m1s[:,fi]>0,fi]; v2=v2s[m2s[:,fi]>0,fi]
                    if not len(v1) or not len(v2): continue
                    sep=abs(v1.mean()-v2.mean()); spread=v1.std()+v2.std()
                    if sep<spread:
                        print(f"  ! {p1} vs {p2}: '{fn}'  sep={sep:.3f}  spread={spread:.3f}")
                        warned=True
        if not warned: print("  None — poses well-separated.")
        print(f"{'='*72}\n")

    def status(self): return dict(self.pose_counts)

    def _save(self):
        try:
            with open(self.save_file,'w') as f:
                json.dump([{'vec':v.tolist(),'mask':m.tolist(),'pose':p}
                           for v,m,p in self.examples],f)
        except Exception as e: print(f"[KNN-{self.side}] Save failed: {e}")

    def _load(self):
        if not os.path.exists(self.save_file):
            print(f"[KNN-{self.side}] No training data at {self.save_file}"); return
        try:
            with open(self.save_file) as f: data=json.load(f)
            if data and len(data[0]['vec']) != 11:
                print(f"[KNN-{self.side}] Old {len(data[0]['vec'])}-feature data — "
                      f"clearing. Re-record all poses (11-feature model now).")
                with open(self.save_file,'w') as f: json.dump([],f)
                return
            self.examples=[(np.array(d['vec']),np.array(d['mask']),d['pose']) for d in data]
            for _,_,p in self.examples: self.pose_counts[p]+=1
            print(f"[KNN-{self.side}] Loaded {len(self.examples)} examples "
                  f"across {len(self.pose_counts)} poses")
        except Exception as e: print(f"[KNN-{self.side}] Load failed: {e}")


# ── KNN instances — both arms ─────────────────────────────────────
right_knn = ArmKNN('right')
left_knn  = ArmKNN('left')

last_right_pose  = None
last_right_time  = 0.0
last_left_pose   = None
last_left_time   = 0.0
right_vote_buf   = deque(maxlen=STABILITY_N)
left_vote_buf    = deque(maxlen=STABILITY_N)


def resolve_servo_command(pose, pose_map):
    """Return (servo_angles_dict, time_ms) for the given pose in the given map."""
    if pose and pose in pose_map:
        cfg = pose_map[pose]
        return cfg['servo_angles'], cfg['time_ms']
    return {}, 2500


class RobotCommand:
    FORWARD="FORWARD"; BACKWARD="BACKWARD"; LEFT="TURN_LEFT"; RIGHT="TURN_RIGHT"; STOP="STOP"
    def __init__(self):
        self.lateral=self.STOP; self.fwd_back=self.STOP
        self.distance=None; self._dist_buf=deque(maxlen=DIST_SMOOTH_N)
    def push_distance(self,r):
        self._dist_buf.append(r); self.distance=float(np.median(self._dist_buf)); return self.distance
    def update(self,smx):
        o=smx-0.5
        self.lateral=(self.STOP if abs(o)<=CENTER_DEAD_ZONE else self.LEFT if o<0 else self.RIGHT)
        self.fwd_back=(self.STOP if self.distance is None else
                       self.BACKWARD if self.distance<DIST_TOO_CLOSE else
                       self.FORWARD  if self.distance>DIST_TOO_FAR else self.STOP)
    def summary_lines(self):
        ds=f"{self.distance:.2f}m" if self.distance is not None else "---"
        return [("DIST",ds,False),("TURN",self.lateral,self.lateral!=self.STOP),
                ("MOVE",self.fwd_back,self.fwd_back!=self.STOP)]

robot_cmd = RobotCommand()


class LockState:
    def __init__(self):
        self.locked=False; self.score=0.0
        self.anchor_x=None; self.anchor_y=None; self.lost_since=None
    def _score(self,lm,w,h):
        sy=(lm[11].y+lm[12].y)/2*h; hy=(lm[23].y+lm[24].y)/2*h
        return abs(hy-sy)*sum(lm[i].visibility for i in KEY_LM)/len(KEY_LM)
    def _vis(self,lm): return sum(lm[i].visibility for i in KEY_LM)/len(KEY_LM)
    def _mid(self,lm): return (lm[11].x+lm[12].x)/2,(lm[11].y+lm[12].y)/2
    def _acquire(self,s,ax,ay):
        self.locked=True; self.score=s; self.anchor_x=ax; self.anchor_y=ay; self.lost_since=None
    def _release(self):
        self.locked=False; self.score=0.0; self.anchor_x=None; self.anchor_y=None; self.lost_since=None
    def update_with_detection(self,lm,w,h):
        now=time.monotonic(); s=self._score(lm,w,h); mv=self._vis(lm); ax,ay=self._mid(lm)
        if not self.locked:
            if mv>=VISIBILITY_FLOOR: self._acquire(s,ax,ay); return True
            return False
        dx=abs(ax-self.anchor_x); dy=abs(ay-self.anchor_y)
        same=(dx<ANCHOR_DRIFT_LIMIT and dy<ANCHOR_DRIFT_LIMIT)
        if same:
            self.anchor_x=0.9*self.anchor_x+0.1*ax; self.anchor_y=0.9*self.anchor_y+0.1*ay
            self.score=0.8*self.score+0.2*s
            if mv>=VISIBILITY_FLOOR: self.lost_since=None; return True
            if self.lost_since is None: self.lost_since=now
            if (now-self.lost_since)>=GRACE_PERIOD_SEC: self._release()
            return False
        else:
            if s>self.score*(1+LOCK_STEAL_MARGIN): self._acquire(s,ax,ay); return True
            if self.lost_since is None: self.lost_since=now
            if (now-self.lost_since)>=GRACE_PERIOD_SEC: self._acquire(s,ax,ay); return True
            return False
    def update_no_detection(self):
        if not self.locked: return False
        now=time.monotonic()
        if self.lost_since is None: self.lost_since=now
        if (now-self.lost_since)>=GRACE_PERIOD_SEC: self._release(); return False
        return True
    @property
    def grace_active(self): return self.locked and self.lost_since is not None
    def grace_elapsed_fraction(self):
        if self.lost_since is None: return 0.0
        return min(1.0,(time.monotonic()-self.lost_since)/GRACE_PERIOD_SEC)

lock_state = LockState()
_in_interaction = False


def calc_angle_3d(a,b,c):
    ba=np.subtract(a,b); bc=np.subtract(c,b)
    return np.degrees(np.arccos(np.clip(np.dot(ba,bc)/(np.linalg.norm(ba)*np.linalg.norm(bc)+1e-6),-1,1)))

def estimate_distance(lm,w,vis_fn):
    if not(vis_fn(11) and vis_fn(12)): return None
    sp=abs(lm[12].x*w-lm[11].x*w)
    return (SHOULDER_WIDTH_M*FOCAL_LENGTH_PX)/sp if sp>=10 else None


def draw_lock_box(frame,lm,w,h):
    xs=[lm[i].x*w for i in range(len(lm)) if lm[i].visibility>0.3]
    ys=[lm[i].y*h for i in range(len(lm)) if lm[i].visibility>0.3]
    if not xs: return
    p=10; x1=max(0,int(min(xs))-p); y1=max(0,int(min(ys))-p)
    x2=min(w,int(max(xs))+p); y2=min(h,int(max(ys))+p)
    col=COL_GRACE if lock_state.grace_active else COL_LOCK_BOX
    cv2.rectangle(frame,(x1,y1),(x2,y2),col,2)
    cv2.putText(frame,"GRACE" if lock_state.grace_active else "LOCKED",
                (x1+2,max(y1-4,10)),FONT,0.38,col,1)
    if lock_state.grace_active:
        bw=x2-x1; fi=int(bw*(1-lock_state.grace_elapsed_fraction()))
        cv2.rectangle(frame,(x1,y1-3),(x1+fi,y1),COL_GRACE,-1)

def draw_centre_guide(frame,w,h,smx):
    cx=w//2; cv2.line(frame,(cx,0),(cx,h-52),(80,80,80),1)
    px=int(smx*w); cv2.line(frame,(px,0),(px,h-52),(0,200,255),1)
    off=px-cx; dp=int(CENTER_DEAD_ZONE*w)
    if abs(off)>dp:
        cv2.arrowedLine(frame,(cx,h//2-52),(cx+off,h//2-52),COL_CMD_ACT,2,tipLength=0.3)

def draw_distance_gauge(frame,w,h,dist_m):
    gx=4; gw=6; gt=10; gb=h-60
    cv2.rectangle(frame,(gx,gt),(gx+gw,gb),(50,50,50),-1)
    if dist_m is None: return
    mn,mx=0.5,4.0; fr=1-(min(max(dist_m,mn),mx)-mn)/(mx-mn)
    fy=gb-int(fr*(gb-gt))
    col=(COL_RED if dist_m<DIST_TOO_CLOSE or dist_m>DIST_TOO_FAR
         else (0,220,0) if DIST_IDEAL_NEAR<=dist_m<=DIST_IDEAL_FAR else (0,180,255))
    cv2.rectangle(frame,(gx,fy),(gx+gw,gb),col,-1)
    for t in [DIST_TOO_CLOSE,DIST_IDEAL_NEAR,DIST_IDEAL_FAR,DIST_TOO_FAR]:
        tf=1-(min(max(t,mn),mx)-mn)/(mx-mn); ty=gb-int(tf*(gb-gt))
        cv2.line(frame,(gx-2,ty),(gx+gw+2,ty),(200,200,200),1)

def draw_command_panel(frame,w,h):
    for i,(label,value,active) in enumerate(robot_cmd.summary_lines()):
        y=8+i*16; col=COL_CMD_ACT if active else COL_NA
        cv2.putText(frame,label,(16,y+12),FONT,0.34,(160,160,160),1)
        cv2.putText(frame,value,(52,y+12),FONT,0.40,col,1)

def draw_pose_panel(frame,w,h):
    """Show both arm KNN candidates and confirmed poses."""
    # Right arm — top right
    rx = w - 110
    ry = 70
    cv2.putText(frame,"R.ARM",(rx,ry),FONT,0.32,(160,160,160),1)
    y = ry + 14
    if right_knn.candidate:
        col = COL_POSE_LOCK if right_knn.confidence >= 0.75 else COL_POSE
        cv2.putText(frame,right_knn.candidate.replace('_',' ').upper(),(rx,y),FONT,0.34,col,1)
        y += 10; bw = int(70*right_knn.confidence)
        cv2.rectangle(frame,(rx,y),(rx+70,y+4),(60,60,60),-1)
        cv2.rectangle(frame,(rx,y),(rx+bw,y+4),col,-1); y += 8
    else:
        cv2.putText(frame,"---",(rx,y),FONT,0.34,COL_NA,1); y += 18
    if last_right_pose:
        cv2.putText(frame,f">{last_right_pose.replace('_',' ').upper()}",(rx,y),FONT,0.34,COL_POSE_LOCK,1)

    # Left arm — below right arm panel
    ly = ry + 60
    cv2.putText(frame,"L.ARM",(rx,ly),FONT,0.32,(160,160,160),1)
    y2 = ly + 14
    if left_knn.candidate:
        col = COL_POSE_LOCK if left_knn.confidence >= 0.75 else COL_LEFT_ARM
        cv2.putText(frame,left_knn.candidate.replace('_',' ').upper(),(rx,y2),FONT,0.34,col,1)
        y2 += 10; bw = int(70*left_knn.confidence)
        cv2.rectangle(frame,(rx,y2),(rx+70,y2+4),(60,60,60),-1)
        cv2.rectangle(frame,(rx,y2),(rx+bw,y2+4),col,-1); y2 += 8
    else:
        cv2.putText(frame,"---",(rx,y2),FONT,0.34,COL_NA,1); y2 += 18
    if last_left_pose:
        cv2.putText(frame,f">{last_left_pose.replace('_',' ').upper()}",(rx,y2),FONT,0.34,COL_POSE_LOCK,1)

def draw_info_bar(frame,angles,z_deltas,wrist,extra):
    h,w=frame.shape[:2]; bh=52
    bc=((0,60,80) if lock_state.grace_active else (20,50,20) if lock_state.locked else (50,20,20))
    cv2.rectangle(frame,(0,h-bh),(w,h),bc,-1)
    cv2.line(frame,(0,h-bh+26),(w,h-bh+26),(60,60,60),1)
    sw=w//4
    for i,(lbl,val,col) in enumerate([
        ('R.Shldr',angles.get('R shoulder'),COL_SHOULDER),
        ('R.Elbw', angles.get('R elbow'),   COL_ELBOW),
        ('L.Shldr',angles.get('L shoulder'),COL_LEFT_ARM),
        ('L.Elbw', angles.get('L elbow'),   COL_LEFT_ARM)]):
        x=i*sw+4
        cv2.putText(frame,lbl,(x,h-bh+11),FONT,0.32,col,1)
        cv2.putText(frame,f"{val:.2f}" if val is not None else "---",
                    (x,h-bh+23),FONT,0.35,col if val is not None else COL_NA,1)
    for i,(lbl,val) in enumerate([
        ('RFa',extra.get('R forearm_angle')),
        ('RUa',extra.get('R upper_arm_angle')),
        ('LFa',extra.get('L forearm_angle')),
        ('LUa',extra.get('L upper_arm_angle'))]):
        x=i*sw+4
        cv2.putText(frame,lbl,(x,h-bh+37),FONT,0.28,COL_EXTRA,1)
        cv2.putText(frame,f"{val:.2f}" if val is not None else "---",
                    (x,h-4),FONT,0.28,COL_EXTRA if val is not None else COL_NA,1)
    st,sc=(("GRACE",COL_GRACE) if lock_state.grace_active
           else ("LOCK",COL_LOCK_BOX) if lock_state.locked else ("SEEK",COL_RED))
    cv2.putText(frame,st,(w-56,16),FONT,0.5,sc,2)


def process_frames():
    global output_frame,last_angles,last_z_deltas,last_wrist,last_extra
    global last_right_pose,last_right_time,last_left_pose,last_left_time
    global _in_interaction

    pose=mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        smooth_landmarks=True,
        enable_segmentation=False,
        smooth_segmentation=False,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6,
    )
    frame_count=0; results=None; ep=[cv2.IMWRITE_JPEG_QUALITY,60]

    while True:
        raw=picam2.capture_array()
        if raw is None: continue
        frame_count+=1
        frame=cv2.cvtColor(raw,cv2.COLOR_BGRA2BGR)
        frame=cv2.flip(frame,-1)

        proc_w, proc_h = 480, 360
        proc = cv2.resize(frame, (proc_w, proc_h))

        lab   = cv2.cvtColor(proc, cv2.COLOR_BGR2LAB)
        l,a,b_ = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        l     = clahe.apply(l)
        proc  = cv2.cvtColor(cv2.merge([l,a,b_]), cv2.COLOR_LAB2BGR)

        h,w=frame.shape[:2]

        run_mp=(frame_count%2==0)
        if run_mp:
            rgb=cv2.cvtColor(proc,cv2.COLOR_BGR2RGB)
            results=pose.process(rgb)

            # Initialise all angle/feature dicts for both arms
            angles  ={k:None for k in ('R shoulder','R elbow','L shoulder','L elbow')}
            z_deltas={k:None for k in ('R','R elbow wrist','L','L elbow wrist')}
            wrist   ={k:None for k in ('R rel_y','R rel_x','L rel_y','L rel_x')}
            extra   ={k:None for k in (
                'R elbow_rel_y','R elbow_rel_x','R forearm_angle','R upper_arm_angle','R elbow_from_centre',
                'L elbow_rel_y','L elbow_rel_x','L forearm_angle','L upper_arm_angle','L elbow_from_centre',
            )}

            if results.pose_landmarks:
                lm=results.pose_landmarks.landmark
                use=lock_state.update_with_detection(lm,w,h)

                if use and not lock_state.grace_active:
                    TORSO  = [(11,12),(11,23),(12,24),(23,24)]
                    R_ARM  = [(12,14),(14,16),(16,18),(16,20),(16,22),(18,20)]
                    L_ARM  = [(11,13),(13,15),(15,17),(15,19),(15,21),(17,19)]
                    L_LEG  = [(23,25),(25,27),(27,29),(27,31),(29,31)]
                    R_LEG  = [(24,26),(26,28),(28,30),(28,32),(30,32)]
                    FACE   = [(0,1),(1,2),(2,3),(3,7),(0,4),(4,5),(5,6),(6,8),(9,10)]

                    seg_colors = {
                        'torso': (100, 220, 100),
                        'r_arm': (0,   200, 255),
                        'l_arm': (255, 128,   0),  # orange for left arm
                        'leg':   (180, 100, 255),
                        'face':  (160, 160, 160),
                    }
                    seg_map = (
                        [(c,'torso') for c in TORSO] +
                        [(c,'r_arm') for c in R_ARM] +
                        [(c,'l_arm') for c in L_ARM] +
                        [(c,'leg')   for c in L_LEG+R_LEG] +
                        [(c,'face')  for c in FACE]
                    )
                    for (a_idx,b_idx), seg in seg_map:
                        if lm[a_idx].visibility < 0.3 or lm[b_idx].visibility < 0.3:
                            continue
                        vis_avg = (lm[a_idx].visibility + lm[b_idx].visibility) / 2
                        thickness = max(1, int(vis_avg * 4))
                        col = seg_colors[seg]
                        p1 = (int(lm[a_idx].x*w), int(lm[a_idx].y*h))
                        p2 = (int(lm[b_idx].x*w), int(lm[b_idx].y*h))
                        cv2.line(frame, p1, p2, col, thickness)

                    KEY_JOINTS = [11,12,13,14,15,16,23,24,25,26]
                    for i in KEY_JOINTS:
                        vis = lm[i].visibility
                        if vis < 0.2: continue
                        cx_ = int(lm[i].x*w); cy_ = int(lm[i].y*h)
                        radius = max(3, int(vis * 8))
                        if vis > 0.7:
                            jcol = (0, 255, 80)
                        elif vis > 0.4:
                            jcol = (0, 180, 255)
                        else:
                            jcol = (0, 80, 255)
                        cv2.circle(frame, (cx_, cy_), radius, jcol, -1)
                        cv2.circle(frame, (cx_, cy_), radius, (0,0,0), 1)
                else:
                    mp_draw.draw_landmarks(
                        frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=mp_draw.DrawingSpec(
                            color=(60,60,60), thickness=1, circle_radius=2),
                        connection_drawing_spec=mp_draw.DrawingSpec(
                            color=(60,60,60), thickness=1))
                if lock_state.locked: draw_lock_box(frame,lm,w,h)

                # Low visibility warnings — both arms
                weak = [name for i,name in [
                    (12,'R.Shldr'),(14,'R.Elbw'),(16,'R.Wrist'),
                    (11,'L.Shldr'),(13,'L.Elbw'),(15,'L.Wrist')] if lm[i].visibility < 0.5]
                if weak:
                    msg = "LOW VIS: " + " ".join(weak)
                    cv2.putText(frame, msg, (w//2 - len(msg)*4, 18),
                                FONT, 0.38, (0,80,255), 1)

                if use and not lock_state.grace_active:
                    def pt3(i): return (lm[i].x*w,lm[i].y*h,lm[i].z*w)
                    def pt2(i): return (int(lm[i].x*w),int(lm[i].y*h))
                    def vis(i): return lm[i].visibility>VISIBILITY_FLOOR

                    # ── Shared reference points ────────────────────
                    smx=(lm[11].x+lm[12].x)/2   # shoulder midpoint x

                    # ── RIGHT ARM ──────────────────────────────────
                    rh=pt3(24); rs=pt3(12); re=pt3(14); rw_pt=pt3(16)

                    if vis(24) and vis(12) and vis(14):
                        angles['R shoulder']=calc_angle_3d(rh,rs,re)
                    elif vis(11) and vis(12) and vis(14):
                        angles['R shoulder']=calc_angle_3d(pt3(11),rs,re)

                    if vis(12) and vis(14) and vis(16):
                        angles['R elbow']=calc_angle_3d(rs,re,rw_pt)

                    if vis(12) and vis(14): z_deltas['R']=lm[14].z-lm[12].z
                    if vis(14) and vis(16): z_deltas['R elbow wrist']=lm[16].z-lm[14].z

                    if vis(16) and vis(12):
                        wrist['R rel_y']=lm[16].y-lm[12].y
                        wrist['R rel_x']=lm[16].x-lm[12].x

                    if vis(14) and vis(12):
                        extra['R elbow_rel_y']=lm[14].y-lm[12].y
                        extra['R elbow_rel_x']=lm[14].x-lm[12].x

                    if vis(14) and vis(16):
                        dy=lm[16].y-lm[14].y; dx=lm[16].x-lm[14].x
                        extra['R forearm_angle']=math.degrees(math.atan2(dy,dx))
                    if vis(12) and vis(14):
                        dy=lm[14].y-lm[12].y; dx=lm[14].x-lm[12].x
                        extra['R upper_arm_angle']=math.degrees(math.atan2(dy,dx))

                    if vis(14):
                        extra['R elbow_from_centre']=lm[14].x-smx

                    # ── LEFT ARM ───────────────────────────────────
                    # MediaPipe: left shoulder=11, left elbow=13, left wrist=15
                    # Left hip=23 used as anchor for shoulder angle
                    ls=pt3(11); le=pt3(13); lw_pt=pt3(15); lh=pt3(23)

                    if vis(23) and vis(11) and vis(13):
                        angles['L shoulder']=calc_angle_3d(lh,ls,le)
                    elif vis(12) and vis(11) and vis(13):
                        angles['L shoulder']=calc_angle_3d(pt3(12),ls,le)

                    if vis(11) and vis(13) and vis(15):
                        angles['L elbow']=calc_angle_3d(ls,le,lw_pt)

                    if vis(11) and vis(13): z_deltas['L']=lm[13].z-lm[11].z
                    if vis(13) and vis(15): z_deltas['L elbow wrist']=lm[15].z-lm[13].z

                    if vis(15) and vis(11):
                        wrist['L rel_y']=lm[15].y-lm[11].y
                        wrist['L rel_x']=lm[15].x-lm[11].x

                    if vis(13) and vis(11):
                        extra['L elbow_rel_y']=lm[13].y-lm[11].y
                        extra['L elbow_rel_x']=lm[13].x-lm[11].x

                    if vis(13) and vis(15):
                        dy=lm[15].y-lm[13].y; dx=lm[15].x-lm[13].x
                        extra['L forearm_angle']=math.degrees(math.atan2(dy,dx))
                    if vis(11) and vis(13):
                        dy=lm[13].y-lm[11].y; dx=lm[13].x-lm[11].x
                        extra['L upper_arm_angle']=math.degrees(math.atan2(dy,dx))

                    if vis(13):
                        extra['L elbow_from_centre']=lm[13].x-smx

                    # ── Annotate angles on frame ───────────────────
                    for i,key,col in [
                        (12,'R shoulder',COL_SHOULDER),(14,'R elbow',COL_ELBOW),
                        (11,'L shoulder',COL_LEFT_ARM),(13,'L elbow', COL_LEFT_ARM)]:
                        if angles[key] is not None:
                            x,y=pt2(i)
                            cv2.putText(frame,f"{angles[key]:.0f}",(x+5,y-5),FONT,0.38,col,1)

                    # Forearm angles on wrists
                    if vis(16):
                        x,y=pt2(16); cv2.circle(frame,(x,y),5,COL_WRIST,-1)
                        fa=extra.get('R forearm_angle')
                        if fa is not None:
                            cv2.putText(frame,f"fa:{fa:.0f}",(x+7,y-5),FONT,0.28,COL_WRIST,1)
                    if vis(15):
                        x,y=pt2(15); cv2.circle(frame,(x,y),5,COL_LEFT_ARM,-1)
                        fa=extra.get('L forearm_angle')
                        if fa is not None:
                            cv2.putText(frame,f"fa:{fa:.0f}",(x+7,y-5),FONT,0.28,COL_LEFT_ARM,1)

                    # Distance and centering
                    rd=estimate_distance(lm,w,vis)
                    if rd is not None: robot_cmd.push_distance(rd)
                    robot_cmd.update(smx)
                    draw_centre_guide(frame,w,h,smx)
                    draw_distance_gauge(frame,w,h,robot_cmd.distance)

                    # ── KNN voting — both arms ─────────────────────
                    now=time.monotonic()

                    vr=right_knn.push_vote(angles,z_deltas,wrist,extra)
                    right_vote_buf.append(vr)
                    if (len(right_vote_buf)==STABILITY_N and len(set(right_vote_buf))==1
                            and vr and vr!=last_right_pose and (now-last_right_time)>=POSE_COOLDOWN_SEC):
                        print(f"[POSE] Right stable: {vr}")
                        last_right_pose=vr; last_right_time=now
                        sa,tm=resolve_servo_command(last_right_pose, RIGHT_POSE_MAP)
                        if sa:
                            print(f"[SERVO] Right → {sa} over {tm}ms")
                            move_right_servos(sa, tm)

                    vl=left_knn.push_vote(angles,z_deltas,wrist,extra)
                    left_vote_buf.append(vl)
                    if (len(left_vote_buf)==STABILITY_N and len(set(left_vote_buf))==1
                            and vl and vl!=last_left_pose and (now-last_left_time)>=POSE_COOLDOWN_SEC):
                        print(f"[POSE] Left stable: {vl}")
                        last_left_pose=vl; last_left_time=now
                        sa,tm=resolve_servo_command(last_left_pose, LEFT_POSE_MAP)
                        if sa:
                            print(f"[SERVO] Left → {sa} over {tm}ms")
                            move_left_servos(sa, tm)

            else:
                ig=lock_state.update_no_detection()
                cv2.putText(frame,"Searching..." if ig else "No person detected",
                            (20,30),FONT,0.6 if ig else 0.7,
                            COL_GRACE if ig else COL_RED,1 if ig else 2)

            now_interaction = lock_state.locked
            if _in_interaction and not now_interaction:
                print("[INTERACTION] Exited — returning arms to arm_down")
                move_right_servos(RIGHT_POSE_MAP['arm_down']['servo_angles'],
                                  RIGHT_POSE_MAP['arm_down']['time_ms'])
                move_left_servos(LEFT_POSE_MAP['arm_down']['servo_angles'],
                                 LEFT_POSE_MAP['arm_down']['time_ms'])
                last_right_pose = None; last_left_pose = None
                right_vote_buf.clear(); left_vote_buf.clear()
            elif not _in_interaction and now_interaction:
                print("[INTERACTION] Entered — arms active")
            _in_interaction = now_interaction

            last_angles=angles; last_z_deltas=z_deltas; last_wrist=wrist; last_extra=extra

        else:
            if (last_angles and results is not None and results.pose_landmarks
                    and lock_state.locked and not lock_state.grace_active):
                lm=results.pose_landmarks.landmark
                smx=(lm[11].x+lm[12].x)/2
                draw_centre_guide(frame,w,h,smx)
                draw_distance_gauge(frame,w,h,robot_cmd.distance)
                for i,key,col in [
                    (12,'R shoulder',COL_SHOULDER),(14,'R elbow',COL_ELBOW),
                    (11,'L shoulder',COL_LEFT_ARM),(13,'L elbow', COL_LEFT_ARM)]:
                    if last_angles.get(key) is not None:
                        cv2.putText(frame,f"{last_angles[key]:.0f}",
                                    (int(lm[i].x*w)+5,int(lm[i].y*h)-5),FONT,0.38,col,1)

        draw_command_panel(frame,w,h)
        draw_pose_panel(frame,w,h)
        draw_info_bar(frame,last_angles,last_z_deltas,last_wrist,last_extra)
        _,buf=cv2.imencode(".jpg",frame,ep)
        with frame_lock: output_frame=buf.tobytes()
        time.sleep(0.08)


def generate_stream():
    while True:
        with frame_lock: frame=output_frame
        if frame is None: time.sleep(0.1); continue
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+frame+b'\r\n'
        time.sleep(0.1)


# ── Flask routes ──────────────────────────────────────────────────

@app.route("/")
def index():
    return ('<html><body style="background:#111;color:white;text-align:center">'
            '<h2>Pose Stream — Both Arms</h2><img src="/video"><br><br>'
            '<a style="color:cyan;font-size:18px" href="/dashboard">→ Dashboard</a><br><br>'
            '<a style="color:cyan" href="/angles">Angles</a> | '
            '<a style="color:cyan" href="/knn/status">KNN Status</a>'
            '</body></html>')

@app.route("/video")
def video():
    return Response(generate_stream(),mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/state")
def state():
    """Minimal endpoint for Pi 5 bridge node — only what it needs."""
    grace_active = lock_state.grace_active
    tracking_valid = lock_state.locked and not grace_active

    return jsonify({
        "right_confirmed": last_right_pose,
        "left_confirmed":  last_left_pose,
        # During grace, keep the target lock visible but do not export a stale
        # turn direction. The Pi 5 behaviour will hold still and wait to
        # reacquire instead of rotating from old data.
        "turn_cmd":        robot_cmd.lateral if tracking_valid else robot_cmd.STOP,
        "locked":          lock_state.locked,
        "grace_active":    grace_active,
        "tracking_valid":  tracking_valid,
    })

@app.route("/angles")
def angles_route():
    def r(v,n=3): return round(v,n) if v is not None else None
    return jsonify({
        'r_shoulder':          r(last_angles.get('R shoulder'),1),
        'r_elbow':             r(last_angles.get('R elbow'),1),
        'r_wrist_y':           r(last_wrist.get('R rel_y')),
        'r_wrist_x':           r(last_wrist.get('R rel_x')),
        'r_elbow_y':           r(last_extra.get('R elbow_rel_y')),
        'r_elbow_x':           r(last_extra.get('R elbow_rel_x')),
        'r_elbow_from_centre': r(last_extra.get('R elbow_from_centre')),
        'r_forearm_angle':     r(last_extra.get('R forearm_angle'),1),
        'r_upper_arm_angle':   r(last_extra.get('R upper_arm_angle'),1),
        'l_shoulder':          r(last_angles.get('L shoulder'),1),
        'l_elbow':             r(last_angles.get('L elbow'),1),
        'l_wrist_y':           r(last_wrist.get('L rel_y')),
        'l_wrist_x':           r(last_wrist.get('L rel_x')),
        'l_elbow_y':           r(last_extra.get('L elbow_rel_y')),
        'l_elbow_x':           r(last_extra.get('L elbow_rel_x')),
        'l_elbow_from_centre': r(last_extra.get('L elbow_from_centre')),
        'l_forearm_angle':     r(last_extra.get('L forearm_angle'),1),
        'l_upper_arm_angle':   r(last_extra.get('L upper_arm_angle'),1),
        'right_confirmed':     last_right_pose,
        'left_confirmed':      last_left_pose,
    })

@app.route("/record/right/<pose_name>")
def record_right(pose_name):
    if pose_name not in RIGHT_POSE_MAP:
        return jsonify({'error':'Unknown pose.','valid':list(RIGHT_POSE_MAP.keys())}),400
    if not last_angles: return jsonify({'error':'No angle data yet'}),400
    right_knn.record(pose_name,last_angles,last_z_deltas,last_wrist,last_extra)
    return jsonify({'recorded':f'right/{pose_name}',
                    'total_examples':right_knn.pose_counts[pose_name],
                    'status':right_knn.status()})

@app.route("/record/left/<pose_name>")
def record_left(pose_name):
    if pose_name not in LEFT_POSE_MAP:
        return jsonify({'error':'Unknown pose.','valid':list(LEFT_POSE_MAP.keys())}),400
    if not last_angles: return jsonify({'error':'No angle data yet'}),400
    left_knn.record(pose_name,last_angles,last_z_deltas,last_wrist,last_extra)
    return jsonify({'recorded':f'left/{pose_name}',
                    'total_examples':left_knn.pose_counts[pose_name],
                    'status':left_knn.status()})

@app.route("/record/clear/right/<pose_name>")
def clear_pose_right(pose_name):
    right_knn.clear_pose(pose_name)
    return jsonify({'cleared':f'right/{pose_name}'})

@app.route("/record/clear/left/<pose_name>")
def clear_pose_left(pose_name):
    left_knn.clear_pose(pose_name)
    return jsonify({'cleared':f'left/{pose_name}'})

@app.route("/record/clear/all")
def clear_all():
    right_knn.clear_all()
    left_knn.clear_all()
    return jsonify({'cleared':'all'})

@app.route("/knn/status")
def knn_status():
    return jsonify({
        'right':            right_knn.status(),
        'right_candidate':  right_knn.candidate,
        'right_confidence': round(right_knn.confidence,2),
        'right_confirmed':  last_right_pose,
        'left':             left_knn.status(),
        'left_candidate':   left_knn.candidate,
        'left_confidence':  round(left_knn.confidence,2),
        'left_confirmed':   last_left_pose,
    })

@app.route("/poses")
def poses():
    return jsonify({
        'right':{k:v['servo_angles'] for k,v in RIGHT_POSE_MAP.items()},
        'left': {k:v['servo_angles'] for k,v in LEFT_POSE_MAP.items()},
    })

@app.route("/diagnose/right")
def diagnose_right():
    right_knn.diagnose()
    return jsonify({'status':'printed to console','side':'right'})

@app.route("/diagnose/left")
def diagnose_left():
    left_knn.diagnose()
    return jsonify({'status':'printed to console','side':'left'})

@app.route("/dashboard")
def dashboard():
    rp=list(RIGHT_POSE_MAP.keys())
    lp=list(LEFT_POSE_MAP.keys())
    rs=right_knn.status()
    ls=left_knn.status()

    def rows(poses,side,status):
        out=""
        for pose in poses:
            cnt=status.get(pose,0)
            col="#e74c3c" if cnt<5 else "#f39c12" if cnt<12 else "#2ecc71"
            lbl=pose.replace("_"," ").title()
            out+=f"""<tr>
                <td style="padding:8px 16px;font-size:15px">{lbl}</td>
                <td style="padding:8px 16px;text-align:center">
                    <span style="color:{col};font-weight:bold">{cnt}</span></td>
                <td style="padding:8px">
                    <button onclick="rec('{side}','{pose}',this)"
                        style="background:#2980b9;color:white;border:none;
                               padding:6px 14px;border-radius:4px;cursor:pointer">
                        + Record</button></td>
                <td style="padding:8px">
                    <button onclick="clr('{side}','{pose}')"
                        style="background:#7f8c8d;color:white;border:none;
                               padding:6px 14px;border-radius:4px;cursor:pointer">
                        Clear</button></td></tr>"""
        return out

    return f"""<!DOCTYPE html><html><head>
    <title>Pose Dashboard — Both Arms</title>
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <style>
        body{{background:#1a1a2e;color:#eee;font-family:Arial,sans-serif;margin:0;padding:16px}}
        h1{{text-align:center;color:#00d2ff;font-size:22px;margin-bottom:4px}}
        .sub{{text-align:center;color:#666;font-size:12px;margin-bottom:16px}}
        .card{{background:#16213e;border-radius:8px;padding:16px;border:1px solid #0f3460;
               max-width:520px;margin:0 auto 16px}}
        .card h2{{margin:0 0 12px;font-size:15px;color:#00d2ff;border-bottom:1px solid #0f3460;padding-bottom:8px}}
        .card h2.left{{color:#ff8000}}
        table{{width:100%;border-collapse:collapse}}tr:hover{{background:#0f3460}}
        .sc{{max-width:520px;margin:0 auto 12px;background:#16213e;border-radius:8px;
             padding:12px;border:1px solid #0f3460;text-align:center}}
        .sc img{{max-width:100%;border-radius:4px}}
        .sb{{max-width:520px;margin:0 auto 12px;background:#16213e;border-radius:8px;
             padding:10px 14px;border:1px solid #0f3460;display:flex;
             justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}}
        .si{{font-size:12px;color:#aaa}}.si span{{color:#00d2ff;font-weight:bold}}
        .si span.left{{color:#ff8000}}
        .bd{{background:#c0392b;color:white;border:none;padding:7px 14px;border-radius:4px;cursor:pointer;font-size:12px}}
        .bp{{background:#8e44ad;color:white;border:none;padding:7px 14px;border-radius:4px;cursor:pointer;font-size:12px}}
        .bg{{background:#27ae60;color:white;border:none;padding:7px 14px;border-radius:4px;cursor:pointer;font-size:12px}}
        .leg{{font-size:11px;color:#666;margin-top:6px;text-align:right}}
        .toast{{position:fixed;bottom:20px;right:20px;background:#2ecc71;color:#111;
                padding:10px 20px;border-radius:6px;font-weight:bold;font-size:13px;
                display:none;z-index:1000}}
        .toast.err{{background:#e74c3c;color:white}}
    </style></head><body>
    <h1>Pose Dashboard — Both Arms</h1>
    <p class="sub">11-feature model · 2.5s movement</p>

    <div class="sc"><img src="/video" alt="Live feed"></div>

    <div class="sb">
        <div class="si">R confirmed: <span id="rc">---</span></div>
        <div class="si">L confirmed: <span id="lc" class="left">---</span></div>
        <div class="si">Turn: <span id="tc">---</span></div>
        <div class="si">Lock: <span id="lk">---</span></div>
        <div style="display:flex;gap:6px;flex-wrap:wrap">
            <button class="bg" onclick="refresh()">↻ Refresh</button>
            <button class="bp" onclick="fetch('/diagnose/right').then(()=>toast('R diagnose → console',0))">Diagnose R</button>
            <button class="bp" onclick="fetch('/diagnose/left').then(()=>toast('L diagnose → console',0))">Diagnose L</button>
            <button class="bd" onclick="if(confirm('Clear ALL training data?'))clrAll()">✕ Clear All</button>
        </div>
    </div>

    <div class="card">
        <h2>Right Arm Poses</h2>
        <table><thead><tr style="color:#666;font-size:11px">
            <th style="text-align:left;padding:4px 16px">Pose</th>
            <th>Examples</th><th>Record</th><th>Clear</th>
        </tr></thead><tbody id="rt">{rows(rp,'right',rs)}</tbody></table>
        <div class="leg">
            <span style="color:#e74c3c">■</span>&lt;5 &nbsp;
            <span style="color:#f39c12">■</span>5–11 &nbsp;
            <span style="color:#2ecc71">■</span>12+
        </div>
    </div>

    <div class="card">
        <h2 class="left">Left Arm Poses</h2>
        <table><thead><tr style="color:#666;font-size:11px">
            <th style="text-align:left;padding:4px 16px">Pose</th>
            <th>Examples</th><th>Record</th><th>Clear</th>
        </tr></thead><tbody id="lt">{rows(lp,'left',ls)}</tbody></table>
        <div class="leg">
            <span style="color:#e74c3c">■</span>&lt;5 &nbsp;
            <span style="color:#f39c12">■</span>5–11 &nbsp;
            <span style="color:#2ecc71">■</span>12+
        </div>
    </div>

    <div class="toast" id="t"></div>
    <script>
        function toast(m,e){{const t=document.getElementById('t');t.textContent=m;
            t.className='toast'+(e?' err':'');t.style.display='block';
            setTimeout(()=>t.style.display='none',2500)}}

        function rec(side,pose,btn){{const o=btn.textContent;btn.textContent='...';btn.disabled=true;
            fetch('/record/'+side+'/'+pose).then(r=>r.json()).then(d=>{{
                if(d.error)toast(d.error,1);
                else{{toast(pose.replace(/_/g,' ')+' — '+d.total_examples+' ex',0);refresh()}}
                btn.textContent=o;btn.disabled=false}})
            .catch(()=>{{toast('Failed',1);btn.textContent=o;btn.disabled=false}})}}

        function clr(side,pose){{if(!confirm('Clear '+side+'/'+pose+'?'))return;
            fetch('/record/clear/'+side+'/'+pose).then(r=>r.json())
            .then(()=>{{toast('Cleared',0);refresh()}})}}

        function clrAll(){{fetch('/record/clear/all').then(r=>r.json())
            .then(()=>{{toast('All cleared',1);refresh()}})}}

        function upd(id,status,side,poses){{const tb=document.getElementById(id);tb.innerHTML='';
            poses.forEach(pose=>{{const cnt=status[pose]||0;
                const col=cnt<5?'#e74c3c':cnt<12?'#f39c12':'#2ecc71';
                const lbl=pose.replace(/_/g,' ').replace(/\\b\\w/g,c=>c.toUpperCase());
                tb.innerHTML+=`<tr>
                    <td style="padding:8px 16px;font-size:14px">${{lbl}}</td>
                    <td style="padding:8px;text-align:center">
                        <span style="color:${{col}};font-weight:bold">${{cnt}}</span></td>
                    <td style="padding:8px"><button onclick="rec('${{side}}','${{pose}}',this)"
                        style="background:#2980b9;color:white;border:none;padding:5px 12px;
                               border-radius:4px;cursor:pointer;font-size:12px">+</button></td>
                    <td style="padding:8px"><button onclick="clr('${{side}}','${{pose}}')"
                        style="background:#7f8c8d;color:white;border:none;padding:5px 12px;
                               border-radius:4px;cursor:pointer;font-size:12px">×</button></td>
                </tr>`}})}}

        function refresh(){{
            fetch('/knn/status').then(r=>r.json()).then(d=>{{
                document.getElementById('rc').textContent=d.right_confirmed||'---';
                document.getElementById('lc').textContent=d.left_confirmed||'---';
                upd('rt',d.right,'right',{list(rp)});
                upd('lt',d.left,'left',{list(lp)});
            }});
            fetch('/state').then(r=>r.json()).then(d=>{{
                document.getElementById('tc').textContent=d.turn_cmd||'---';
                document.getElementById('lk').textContent=d.locked?'YES':'NO';
            }});
        }}

        setInterval(refresh,2000);refresh();
    </script></body></html>"""


if __name__=="__main__":
    t=threading.Thread(target=process_frames,daemon=True); t.start()
    import socket; ip=socket.gethostbyname(socket.gethostname())
    print(f"\nStream:    http://{ip}:5000")
    print(f"Dashboard: http://{ip}:5000/dashboard")
    print(f"Angles:    http://{ip}:5000/angles")
    print(f"State:     http://{ip}:5000/state")
    print(f"Diagnose R: http://{ip}:5000/diagnose/right")
    print(f"Diagnose L: http://{ip}:5000/diagnose/left\n")
    app.run(host="0.0.0.0",port=5000,threaded=True)
