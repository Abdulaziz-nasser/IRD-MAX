#!/usr/bin/env python3
# -- coding: utf-8 --
"""
Slow open comparison — starts on S or the Arduino button.
The two files differ only in STRAIGHT_YAW_ENABLED.
Off: centered steering between corners. On: proven [STRAIGHT] heading hold.
Both retain IMU corner turns, camera/ultrasonic corner gates and speed rules.
Side-wall steering is disabled in both versions to isolate heading hold.
Run beside the original config folder; the Arduino code is unchanged.

Hotkeys:
  S -> send START (manual start)
  X -> reset Arduino (with backoff)   [kept as a manual tool only]
  Q -> STOP and quit
"""

import cv2 as cv
import numpy as np
import yaml, glob, time, os, re, math, sys, gc, atexit
from collections import deque

try:
    import serial
except Exception:
    serial = None

# Straight correction only; IMU corner turns remain active in both files.
STRAIGHT_YAW_ENABLED = True

# ==========================================================
# =================== TUNING PANEL =========================
# ==========================================================
TUNE = {
    # --- Turn / geometry ---
    "TURN_DEG": float(os.environ.get("TURN_DEG", "90.0")),
    "TURN_TRIGGER_TOL_DEG": float(os.environ.get("TURN_TRIGGER_TOL_DEG", "4.0")),
    "CENTER_TURN_STEP_DEG": float(os.environ.get("CENTER_TURN_STEP_DEG", "90.0")),

    # --- Turn timeout ---
    "TURN_TIMEOUT_MS": int(os.environ.get("TURN_TIMEOUT_MS", "1000")),

    # --- Run & stop caps ---
    "MAX_TURNS": int(os.environ.get("MAX_TURNS", "12")),
    "STOP_AT_END_CM": float(os.environ.get("STOP_AT_END_CM", "170.0")),

    # --- Steering servo geometry (absolute degrees) ---
    "CENTER_DEG": int(os.environ.get("CENTER_DEG", "90")),
    "LEFT_LIMIT": int(os.environ.get("LEFT_LIMIT", "60")),
    "RIGHT_LIMIT": int(os.environ.get("RIGHT_LIMIT", "110")),
    "SERVO_REVERSED": int(os.environ.get("SERVO_REVERSED", "1")),

    # --- Drive speeds (PWM 0..255) ---
    "DRIVE_SPEED": int(os.environ.get("DRIVE_SPEED", "46")),
    "TURN_SPEED": int(os.environ.get("TURN_SPEED", "40")),
    "POST_TURN_SPEED": int(os.environ.get("POST_TURN_SPEED", "35")),
    "POST_TURN_SEC": float(os.environ.get("POST_TURN_SEC", ".5")),
    "SEE_COLOR_SPEED": int(os.environ.get("SEE_COLOR_SPEED", "35")),
    "MIN_SPEED": int(os.environ.get("MIN_SPEED", "35")),

    # --- Wall proximity slowdowns ---
    "WALL_SLOW_SIDE_ON_CM": float(os.environ.get("WALL_SLOW_SIDE_ON_CM", "11.50")),
    "WALL_SLOW_SIDE_OFF_CM": float(os.environ.get("WALL_SLOW_SIDE_OFF_CM", "12.50")),
    "SIDE_SLOWDOWN_FACTOR": float(os.environ.get("SIDE_SLOWDOWN_FACTOR", "0.55")),
    "WALL_SLOW_FRONT_ON_CM": float(os.environ.get("WALL_SLOW_FRONT_ON_CM", "80.0")),
    "FRONT_SLOWDOWN_FACTOR": float(os.environ.get("FRONT_SLOWDOWN_FACTOR", "0.550")),
    "WALL_SLOW_MIN_SPEED": int(os.environ.get("WALL_SLOW_MIN_SPEED", "42")),

    # --- Wall-safety steering authority ---
    "WALL_STEER_GAIN": float(os.environ.get("WALL_STEER_GAIN", ".6")),
    "WALL_STEER_MAX_ADD_DEG": float(os.environ.get("WALL_STEER_MAX_ADD_DEG", "2.80")),
    "WALL_STEER_EXPAND_CENTER_CLAMP": int(os.environ.get("WALL_STEER_EXPAND_CENTER_CLAMP", "1")),
    "WALL_STEER_CENTER_MIN": int(os.environ.get("WALL_STEER_CENTER_MIN", "80")),
    "WALL_STEER_CENTER_MAX": int(os.environ.get("WALL_STEER_CENTER_MAX", "100")),

    # --- Strong Wall AVOID ---
    "WALL_AVOID_ON_CM": float(os.environ.get("WALL_AVOID_ON_CM", "11.5")),
    "WALL_AVOID_OFF_CM": float(os.environ.get("WALL_AVOID_OFF_CM", "14.5")),
    "WALL_AVOID_MAX_BIAS_DEG": float(os.environ.get("WALL_AVOID_MAX_BIAS_DEG", "20.0")),
    "WALL_AVOID_GAIN": float(os.environ.get("WALL_AVOID_GAIN", "1.3")),
    "WALL_AVOID_SPEED": int(os.environ.get("WALL_AVOID_SPEED", "42")),
    "WALL_AVOID_TIMEOUT_S": float(os.environ.get("WALL_AVOID_TIMEOUT_S", ".2")),

    # --- Contact failsafe ---
    "CONTACT_ON_CM": float(os.environ.get("CONTACT_ON_CM", "12.0")),
    "CONTACT_TREAT_INVALID_AS_ON": int(os.environ.get("CONTACT_TREAT_INVALID_AS_ON", "1")),
    "CONTACT_STEER_DEG": float(os.environ.get("CONTACT_STEER_DEG", "14.0")),
    "CONTACT_HOLD_S": float(os.environ.get("CONTACT_HOLD_S", "0.3")),
    "CONTACT_SPEED_FACTOR": float(os.environ.get("CONTACT_SPEED_FACTOR", "0.6")),

    # --- Turn gating (FRONT + SIDE) ---
    "FRONT_TURN_THRESH_CM": float(os.environ.get("FRONT_TURN_THRESH_CM", "35.0")),
    "TURN_USE_SIDE_GATE": int(os.environ.get("TURN_USE_SIDE_GATE", "1")),
    "TURN_SIDE_OPEN_CM": float(os.environ.get("TURN_SIDE_OPEN_CM", "80.0")),
    "TURN_SIDE_LOGIC": os.environ.get("TURN_SIDE_LOGIC", "OR").upper(),
    "TURN_GATE_COMBINE": os.environ.get("TURN_GATE_COMBINE", "OR").upper(),

    # --- Color detection / ROI ---
    "COLOR_CONFIRM_FRAMES": int(os.environ.get("COLOR_CONFIRM_FRAMES", "1")),
    "DETECT_COOLDOWN_S": float(os.environ.get("DETECT_COOLDOWN_S", "0.9")),
    "MIN_PIX_COLOR": int(os.environ.get("MIN_PIX_COLOR", "1")),
    "COLOR_ERODE_IT": int(os.environ.get("COLOR_ERODE_IT", "0")),
    "COLOR_DILATE_IT": int(os.environ.get("COLOR_DILATE_IT", "1")),
    "COLOR_ROI_WIDTH_FRAC": float(os.environ.get("COLOR_ROI_WIDTH_FRAC", "0.4")),
    "COLOR_ROI_HEIGHT_FRAC": float(os.environ.get("COLOR_ROI_HEIGHT_FRAC", "0.25")),
    "ROI_SCALE_AFTER_FIRST_TURN": float(os.environ.get("ROI_SCALE_AFTER_FIRST_TURN", "1.8")),
    "COLOR_ROI_AFTER_TURNS": int(os.environ.get("COLOR_ROI_AFTER_TURNS", "0")),
    "COLOR_PICK_MODE": "maxpix",
    "COLOR_TURN_MAP": {"BLUE": "LEFT", "ORANGE": "RIGHT"},
    "SAT_BOOST": float(os.environ.get("SAT_BOOST", "1.15")),
    "BLUR_KSIZE": int(os.environ.get("BLUR_KSIZE", "1")),
    "COLOR_MIN_PIX_FRAC": float(os.environ.get("COLOR_MIN_PIX_FRAC", "0.0010")),
    "COLOR_SUSPEND_SEC": float(os.environ.get("COLOR_SUSPEND_SEC", ".5")),

    # --- Color re-arm ---
    "COLOR_REARM_FRONT_CM": float(os.environ.get("COLOR_REARM_FRONT_CM", "90.0")),
    "COLOR_REARM_DELAY_S": float(os.environ.get("COLOR_REARM_DELAY_S", "0.6")),
    "DISABLE_DETECT_DURING_TURN": True,

    # --- Yaw centering controller ---
    "STEER_SIGN": float(os.environ.get("STEER_SIGN", "1.0")),
    "STEER_KP_DEG_PER_DEG": float(os.environ.get("STEER_KP_DEG_PER_DEG", ".32")),
    "STEER_MAX_DEG": float(os.environ.get("STEER_MAX_DEG", "5.0")),
    "STEER_DEAD_ERR_DEG": float(os.environ.get("STEER_DEAD_ERR_DEG", "1.5")),
    "ERR_FILTER_ALPHA": float(os.environ.get("ERR_FILTER_ALPHA", "0.30")),
    "STEER_KD_DEG_PER_DPS": float(os.environ.get("STEER_KD_DEG_PER_DPS", "0.10")),
    "YAW_RATE_FILTER_ALPHA": float(os.environ.get("YAW_RATE_FILTER_ALPHA", "0.25")),
    "STEER_SLEW_DEG_PER_SEC": float(os.environ.get("STEER_SLEW_DEG_PER_SEC", "15.0")),
    "CONTROL_PERIOD_S": 0.05,
    "CENTER_STEER_MIN": 70,
    "CENTER_STEER_MAX": 110,

    # --- Ultrasonic averaging & repel ---
    "US_MAX_CM": float(os.environ.get("US_MAX_CM", "300.0")),
    "US_EMA_ALPHA": float(os.environ.get("US_EMA_ALPHA", "0.35")),
    "US_REPEL_ON_CM": float(os.environ.get("US_REPEL_ON_CM", "18.0")),
    "US_REPEL_OFF_CM": float(os.environ.get("US_REPEL_OFF_CM", "22.0")),
    "US_REPEL_K": float(os.environ.get("US_REPEL_K", "0.06")),
    "US_REPEL_STEER_BOOST_DEG": float(os.environ.get("US_REPEL_STEER_BOOST_DEG", "6.0")),
    "US_REPEL_EXPAND_CENTER_CLAMP": int(os.environ.get("US_REPEL_EXPND_CENTER_CLAMP", "1")),
    "US_REPEL_CENTER_MIN": int(os.environ.get("US_REPEL_CENTER_MIN", "80")),
    "US_REPEL_CENTER_MAX": int(os.environ.get("US_REPEL_CENTER_MAX", "100")),

    # --- Camera ---
    "ALLOW_NO_CAMERA": int(os.environ.get("ALLOW_NO_CAMERA", "1")),
    "CAMERA_BACKEND": os.environ.get("CAMERA_BACKEND", "AUTO").upper(),
    "FRAME_W": int(os.environ.get("FRAME_W", "640")),
    "FRAME_H": int(os.environ.get("FRAME_H", "480")),

    # --- Final-run logic ---
    "FINAL_BURST_S": float(os.environ.get("FINAL_BURST_S", "0")),
    "FINAL_US_DELAY_S": float(os.environ.get("FINAL_US_DELAY_S", "0")),
    "REVERSE_SUPPORTED": int(os.environ.get("REVERSE_SUPPORTED", "1")),
    "REVERSE_SPEED": int(os.environ.get("REVERSE_SPEED", "30")),
    "REVERSE_MAX_S": float(os.environ.get("REVERSE_MAX_S", "2.0")),

    # --- Serial link & watchdog (we DO NOT auto-reset from Jetson anymore) ---
    "ROBOT_PORT": os.environ.get("ROBOT_PORT", "/dev/ttyUSB0"),
    "ROBOT_BAUD": int(os.environ.get("ROBOT_BAUD", "115200")),
    "STALE_TLM_SEC": float(os.environ.get("STALE_TLM_SEC", "1.2")),
    "STALE_LINK_SEC": float(os.environ.get("STALE_LINK_SEC", "2.5")),
    "PING_PERIOD_SEC": float(os.environ.get("PING_PERIOD_SEC", "0.7")),
    "RESET_BACKOFF_SEC": float(os.environ.get("RESET_BACKOFF_SEC", "4.0")),

    "ARD_RESET_METHOD": os.environ.get("ARD_RESET_METHOD", "AUTO").upper(),
    "ARD_RESET_TOUCH_DELAY_S": float(os.environ.get("ARD_RESET_TOUCH_DELAY_S", "0.8")),
    "ARD_RESET_DTR_PULSE_S": float(os.environ.get("ARD_RESET_DTR_PULSE_S", "0.12")),

    # --- REF handling ---
    "FIX_REF_TO_START": int(os.environ.get("FIX_REF_TO_START", "0")),
    "REF_STEP_AFTER_TURN_DEG": float(os.environ.get("REF_STEP_AFTER_TURN_DEG",
                                          os.environ.get("TURN_DEG", "90.0"))),

    # Legacy lane steering settings are unused in this comparison.
    "PID_KP": float(os.environ.get("PID_KP", ".25")),
    "PID_KI": float(os.environ.get("PID_KI", "0.0")),
    "PID_KD": float(os.environ.get("PID_KD", "0.12")),
    "PID_DEAD_DEG": float(os.environ.get("PID_DEAD_DEG", "0.014")),
    "PID_I_CLAMP_DEG": float(os.environ.get("PID_I_CLAMP_DEG", "6.0")),

    "LANE_WALL_DEAD_CM": float(os.environ.get("LANE_WALL_DEAD_CM", "2.13")),
    "LANE_WALL_MAX_CM": float(os.environ.get("LANE_WALL_MAX_CM", "30.0")),
    "LANE_K_WALL_DEG": float(os.environ.get("LANE_K_WALL_DEG", "2.2")),
    "LANE_WALL_EXP": float(os.environ.get("LANE_WALL_EXP", "1.60")),
    "LANE_ADAPT_BY_SPEED": float(os.environ.get("LANE_ADAPT_BY_SPEED", ".20")),
    "SPEED_BASE": int(os.environ.get("SPEED_BASE", "65")),

    # ===== Strict color-gate for corner turns =====
    "REQUIRE_COLOR_FOR_TURN": int(os.environ.get("REQUIRE_COLOR_FOR_TURN", "1")),

    # ===== Final align on color only =====
    "FINAL_ALIGN": int(os.environ.get("FINAL_ALIGN", "1")),
    "FINAL_ALIGN_SPEED": int(os.environ.get("FINAL_ALIGN_SPEED", "45")),
    "FINAL_ALIGN_TIMEOUT_S": float(os.environ.get("FINAL_ALIGN_TIMEOUT_S", "8.0")),
    "FINAL_ALIGN_CONFIRM_FRAMES": int(os.environ.get("FINAL_ALIGN_CONFIRM_FRAMES", "2")),
    "FINAL_ALIGN_ROI_W_FRAC": float(os.environ.get("FINAL_ALIGN_ROI_W_FRAC", "0.28")),
    "FINAL_ALIGN_ROI_H_FRAC": float(os.environ.get("FINAL_ALIGN_ROI_H_FRAC", "0.32")),
    "FINAL_ALIGN_DETECT_DELAY_S": float(os.environ.get("FINAL_ALIGN_DETECT_DELAY_S", "1.40")),
}

# ================ CONSTANTS / HELPERS ================
MIN_SPEED   = TUNE["MIN_SPEED"]
CENTER_DEG  = TUNE["CENTER_DEG"]
LEFT_LIMIT  = TUNE["LEFT_LIMIT"]
RIGHT_LIMIT = TUNE["RIGHT_LIMIT"]
SPAN_LEFT   = abs(CENTER_DEG - LEFT_LIMIT)
SPAN_RIGHT  = abs(RIGHT_LIMIT - CENTER_DEG)
SERVO_DIR   = -1 if int(TUNE.get("SERVO_REVERSED", 1)) else +1

# ---------- Camera helpers (robust across restarts) ----------
def gstreamer_pipeline(sensor_id=0, capture_width=1280, capture_height=720,
                       display_width=640, display_height=480, framerate=30, flip_method=0):
    return (f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, "
            f"format=(string)NV12, framerate={framerate}/1 ! "
            f"nvvidconv flip-method={flip_method} ! "
            f"video/x-raw, width={display_width}, height={display_height}, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=1 sync=false")

_cap_global = None

def _release_cap():
    global _cap_global
    if _cap_global is not None:
        try:
            _cap_global.release()
        except Exception:
            pass
        _cap_global = None
    try:
        cv.destroyAllWindows()
    except Exception:
        pass
    gc.collect()

atexit.register(_release_cap)

def open_camera():
    global _cap_global
    back = TUNE["CAMERA_BACKEND"]; w, h = TUNE["FRAME_W"], TUNE["FRAME_H"]

    def try_gst():
        try:
            cap = cv.VideoCapture(gstreamer_pipeline(display_width=w, display_height=h), cv.CAP_GSTREAMER)
            return cap if (cap and cap.isOpened()) else None
        except Exception:
            return None

    def try_v4l2():
        try:
            cap = cv.VideoCapture(0, cv.CAP_V4L2)
            if not (cap and cap.isOpened()):
                if cap: cap.release()
                cap = cv.VideoCapture(0, cv.CAP_ANY)
            return cap if (cap and cap.isOpened()) else None
        except Exception:
            return None

    orders = {
        "GST":  [try_gst, try_v4l2],
        "V4L2": [try_v4l2, try_gst],
        "AUTO": [try_gst, try_v4l2],
    }
    order = orders.get(back, orders["AUTO"])

    attempts = 8
    for i in range(attempts):
        _release_cap()
        for maker in order:
            cap = maker()
            if cap:
                _cap_global = cap
                ok, _ = cap.read()
                if not ok:
                    time.sleep(0.05); ok, _ = cap.read()
                if ok:
                    return cap, False
                time.sleep(0.06); ok, _ = cap.read()
                if ok:
                    return cap, False
                cap.release(); _cap_global = None
        time.sleep(0.15 + 0.05 * i)

    if TUNE["ALLOW_NO_CAMERA"]:
        print("WARNING: camera open failed; using dummy frames.")
        class DummyCap:
            def isOpened(self): return True
            def read(self):
                frame = np.zeros((h, w, 3), dtype=np.uint8)
                cv.putText(frame, "NO CAMERA (dummy)", (10,30),
                           cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                return True, frame
            def release(self): pass
        return DummyCap(), True

    print("ERROR: camera open failed permanently.")
    return None, False

# ---------- Misc helpers ----------
def load_latest_yaml():
    override = os.environ.get("VISION_CFG", "").strip()
    if override and os.path.isfile(override): path = override
    else:
        cands = sorted(glob.glob("config/vision_*.yaml"))
        if not cands: raise FileNotFoundError("No config in ./config — run tuner_multi.py and press S to save.")
        path = cands[-1]
    with open(path, "r") as f: return yaml.safe_load(f), path

def wrap180(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a

def shortest_err(target_deg, now_deg): return wrap180(target_deg - now_deg)

def deg_to_norm(angle_deg):
    angle_deg = float(max(LEFT_LIMIT, min(RIGHT_LIMIT, angle_deg)))
    if angle_deg >= CENTER_DEG:
        return (angle_deg - CENTER_DEG) / float(max(1.0, SPAN_RIGHT))
    else:
        return (angle_deg - CENTER_DEG) / float(max(1.0, SPAN_LEFT))

def slew_limit(prev, target, max_delta):
    if prev is None: return target
    if target > prev + max_delta: return prev + max_delta
    if target < prev - max_delta: return prev - max_delta
    return target

class HeadingHold:
    """Straight heading correction updated once per fresh yaw sample."""
    def __init__(self, tune, center_deg):
        self.tune = tune
        self.center = float(center_deg)
        self.reset()

    def reset(self):
        self.context = None
        self.target = None
        self.sample_time = None
        self.yaw = None
        self.error = 0.0
        self.rate = 0.0
        self.command = self.center
        self.command_time = None
        self.command_direction = None

    def update(self, yaw, target, sample_time, context, kp, max_offset):
        if sample_time is None or not math.isfinite(float(yaw)):
            return self.center
        yaw, target = float(yaw), float(target)
        new_section = (context != self.context or self.target is None or
                       abs(shortest_err(target, self.target)) > 1e-6 or
                       self.sample_time is None)
        if new_section:
            self.reset()
            self.context, self.target = context, target
            self.yaw, self.sample_time = yaw, sample_time
            self.error = shortest_err(target, yaw)
        elif sample_time > self.sample_time:
            dt = sample_time - self.sample_time
            if dt > 0.30:
                self.error = shortest_err(target, yaw)
                self.rate = 0.0
                self.yaw, self.sample_time = yaw, sample_time
            elif dt >= 0.01:
                alpha = max(0.001, min(1.0, float(self.tune['ERR_FILTER_ALPHA'])))
                a = 1.0 - (1.0 - alpha) ** (dt / 0.05)
                self.error = wrap180(
                    self.error + a * shortest_err(shortest_err(target, yaw), self.error))
                rate_alpha = max(0.001, min(1.0, float(self.tune['YAW_RATE_FILTER_ALPHA'])))
                ar = 1.0 - (1.0 - rate_alpha) ** (dt / 0.05)
                measured_rate = shortest_err(yaw, self.yaw) / dt
                measured_rate = max(-180.0, min(180.0, measured_rate))
                self.rate += ar * (measured_rate - self.rate)
                self.yaw, self.sample_time = yaw, sample_time

        deadband = max(0.0, float(self.tune['STEER_DEAD_ERR_DEG']))
        raw_error = shortest_err(target, self.yaw)
        if abs(raw_error) <= deadband:
            self.error = 0.0
            if abs(self.rate) < 3.0:
                return self.center
        effective_error = math.copysign(
            max(0.0, abs(self.error) - deadband), self.error)
        correction = (float(kp) * effective_error -
                      float(self.tune['STEER_KD_DEG_PER_DPS']) * self.rate)
        correction *= float(self.tune['STEER_SIGN'])
        correction = max(-max_offset, min(max_offset, correction))
        return self.center + correction

    def limit_command(self, target, now, direction):
        if self.command_direction != direction:
            self.command, self.command_time = self.center, None
        dt = (0.05 if self.command_time is None
              else max(0.0, min(0.10, now - self.command_time)))
        rate = max(0.1, float(self.tune['STEER_SLEW_DEG_PER_SEC']))
        self.command = slew_limit(self.command, float(target), rate * dt)
        self.command_time, self.command_direction = now, direction
        return self.command


def mask_hsv_single(hsv, low, high, ek=0, dk=0):
    m = cv.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    if ek > 0: m = cv.erode(m, cv.getStructuringElement(cv.MORPH_ELLIPSE, (ek, ek)))
    if dk > 0: m = cv.dilate(m, cv.getStructuringElement(cv.MORPH_ELLIPSE, (dk, dk)))
    return m

def mask_hsv_multi(hsv, ranges, ek=0, dk=0):
    if isinstance(ranges, dict) and "low" in ranges and "high" in ranges:
        return mask_hsv_single(hsv, ranges["low"], ranges["high"], ek, dk)
    mask = None
    for r in (ranges if isinstance(ranges, list) else []):
        if "low" in r and "high" in r:
            m = mask_hsv_single(hsv, r["low"], r["high"], ek, dk)
            mask = m if mask is None else cv.bitwise_or(mask, m)
    if mask is None: mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    return mask

def compute_center_roi(w, h):
    rwf = max(0.06, min(0.80, float(TUNE["FINAL_ALIGN_ROI_W_FRAC"])))
    rhf = max(0.05, min(0.60, float(TUNE["FINAL_ALIGN_ROI_H_FRAC"])))
    rw = max(12, int(rwf * w))
    rh = max(12, int(rhf * h))
    x0 = (w - rw)//2; x1 = x0 + rw
    y0 = (h - rh)//2; y1 = y0 + rh
    return (x0, y0, x1, y1)

def compute_color_roi(w, h):
    base_wf = float(TUNE["COLOR_ROI_WIDTH_FRAC"])
    base_hf = float(TUNE["COLOR_ROI_HEIGHT_FRAC"])
    scale = 1.0
    rwf = max(0.06, min(0.80, base_wf * scale))
    rhf = max(0.05, min(0.40, base_hf * scale))
    rw = max(12, int(rwf * w))
    rh = max(12, int(rhf * h))
    x0 = (w - rw)//2; x1 = x0 + rw
    y1 = h; y0 = h - rh
    return (x0,y0,x1,y1)

# =================== MAIN ===================
def main():
    cfg, cfg_path = load_latest_yaml()
    print("Loaded vision config:", cfg_path)

    # ---- Serial (robust) ----
    ser = None
    device = TUNE["ROBOT_PORT"]
    baud = TUNE["ROBOT_BAUD"]

    last_tlm_t = 0.0
    last_yaw_t = None
    last_pong_t = 0.0
    last_ping_t = 0.0

    rx_buf = b""
    heading_hold = HeadingHold(TUNE, CENTER_DEG)
    tlm_re = re.compile(r"^TLM,.*yaw=([\-0-9\.]+),state=([0-9]),steer=([\-0-9\.]+),speed=([0-9]+),dF=([\-0-9]+),dL=([\-0-9]+),dR=([\-0-9]+)")

    # restart logic flags
    ready_count = 0
    restart_requested = False

    def self_restart():
        """Cleanly re-exec the current Python program (release cam/serial first)."""
        print("[SYS] Arduino reset detected -> restarting Python process...")
        try:
            if ser:
                try: ser.close()
                except Exception: pass
        except Exception:
            pass
        _release_cap()
        sys.stdout.flush(); sys.stderr.flush()
        os.execv(sys.executable, [sys.executable] + sys.argv)

    def open_serial():
        nonlocal ser, last_yaw_t
        last_yaw_t = None
        heading_hold.reset()
        if serial is None:
            print("PySerial not available."); return
        try:
            ser = serial.Serial(device, baudrate=baud, timeout=0.01, write_timeout=0.2, dsrdtr=False)
            time.sleep(0.35)
            ser.reset_input_buffer(); ser.reset_output_buffer()
            print("Serial OK on", device)
        except Exception as e:
            print("WARNING: serial not open ->", e); ser = None

    def safe_write(line_bytes):
        nonlocal ser
        command = line_bytes.strip().split(b',', 1)[0].upper()
        if command in (b'START', b'STOP', b'TURN_ABS'):
            heading_hold.reset()
        if serial is None: return False
        if ser is None:
            open_serial()
            if ser is None: return False
        try:
            ser.write(line_bytes); return True
        except Exception as e:
            print("Serial write error:", e)
            try: ser.close()
            except Exception: pass
            ser=None
            open_serial()
            if ser:
                try: ser.write(line_bytes); return True
                except Exception as e2: print("Write retry failed:", e2)
        return False

    # NOTE: keep this for manual key 'X' only; we never auto-call it.
    def reset_arduino(method="AUTO"):
        nonlocal ser
        meth = (method or "AUTO").upper()
        port = device
        def do_touch1200():
            try:
                print("[RESET] 1200-bps touch...")
                tmp = serial.Serial(port, 1200, timeout=0.05)
                time.sleep(0.05); tmp.setDTR(False); tmp.flush(); tmp.close()
                time.sleep(TUNE["ARD_RESET_TOUCH_DELAY_S"])
                return True
            except Exception as e:
                print("[RESET] 1200-bps touch failed:", e); return False
        def do_dtr():
            try:
                print("[RESET] DTR pulse...")
                s = serial.Serial(port, baudrate=baud, timeout=0.05, dsrdtr=False)
                s.dtr = False; time.sleep(TUNE["ARD_RESET_DTR_PULSE_S"]); s.dtr = True
                time.sleep(0.25); s.close()
                return True
            except Exception as e:
                print("[RESET] DTR pulse failed:", e); return False
        if ser and ser.is_open:
            try: ser.close()
            except Exception: pass
            ser = None
            time.sleep(0.1)
        ok = False
        if meth in ("AUTO", "TOUCH1200"):
            ok = do_touch1200()
        if meth == "AUTO" and not ok:
            ok = do_dtr()
        elif meth == "DTR":
            ok = do_dtr()
        open_serial()
        print("[RESET] Done." if ok else "[RESET] Failed.")
        return ok

    def link_alive(now):
        return (now - max(last_tlm_t, last_pong_t)) < TUNE["STALE_LINK_SEC"]

    def maybe_ping(now):
        nonlocal last_ping_t
        if (now - last_ping_t) >= TUNE["PING_PERIOD_SEC"]:
            safe_write(b"PING\n")
            last_ping_t = now

    # --- Telemetry / parsing ---
    last_yaw = 0.0
    last_US = {"dF":-1, "dL":-1, "dR":-1}
    us_ema = {"dF":-1.0, "dL":-1.0, "dR":-1.0}

    # Start-control flags
    arduino_armed = False
    run_start_t = None
    ever_had_tlm = False

    def sane_cm(x):
        if x is None: return -1
        if x < 0: return -1
        return float(min(x, TUNE["US_MAX_CM"]))

    def yaw_ready():
        return (last_yaw_t is not None and
                0.0 <= time.monotonic() - last_yaw_t < TUNE["STALE_TLM_SEC"])

    def continuous_heading_active():
        return state in (STATE_CENTER, STATE_FINAL_BURST, STATE_FINAL_FWD, STATE_FINAL_ALIGN)

    def read_tlm():
        nonlocal rx_buf, last_yaw, last_yaw_t, last_tlm_t, last_US, us_ema, last_pong_t, arduino_armed, ready_count, restart_requested, ever_had_tlm
        if ser is None: return
        try:
            data = ser.read(max(256, ser.in_waiting))
            if not data: return
            rx_buf += data
            while b"\n" in rx_buf:
                line, rx_buf = rx_buf.split(b"\n", 1)
                s = line.decode("utf-8", "ignore").strip()
                if s == "PONG":
                    last_pong_t = time.time(); continue
                if s == "READY":
                    # If we see READY after we've already been running or had TLM before -> Arduino was reset
                    ready_count += 1
                    last_yaw_t = None
                    heading_hold.reset()
                    last_pong_t = time.time()
                    if ready_count >= 2 or ever_had_tlm:
                        restart_requested = True
                    continue
                if s == "ARMED":
                    arduino_armed = True
                    last_pong_t = time.time()
                    continue
                m = tlm_re.match(s)
                if m:
                    ever_had_tlm = True
                    yaw_sample = float(m.group(1))
                    if not math.isfinite(yaw_sample): continue
                    last_yaw = wrap180(yaw_sample % 360.0)
                    last_yaw_t = time.monotonic()
                    last_tlm_t = time.time()
                    dF = int(m.group(5)); dL = int(m.group(6)); dR = int(m.group(7))
                    a = TUNE["US_EMA_ALPHA"]
                    for k, v in (("dF",dF),("dL",dL),("dR",dR)):
                        v = float(v)
                        if v < 0: last_US[k] = -1; continue
                        if us_ema[k] < 0: us_ema[k] = v
                        else: us_ema[k] = (1-a)*us_ema[k] + a*v
                        last_US[k] = int(us_ema[k])
        except Exception as e:
            print("Serial read error:", e)
            try: ser.close()
            except Exception: pass
            open_serial()

    # ---- Camera ----
    cap, using_dummy = open_camera()
    if cap is None: return

    # ---- States & vars ----
    STATE_WAIT_START, STATE_CENTER, STATE_WAIT_TURN, STATE_FINAL_BURST, STATE_FINAL_FWD, STATE_FINAL_ALIGN, STATE_FINAL_BACK, STATE_DONE = 0,1,2,3,4,5,6,7
    state = STATE_WAIT_START
    print("READY. Keys: S=start, X=reset Arduino, Q=quit.")
    print("Comparison: " + ("[STRAIGHT] heading hold ON" if STRAIGHT_YAW_ENABLED
                            else "[NO_YAW] centered steering between turns"))

    yaw_start = None
    yaw_ref = 0.0
    last_turn_target = None
    last_turn_t = -1e9
    turn_count = 0

    detect_enabled = True
    strict_latch_active = False
    rearm_started_t = -1e9
    REARM_AFTER_TURN_T = -1.0
    color_q = deque(maxlen=TUNE["COLOR_CONFIRM_FRAMES"])

    turn_latched_dir = None
    CURRENT_TURN_DIR = None
    turn_requested = False

    see_color_slow_active = False
    see_color_consumed = False

    side_slow_active = False

    contact_active = False
    contact_dir = None
    contact_until = -1.0

    color_pause_until = -1.0

    # Heading-hold history is kept in heading_hold.
    last_straight_log_t = -1.0
    last_tx = 0.0
    last_spd_sent = 0

    final_burst_start = -1.0
    final_back_start = -1.0
    POST_BOOST_UNTIL  = -1.0
    final_eval_not_before = -1.0

    # Final-align state
    final_align_start_t = -1.0
    final_align_q = deque(maxlen=TUNE["FINAL_ALIGN_CONFIRM_FRAMES"])

    # -------- Wall AVOID FSM --------
    WA_IDLE, WA_ACTIVE = 0, 1
    wall_avoid_state = WA_IDLE
    wall_avoid_start_t = -1.0
    wall_avoid_side = None  # 'L' or 'R'

    cv.namedWindow("robot", cv.WINDOW_AUTOSIZE)

    def send_center(steer_deg, speed_pwm):
        nonlocal last_spd_sent, last_straight_log_t
        if continuous_heading_active():
            if STRAIGHT_YAW_ENABLED and speed_pwm > 0:
                steer_deg = heading_hold.limit_command(steer_deg, time.monotonic(), +1)
            else:
                steer_deg = float(CENTER_DEG)
        steer_norm = deg_to_norm(steer_deg)
        spd = max(MIN_SPEED, min(255, int(speed_pwm)))
        last_spd_sent = spd
        safe_write(f"CENTER,{steer_norm:.3f},{spd}\n".encode("ascii"))
        if continuous_heading_active() and time.monotonic() - last_straight_log_t >= 0.5:
            tag = "STRAIGHT" if STRAIGHT_YAW_ENABLED else "NO_YAW"
            print(f"[{tag}] yaw={last_yaw:.2f} target={yaw_ref:.2f} "
                  f"err={shortest_err(yaw_ref, last_yaw):+.2f} "
                  f"steer={steer_deg:.2f} speed={spd}")
            last_straight_log_t = time.monotonic()

    def send_back(speed_pwm):
        spd = max(0, min(255, int(speed_pwm)))
        safe_write(f"BACK,{spd}\n".encode("ascii"))

    def send_turn_timeout_ms(ms):
        safe_write(f"SET_TURN_TIMEOUT,{int(ms)}\n".encode("ascii"))

    # ---------- main loop ----------
    while True:
        now = time.time()

        maybe_ping(now)
        read_tlm()

        # If Arduino was reset -> restart ourselves (after clean release)
        if restart_requested:
            self_restart()
            return  # not reached

        # Auto-start when Arduino is armed
        if state == STATE_WAIT_START and arduino_armed and yaw_ready():
            if yaw_start is None: yaw_start = float(last_yaw)
            yaw_ref = float(yaw_start)
            send_turn_timeout_ms(TUNE["TURN_TIMEOUT_MS"])
            state = STATE_CENTER
            run_start_t = time.time()
            heading_hold.reset()
            print("[BUTTON] Arduino armed -> starting motion")

        # Both modes need fresh IMU telemetry for their corner turns.
        if state not in (STATE_WAIT_START, STATE_DONE) and not yaw_ready():
            safe_write(b"STOP\n")
            state = STATE_WAIT_START
            arduino_armed = False
            print("[YAW] Waiting for fresh telemetry; stopped.")

        # Link watchdog: DO NOT auto-reset Arduino anymore
        if not link_alive(now):
            # Just fall back to WAIT_START; keep process alive; do not touch Arduino
            if state != STATE_WAIT_START:
                print("[LINK] stale; waiting for Arduino to reappear (no auto reset).")
            state = STATE_WAIT_START
            arduino_armed = False
            heading_hold.reset()

        # --- Frame ---
        ok, frame = cap.read()
        if not ok:
            print("[CAM] frame read failed; attempting re-open...")
            cap, using_dummy = open_camera()
            if cap is None:
                print("[CAM] fatal open failure; exiting.")
                break
            ok, frame = cap.read()
            if not ok:
                print("[CAM] still cannot read; using black frame.")
                frame = np.zeros((TUNE["FRAME_H"], TUNE["FRAME_W"], 3), dtype=np.uint8)
        h, w = frame.shape[:2]
        mrx0,mry0,mrx1,mry1 = compute_color_roi(w,h)
        cx0, cy0, cx1, cy1 = compute_center_roi(w, h)  # final-align ROI for HUD

        # ---------- COLOR DETECTION (pre-corner, bottom ROI) ----------
        roi_bgr = frame[mry0:mry1, mrx0:mrx1]
        k = max(3, TUNE["BLUR_KSIZE"] | 1)
        roi_bgr = cv.GaussianBlur(roi_bgr, (k,k), 0)

        pause_active = (now < color_pause_until)
        cand_color = None
        if not pause_active:
            roi_hsv = cv.cvtColor(roi_bgr, cv.COLOR_BGR2HSV)
            if TUNE["SAT_BOOST"] != 1.0:
                hch, sch, vch = cv.split(roi_hsv)
                sch = np.clip(sch.astype(np.float32) * float(TUNE["SAT_BOOST"]), 0, 255).astype(np.uint8)
                roi_hsv = cv.merge([hch, sch, vch])

            ck_erode  = int(cfg.get("erode",  TUNE["COLOR_ERODE_IT"]))
            ck_dilate = int(cfg.get("dilate", TUNE["COLOR_DILATE_IT"]))

            blue_cfg   = cfg.get("BLUE")
            orange_cfg = cfg.get("ORANGE")

            m_bl = mask_hsv_multi(roi_hsv, blue_cfg,   ek=ck_erode, dk=ck_dilate)
            m_or = mask_hsv_multi(roi_hsv, orange_cfg, ek=ck_erode, dk=ck_dilate)

            roi_area = max(1, (mry1 - mry0) * (mrx1 - mrx0))
            dyn_min_pix = max(int(roi_area * float(TUNE["COLOR_MIN_PIX_FRAC"])),
                              int(TUNE["MIN_PIX_COLOR"]))

            pix_bl = int(np.count_nonzero(m_bl))
            pix_or = int(np.count_nonzero(m_or))
            pass_bl = (pix_bl >= dyn_min_pix)
            pass_or = (pix_or >= dyn_min_pix)

            recent_turn = (now - last_turn_t) < TUNE["DETECT_COOLDOWN_S"]
            in_blackout = (REARM_AFTER_TURN_T > 0) and (now < REARM_AFTER_TURN_T)
            if detect_enabled and (not recent_turn) and (not in_blackout) and state in (STATE_CENTER, STATE_FINAL_FWD):
                if pass_bl or pass_or:
                    if TUNE["COLOR_PICK_MODE"] == "maxpix":
                        if pass_bl and pass_or:
                            cand_color = "BLUE" if pix_bl >= pix_or else "ORANGE"
                        elif pass_bl:
                            cand_color = "BLUE"
                        else:
                            cand_color = "ORANGE"
                    else:
                        cand_color = "BLUE" if pass_bl else "ORANGE"
            else:
                cand_color = None
                color_q.clear()
        else:
            cand_color = None
            color_q.clear()

        if cand_color is not None: color_q.append(cand_color)
        else: color_q.clear()

        # ---- STRICT COLOR LATCH → sets the turn direction ----
        if (not pause_active) and detect_enabled \
           and len(color_q) == color_q.maxlen and len(color_q)>0 and all(c == color_q[0] for c in color_q):
            seen_color = color_q[0]; color_q.clear()
            raw_dir = TUNE["COLOR_TURN_MAP"].get(seen_color, None)
            if raw_dir in ("LEFT","RIGHT"):
                turn_latched_dir = raw_dir
                detect_enabled = False
                strict_latch_active = True
                sign = -1.0 if turn_latched_dir == "LEFT" else +1.0
                last_turn_target = wrap180(yaw_ref + sign * TUNE["TURN_DEG"])
                turn_requested = True
                print(f"[LATCH] color={seen_color} dir={turn_latched_dir} -> target={last_turn_target:.1f}°")

                color_pause_until = time.time() + float(TUNE["COLOR_SUSPEND_SEC"])
                if not see_color_consumed:
                    see_color_slow_active = True
                    see_color_consumed = True
                    print("[SEE-COLOR] slowdown ACTIVE until turn starts")

        # ---------- TURN GATE ----------
        def side_open_by_dir(turn_dir, dL, dR, cm):
            if turn_dir == "LEFT":
                return (dL >= 0 and dL > cm)
            elif turn_dir == "RIGHT":
                return (dR >= 0 and dR > cm)
            return False

        if state == STATE_CENTER and TUNE["REQUIRE_COLOR_FOR_TURN"]:
            if turn_requested:
                dF = sane_cm(last_US.get("dF", -1))
                dL = sane_cm(last_US.get("dL", -1)); dR = sane_cm(last_US.get("dR", -1))
                fresh = (time.time() - last_tlm_t) < 0.5
                front_ready = fresh and dF >= 0 and dF < TUNE["FRONT_TURN_THRESH_CM"]
                side_ready = side_open_by_dir(turn_latched_dir, dL, dR, TUNE["TURN_SIDE_OPEN_CM"]) if TUNE["TURN_USE_SIDE_GATE"] else False
                allow_turn = (front_ready or side_ready)
                if allow_turn and last_turn_target is not None:
                    CURRENT_TURN_DIR = turn_latched_dir
                    safe_write(f"TURN_ABS,{last_turn_target:.1f},{int(TUNE['TURN_SPEED'])}\n".encode("ascii"))
                    state = STATE_WAIT_TURN
                    last_turn_t = time.time()
                    turn_requested = False
                    see_color_slow_active = False
                    strict_latch_active = False
                    heading_hold.reset()
                    print("[TURN] started after color-gate")

        # ===== TURN COMPLETE → re-center and clear latch =====
        if state == STATE_WAIT_TURN:
            # keep rolling during the turn (no brake)
            if (time.time() - last_tx) > 0.01:
                send_center(TUNE["CENTER_DEG"], TUNE["TURN_SPEED"])
                last_tx = time.time()

            if TUNE["DISABLE_DETECT_DURING_TURN"]:
                detect_enabled = False

            telemetry_fresh = (time.time() - last_tlm_t) < 0.5
            reached_angle = (last_turn_target is not None and telemetry_fresh and
                             abs(shortest_err(last_turn_target, last_yaw)) <= TUNE["TURN_TRIGGER_TOL_DEG"])
            timed_out = (time.time() - last_turn_t) >= (TUNE["TURN_TIMEOUT_MS"] / 1000.0)

            if reached_angle or timed_out:
                if not TUNE.get("FIX_REF_TO_START", 0):
                    ref_step = float(TUNE.get("REF_STEP_AFTER_TURN_DEG", TUNE["TURN_DEG"]))
                    dir_sign = -1.0 if (CURRENT_TURN_DIR == "LEFT") else +1.0
                    yaw_ref = wrap180(yaw_ref + dir_sign * ref_step)
                else:
                    yaw_ref = float(yaw_start)

                print(f"[TURN] complete ({'timeout' if timed_out else 'angle'}) -> new REF {yaw_ref:.1f}")

                last_turn_target = None
                turn_latched_dir = None
                turn_requested = False
                turn_count += 1
                REARM_AFTER_TURN_T = time.time() + 1.0
                detect_enabled = False

                # Immediately keep moving
                immediate_spd = int(TUNE["POST_TURN_SPEED"] if TUNE["POST_TURN_SEC"] > 0 else TUNE["DRIVE_SPEED"])
                send_center(TUNE["CENTER_DEG"], immediate_spd)
                last_tx = time.time()

                if turn_count >= TUNE["MAX_TURNS"]:
                    state = STATE_FINAL_BURST
                    final_burst_start = time.time()
                    print("[FINAL] Burst forward for %.1fs..." % TUNE["FINAL_BURST_S"])
                else:
                    state = STATE_CENTER
                    rearm_started_t = time.time()
                    POST_BOOST_UNTIL = time.time() + float(TUNE["POST_TURN_SEC"])
                    see_color_slow_active = False
                    strict_latch_active = False
                    heading_hold.reset()

        # ---------------- Straight comparison ----------------
        steer_deg = float(CENTER_DEG)
        if continuous_heading_active():
            if STRAIGHT_YAW_ENABLED:
                yaw_target = heading_hold.update(
                    last_yaw, yaw_ref, last_yaw_t, state,
                    TUNE["STEER_KP_DEG_PER_DEG"], TUNE["STEER_MAX_DEG"])
                offset = yaw_target - CENTER_DEG
                target = CENTER_DEG + SERVO_DIR * offset
                steer_deg = float(max(LEFT_LIMIT, min(RIGHT_LIMIT, target)))
            else:
                heading_hold.reset()

            # Preserve the upload's proximity speed signals, with no wall steering.
            dL = sane_cm(last_US.get("dL", -1)); dR = sane_cm(last_US.get("dR", -1))
            left_is_contact = (dL >= 0 and dL < TUNE["CONTACT_ON_CM"]) or (TUNE["CONTACT_TREAT_INVALID_AS_ON"] and dL < 0)
            right_is_contact = (dR >= 0 and dR < TUNE["CONTACT_ON_CM"]) or (TUNE["CONTACT_TREAT_INVALID_AS_ON"] and dR < 0)
            if (left_is_contact or right_is_contact):
                if left_is_contact and right_is_contact:
                    contact_dir = 'LEFT' if (dL if dL>=0 else 0) <= (dR if dR>=0 else 0) else 'RIGHT'
                else:
                    contact_dir = 'LEFT' if left_is_contact else 'RIGHT'
                contact_active = True
                contact_until = now + float(TUNE["CONTACT_HOLD_S"])
            elif contact_active and now < contact_until:
                pass
            else:
                contact_active = False

            side = None; near = None
            if dL >= 0 and (dR < 0 or dL <= dR): side, near = 'L', dL
            elif dR >= 0: side, near = 'R', dR
            if wall_avoid_state == WA_IDLE:
                if side is not None and near <= float(TUNE["WALL_AVOID_ON_CM"]):
                    wall_avoid_state = WA_ACTIVE
                    wall_avoid_start_t = now
                    wall_avoid_side = side
                    print(f"[WALL_AVOID] START side={side} at {near:.1f}cm")
            else:
                timeout = (now - wall_avoid_start_t) > float(TUNE["WALL_AVOID_TIMEOUT_S"])
                recovered = (side is None) or (side != wall_avoid_side) or (near >= float(TUNE["WALL_AVOID_OFF_CM"]))
                if timeout or recovered:
                    wall_avoid_state = WA_IDLE
                    wall_avoid_side = None
                    print("[WALL_AVOID] STOP")
        else:
            heading_hold.reset()

        # --- FINAL_BURST → FINAL_ALIGN or FINAL_FWD ---
        if state == STATE_FINAL_BURST:
            if (time.time() - final_burst_start) >= TUNE["FINAL_BURST_S"]:
                if TUNE["FINAL_ALIGN"]:
                    state = STATE_FINAL_ALIGN
                    final_align_start_t = time.time()
                    final_align_q.clear()
                    print("[FINAL] Burst finished -> FINAL_ALIGN (center ROI, stop on BLUE/ORANGE only)")
                else:
                    state = STATE_FINAL_FWD
                    final_eval_not_before = time.time() + float(TUNE["FINAL_US_DELAY_S"])
                    print(f"[FINAL] Burst finished -> delay {TUNE['FINAL_US_DELAY_S']:.2f}s before reading front US")

        # ---------- FINAL ALIGN ----------
        if state == STATE_FINAL_ALIGN:
            if (time.time() - last_tx) > TUNE["CONTROL_PERIOD_S"]:
                send_center(steer_deg, TUNE["FINAL_ALIGN_SPEED"])
                last_tx = time.time()

            detect_delay = float(TUNE["FINAL_ALIGN_DETECT_DELAY_S"])
            detect_phase = (time.time() - final_align_start_t) >= detect_delay

            if detect_phase:
                fa_bgr = frame[cy0:cy1, cx0:cx1]
                k = max(3, TUNE["BLUR_KSIZE"] | 1)
                fa_bgr = cv.GaussianBlur(fa_bgr, (k,k), 0)
                fa_hsv = cv.cvtColor(fa_bgr, cv.COLOR_BGR2HSV)
                if TUNE["SAT_BOOST"] != 1.0:
                    hch, sch, vch = cv.split(fa_hsv)
                    sch = np.clip(sch.astype(np.float32) * float(TUNE["SAT_BOOST"]), 0, 255).astype(np.uint8)
                    fa_hsv = cv.merge([hch, sch, vch])

                ck_erode  = int(cfg.get("erode",  TUNE["COLOR_ERODE_IT"]))
                ck_dilate = int(cfg.get("dilate", TUNE["COLOR_DILATE_IT"]))

                m_bl = mask_hsv_multi(fa_hsv, cfg.get("BLUE"),   ek=ck_erode, dk=ck_dilate)
                m_or = mask_hsv_multi(fa_hsv, cfg.get("ORANGE"), ek=ck_erode, dk=ck_dilate)

                roi_area = max(1, (cy1 - cy0) * (cx1 - cx0))
                dyn_min_pix = max(int(roi_area * float(TUNE["COLOR_MIN_PIX_FRAC"])),
                                  int(TUNE["MIN_PIX_COLOR"]))

                pix_bl = int(np.count_nonzero(m_bl))
                pix_or = int(np.count_nonzero(m_or))
                pass_bl = (pix_bl >= dyn_min_pix)
                pass_or = (pix_or >= dyn_min_pix)

                cand_final = None
                if pass_bl or pass_or:
                    if TUNE["COLOR_PICK_MODE"] == "maxpix":
                        if pass_bl and pass_or:
                            cand_final = "BLUE" if pix_bl >= pix_or else "ORANGE"
                        elif pass_bl:
                            cand_final = "BLUE"
                        else:
                            cand_final = "ORANGE"
                    else:
                        cand_final = "BLUE" if pass_bl else "ORANGE"

                if cand_final is not None:
                    final_align_q.append(cand_final)
                else:
                    final_align_q.clear()

                timeout = (time.time() - final_align_start_t) >= float(TUNE["FINAL_ALIGN_TIMEOUT_S"])
                color_confirmed = (len(final_align_q) == final_align_q.maxlen
                                   and len(final_align_q) > 0
                                   and all(c == final_align_q[0] for c in final_align_q))
                if color_confirmed:
                    safe_write(b"STOP\n")
                    print(f"[FINAL_ALIGN] STOP on color {final_align_q[0]}")
                    state = STATE_DONE
                elif timeout:
                    safe_write(b"STOP\n")
                    print(f"[FINAL_ALIGN] TIMEOUT {TUNE['FINAL_ALIGN_TIMEOUT_S']:.1f}s — stopping anyway")
                    state = STATE_DONE

        # ---- FINAL_FWD legacy path ----
        if state == STATE_FINAL_FWD:
            if (time.time() - last_tx) > TUNE["CONTROL_PERIOD_S"]:
                send_center(steer_deg, TUNE["DRIVE_SPEED"]); last_tx = time.time()
            if time.time() >= final_eval_not_before:
                dF_now = sane_cm(last_US.get("dF", -1))
                if dF_now >= 0 and dF_now <= TUNE["STOP_AT_END_CM"]:
                    safe_write(b"STOP\n")
                    print(f"[FINAL] Reached end: front={dF_now:.0f}cm (thr={TUNE['STOP_AT_END_CM']:.0f}cm)")
                    state = STATE_DONE

        # ---- FINAL BACKUP (optional) ----
        if state == STATE_FINAL_BACK:
            dF_now = sane_cm(last_US.get("dF", -1))
            if (time.time() - last_tx) > 0.05:
                if TUNE["REVERSE_SUPPORTED"]: send_back(TUNE["REVERSE_SPEED"])
                last_tx = time.time()
            timed_out = (time.time() - final_back_start) >= TUNE["REVERSE_MAX_S"]
            if (dF_now >= 0 and dF_now > TUNE["STOP_AT_END_CM"]) or timed_out:
                safe_write(b"STOP\n"); print("[FINAL] Backup stop (gap found or timeout)."); state = STATE_DONE

        # ---- Color RE-ARM ----
        if (not detect_enabled) and rearm_started_t > 0 and state == STATE_CENTER:
            if (time.time() - rearm_started_t) >= TUNE["COLOR_REARM_DELAY_S"]:
                dF_now = sane_cm(last_US.get("dF", -1))
                if dF_now >= 0 and dF_now > TUNE["COLOR_REARM_FRONT_CM"]:
                    detect_enabled = True; rearm_started_t = -1e9
                    see_color_consumed = False
                    print("Color detection RE-ARMED (front clear).")

        # ---- Choose speed in CENTER (20 Hz, identical in both modes) ----
        if state == STATE_CENTER:
            if (time.time() - last_tx) > TUNE["CONTROL_PERIOD_S"]:
                now2 = time.time()
                if now2 < POST_BOOST_UNTIL: base_spd = TUNE["POST_TURN_SPEED"]
                elif see_color_slow_active: base_spd = TUNE["SEE_COLOR_SPEED"]
                else: base_spd = TUNE["DRIVE_SPEED"]

                dF = sane_cm(last_US.get("dF", -1))
                dL = sane_cm(last_US.get("dL", -1)); dR = sane_cm(last_US.get("dR", -1))
                min_side = min(x for x in (dL, dR) if x >= 0) if (dL >= 0 or dR >= 0) else -1

                if min_side >= 0:
                    if side_slow_active:
                        if min_side > TUNE["WALL_SLOW_SIDE_OFF_CM"]: side_slow_active = False
                    else:
                        if min_side < TUNE["WALL_SLOW_SIDE_ON_CM"]: side_slow_active = True

                spd = base_spd
                if side_slow_active:
                    spd = max(int(spd * TUNE["SIDE_SLOWDOWN_FACTOR"]), int(TUNE["WALL_SLOW_MIN_SPEED"]))
                if dF >= 0 and dF < TUNE["WALL_SLOW_FRONT_ON_CM"]:
                    spd = max(int(spd * TUNE["FRONT_SLOWDOWN_FACTOR"]), int(TUNE["WALL_SLOW_MIN_SPEED"]))
                if contact_active:
                    spd = max(int(spd * TUNE["CONTACT_SPEED_FACTOR"]), int(TUNE["WALL_SLOW_MIN_SPEED"]))

                if wall_avoid_state == WA_ACTIVE:
                    spd = min(spd, int(TUNE["WALL_AVOID_SPEED"]))
                    spd = max(spd, int(TUNE["WALL_SLOW_MIN_SPEED"]))

                send_center(steer_deg, spd); last_tx = now2

        # ---- HUD ----
        vis = frame.copy()
        # Bottom color ROI (orange box)
        cv.rectangle(vis, (mrx0, mry0), (mrx1, mry1), (0,165,255), 2)
        cv.putText(vis, "PRE-TURN COLOR ROI", (mrx0, max(0, mry0-6)),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,165,255), 1)
        # Final-align ROI (green box)
        cv.rectangle(vis, (cx0, cy0), (cx1, cy1), (0,255,0), 2)
        cv.putText(vis, "FINAL ALIGN ROI", (cx0, max(0, cy0-6)),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        st_name = {
            0:"WAIT_START",1:"CENTER",2:"WAIT_TURN",
            3:"FINAL_BURST",4:"FINAL_FWD",5:"FINAL_ALIGN",
            6:"FINAL_BACK",7:"DONE"
        }.get(state, "?")

        cv.putText(vis, f"STATE:{st_name}  YAW:{last_yaw:5.1f}  REF:{yaw_ref:5.1f}", (10,24),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        dF = last_US.get("dF", -1); dL = last_US.get("dL", -1); dR = last_US.get("dR", -1)

        pause_tag = "PAUSED" if (time.time() < color_pause_until) else "ON"
        run_secs = 0.0 if run_start_t is None else (time.time() - run_start_t)
        color_gate = "REQ_COLOR" if TUNE["REQUIRE_COLOR_FOR_TURN"] else "COLOR_OPT"
        latch_tag = f"LAT:{turn_latched_dir or '-'}"
        cv.putText(vis, f"US F:{dF}  L:{dL}  R:{dR}  COLOR:{pause_tag}  RUN:{run_secs:5.1f}s  {color_gate} {latch_tag}",
                   (10,46), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

        now3 = time.time()
        if now3 < POST_BOOST_UNTIL: curr_base = TUNE["POST_TURN_SPEED"]
        elif see_color_slow_active: curr_base = TUNE["SEE_COLOR_SPEED"]
        else: curr_base = TUNE["DRIVE_SPEED"]
        side_tag = "SIDE!" if side_slow_active else "-"
        front_tag = "FRONT!" if (dF >= 0 and dF < TUNE["WALL_SLOW_FRONT_ON_CM"]) else "-"
        avoid_tag = f"AVOID:{wall_avoid_side or '-'}" if wall_avoid_state == WA_ACTIVE else "AVOID:-"
        alive_tag = "LINK:OK" if link_alive(time.time()) else "LINK:STALE"
        arm_tag = "ARMED" if arduino_armed else "WAIT"
        cv.putText(vis, f"BaseSPD:{curr_base} [{side_tag}|{front_tag}] {avoid_tag} {alive_tag} BTN:{arm_tag}",
                   (10,68), cv.FONT_HERSHEY_SIMPLEX, 0.6, (200,220,255), 2)

        cv.imshow("robot", vis)

        # ---- HOTKEYS ----
        key = cv.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q')):
            safe_write(b"STOP\n"); break
        elif key in (ord('s'), ord('S')):
            if not yaw_ready():
                print("[START] Waiting for fresh yaw telemetry.")
            else:
                if yaw_start is None: yaw_start = float(last_yaw)
                yaw_ref = float(yaw_start)
                safe_write(b"START\n")
                send_turn_timeout_ms(TUNE["TURN_TIMEOUT_MS"])
                state = STATE_CENTER
                arduino_armed = True
                run_start_t = time.time()
                heading_hold.reset()
                turn_requested = False; turn_latched_dir = None
                print("START sent. Running…")
        elif key in (ord('x'), ord('X')):
            try: safe_write(b"STOP\n")
            except Exception: pass
            print("Resetting Arduino (manual)…")
            reset_arduino(TUNE["ARD_RESET_METHOD"])
            # Do NOT auto-restart Python here; Arduino reboot will print READY and trigger self-restart anyway.

    # ---- Cleanup ----
    try:
        if ser: ser.close()
    except Exception:
        pass
    _release_cap()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: exiting.")
        sys.exit(0)

