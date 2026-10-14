#!/usr/bin/env python3
"""IRD MAX robot controller for the WRO Future Engineers obstacle challenge."""


import cv2 as cv
import numpy as np
import yaml, glob, time, os, re, math, sys
from collections import deque

try:
    import serial
except Exception:
    serial = None


# TUNING PANEL

TUNE = {
    # Corridor turns (BLUE/ORANGE lines)
    "TURN_DEG": float(os.environ.get("TURN_DEG", "90.0")),
    "TURN_TOL_DEG": float(os.environ.get("TURN_TURN_TOL_DEG", os.environ.get("TURN_TOL_DEG", "45.0"))),

    # Pillar sidestep turns (RED/GREEN pillars)
    "PILLAR_TURN_DEG": float(os.environ.get("PILLAR_TURN_DEG", "90.0")),
    "PILLAR_TURN_TOL_DEG": float(os.environ.get("PILLAR_TURN_TOL_DEG", "30.0")),

    # Flip (optional)
    "FLIP_ENABLED": int(os.environ.get("FLIP_ENABLED", "0")),
    "FLIP_TURN_DEG": float(os.environ.get("FLIP_TURN_DEG", "180.0")),
    "FLIP_FWD_HOLD_S": float(os.environ.get("FLIP_FWD_HOLD_S", "1.50")),
    "FLIP_LEFT_SIGN": float(os.environ.get("FLIP_LEFT_SIGN", "1.0")),

    # Pre-run alignment

    "ESCAPE_ENABLED": int(os.environ.get("ESCAPE_ENABLED", "1")),
    "ESC_S1_TARGET_DEG": float(os.environ.get("ESC_S1_TARGET_DEG", "0.0")),
    "ESC_S2_TARGET_DEG": float(os.environ.get("ESC_S2_TARGET_DEG", "0.0")),
    "ESC_S3_TARGET_DEG": float(os.environ.get("ESC_S3_TARGET_DEG", "0.0")),
    "ESC_TURN_TOL_DEG":  float(os.environ.get("ESC_TURN_TOL_DEG", "50.0")),
    "ESC_TURN_MAX_S":    float(os.environ.get("ESC_TURN_MAX_S", "7.17015")),
    "ESC_TURN_SPEED_FWD": int(os.environ.get("ESC_TURN_SPEED_FWD", "34")),
    "ESC_TURN_SPEED_BACK": int(os.environ.get("ESC_TURN_SPEED_BACK", "34")),
    "ESC_S3_HOLD_S": float(os.environ.get("ESC_S3_HOLD_S", ".50")),
    "ESC_S3_HOLD_SPEED": int(os.environ.get("ESC_S3_HOLD_SPEED", "45")),
    "ESC_SIDE_INVERT": int(os.environ.get("ESC_SIDE_INVERT", "0")),
    "ESC_DIR_SIGN": float(os.environ.get("ESC_DIR_SIGN", "1.0")),
    "ESC_RTN_BIAS_DEG": float(os.environ.get("ESC_RTN_BIAS_DEG", "20.0")),
    "ESC_RTN_TOL_DEG":  float(os.environ.get("ESC_RTN_TOL_DEG", "20.0")),
    "ESC_USE_ENCODER": int(os.environ.get("ESC_USE_ENCODER", "1")),
    "ESC_S1_CM": float(os.environ.get("ESC_S1_CM", "9.0")),
    "ESC_S2_CM": float(os.environ.get("ESC_S2_CM", "10.0")),
    "ESC_S3_CM": float(os.environ.get("ESC_S3_CM", "30.0")),
    "ESC_S4_CM": float(os.environ.get("ESC_S4_CM", "30.0")),
    "ESC_S1_PWM": int(os.environ.get("ESC_S1_PWM", "40")),
    "ESC_S2_PWM": int(os.environ.get("ESC_S2_PWM", "42")),
    "ESC_S3_PWM": int(os.environ.get("ESC_S3_PWM", "42")),
    "ESC_S4_PWM": int(os.environ.get("ESC_S4_PWM", "42")),

    # Direction lock across all 12 corners
    "LOCK_ALL_12_TURNS_ONE_DIRECTION": int(os.environ.get("LOCK_ALL_12_TURNS_ONE_DIRECTION", "1")),
    "ALL_TURNS_DIR": os.environ.get("ALL_TURNS_DIR", "").strip().upper(),

    # Turn watchdogs
    "CORNER_TURN_MAX_S": float(os.environ.get("CORNER_TURN_MAX_S", "1.5")),
    "PILLAR_TURN_MAX_S": float(os.environ.get("PILLAR_TURN_MAX_S", "1.0")),

    # Run cap
    "MAX_TURNS": int(os.environ.get("MAX_TURNS", "4")),

    # Final forward-centering hold after last turn
    "FINAL_HOLD_S": float(os.environ.get("FINAL_HOLD_S", "0.0")),

    # Servo geometry (deg)
    "CENTER_DEG": int(os.environ.get("CENTER_DEG", "90")),
    "LEFT_LIMIT": int(os.environ.get("LEFT_LIMIT", "60")),
    "RIGHT_LIMIT": int(os.environ.get("RIGHT_LIMIT", "115")),
    "CENTER_STEER_MIN": 60,
    "CENTER_STEER_MAX": 115,

    # General speeds (PWM 0..255)
    "DRIVE_SPEED": int(os.environ.get("DRIVE_SPEED", "55")),
    "SEE_COLOR_SPEED": int(os.environ.get("SEE_COLOR_SP5EED", "55")),
    "TURN_SPEED": int(os.environ.get("TURN_SPEED", "45")),

    # Pillar pass speeds
    "PILLAR_TURN_SPEED": int(os.environ.get("PILLAR_TURN_SPEED", "45")),
    "PILLAR_FWD_SPEED": int(os.environ.get("PILLAR_FWD_SPEED", "45")),
    "PILLAR_BACK_SPEED": int(os.environ.get("PILLAR_BACK_SPEED", "55")),  # legacy

    # Back-center stop speed
    "BACK_STOP_SPEED": int(os.environ.get("BACK_STOP_SPEED", "50")),

    "PILLAR_BACK_USE_ENCODER": int(os.environ.get("PILLAR_BACK_USE_ENCODER", "1")),
    "PILLAR_BACK_ENC_CM": float(os.environ.get("PILLAR_BACK_ENC_CM", "16.0")),
    "PILLAR_BACK_ENC_SPEED": int(os.environ.get("PILLAR_BACK_ENC_SPEED", "45")),

    # Pillar forward distance (encoder)
    "PILLAR_FWD_USE_ENCODER": int(os.environ.get("PILLAR_FWD_USE_ENCODER", "1")),
    "PILLAR_FWD_BASE_CM": float(os.environ.get("PILLAR_FWD_BASE_CM", "0.0")),
    "PILLAR_FWD_GAIN_CM": float(os.environ.get("PILLAR_FWD_GAIN_CM", "15.0")),
    "PILLAR_FWD_MIN_CM": float(os.environ.get("PILLAR_FWD_MIN_CM", "0.0")),
    "PILLAR_FWD_MAX_CM": float(os.environ.get("PILLAR_FWD_MAX_CM", "22.0")),

    # Corner fixed servo angles (hardware mapping)
    "BACK_LEFT_SERVO_DEG": int(os.environ.get("BACK_LEFT_SERVO_DEG", "65")),
    "BACK_RIGHT_SERVO_DEG": int(os.environ.get("BACK_RIGHT_SERVO_DEG", "120")),
    "FWD_LEFT_SERVO_DEG": int(os.environ.get("FWD_LEFT_SERVO_DEG", "115")),
    "FWD_RIGHT_SERVO_DEG": int(os.environ.get("FWD_RIGHT_SERVO_DEG", "60")),

    # Distances (cm)
    "FRONT_TURN_CM_1": float(os.environ.get("FRONT_TURN_CM_1", "85.0")),
    "FRONT_TURN_CM_2": float(os.environ.get("FRONT_TURN_CM_2", "45.0")),
    "FRONT_TURN_CM_3": float(os.environ.get("FRONT_TURN_CM_3", "45.0")),
    "FRONT_TURN_CM_4": float(os.environ.get("FRONT_TURN_CM_4", "45.0")),
    "FRONT_TURN_MAX_AGE_S": float(os.environ.get("FRONT_TURN_MAX_AGE_S", "0.50")),
    "FRONT_TURN_CONFIRM_SAMPLES": int(os.environ.get("FRONT_TURN_CONFIRM_SAMPLES", "3")),
    "BACK_STOP_CM": float(os.environ.get("BACK_STOP_CM", "55.0")),
    "PILLAR_ACCEPT_CM": float(os.environ.get("PILLAR_ACCEPT_CM", "55.0")),
    "PILLAR_APPROACH_FRONT_CM": float(os.environ.get("PILLAR_APPROACH_FRONT_CM", "12.0")),  # legacy
    "PILLAR_BACK_TO_FRONT_CM": float(os.environ.get("PILLAR_BACK_TO_FRONT_CM", "14.0")),    # legacy

    # Per-corner turn types
    "CORNER_TURN_TYPE_1": os.environ.get("CORNER_TURN_TYPE_1", "FORWARD"),
    "CORNER_TURN_TYPE_2": os.environ.get("CORNER_TURN_TYPE_2", "FORWARD"),
    "CORNER_TURN_TYPE_3": os.environ.get("CORNER_TURN_TYPE_3", "FORWARD"),
    "CORNER_TURN_TYPE_4": os.environ.get("CORNER_TURN_TYPE_4", "FORWARD"),

    # Color detection / ROI
    "COLOR_CONFIRM_FRAMES": int(os.environ.get("COLOR_CONFIRM_FRAMES", "1")),
    "MIN_PIX_COLOR": int(os.environ.get("MIN_PIX_COLOR", "1")),
    "BLUR_KSIZE": int(os.environ.get("BLUR_KSIZE", "1")),
    "COLOR_ROI_WIDTH_FRAC": float(os.environ.get("COLOR_ROI_WIDTH_FRAC", "0.85")),
    "COLOR_ROI_HEIGHT_FRAC": float(os.environ.get("COLOR_ROI_HEIGHT_FRAC", "0.750")),
    "ROI_SCALE_AFTER_FIRST_TURN": float(os.environ.get("ROI_SCALE_AFTER_FIRST_TURN", "1.3")),
    "SAT_BOOST": float(os.environ.get("SAT_BOOST", "1.10")),
    "COLOR_SUSPEND_SEC": float(os.environ.get("COLOR_SUSPEND_SEC", "2.0")),
    # Re-arm only after the old corner stripe has actually left the camera.

    "CORNER_REARM_MIN_S": float(os.environ.get("CORNER_REARM_MIN_S", "0.50")),
    "CORNER_REARM_CLEAR_FRAMES": int(os.environ.get("CORNER_REARM_CLEAR_FRAMES", "1")),
    # If a locked-direction run reaches the wall without a line latch, stop
    # and use the already-proven course direction instead of driving forward.
    "MISSED_CORNER_FRONT_CM": float(os.environ.get("MISSED_CORNER_FRONT_CM", "18.0")),
    "MISSED_CORNER_CONFIRM_FRAMES": int(os.environ.get("MISSED_CORNER_CONFIRM_FRAMES", "2")),
    # A corner marker must be a long floor stripe crossing the camera centre.
    # These gates reject the blue centre artwork, barriers and line extensions
    # outside the car's forward driving path.
    "LINE_MIN_WIDTH_FRAC": float(os.environ.get("LINE_MIN_WIDTH_FRAC", "0.12")),
    "LINE_MIN_ASPECT": float(os.environ.get("LINE_MIN_ASPECT", "1.00")),
    "LINE_MAX_HEIGHT_FRAC": float(os.environ.get("LINE_MAX_HEIGHT_FRAC", "1.0")),
    "LINE_MAX_AREA_FRAC": float(os.environ.get("LINE_MAX_AREA_FRAC", "1.0")),
    "LINE_MIN_CENTER_Y_FRAC": float(os.environ.get("LINE_MIN_CENTER_Y_FRAC", "0.20")),
    "LINE_CENTER_SPAN_FRAC": float(os.environ.get("LINE_CENTER_SPAN_FRAC", "0.08")),

    # Pillar detection (RED/GREEN)
    "PILLAR_CONFIRM_FRAMES": int(os.environ.get("PILLAR_CONFIRM_FRAMES", "2")),
    "PILLAR_MIN_PIX": int(os.environ.get("PILLAR_MIN_PIX", "500")),
    "PILLAR_ROI_WIDTH_FRAC": float(os.environ.get("PILLAR_ROI_WIDTH_FRAC", "0.99")),
    "PILLAR_ROI_HEIGHT_FRAC": float(os.environ.get("PILLAR_ROI_HEIGHT_FRAC", "0.75")),
    "PILLAR_DIST_K": float(os.environ.get("PILLAR_DIST_K", "6500.0")),
    "PILLAR_MIN_AREA": int(os.environ.get("PILLAR_MIN_AREA", "2000")),
    "PILLAR_DILATE_ITER": int(os.environ.get("PILLAR_DILATE_ITER", "3")),
    "PILLAR_ERODE_ITER": int(os.environ.get("PILLAR_ERODE_ITER", "1")),
    "PILLAR_MIN_ASPECT": float(os.environ.get("PILLAR_MIN_ASPECT", "1.15")),
    "PILLAR_MIN_SOLIDITY": float(os.environ.get("PILLAR_MIN_SOLIDITY", "0.35")),
    "PILLAR_CLOSE_GAP_PX": int(os.environ.get("PILLAR_CLOSE_GAP_PX", "8")),

    # Pillar gates (side validation)
    "RED_LINE_X_FRAC": float(os.environ.get("RED_LINE_X_FRAC", "0.3")),
    "GREEN_LINE_X_FRAC": float(os.environ.get("GREEN_LINE_X_FRAC", "0.7")),

    # Yaw centering controller
    "STEER_SIGN": float(os.environ.get("STEER_SIGN", "1.0")),
    "STEER_KP_DEG_PER_DEG": float(os.environ.get("STEER_KP_DEG_PER_DEG", ".32")),
    "STEER_MAX_DEG": float(os.environ.get("STEER_MAX_DEG", "5.0")),
    "STEER_DEAD_ERR_DEG": float(os.environ.get("STEER_DEAD_ERR_DEG", "1.5")),
    "ERR_FILTER_ALPHA": float(os.environ.get("ERR_FILTER_ALPHA", "0.30")),
    "STEER_KD_DEG_PER_DPS": float(os.environ.get("STEER_KD_DEG_PER_DPS", "0.10")),
    "YAW_RATE_FILTER_ALPHA": float(os.environ.get("YAW_RATE_FILTER_ALPHA", "0.25")),
    "STEER_SLEW_DEG_PER_SEC": float(os.environ.get("STEER_SLEW_DEG_PER_SEC", "15.0")),
    # 20 Hz avoids filling the Uno RX buffer while it measures ultrasound.
    "CONTROL_PERIOD_S": 0.05,
    # About 3 m at room temperature; a double front ping waits at most 36 ms.
    "CONTROL_US_TIMEOUT_US": 18000,

    # Post-flip centering softener
    "POST_FLIP_KP_GAIN": float(os.environ.get("POST_FLIP_KP_GAIN", "4.825")),
    "POST_FLIP_MAX_DEG": float(os.environ.get("POST_FLIP_MAX_DEG", "4.0")),
    "POST_FLIP_KP_HOLD_S": float(os.environ.get("POST_FLIP_KP_HOLD_S", ".0000005")),

    # Wall slowdowns
    "WALL_SLOW_SIDE_ON_CM": float(os.environ.get("WALL_SLOW_SIDE_ON_CM", "15.5")),
    "WALL_SLOW_SIDE_OFF_CM": float(os.environ.get("WALL_SLOW_SIDE_OFF_CM", "16.5")),
    "SIDE_SLOWDOWN_FACTOR": float(os.environ.get("SIDE_SLOWDOWN_FACTOR", "0.55")),
    "WALL_SLOW_FRONT_ON_CM": float(os.environ.get("WALL_SLOW_FRONT_ON_CM", "55.0")),
    "FRONT_SLOWDOWN_FACTOR": float(os.environ.get("FRONT_SLOWDOWN_FACTOR", "0.55")),
    "WALL_SLOW_MIN_SPEED": int(os.environ.get("WALL_SLOW_MIN_SPEED", "55")),

    # Wall avoid
    "WALL_AVOID_ON_CM": float(os.environ.get("WALL_AVOID_ON_CM", "15.5")),
    "WALL_AVOID_OFF_CM": float(os.environ.get("WALL_AVOID_OFF_CM", "16.5")),
    "WALL_AVOID_HOLD_S": float(os.environ.get("WALL_AVOID_HOLD_S", "0.35")),
    "WALL_AVOID_KP_DEG_PER_CM": float(os.environ.get("WALL_AVOID_KP_DEG_PER_CM", "3.8")),
    "WALL_AVOID_EXP": float(os.environ.get("WALL_AVOID_EXP", "1.0")),
    "WALL_AVOID_MIN_DEG": float(os.environ.get("WALL_AVOID_MIN_DEG", "2.0")),
    "WALL_AVOID_MAX_BIAS_DEG": float(os.environ.get("WALL_AVOID_MAX_BIAS_DEG", "5.0")),
    "WALL_AVOID_GAIN": float(os.environ.get("WALL_AVOID_GAIN", "1.0")),
    "AVOID_INVERT_DIR": int(os.environ.get("AVOID_INVERT_DIR", "-1")),
    "WALL_AVOID_SPEED": int(os.environ.get("WALL_AVOID_SPEED", "50")),

    # First-Step Pillar Avoid
    "PREAVOID_ENABLED": int(os.environ.get("PREAVOID_ENABLED", "0")),
    "PREAVOID_RED_DIR": os.environ.get("PREAVOID_RED_DIR", "RIGHT").strip().upper(),
    "PREAVOID_GREEN_DIR": os.environ.get("PREAVOID_GREEN_DIR", "LEFT").strip().upper(),
    "PREAVOID_ON_CM": float(os.environ.get("PREAVOID_ON_CM", "65.0")),
    "PREAVOID_SPEED": int(os.environ.get("PREAVOID_SPEED", "43")),
    "PREAVOID_MIN_BIAS_DEG": float(os.environ.get("PREAVOID_MIN_BIAS_DEG", "2.0")),
    "PREAVOID_MAX_BIAS_DEG": float(os.environ.get("PREAVOID_MAX_BIAS_DEG", "20.0")),
    "PREAVOID_SAFE_FRAC": float(os.environ.get("PREAVOID_SAFE_FRAC", "0.02")),
    "PREAVOID_LOSS_GRACE_S": float(os.environ.get("PREAVOID_LOSS_GRACE_S", ".0205")),
    "PREAVOID_HUD_TIMER": int(os.environ.get("PREAVOID_HUD_TIMER", "1")),
    "PREAVOID_LOST_CONFIRM": int(os.environ.get("PREAVOID_LOST_CONFIRM", "6")),
    "PREAVOID_SUPPRESS_AFTER_S": float(os.environ.get("PREAVOID_SUPPRESS_AFTER_S", "0.30")),
    "PREAVOID_USE_ENCODER": int(os.environ.get("PREAVOID_USE_ENCODER", "1")),
    "PREAVOID_MAX_TRAVEL_CM": float(os.environ.get("PREAVOID_MAX_TRAVEL_CM", "18.0")),
    "PREAVOID_RAMP_CM": float(os.environ.get("PREAVOID_RAMP_CM", "4.0")),
    "PREAVOID_MIN_TRAVEL_CM_EXIT": float(os.environ.get("PREAVOID_MIN_TRAVEL_CM_EXIT", "5.0")),

    # Camera / serial
    "ALLOW_NO_CAMERA": int(os.environ.get("ALLOW_NO_CAMERA", "0")),
    "FRAME_W": int(os.environ.get("FRAME_W", "640")),
    "FRAME_H": int(os.environ.get("FRAME_H", "480")),
    "CAMERA_FAIL_STOP_FRAMES": int(os.environ.get("CAMERA_FAIL_STOP_FRAMES", "10")),
    "ROBOT_PORT": os.environ.get("ROBOT_PORT", "/dev/ttyACM0"),
    "ROBOT_BAUD": int(os.environ.get("ROBOT_BAUD", "115200")),

    # Link watchdogs
    "STALE_TLM_SEC": float(os.environ.get("STALE_TLM_SEC", "1.2")),
    "STALE_LINK_SEC": float(os.environ.get("STALE_LINK_SEC", "2.5")),
    "PING_PERIOD_SEC": float(os.environ.get("PING_PERIOD_SEC", "0.7")),
    "RESET_BACKOFF_SEC": float(os.environ.get("RESET_BACKOFF_SEC", "80.0")),

    # Jetson <-> Arduino reset sync
    "JETSON_RESET_ON_ARDUINO_READY": int(os.environ.get("JETSON_RESET_ON_ARDUINO_READY", "1")),
    "JETSON_RESET_GRACE_S": float(os.environ.get("JETSON_RESET_GRACE_S", "1.0")),

    # Misc
    "US_MAX_CM": float(os.environ.get("US_MAX_CM", "300.0")),

    # Post-12 extra maneuver (optional)
    "POST12_ENABLED": int(os.environ.get("POST12_ENABLED", "0")),
    "POST12_DIR": os.environ.get("POST12_DIR", "RIGHT").strip().upper(),
    "POST12_TURN_DEG": float(os.environ.get("POST12_TURN_DEG", "90.0")),
    "POST12_TURN_TOL_DEG": float(os.environ.get("POST12_TURN_TOL_DEG", "20.0")),
    "POST12_TURN_MAX_S": float(os.environ.get("POST12_TURN_MAX_S", "2.5")),
    "POST12_TURN_SPEED": int(os.environ.get("POST12_TURN_SPEED", "31")),
    "POST12_FWD_SPEED": int(os.environ.get("POST12_FWD_SPEED", "28")),
    "POST12_FRONT_STOP_CM": float(os.environ.get("POST12_FRONT_STOP_CM", "8.0")),

    # Stuck / wall escape
    "STUCK_FRONT_CM": -20.0,
    "STUCK_YAW_ERR_DEG": 0.00,
    "STUCK_MIN_S": .0,
    "WESC_BACK_S": 0.0,
    "WESC_BACK_SPEED": 0,
    "WESC_SPIN_DEG": 22.0,
    "WESC_SPIN_SPEED": 0,
    "WESC_MAX_SPIN_S": float(os.environ.get("WESC_MAX_SPIN_S", "2.5")),
    "WESC_EXIT_FRONT_CM": float(os.environ.get("WESC_EXIT_FRONT_CM", "18.0")),
    "WESC_FRONT_MAX_AGE_S": float(os.environ.get("WESC_FRONT_MAX_AGE_S", "0.50")),
    "WESC_EXIT_YAW_DEG": 180.0,
    "WESC_COOLDOWN_S": .8,

    # Encoder-based stuck detection
    "STUCK_ENC_FREEZE_S": float(os.environ.get("STUCK_ENC_FREEZE_S", "1.850")),
    "STUCK_ENC_MIN_DELTA_CM": float(os.environ.get("STUCK_ENC_MIN_DELTA_CM", "0.9")),
    "WESC_BACK_CM": float(os.environ.get("WESC_BACK_CM", "15.0")),
    "WESC_BACK_PWM": int(os.environ.get("WESC_BACK_PWM", "38")),
    "WESC_BACK_MAX_S": float(os.environ.get("WESC_BACK_MAX_S", "2.0")),
    "WESC_TURN_PWM": int(os.environ.get("WESC_TURN_PWM", "41")),
    "WESC_EXIT_TOL_DEG": float(os.environ.get("WESC_EXIT_TOL_DEG", "5.0")),
    "WESC_MAX_BACK_CYCLES": int(os.environ.get("WESC_MAX_BACK_CYCLES", "2")),
    "WESC_REAR_STOP_CM": float(os.environ.get("WESC_REAR_STOP_CM", "12.0")),

    # magnet-wall avoid (corridor)
    "MAG_AVOID_ON_CM": float(os.environ.get("MAG_AVOID_ON_CM", "18.0")),
    "MAG_AVOID_OFF_CM": float(os.environ.get("MAG_AVOID_OFF_CM", "19.0")),
    "MAG_AVOID_SAFE_FRAC": float(os.environ.get("MAG_AVOID_SAFE_FRAC", "0.08")),
    "MAG_AVOID_MIN_AREA": int(os.environ.get("MAG_AVOID_MIN_AREA", "1800")),
    "MAG_AVOID_DILATE_ITER": int(os.environ.get("MAG_AVOID_DILATE_ITER", "2")),
    "MAG_AVOID_KP_DEG_PER_CM": float(os.environ.get("MAG_AVOID_KP_DEG_PER_CM", "7.0")),
    "MAG_AVOID_EXP": float(os.environ.get("MAG_AVOID_EXP", "1.0")),
    "MAG_AVOID_MIN_DEG": float(os.environ.get("MAG_AVOID_MIN_DEG", "1.5")),
    "MAG_AVOID_MAX_BIAS_DEG": float(os.environ.get("MAG_AVOID_MAX_BIAS_DEG", "6.0")),
    "MAG_AVOID_GAIN": float(os.environ.get("MAG_AVOID_GAIN", "1.0")),

    # Last-turn parking sequence
    "LAST_ENABLED": int(os.environ.get("LAST_ENABLED", "1")),
    "LAST_PRETURN_FRONT_CM": float(os.environ.get("LAST_PRETURN_FRONT_CM", "5.0")),
    "LAST_APPROACH_5_PWM": int(os.environ.get(
        "LAST_APPROACH_5_PWM", os.environ.get("DRIVE_SPEED", "45")
    )),
    "LAST_APPROACH_5_MAX_S": float(os.environ.get("LAST_APPROACH_5_MAX_S", "12.0")),
    "LAST_BACK_ENC_CM": float(os.environ.get("LAST_BACK_ENC_CM", "95.0")),
    "LAST_BACK_PWM": int(os.environ.get("LAST_BACK_PWM", "40")),
    "LAST_BACK_MAX_S": float(os.environ.get("LAST_BACK_MAX_S", "8.0")),

    "LAST_CORNER_TURN_DEG": float(os.environ.get("LAST_CORNER_TURN_DEG", "90.0")),
    "LAST_CORNER_TURN_TOL_DEG": float(os.environ.get("LAST_CORNER_TURN_TOL_DEG", "18.0")),
    "LAST_CORNER_TURN_SPEED": int(os.environ.get(
        "LAST_CORNER_TURN_SPEED", os.environ.get("TURN_SPEED", "45")
    )),
    "LAST_CORNER_TURN_MAX_S": float(os.environ.get("LAST_CORNER_TURN_MAX_S", "3.0")),

    # Reverse with straight wheels after the last corridor turn.
    # Arduino must report the REAR ultrasonic as db=... in TLM packets.
    "LAST_WALL_REAR_CM": float(os.environ.get("LAST_WALL_REAR_CM", "5.0")),
    "LAST_WALL_BACK_PWM": int(os.environ.get("LAST_WALL_BACK_PWM", "40")),
    "LAST_WALL_PRESS_S": float(os.environ.get("LAST_WALL_PRESS_S", "0.50")),
    "LAST_WALL_SEEK_MAX_S": float(os.environ.get("LAST_WALL_SEEK_MAX_S", "8.0")),
    "LAST_WALL_SENSOR_MAX_AGE_S": float(os.environ.get("LAST_WALL_SENSOR_MAX_AGE_S", "0.50")),
    "LAST_WALL_SETTLE_S": float(os.environ.get("LAST_WALL_SETTLE_S", "0.20")),
    "LAST_WALL_SETTLE_MAX_S": float(os.environ.get("LAST_WALL_SETTLE_MAX_S", "2.0")),
    "LAST_STRAIGHT_DEADBAND_DEG": float(os.environ.get("LAST_STRAIGHT_DEADBAND_DEG", "2.0")),
    "LAST_STRAIGHT_MAX_STEER_DEG": float(os.environ.get("LAST_STRAIGHT_MAX_STEER_DEG", "3.0")),

    # Forward centering completes by encoder travel, in cm, for each color.

    "LAST_CENTER_YAW_CM": float(os.environ.get("LAST_CENTER_YAW_CM", "215.0")),
    "LAST_CENTER_YAW_ORANGE_CM": float(os.environ.get("LAST_CENTER_YAW_ORANGE_CM", "115.0")),
    # A timeout stops a failed stage; it never advances the parking sequence.
    "LAST_CENTER_YAW_MAX_S": float(os.environ.get("LAST_CENTER_YAW_MAX_S", "8.0")),
    "LAST_CENTER_YAW_PWM": int(os.environ.get(
        "LAST_CENTER_YAW_PWM", os.environ.get("DRIVE_SPEED", "45")
    )),
    "LAST_FRONT_TRIGGER_CM": float(os.environ.get("LAST_FRONT_TRIGGER_CM", "150.0")),
    "LAST_APPROACH_100_PWM": int(os.environ.get(
        "LAST_APPROACH_100_PWM", os.environ.get("DRIVE_SPEED", "45")
    )),
    "LAST_APPROACH_100_MAX_S": float(os.environ.get("LAST_APPROACH_100_MAX_S", "12.0")),

    "LAST_ENCODER_FWD_CM": float(os.environ.get("LAST_ENCODER_FWD_CM", "0.0")),
    "LAST_ENCODER_FWD_PWM": int(os.environ.get("LAST_ENCODER_FWD_PWM", "50")),
    "LAST_ENCODER_FWD_MAX_S": float(os.environ.get("LAST_ENCODER_FWD_MAX_S", "6.0")),

    "LAST_PARK_TURN_DEG": float(os.environ.get("LAST_PARK_TURN_DEG", "85.0")),
    "LAST_PARK_TURN_TOL_DEG": float(os.environ.get("LAST_PARK_TURN_TOL_DEG", "15.0")),
    "LAST_PARK_TURN_SPEED": int(os.environ.get(
        "LAST_PARK_TURN_SPEED", os.environ.get("TURN_SPEED", "45")
    )),
    "LAST_PARK_TURN_MAX_S": float(os.environ.get("LAST_PARK_TURN_MAX_S", "3.0")),

    "LAST_PARK_STOP_CM": float(os.environ.get("LAST_PARK_STOP_CM", "5.0")),
    "LAST_PARK_STOP_ARM_S": float(os.environ.get("LAST_PARK_STOP_ARM_S", "0.5")),
    "LAST_PARK_APPROACH_PWM": int(os.environ.get(
        "LAST_PARK_APPROACH_PWM", os.environ.get("DRIVE_SPEED", "45")
    )),
    "LAST_PARK_APPROACH_MAX_S": float(os.environ.get("LAST_PARK_APPROACH_MAX_S", "12.0")),


}

# CONSTANTS / HELPERS
MIN_SPEED   = 0
CENTER_DEG  = TUNE["CENTER_DEG"]
LEFT_LIMIT  = TUNE["LEFT_LIMIT"]
RIGHT_LIMIT = TUNE["RIGHT_LIMIT"]
SPAN_LEFT   = abs(CENTER_DEG - LEFT_LIMIT)
SPAN_RIGHT  = abs(RIGHT_LIMIT - CENTER_DEG)

def gstreamer_pipeline(sensor_id=0, capture_width=1280, capture_height=720,
                       display_width=640, display_height=480, framerate=30, flip_method=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, "
        f"format=(string)NV12, framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={display_width}, height={display_height}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=1"
    )

def open_camera():
    w, h = TUNE["FRAME_W"], TUNE["FRAME_H"]
    cap = None
    try:
        cap = cv.VideoCapture(gstreamer_pipeline(display_width=w, display_height=h), cv.CAP_GSTREAMER)
    except Exception:
        cap = None
    if not (cap and cap.isOpened()):
        try:
            cap = cv.VideoCapture(0, cv.CAP_V4L2)
        except Exception:
            cap = None
    if cap and cap.isOpened():
        return cap, False
    if TUNE["ALLOW_NO_CAMERA"]:
        print("WARNING: camera open failed; using dummy frames.")
        class DummyCap:
            def isOpened(self): return True
            def read(self):
                frame = np.zeros((h, w, 3), dtype=np.uint8)
                cv.putText(frame, "NO CAMERA", (10,30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)
                return True, frame
            def release(self): pass
        return DummyCap(), True
    print("ERROR: camera open failed.")
    return None, False

def load_latest_yaml():
    override = os.environ.get("VISION_CFG", "").strip()
    if override and os.path.isfile(override):
        path = override
    else:
        cands = sorted(glob.glob("config/vision_*.yaml"))
        if not cands:
            raise FileNotFoundError("No config in ./config — need RED/GREEN/BLUE/ORANGE ranges.")
        path = cands[-1]
    with open(path, "r") as f:
        return yaml.safe_load(f), path

def wrap180(a):
    while a > 180: a -= 360
    while a < -180: a += 360
    return a

def shortest_err(target_deg, now_deg):
    return wrap180(target_deg - now_deg)

def parse_tlm_line(line):
    """Read Arduino TLM fields independently; a rear sensor is optional.

    Yaw is an angle in degrees, supplied by the Arduino. For a DFRobot
    BNO055 this is getEuler().head, not gyroscope angular velocity.
    Accept head/heading aliases if the controller uses those field names.
    """
    parts = line.strip().split(',')
    if not parts or parts[0].strip().upper() != 'TLM':
        return None
    fields = {}
    for part in parts[1:]:
        key, sep, value = part.partition('=')
        if sep:
            fields[key.strip().lower()] = value.strip()

    packet = {}
    for key in ('yaw', 'head', 'heading', 'df', 'dl', 'dr', 'db', 'enc_cm'):
        if key not in fields:
            continue
        try:
            value = float(fields[key])
        except (ValueError, OverflowError):
            continue
        if math.isfinite(value):
            packet[key] = value
    for alias in ('head', 'heading'):
        if 'yaw' not in fields and alias in packet:
            packet['yaw'] = packet[alias]
            break
    return packet

def deg_to_norm(angle_deg):
    # These are the existing command coordinates, not physical servo degrees.
    # Arduino applies SERVO_DIR=-1 and also flips steering in reverse.
    angle_deg = float(max(LEFT_LIMIT, min(RIGHT_LIMIT, float(angle_deg))))
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
    """Heading correction updated once per fresh yaw sample, not per frame.

    Keep the corridor target supplied by the mission. Derivative on measured
    yaw slows an existing rotation without a derivative kick on target changes.
    The final command limiter also covers wall biases added by the mission.
    """
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
            # Serial packets in a batch have no sensor timestamps. Do not
            # derive a turn rate from two nearly simultaneous reads.
            if dt > 0.30:
                # A slow camera frame loses rate information, but must not
                # restart the final steering ramp at center on every frame.
                self.error = shortest_err(target, yaw)
                self.rate = 0.0
                self.yaw, self.sample_time = yaw, sample_time
            elif dt >= 0.01:
                alpha = max(0.001, min(1.0, float(self.tune['ERR_FILTER_ALPHA'])))
                a = 1.0 - (1.0 - alpha) ** (dt / 0.05)
                self.error = wrap180(self.error + a * shortest_err(shortest_err(target, yaw), self.error))
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
            # At rest, tiny heading noise must not move the wheels. Retain
            # damping when crossing the target with real angular motion.
            if abs(self.rate) < 3.0:
                return self.center
        effective_error = math.copysign(max(0.0, abs(self.error) - deadband), self.error)
        correction = (float(kp) * effective_error -
                      float(self.tune['STEER_KD_DEG_PER_DPS']) * self.rate)
        correction *= float(self.tune['STEER_SIGN'])
        correction = max(-max_offset, min(max_offset, correction))
        return self.center + correction

    def limit_command(self, target, now, direction):
        if self.command_direction != direction:
            self.command, self.command_time = self.center, None
        dt = 0.05 if self.command_time is None else max(0.0, min(0.10, now - self.command_time))
        rate = max(0.1, float(self.tune['STEER_SLEW_DEG_PER_SEC']))
        self.command = slew_limit(self.command, float(target), rate * dt)
        self.command_time, self.command_direction = now, direction
        return self.command

def parking_straight_steer(hold, yaw, target, sample_time, context, tune):
    """Keep wheels neutral inside the parking noise band; correct real drift."""
    max_offset = min(float(tune["STEER_MAX_DEG"]),
                     max(0.0, float(tune["LAST_STRAIGHT_MAX_STEER_DEG"])))
    command = hold.update(yaw, target, sample_time, context,
                          float(tune["STEER_KP_DEG_PER_DEG"]), max_offset)
    if abs(shortest_err(float(target), float(yaw))) <= max(
            0.0, float(tune["LAST_STRAIGHT_DEADBAND_DEG"])):
        # Do not let a rate spike or the preceding turn move straight wheels.
        hold.error = 0.0
        hold.rate = 0.0
        hold.command = hold.center
        return hold.center
    return command


class ParkingWallAlign:
    """Rear-wall seek, timed straight push, motor-off settle, fresh yaw check.

    The <5 cm reading starts the push timer; it is not proof of contact.
    After that reading, the short push remains latched even if the sensor
    loses its echo at the wall. No encoder distance is used while pushing.
    """
    def __init__(self, tune):
        self.tune = tune
        self.reset()

    def reset(self):
        self.phase = "IDLE"
        self.started_at = None
        self.after_rear_sequence = 0
        self.press_started_at = None
        self.stopped_at = None
        self.failure_reason = ""

    def start(self, now, rear_sequence):
        self.reset()
        self.started_at = float(now)
        self.after_rear_sequence = int(rear_sequence)
        self.phase = "SEEK"

    def _fresh_number(self, value, received_at, now):
        try:
            value, received_at = float(value), float(received_at)
        except (TypeError, ValueError, OverflowError):
            return None
        if (not math.isfinite(value) or not math.isfinite(received_at) or
                not 0.0 <= now - received_at <=
                float(self.tune["LAST_WALL_SENSOR_MAX_AGE_S"])):
            return None
        return value

    def step(self, rear_cm, rear_time, rear_sequence, yaw, yaw_time, now):
        now = float(now)
        if self.phase == "READY":
            return "COMPLETE"
        if self.phase in ("IDLE", "FAULT"):
            return "FAIL"
        yaw_value = self._fresh_number(yaw, yaw_time, now)
        if self.phase == "SEEK":
            if now - self.started_at >= float(self.tune["LAST_WALL_SEEK_MAX_S"]):
                self.phase = "FAULT"
                self.failure_reason = "rear-wall seek timed out; check rear db telemetry"
                return "FAIL"
            rear = self._fresh_number(rear_cm, rear_time, now)
            if (yaw_value is None or rear is None or rear <= 0.0 or
                    int(rear_sequence) <= self.after_rear_sequence or
                    float(rear_time) < self.started_at):
                return "WAIT_SENSOR"
            if rear < float(self.tune["LAST_WALL_REAR_CM"]):
                self.phase = "PRESS"
                self.press_started_at = now
            return "REVERSE"
        if self.phase == "PRESS":
            if yaw_value is None:
                self.phase = "FAULT"
                self.failure_reason = "fresh yaw lost during rear-wall push"
                return "FAIL"
            if now - self.press_started_at >= float(self.tune["LAST_WALL_PRESS_S"]):
                self.phase = "SETTLE"
                self.stopped_at = now
                return "STOP"
            return "REVERSE"
        if self.phase == "SETTLE":
            if now - self.stopped_at >= float(self.tune["LAST_WALL_SETTLE_MAX_S"]):
                self.phase = "FAULT"
                self.failure_reason = "fresh settled yaw unavailable after rear-wall push"
                return "FAIL"
            settled_at = self.stopped_at + float(self.tune["LAST_WALL_SETTLE_S"])
            if now < settled_at or yaw_value is None or float(yaw_time) < settled_at:
                return "STOP"
            self.phase = "READY"
            return "COMPLETE"
        return "FAIL"


class CrashRecoveryHeading:
    """Freeze the active straight target for the whole crash recovery."""
    def __init__(self, tune):
        self.tune = tune
        self.saved_target = None

    def clear(self):
        self.saved_target = None

    def latch(self, active_target):
        self.saved_target = wrap180(float(active_target))

    def target(self, fallback_target):
        if self.saved_target is None:
            return wrap180(float(fallback_target))
        return self.saved_target

    def status(self, now_yaw, start_time, now_time, fallback_target):
        target = self.target(fallback_target)
        tolerance = float(self.tune["WESC_EXIT_TOL_DEG"])
        hit = abs(shortest_err(target, float(now_yaw))) <= tolerance
        timed_out = (float(now_time) - float(start_time)) >= float(self.tune["WESC_MAX_SPIN_S"])
        return (hit or timed_out), hit, timed_out


class CornerLineRearm:
    """Reject the old corner stripe until it has left the camera view."""
    def __init__(self, tune):
        self.tune = tune
        self.reset()

    def reset(self):
        self.armed = True
        self.locked_at = None
        self.clear_frames = 0

    def lock(self, now):
        self.armed = False
        self.locked_at = float(now)
        self.clear_frames = 0

    def update(self, marker_visible, now):
        if self.armed:
            return True

        if marker_visible:
            self.clear_frames = 0
            return False

        # These are safety minima, even if an environment override is lower.
        min_hold = max(0.50, float(self.tune["CORNER_REARM_MIN_S"]))
        if self.locked_at is None or float(now) - self.locked_at < min_hold:
            self.clear_frames = 0
            return False

        self.clear_frames += 1
        required = max(3, int(self.tune["CORNER_REARM_CLEAR_FRAMES"]))
        if self.clear_frames >= required:
            self.armed = True
            self.locked_at = None
            self.clear_frames = 0
        return self.armed


def update_corner_line_rearm(rearm, frame_ok, marker_visible, now):
    """A missing camera frame is unknown, never proof that the line is gone."""
    if not bool(frame_ok):
        # Clear observations must be consecutive and camera-backed.
        rearm.clear_frames = 0
        return bool(rearm.armed)
    return bool(rearm.update(bool(marker_visible), float(now)))


class RangeSampleClock:
    """Keep one range value, sequence and receipt time as an atomic sample."""
    def __init__(self, field):
        self.field = str(field).lower()
        self.reset()

    def reset(self):
        self.sequence = 0
        self.received_at = None
        self.value = None

    def observe(self, packet, received_at):
        if packet is None or self.field not in packet:
            return False
        self.sequence += 1
        self.received_at = float(received_at)
        self.value = float(packet[self.field])
        return True


class FrontRangeSampleClock(RangeSampleClock):
    """Compatibility name for the front-ultrasonic sample clock."""
    def __init__(self):
        super().__init__("df")


class CameraFailureGuard:
    """Stop an active run after a short bounded sequence of camera failures."""
    def __init__(self, tune):
        self.tune = tune
        self.reset()

    def reset(self):
        self.failed_frames = 0

    def update(self, frame_ok, motion_active):
        if bool(frame_ok) or not bool(motion_active):
            self.failed_frames = 0
            return False
        self.failed_frames += 1
        required = min(10, max(
            1, int(self.tune.get("CAMERA_FAIL_STOP_FRAMES", 10))))
        return self.failed_frames >= required


class MissedCornerGuard:
    """Confirm a close front wall before using the locked corner direction."""
    def __init__(self, tune):
        self.tune = tune
        self.reset()

    def reset(self):
        self.close_frames = 0
        self.last_sample_seq = None

    def update(self, front_cm, direction_locked, corner_pending,
               pillar_active, preavoid_active, recovery_active,
               sample_seq=None):
        try:
            front = float(front_cm)
        except (TypeError, ValueError):
            front = -1.0
        eligible = (
            bool(direction_locked) and not bool(corner_pending) and
            not bool(pillar_active) and not bool(preavoid_active) and
            not bool(recovery_active) and math.isfinite(front) and
            0.0 <= front <= min(
                18.0, max(0.0, float(self.tune["MISSED_CORNER_FRONT_CM"])))
        )
        if not eligible:
            self.close_frames = 0
            self.last_sample_seq = sample_seq
            return False

        # Camera loops can run faster than telemetry. A retained dF value is
        # one sample, regardless of how many times the main loop sees it.
        if sample_seq is None or sample_seq == self.last_sample_seq:
            return False
        self.last_sample_seq = sample_seq

        self.close_frames += 1
        required = max(2, int(self.tune["MISSED_CORNER_CONFIRM_FRAMES"]))
        if self.close_frames >= required:
            self.close_frames = 0
            return True
        return False


def recovery_front_is_clear(front_cm, tune):
    """Only resume forward motion with a valid, genuinely clear front range."""
    return recovery_front_status(front_cm, 0.0, 0.0, tune) == "CLEAR"


def _range_sample_is_fresh(sample_time, now, tune,
                           sample_sequence=None, after_sequence=None):
    """Return True only for a recent sample newer than an optional baseline."""
    max_age = max(0.05, min(
        0.50, float(tune.get("WESC_FRONT_MAX_AGE_S", 0.50))))
    try:
        sample_time = float(sample_time)
        now = float(now)
    except (TypeError, ValueError):
        return False
    if (not math.isfinite(sample_time) or not math.isfinite(now) or
            sample_time > now + 0.05 or now - sample_time > max_age):
        return False
    if after_sequence is not None:
        try:
            if sample_sequence is None or int(sample_sequence) <= int(after_sequence):
                return False
        except (TypeError, ValueError):
            return False
    return True


def recovery_front_status(front_cm, sample_time, now, tune,
                          sample_sequence=None, after_sequence=None):
    """Classify fresh raw front clearance after the recovery turn."""
    if not _range_sample_is_fresh(
            sample_time, now, tune, sample_sequence, after_sequence):
        return "STALE"

    try:
        front = float(front_cm)
    except (TypeError, ValueError):
        return "INVALID"
    if not math.isfinite(front) or front < 0.0:
        return "INVALID"
    threshold = max(18.0, float(tune["WESC_EXIT_FRONT_CM"]))
    return "CLEAR" if front >= threshold else "BLOCKED"


def recovery_rear_status(rear_cm, sample_time, now, tune,
                         sample_sequence=None, after_sequence=None):
    """Require current rear room for the whole reverse plus a stop margin."""
    if not _range_sample_is_fresh(
            sample_time, now, tune, sample_sequence, after_sequence):
        return "STALE"
    try:
        rear = float(rear_cm)
    except (TypeError, ValueError):
        return "INVALID"
    if not math.isfinite(rear) or rear < 0.0:
        return "INVALID"
    back_cm = max(0.0, float(tune.get("WESC_BACK_CM", 15.0)))
    stop_margin = max(12.0, float(tune.get("WESC_REAR_STOP_CM", 12.0)))
    return "CLEAR" if rear >= back_cm + stop_margin else "BLOCKED"


def recovery_back_timed_out(start_time, now, tune):
    """Bound an encoder reverse even if DIST_DONE never arrives."""
    try:
        if start_time is None:
            return False
        elapsed = float(now) - float(start_time)
    except (TypeError, ValueError):
        return False
    timeout = max(0.20, min(
        2.0, float(tune.get("WESC_BACK_MAX_S", 2.0))))
    return elapsed >= timeout


def manual_start_allowed(recovery_active, recovery_phase):
    """A START command may not clear or bypass a latched recovery state."""
    return (not bool(recovery_active) and
            str(recovery_phase).upper() != "EFAULT")


def manual_start_state_allowed(state, wait_state):
    """Require a full program restart before starting another completed run."""
    return state == wait_state


def choose_recovery_action(front_status, aligned, back_cycles,
                           rear_status, tune):
    """Choose a bounded recovery action; unknown sensing always means STOP."""
    if not bool(aligned):
        return "HOLD"
    if front_status == "CLEAR":
        return "RESUME"
    if front_status != "BLOCKED":
        return "HOLD"

    max_cycles = min(2, max(1, int(tune.get("WESC_MAX_BACK_CYCLES", 2))))
    if int(back_cycles) >= max_cycles:
        return "HOLD"
    if rear_status != "CLEAR":
        return "HOLD"
    return "BACK"


def choose_corner_event(confirmed_line_color, front_guard_confirmed,
                        locked_direction):
    """Prefer a real stripe; otherwise allow a valid locked-direction guard."""
    if confirmed_line_color in ("BLUE", "ORANGE"):
        return "LINE", confirmed_line_color, None
    if front_guard_confirmed and locked_direction in (-1, +1):
        return "FRONT_GUARD", "FRONT_GUARD", int(locked_direction)
    return None, None, None


def latch_corner_event(latched_source, latched_event,
                       observed_source, observed_event):
    """Keep the accepted intersection identity until its turn is consumed."""
    if latched_event is not None:
        return latched_source, latched_event, False
    if observed_event is None:
        return None, None, False
    return observed_source, observed_event, True

def mask_hsv_ranges(hsv, ranges):
    if isinstance(ranges, dict) and "low" in ranges and "high" in ranges:
        low, high = np.array(ranges["low"], np.uint8), np.array(ranges["high"], np.uint8)
        return cv.inRange(hsv, low, high)
    mask = None
    for r in (ranges if isinstance(ranges, list) else []):
        if "low" in r and "high" in r:
            m = cv.inRange(hsv, np.array(r["low"], np.uint8), np.array(r["high"], np.uint8))
            mask = m if mask is None else cv.bitwise_or(mask, m)
    if mask is None:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    return mask


def normalized_odd_kernel(value):
    """Return an odd blur size while allowing 1 to mean no blur."""
    size = max(1, int(value))
    return size if size % 2 == 1 else size + 1


def color_roi_rect_from_cfg(w, h, cfg, tune):
    """Use AutoTune's saved vertical ROI and the main detector's centre width."""
    width_frac = max(0.06, min(1.0, float(tune["COLOR_ROI_WIDTH_FRAC"])))
    roi_width = max(1, min(int(w), int(round(float(w) * width_frac))))
    x0 = (int(w) - roi_width) // 2
    x1 = x0 + roi_width

    saved_y = cfg.get("roi_y") if isinstance(cfg, dict) else None
    if isinstance(saved_y, (list, tuple)) and len(saved_y) >= 2:
        try:
            y0 = max(0, min(int(h) - 1, int(saved_y[0])))
            y1 = max(1, min(int(h), int(saved_y[1])))
            if y1 > y0:
                return x0, y0, x1, y1
        except (TypeError, ValueError, OverflowError):
            pass

    # Safe fallback for an older config without AutoTune's roi_y field.
    height_frac = max(
        0.05, min(0.40, float(tune["COLOR_ROI_HEIGHT_FRAC"])))
    roi_height = max(1, min(int(h), int(round(float(h) * height_frac))))
    return x0, int(h) - roi_height, x1, int(h)


def apply_intersection_morphology(mask, cfg, cv_module=None):
    """Apply the same erode/dilate cleanup saved by AutoTune."""
    cvm = cv if cv_module is None else cv_module
    result = mask
    try:
        erode_size = max(0, min(31, int(cfg.get("erode", 0))))
        dilate_size = max(0, min(31, int(cfg.get("dilate", 0))))
    except (AttributeError, TypeError, ValueError, OverflowError):
        erode_size = 0
        dilate_size = 0

    if erode_size > 0:
        kernel = cvm.getStructuringElement(
            cvm.MORPH_ELLIPSE, (erode_size, erode_size))
        result = cvm.erode(result, kernel)
    if dilate_size > 0:
        kernel = cvm.getStructuringElement(
            cvm.MORPH_ELLIPSE, (dilate_size, dilate_size))
        result = cvm.dilate(result, kernel)
    return result


def intersection_blob_status(rect, contour_area, roi_w, roi_h, tune):
    """Explain whether a color contour is a playable intersection stripe."""
    if rect is None or roi_w <= 0 or roi_h <= 0:
        return "invalid"
    x, y, bw, bh = rect
    if bw <= 0 or bh <= 0:
        return "invalid"

    area = float(contour_area)
    roi_area = float(roi_w * roi_h)
    centre_x = 0.5 * float(roi_w)
    centre_half_span = (
        0.5 * float(tune["LINE_CENTER_SPAN_FRAC"]) * float(roi_w))
    right_pixel = float(x + bw - 1)
    centre_y = float(y) + 0.5 * float(bh)

    if area < float(tune["MIN_PIX_COLOR"]):
        return "too_small"
    if area > float(tune["LINE_MAX_AREA_FRAC"]) * roi_area:
        return "too_large"
    if float(bh) > float(tune["LINE_MAX_HEIGHT_FRAC"]) * float(roi_h):
        return "too_tall"
    if float(bw) < float(tune["LINE_MIN_WIDTH_FRAC"]) * float(roi_w):
        return "too_narrow"
    if (float(bw) / float(max(1, bh))) < float(tune["LINE_MIN_ASPECT"]):
        return "too_vertical"
    if centre_y < float(tune["LINE_MIN_CENTER_Y_FRAC"]) * float(roi_h):
        return "too_high"
    if (float(x) > (centre_x - centre_half_span) or
            right_pixel < (centre_x + centre_half_span)):
        return "off_path"
    return "accepted"


def valid_intersection_blob(rect, contour_area, roi_w, roi_h, tune):
    """Return True only for a floor stripe crossing the car's forward path."""
    return intersection_blob_status(
        rect, contour_area, roi_w, roi_h, tune) == "accepted"


def find_valid_intersection_rect(mask, tune, cv_module=None, diagnostics=False):
    """Choose the largest valid intersection stripe, ignoring false regions."""
    cvm = cv if cv_module is None else cv_module
    cnts, _ = cvm.findContours(mask, cvm.RETR_EXTERNAL, cvm.CHAIN_APPROX_SIMPLE)
    roi_h, roi_w = mask.shape[:2]
    best_rect = None
    best_area = 0.0
    rejected_rect = None
    rejected_area = 0.0
    rejected_reason = None
    for contour in cnts:
        area = float(cvm.contourArea(contour))
        rect = cvm.boundingRect(contour)
        status = intersection_blob_status(rect, area, roi_w, roi_h, tune)
        if status == "accepted" and area > best_area:
            best_rect = rect
            best_area = area
        elif status != "accepted" and area > rejected_area:
            rejected_rect = rect
            rejected_area = area
            rejected_reason = status
    result = best_rect, int(round(best_area))
    if diagnostics:
        return result + (rejected_rect, rejected_reason)
    return result


def confirmed_intersection_color(samples, required_frames):
    """Confirm only repeated observations of one intersection-line color."""
    required = max(1, int(required_frames))
    values = list(samples)
    if len(values) != required:
        return None
    first = values[0]
    if first not in ("BLUE", "ORANGE"):
        return None
    return first if all(value == first for value in values) else None


def pillar_fsm_state_allowed(state, run_state, *final_approach_states):
    """Allow pillar avoidance only during normal corridor running.

    Final parking is intentionally independent of red/green pillar color.
    The variadic argument is retained for compatibility with older callers,
    but final-sequence states are never allowed to activate the pillar FSM.
    """
    return state == run_state


def build_last_turn_plan(entry_yaw, intersection_color, turn_deg,
                         fallback_sign=None, parking_turn_deg=None):
    """Return corner sign/target and the exact opposite parking sign/target."""
    color = str(intersection_color or "").strip().upper()
    if color == "BLUE":
        corner_sign = -1
    elif color == "ORANGE":
        corner_sign = +1
    elif fallback_sign in (-1, +1):
        corner_sign = int(fallback_sign)
    else:
        return None
    try:
        entry = float(entry_yaw)
        corner_degrees = abs(float(turn_deg))
        parking_degrees = (
            corner_degrees if parking_turn_deg is None
            else abs(float(parking_turn_deg))
        )
    except (TypeError, ValueError, OverflowError):
        return None
    if (not all(math.isfinite(value) for value in
                (entry, corner_degrees, parking_degrees)) or
            corner_degrees <= 0.0 or parking_degrees <= 0.0):
        return None
    corner_target = wrap180(entry + corner_sign * corner_degrees)
    parking_sign = -corner_sign
    parking_target = wrap180(corner_target + parking_sign * parking_degrees)
    return corner_sign, corner_target, parking_sign, parking_target


def last_front_stage_status(front_cm, received_at, now, threshold_cm,
                            max_age_s, inclusive):
    """Classify one front sample for a last-sequence distance gate."""
    if received_at is None:
        return "STALE"
    try:
        front = float(front_cm)
        age = float(now) - float(received_at)
        threshold = max(0.0, float(threshold_cm))
        max_age = max(0.05, float(max_age_s))
    except (TypeError, ValueError, OverflowError):
        return "INVALID"
    if not all(math.isfinite(value) for value in (front, age, threshold, max_age)):
        return "INVALID"
    if age < 0.0 or age > max_age:
        return "STALE"
    if front < 0.0:
        return "INVALID"
    reached = front <= threshold if bool(inclusive) else front < threshold
    return "REACHED" if reached else "WAIT"


def last_front_stage_action(front_status, elapsed_s, max_s):
    """Choose a front-distance stage action with timeout taking priority."""
    try:
        elapsed = float(elapsed_s)
        timeout = float(max_s)
    except (TypeError, ValueError, OverflowError):
        return "FAIL"
    if (not math.isfinite(elapsed) or not math.isfinite(timeout) or
            elapsed < 0.0 or timeout <= 0.0):
        return "FAIL"
    if elapsed >= timeout:
        return "FAIL"
    if front_status == "REACHED":
        return "COMPLETE"
    if front_status == "WAIT":
        return "MOVE"
    return "WAIT_SENSOR"


def last_encoder_action(start_enc_cm, current_enc_cm, target_cm,
                        elapsed_s, max_s, telemetry_fresh=True):
    """Supervise either forward or reverse travel from live encoder distance."""
    try:
        start = float(start_enc_cm)
        current = float(current_enc_cm)
        target = float(target_cm)
        elapsed = float(elapsed_s)
        timeout = float(max_s)
    except (TypeError, ValueError):
        return "FAIL"
    values = (start, current, target, elapsed, timeout)
    if (not all(math.isfinite(value) for value in values) or
            target < 0.0 or elapsed < 0.0 or timeout <= 0.0):
        return "FAIL"
    if not bool(telemetry_fresh) or elapsed >= timeout:
        return "FAIL"
    if abs(current - start) >= target:
        return "COMPLETE"
    return "MOVE"


def last_turn_action(yaw_hit, timed_out, yaw_fresh):
    """Require fresh yaw success; a timeout always fails safely."""
    if bool(timed_out):
        return "FAIL"
    if bool(yaw_fresh) and bool(yaw_hit):
        return "COMPLETE"
    if not bool(yaw_fresh):
        return "WAIT_SENSOR"
    return "TURN"


def last_final_stop_allowed(front_cm, threshold_cm, received_at, now,
                            max_age_s, sample_sequence, after_sequence,
                            arm_until):
    """Stop only on a new, fresh sample strictly below the parking limit."""
    try:
        sequence_is_new = int(sample_sequence) > int(after_sequence)
        arm_elapsed = float(now) >= float(arm_until)
    except (TypeError, ValueError, OverflowError):
        return False
    status = last_front_stage_status(
        front_cm, received_at, now, threshold_cm, max_age_s, False
    )
    return sequence_is_new and arm_elapsed and status == "REACHED"


def intersection_hud_style(corner_color):
    """Return GUI text and BGR color only for a confirmed intersection."""
    if corner_color == "BLUE":
        return "INTERSECTION: BLUE", (255, 128, 0)
    if corner_color == "ORANGE":
        return "INTERSECTION: ORANGE", (0, 128, 255)
    return None


def intersection_line_overlay(rect, roi_x0, roi_y0, color_name):
    """Map an accepted ROI stripe to full-frame GUI coordinates."""
    if rect is None or color_name not in ("BLUE", "ORANGE"):
        return None
    x, y, width, height = rect
    top_left = (int(roi_x0 + x), int(roi_y0 + y))
    bottom_right = (int(roi_x0 + x + width), int(roi_y0 + y + height))
    if color_name == "BLUE":
        bgr = (255, 128, 0)
    else:
        bgr = (0, 128, 255)
    return top_left, bottom_right, f"{color_name} INTERSECTION", bgr

# Flip helpers
def flip_delta_deg(color):
    s = float(TUNE["FLIP_LEFT_SIGN"])
    ang = float(TUNE["FLIP_TURN_DEG"])
    return (s*ang) if color == "RED" else (-s*ang)

def flip_target_from(base_yaw, color):
    return wrap180(base_yaw + flip_delta_deg(color))

# Four front-distance thresholds, repeated for all three laps
def front_turn_cm_for(next_turn_number: int) -> float:
    idx = ((int(next_turn_number) - 1) % 4) + 1
    return float(TUNE[f"FRONT_TURN_CM_{idx}"])


def corner_front_status(front_cm, received_at, now, threshold_cm, max_age_s):
    """Classify a raw front sample before a latched corner may proceed."""
    if received_at is None:
        return "STALE"
    try:
        age = float(now) - float(received_at)
        front = float(front_cm)
        threshold = max(0.0, float(threshold_cm))
        max_age = max(0.05, float(max_age_s))
    except (TypeError, ValueError, OverflowError):
        return "INVALID"
    if not math.isfinite(age) or age < 0.0 or age > max_age:
        return "STALE"
    if not math.isfinite(front) or front < 0.0:
        return "INVALID"
    return "REACHED" if front < threshold else "WAIT"


class CornerDistanceChecker:
    """Confirm a corner distance using separate consecutive range packets."""
    def __init__(self, required_samples=3):
        self.required_samples = max(2, int(required_samples))
        self.reset()

    def reset(self, sample_seq=None):
        self.confirmed_samples = 0
        self.last_sample_seq = sample_seq
        self.last_status = "WAIT"
        self.last_update_was_new = False

    def update(self, front_cm, received_at, now, threshold_cm, max_age_s,
               sample_seq=None):
        self.last_update_was_new = (
            sample_seq is not None and sample_seq != self.last_sample_seq
        )
        if not self.last_update_was_new:
            return self.confirmed_samples >= self.required_samples

        self.last_sample_seq = sample_seq
        self.last_status = corner_front_status(
            front_cm, received_at, now, threshold_cm, max_age_s
        )
        if self.last_status == "REACHED":
            self.confirmed_samples += 1
        else:
            self.confirmed_samples = 0
        return self.confirmed_samples >= self.required_samples

# Per-corner turn type
def corner_turn_type_for(next_turn_number: int) -> str:
    idx = ((int(next_turn_number) - 1) % 4) + 1
    val = str(TUNE.get(f"CORNER_TURN_TYPE_{idx}", "BACKWARD")).strip().upper()
    return "FORWARD" if val.startswith("FOR") else "BACKWARD"

# Servo chooser
def select_servo_deg(mode: str, dir_sign: int) -> int:
    if mode == "FORWARD":
        return (TUNE["FWD_LEFT_SERVO_DEG"] if dir_sign == +1 else TUNE["FWD_RIGHT_SERVO_DEG"])
    else:
        return (TUNE["BACK_RIGHT_SERVO_DEG"] if dir_sign == +1 else TUNE["BACK_LEFT_SERVO_DEG"])

def dir_sign_from_string(s: str) -> int:
    s = (s or "").strip().upper()
    return +1 if s == "RIGHT" else -1

# Helpers for wall-escape
def yaw_err_deg(yaw_ref, yaw_now): return abs(wrap180(yaw_ref - yaw_now))

def choose_escape_dir(dL, dR):
    # +1 RIGHT, -1 LEFT favor more open side
    if dL >= 0 and dR >= 0: return +1 if dR >= dL else -1
    if dL >= 0 and dR < 0:  return -1 if dL > 0 else +1
    if dR >= 0 and dL < 0:  return +1
    return +1

# MAIN
def main():
    cfg, cfg_path = load_latest_yaml()
    print("Loaded vision config:", cfg_path)

    # Serial & parsing
    ser = None
    device = TUNE["ROBOT_PORT"]
    baud = TUNE["ROBOT_BAUD"]

    last_tlm_t = 0.0
    last_pong_t = 0.0
    last_ping_t = 0.0
    last_reset_t = -1e9

    rx_buf = b""
    heading_hold = HeadingHold(TUNE, CENTER_DEG)
    crash_recovery = CrashRecoveryHeading(TUNE)
    corner_line_rearm = CornerLineRearm(TUNE)
    missed_corner_guard = MissedCornerGuard(TUNE)
    corner_distance_checker = CornerDistanceChecker(
        TUNE["FRONT_TURN_CONFIRM_SAMPLES"]
    )
    front_range_clock = FrontRangeSampleClock()
    rear_range_clock = RangeSampleClock("db")
    wall_alignment = ParkingWallAlign(TUNE)
    camera_failure_guard = CameraFailureGuard(TUNE)
    arduino_turn_pending = False
    dist_done_re = re.compile(r"^DIST_DONE")
    dist_prog_re = re.compile(r"^DIST,cm=([-0-9.]+)")

    # Arduino reset sync
    program_start_t = time.time()
    had_ready_once = False
    arduino_reset_request = False

    def open_serial():
        nonlocal ser, rx_buf, last_yaw_t, arduino_armed, arduino_turn_pending
        rx_buf = b''
        last_yaw_t = None
        arduino_armed = False
        arduino_turn_pending = False
        heading_hold.reset()
        corner_line_rearm.reset()
        missed_corner_guard.reset()
        front_range_clock.reset()
        rear_range_clock.reset()
        camera_failure_guard.reset()
        if serial is None: return
        try:
            ser = serial.Serial(device, baudrate=baud, timeout=0.01, write_timeout=0.2, dsrdtr=False)
            time.sleep(0.30)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            # Repeat on READY in case the board is still booting now.
            ser.write(f"SET_US_TIMEOUT,{TUNE['CONTROL_US_TIMEOUT_US']}\n".encode('ascii'))
            print("Serial OK on", device)
        except Exception as e:
            print("WARNING: serial not open ->", e)
            ser = None

    def safe_write(line_bytes):
        nonlocal ser, arduino_turn_pending
        command = line_bytes.strip().split(b',', 1)[0].upper()
        motion = command in (
            b'START', b'S', b'CENTER', b'BACK', b'BACKC', b'TURN_ABS',
            b'FWD_CM', b'BACK_CM'
        )
        if serial is None:
            return False
        if ser is None:
            open_serial()
            if ser is None:
                return False
        try:
            if motion and not yaw_ready():
                ser.write(b'STOP\n')
                heading_hold.reset()
                arduino_turn_pending = False
                return False
            takeover = arduino_turn_pending and command in (b'CENTER', b'BACK', b'BACKC', b'FWD_CM', b'BACK_CM')
            # The supplied Uno code ignores CENTER/BACKC as a state change
            # while TURNING. STOP explicitly hands control back to Python.
            payload = (b'STOP\n' if takeover else b'') + line_bytes
            ser.write(payload)
            if takeover or command == b'STOP':
                arduino_turn_pending = False
            if command == b'TURN_ABS':
                arduino_turn_pending = True
            if command in (b'STOP', b'TURN_ABS', b'FWD_CM', b'BACK_CM'):
                heading_hold.reset()
            return True
        except Exception as e:
            print("Serial write error:", e)
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            open_serial()
            if ser:
                try:
                    if motion and not yaw_ready():
                        ser.write(b'STOP\n')
                        return False
                    ser.write(line_bytes)
                    arduino_turn_pending = (command == b'TURN_ABS')
                    heading_hold.reset()
                    return True
                except Exception as e2:
                    print("Write retry failed:", e2)
        return False

    def reset_arduino():
        nonlocal ser, last_reset_t
        now = time.time()
        if now - last_reset_t < TUNE["RESET_BACKOFF_SEC"]:
            return
        last_reset_t = now
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass
        time.sleep(0.2)
        open_serial()
        print("[RESET] (basic reopen)")

    def link_alive(now):
        return (now - max(last_tlm_t, last_pong_t)) < TUNE["STALE_LINK_SEC"]

    def maybe_ping(now):
        nonlocal last_ping_t
        if (now - last_ping_t) >= TUNE["PING_PERIOD_SEC"]:
            safe_write(b"PING\n")
            last_ping_t = now

    # Telemetry
    last_yaw = 0.0  # Display placeholder only; last_yaw_t marks a real sample.
    last_yaw_t = None
    last_tlm_line = ''
    last_US = {"dF": -1, "dL": -1, "dR": -1, "dB": -1}
    us_ema  = {"dF": -1.0, "dL": -1.0, "dR": -1.0, "dB": -1.0}
    arduino_armed = False

    # encoder & distance move flags
    last_enc_cm = 0.0
    last_enc_t = None
    dist_done_flag = False
    last_dist_progress_cm = None

    def yaw_ready():
        return (last_yaw_t is not None and
                0.0 <= time.monotonic() - last_yaw_t < TUNE['STALE_TLM_SEC'])

    def encoder_ready():
        return (last_enc_t is not None and
                0.0 <= time.monotonic() - last_enc_t < TUNE['STALE_TLM_SEC'])

    def sane_cm(x):
        if x is None or x < 0:
            return -1
        return float(min(x, TUNE["US_MAX_CM"]))

    def read_tlm():
        nonlocal rx_buf, last_yaw, last_tlm_t, last_US, us_ema, last_pong_t, arduino_armed
        nonlocal last_enc_cm, last_enc_t, dist_done_flag, last_dist_progress_cm
        nonlocal had_ready_once, arduino_reset_request
        nonlocal last_yaw_t, last_tlm_line

        if ser is None:
            return
        try:
            # Drain queued messages so camera processing does not leave yaw
            # seconds behind the car's current heading.
            data = ser.read(max(256, ser.in_waiting))
            if not data:
                return
            rx_buf += data
            while b"\n" in rx_buf:
                line, rx_buf = rx_buf.split(b"\n", 1)
                s = line.decode("utf-8", "ignore").strip()

                # Handshake lines from Arduino
                if s == "PONG":
                    last_pong_t = time.time()
                    continue

                if s == "READY":
                    last_yaw_t = None
                    last_enc_t = None
                    arduino_armed = False
                    heading_hold.reset()
                    front_range_clock.reset()
                    rear_range_clock.reset()
                    last_US["dF"] = -1
                    last_US["dB"] = -1
                    us_ema["dF"] = -1.0
                    us_ema["dB"] = -1.0
                    safe_write(f"SET_US_TIMEOUT,{TUNE['CONTROL_US_TIMEOUT_US']}\n".encode('ascii'))
                    last_pong_t = time.time()
                    if int(TUNE.get("JETSON_RESET_ON_ARDUINO_READY", 1)):
                        if had_ready_once and (time.time() - program_start_t) >= float(TUNE["JETSON_RESET_GRACE_S"]):
                            arduino_reset_request = True
                        had_ready_once = True
                    continue

                if s == "ARMED":
                    arduino_armed = True
                    last_pong_t = time.time()
                    continue

                # Distance move messages
                if dist_done_re.match(s):
                    dist_done_flag = True
                    continue
                mprog = dist_prog_re.match(s)
                if mprog:
                    try:
                        last_dist_progress_cm = float(mprog.group(1))
                    except:
                        last_dist_progress_cm = None
                    continue

                # Telemetry packet
                packet = parse_tlm_line(s)
                if packet is not None:
                    last_tlm_line = s
                    if packet:
                        last_tlm_t = time.time()
                    front_range_clock.observe(packet, time.monotonic())
                    rear_range_clock.observe(packet, time.monotonic())
                    if 'yaw' in packet:
                        last_yaw = wrap180(packet['yaw'] % 360.0)
                        last_yaw_t = time.monotonic()

                    for k in ("dF", "dL", "dR", "dB"):
                        if k.lower() not in packet:
                            continue
                        v = packet[k.lower()]
                        a = 0.35
                        if v < 0:
                            last_US[k] = -1
                        else:
                            if us_ema[k] < 0:
                                us_ema[k] = v
                            else:
                                us_ema[k] = (1-a)*us_ema[k] + a*v
                            last_US[k] = int(us_ema[k])

                    if 'enc_cm' in packet:
                        last_enc_cm = packet['enc_cm']
                        last_enc_t = time.monotonic()

        except Exception as e:
            print("Serial read error:", e)
            try:
                ser.close()
            except Exception:
                pass
            open_serial()

    # Camera
    cap, using_dummy = open_camera()
    if cap is None:
        return


    def restart_program(reason=""):
        try:
            safe_write(b"STOP\n")
        except Exception:
            pass
        try:
            cap.release()
            cv.destroyAllWindows()
        except Exception:
            pass
        if reason:
            print(f"[JETSON] Restarting program: {reason}")
        else:
            print("[JETSON] Restarting program")
        os.execl(sys.executable, sys.executable, *sys.argv)

    # State machine IDs
    (STATE_WAIT_START,
     STATE_ESC_S1_FWD_L, STATE_ESC_S2_BACK_L, STATE_ESC_S3_FWD_L, STATE_ESC_S3_FWD_HOLD, STATE_ESC_S4_FWD_RTN,
     STATE_RUN, STATE_CORNER_TURNING, STATE_CORNER_BACKCENTER,
     STATE_FLIP_WAIT, STATE_FLIP_TURN, STATE_FINAL_HOLD,
     STATE_POST12_TURN, STATE_POST12_FWD, STATE_DONE,
     STATE_PARK_SEEK, STATE_PARK_APPROACH, STATE_PARK_ALIGN, STATE_PARK_BACK_IN, STATE_PARKED,
     STATE_PARK_TO_30, STATE_PARK_EXTRA_FWD, STATE_PARK_TURN_IN,
     # Last-turn parking sequence
     STATE_LAST_APPROACH_5, STATE_LAST_BACK_CENTER, STATE_LAST_CORNER_TURN,
     STATE_LAST_CENTER_ENCODER, STATE_LAST_APPROACH_100,
     STATE_LAST_ENCODER_FWD, STATE_LAST_PARK_TURN, STATE_LAST_PARK_APPROACH,
     STATE_LAST_WALL_ALIGN
     ) = range(32)
    state = STATE_WAIT_START

    # Main run bookkeeping
    start_yaw = 0.0
    corridor_idx = 0
    orientation_offset_deg = 0.0

    run_start_time = None
    run_end_time = None

    yaw_ref = 0.0
    def set_ref_to_corridor():
        nonlocal yaw_ref
        yaw_ref = wrap180(start_yaw + corridor_idx * TUNE["TURN_DEG"] + orientation_offset_deg)

    # Corner book-keeping
    yaw_at_corner_start = None
    intended_corner_yaw = None
    corner_turn_start_t = -1.0
    turn_count = 0
    target_idx = 0

    # Direction lock
    corner_dir_latch = None
    dir_locked = False
    dir_first_base = None
    force_all_dir = None

    # Flip scheduling
    waiting_first_pillar_after_turn8 = False
    flip_scheduled_color = None
    flip_wait_until = -1.0
    rotate_target = None
    rotate_start_t = -1.0
    post_flip_soft_until = -1.0

    # Final hold
    final_hold_until = -1.0

    # Post-12 vars
    post12_turn_target = None
    post12_turn_start_t = -1.0

    # Corner detection control
    detect_enabled = True
    color_pause_until = -1.0
    color_q = deque(maxlen=TUNE["COLOR_CONFIRM_FRAMES"])
    see_color_slow = False
    corner_color = None
    corner_source = None

    # HUD
    line_rect_blue = None
    line_rect_orange = None
    rejected_rect_blue = None
    rejected_rect_orange = None
    rejected_reason_blue = None
    rejected_reason_orange = None

    # Pillar trackers
    first_pillar_after_first4 = None
    last_pillar_before_5th = None

    # Flip policy override
    flip_policy_enable = None
    def flip_enabled_now():
        base = int(TUNE["FLIP_ENABLED"])
        if flip_policy_enable is None:
            return base
        return 1 if (base and flip_policy_enable == 1) else 0

    # Current corner plan
    current_corner_dir = None    # +1 = RIGHT, -1 = LEFT
    current_corner_mode = None   # FORWARD/BACKWARD

    # Pillar detection control
    pillar_q = deque(maxlen=TUNE["PILLAR_CONFIRM_FRAMES"])
    pillar_active = False
    pillar_color = None
    pillar_state = "IDLE"
    pillar_turn_target = None
    pillar_turn_start_t = -1.0
    corridor_yaw_before_pillar = None

    # First-Step Pillar AVOID
    preavoid_active = False
    preavoid_state = "ACTIVE"
    preavoid_started_t = -1.0
    pillar_timer_start = None
    last_seen_pillar_t = -1.0
    preavoid_color = None
    preavoid_lost_frames = 0
    preavoid_last_bias_deg = 0.0
    preavoid_available = True
    preavoid_post_suppress_until = -1.0
    preavoid_enc_start_cm = None

    # Pillar suppression
    pillar_suppress = False

    # Steering history lives in heading_hold and resets between maneuvers.
    last_tx = 0.0
    last_yaw_debug_t = -1.0

    # Wall avoid FSM
    avoid_active = False
    avoid_side = None
    avoid_until = -1.0

    # Encoder-stuck escape
    enc_progress_cm_at = 0.0
    enc_progress_t = time.time()
    enc_stuck_active = False
    enc_stuck_phase = "EIDLE"  # EIDLE, EBACK, ETURN, ECHECK, EFAULT
    enc_cmd_sent = False
    enc_waiting_dist = False
    enc_cooldown_until = -1.0
    enc_turn_start_t = -1.0
    enc_back_cycles = 0
    enc_hold_reason = None
    enc_back_after_rear_seq = 0
    enc_check_after_front_seq = 0
    enc_back_start_t = None

    # Legacy wall-escape (inactive but kept)
    wall_escape_active = False

    # Parking helpers (legacy keep)
    park_start_t = -1.0
    park_align_start_t = -1.0
    last_magnet_area = 0
    park_magnet_side = None
    park_magnet_last_dist_cm = None
    park_extra_until = -1.0
    park_turn_target = None
    park_turn_start_t = -1.0

    # Always-on magnet left/right bands
    mag_wall_left_cm  = None
    mag_wall_right_cm = None

    # Last-turn parking sequence VARS
    lastseq_active = False
    last_corner_sign = None
    last_corner_target = None
    last_parking_sign = None
    last_parking_target = None
    last_stage_start_t = -1.0
    last_center_after_enc_t = -1.0
    last_center_target_cm = None
    last_turn_start_t = -1.0
    last_move_start_enc = None
    last_move_start_t = -1.0
    last_final_front_after_seq = 0
    last_final_stop_arm_until = -1.0
    last_wall_alignment_done = False
    last_wall_action = None
    last_no_avoid_override = False  # disable wall/mag avoid during last seq

    # Pillar lateral position snapshot at trigger
    pillar_pre_cx = None
    pillar_side_frac = 0.0     # 0..1 normalize how close pillar is to its wall
    pillar_fwd_cmd_sent = False
    pillar_fwd_waiting_dist = False

    # HUD window
    cv.namedWindow("robot", cv.WINDOW_AUTOSIZE)

    # helpers to send
    def continuous_heading_active():
        return (not pillar_active and not preavoid_active and not enc_stuck_active and
                state in (STATE_RUN, STATE_CORNER_BACKCENTER, STATE_ESC_S3_FWD_HOLD,
                          STATE_FLIP_WAIT, STATE_FINAL_HOLD, STATE_POST12_FWD,
                          STATE_LAST_APPROACH_5, STATE_LAST_BACK_CENTER,
                          STATE_LAST_CENTER_ENCODER, STATE_LAST_APPROACH_100,
                          STATE_LAST_ENCODER_FWD, STATE_LAST_PARK_APPROACH))

    def send_center_abs_deg(steer_deg, speed_pwm):
        steer_deg = int(max(LEFT_LIMIT, min(RIGHT_LIMIT, int(steer_deg))))
        steer_norm = deg_to_norm(steer_deg)
        spd = max(MIN_SPEED, min(255, int(speed_pwm)))
        safe_write(f"CENTER,{steer_norm:.3f},{spd}\n".encode("ascii"))

    def send_center_deg(steer_deg, speed_pwm):
        # Limit AFTER wall/magnet corrections are added, not just the yaw part.
        if continuous_heading_active() and speed_pwm > 0:
            steer_deg = heading_hold.limit_command(steer_deg, time.monotonic(), +1)
        steer_norm = deg_to_norm(steer_deg)
        spd = max(MIN_SPEED, min(255, int(speed_pwm)))
        safe_write(f"CENTER,{steer_norm:.3f},{spd}\n".encode("ascii"))

    def send_back(speed_pwm):
        spd = max(0, min(255, int(speed_pwm)))
        safe_write(f"BACK,{spd}\n".encode("ascii"))

    def send_backc_norm(norm, speed_pwm):
        spd = max(0, min(255, int(speed_pwm)))
        safe_write(f"BACKC,{norm:.3f},{spd}\n".encode("ascii"))

    def send_backc_deg(steer_deg, speed_pwm):
        if continuous_heading_active() and speed_pwm > 0:
            steer_deg = heading_hold.limit_command(steer_deg, time.monotonic(), -1)
        u = deg_to_norm(steer_deg)
        send_backc_norm(u, speed_pwm)

    # Encoder move helpers
    def send_fwd_cm(cm, pwm):
        safe_write(f"FWD_CM,{float(cm):.2f},{int(pwm)}\n".encode("ascii"))

    def send_back_cm(cm, pwm):
        safe_write(f"BACK_CM,{float(cm):.2f},{int(pwm)}\n".encode("ascii"))

    def finish_run(message):
        nonlocal state, run_end_time
        safe_write(b"STOP\n")
        state = STATE_DONE
        if run_end_time is None:
            run_end_time = time.time()
        print(f"[DONE] {message} -> STOP")

    def finish_last_sequence(success, message):
        nonlocal state, lastseq_active, last_no_avoid_override, run_end_time
        nonlocal pillar_suppress
        safe_write(b"STOP\n")
        lastseq_active = False
        last_no_avoid_override = False
        pillar_suppress = False
        pillar_q.clear()
        state = STATE_DONE
        run_end_time = time.time()
        result = "PARKED" if success else "NOT PARKED"
        print(f"[LAST] {message} -> STOP ({result})")

    # ROI helpers
    def color_roi_rect(w,h):
        return color_roi_rect_from_cfg(w, h, cfg, TUNE)

    def pillar_roi_rect(w,h):
        wf, hf = TUNE["PILLAR_ROI_WIDTH_FRAC"], TUNE["PILLAR_ROI_HEIGHT_FRAC"]
        rw, rh = int(w*wf), int(h*hf)
        x0 = (w-rw)//2; x1 = x0+rw
        y0 = h-rh-20; y1 = h-2
        return (x0,y0,x1,y1)

    def magnet_roi_rect(w,h):
        wf = max(0.10, min(1.00, float(TUNE.get("PARK_ROI_WIDTH_FRAC",  0.90))))
        hf = max(0.10, min(1.00, float(TUNE.get("PARK_ROI_HEIGHT_FRAC", 0.50))))
        rw, rh = int(w * wf), int(h * hf)
        x0 = (w - rw) // 2
        x1 = x0 + rw
        y1 = h - max(0, int(TUNE.get("PARK_ROI_BOT_PAD_PX", 2)))
        y0 = max(0, y1 - rh - max(0, int(TUNE.get("PARK_ROI_TOP_PAD_PX", 10))))
        return (x0, y0, x1, y1)

    def pillar_distance_cm_from_bbox(h_px):
        h_px = max(1.0, float(h_px))
        return TUNE["PILLAR_DIST_K"] / h_px

    def magnet_distance_cm_from_bbox(h_px):
        h_px = max(1.0, float(h_px))
        return TUNE.get("PARK_MAGNET_DIST_K", 6000.0) / h_px

    # steering for yaw centering
    def compute_center_steer_deg(yaw_now):
        if lastseq_active and last_wall_alignment_done:
            # Parking keeps the existing mission heading after wall alignment. The ordinary
            # corridor controller and its tuning remain unchanged.
            return parking_straight_steer(
                heading_hold, yaw_now, yaw_ref, last_yaw_t, state, TUNE
            )
        kp = float(TUNE["STEER_KP_DEG_PER_DEG"])
        max_off = float(TUNE["STEER_MAX_DEG"])
        if time.time() < post_flip_soft_until:
            kp *= float(TUNE["POST_FLIP_KP_GAIN"])
            max_off = min(max_off, float(TUNE["POST_FLIP_MAX_DEG"]))
        context = (state, pillar_active, pillar_state, preavoid_active,
                   enc_stuck_active, enc_stuck_phase)
        target = heading_hold.update(yaw_now, yaw_ref, last_yaw_t, context, kp, max_off)
        return max(float(TUNE["CENTER_STEER_MIN"]), min(float(TUNE["CENTER_STEER_MAX"]), target))

    def at_yaw(target_deg, now_deg, tol_deg):
        return abs(shortest_err(target_deg, now_deg)) <= float(tol_deg)

    def reached_or_timeout(target_deg, start_t, tol_deg, max_s, now_yaw):
        hit = at_yaw(target_deg, now_yaw, tol_deg)
        tmo = (time.time() - start_t) >= float(max_s)
        return (hit or tmo), hit, tmo

    print("READY. Keys: S=start, X=reset Arduino + restart Python, Q=quit.")
    print("[STRAIGHT] kp=%.2f kd=%.2f deadband=%.1f alpha=%.2f slew=%.1f deg/s; commands <=20 Hz" %
          (TUNE['STEER_KP_DEG_PER_DEG'], TUNE['STEER_KD_DEG_PER_DPS'],
           TUNE['STEER_DEAD_ERR_DEG'], TUNE['ERR_FILTER_ALPHA'], TUNE['STEER_SLEW_DEG_PER_SEC']))

    # If user forced a global direction, lock it now
    if TUNE["ALL_TURNS_DIR"] in ("LEFT", "RIGHT") and int(TUNE["LOCK_ALL_12_TURNS_ONE_DIRECTION"]):
        force_all_dir = (-1 if TUNE["ALL_TURNS_DIR"] == "LEFT" else +1)
        dir_locked = True
        print(f"[DIR] Forced all-12 direction from TUNE: {TUNE['ALL_TURNS_DIR']} ({force_all_dir:+d})")

    # helper to choose ESC sequence direction
    def decide_esc_seq_dir():
        L = sane_cm(last_US.get("dL", -1))
        R = sane_cm(last_US.get("dR", -1))
        raw_dir = (+1 if L < R else -1)   # left<right -> RIGHT-first
        if int(TUNE["ESC_SIDE_INVERT"]):
            raw_dir *= -1
        if float(TUNE["ESC_DIR_SIGN"]) < 0:
            raw_dir *= -1
        return +1 if raw_dir >= 0 else -1

    # ESC targets & servos
    esc_s1_target = esc_s2_target = esc_s3_target = esc_s4_target = None
    esc_seq_dir = +1
    esc_fwd_first_servo = esc_back_first_servo = esc_fwd_return_servo = None
    esc_turn_start_t = -1.0
    esc_cmd_sent = False
    esc_waiting_dist = False
    pillar_back_cmd_sent = False
    pillar_back_waiting_dist = False

    def build_escape_targets():
        nonlocal esc_s1_target, esc_s2_target, esc_s3_target, esc_s4_target
        nonlocal esc_fwd_first_servo, esc_back_first_servo, esc_fwd_return_servo
        dir_first = esc_seq_dir  # +1 RIGHT-first, -1 LEFT-first
        def directed(mag_deg, sign): return wrap180(yaw_ref + sign * abs(float(mag_deg)))
        esc_s1_target = directed(TUNE["ESC_S1_TARGET_DEG"], dir_first)
        esc_s2_target = directed(TUNE["ESC_S2_TARGET_DEG"], dir_first)
        esc_s3_target = directed(TUNE["ESC_S3_TARGET_DEG"], dir_first)
        rtn_bias = float(TUNE["ESC_RTN_BIAS_DEG"])
        esc_s4_target = wrap180(yaw_ref + (-dir_first) * abs(rtn_bias))
        esc_fwd_first_servo   = select_servo_deg("FORWARD",  (+1 if dir_first==+1 else -1))
        esc_back_first_servo  = select_servo_deg("BACKWARD", (+1 if dir_first==+1 else -1))
        esc_fwd_return_servo  = select_servo_deg("FORWARD",  (-1 if dir_first==+1 else +1))

    # robust pillar detector
    def detect_pillar(roi_bgr, roi_full_x0, gate_red_x, gate_green_x):
        roi_blur = cv.GaussianBlur(roi_bgr, (5,5), 0)
        hsv = cv.cvtColor(roi_blur, cv.COLOR_BGR2HSV)
        m_red = mask_hsv_ranges(hsv, cfg.get("RED", {}))
        m_grn = mask_hsv_ranges(hsv, cfg.get("GREEN", {}))

        er_it = max(0, int(TUNE["PILLAR_ERODE_ITER"]))
        di_it = max(0, int(TUNE["PILLAR_DILATE_ITER"]))
        gap   = max(0, int(TUNE["PILLAR_CLOSE_GAP_PX"]))
        k3 = np.ones((3,3), np.uint8)

        def refine(mask):
            if er_it > 0: mask = cv.erode(mask, k3, iterations=er_it)
            if gap > 0:  mask = cv.morphologyEx(mask, cv.MORPH_CLOSE,
                                                cv.getStructuringElement(cv.MORPH_RECT,(gap, gap)))
            if di_it > 0: mask = cv.dilate(mask, k3, iterations=di_it)
            return mask

        m_red = refine(m_red)
        m_grn = refine(m_grn)

        min_area = int(TUNE["PILLAR_MIN_AREA"])
        min_aspect = float(TUNE["PILLAR_MIN_ASPECT"])
        min_sol = float(TUNE["PILLAR_MIN_SOLIDITY"])

        def best_blob(mask):
            cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            best = None
            best_area = 0
            for c in cnts:
                x,y,wc,hc = cv.boundingRect(c)
                if wc <= 0 or hc <= 0:
                    continue
                area = wc*hc
                if area < min_area:
                    continue
                aspect = float(hc)/float(wc)
                if aspect < min_aspect:
                    continue
                cnt_area = float(cv.contourArea(c))
                solidity = (cnt_area/float(area)) if area>1 else 0.0
                if solidity < min_sol:
                    continue
                if area > best_area:
                    best_area = area
                    best = (x,y,wc,hc,area)
            return best

        red_blob = best_blob(m_red)
        grn_blob = best_blob(m_grn)

        pillar_seen, pillar_bbox, pillar_cx_frame, pillar_dist_cm = None, None, None, None
        cand = None
        col = None
        if red_blob is not None:
            cand, col = red_blob, "RED"
        if grn_blob is not None and (cand is None or grn_blob[4] > cand[4]):
            cand, col = grn_blob, "GREEN"
        if cand is not None:
            x,y,wc,hc,area = cand
            cx_frame = roi_full_x0 + x + wc/2.0
            gate_ok = (cx_frame >= gate_red_x) if col == "RED" else (cx_frame <= gate_green_x)
            if gate_ok:
                pillar_seen = col
                pillar_bbox = (x,y,wc,hc)
                pillar_cx_frame = cx_frame
                pillar_dist_cm = pillar_distance_cm_from_bbox(hc)
        return pillar_seen, pillar_bbox, pillar_cx_frame, pillar_dist_cm, red_blob, grn_blob, m_red, m_grn


    # helper: when should we watch encoder for stuck?
    def should_watch_encoder(st):
        """
        Return True if robot is supposed to be moving FORWARD under continuous drive,
        so if encoders stop changing we call E-WESC.
        We include normal run, escape forward steps, flip wait crawl, final hold,
        and the post12 forward push.
        We do NOT include pillar FSM FWD (that's encoder FWD_CM already)
        or the clean last-turn sequence, which owns its own bounded safety
        timeouts and must never launch an unrelated backward/forward recovery.
        """
        forward_states = {
            STATE_RUN,
            STATE_ESC_S1_FWD_L,
            STATE_ESC_S3_FWD_L,
            STATE_ESC_S3_FWD_HOLD,
            STATE_ESC_S4_FWD_RTN,
            STATE_FLIP_WAIT,
            STATE_FINAL_HOLD,
            STATE_POST12_FWD,
        }
        return (st in forward_states)

    # MAIN LOOP
    while True:
        now = time.time()
        # Preserve the state at loop entry. A maneuver that finishes later in
        # this iteration must not donate its last distance sample to RUN.
        loop_started_in_run = (state == STATE_RUN)
        pillar_active_at_loop_start = pillar_active
        preavoid_active_at_loop_start = preavoid_active
        enc_stuck_active_at_loop_start = enc_stuck_active
        maybe_ping(now)
        read_tlm()

        # Did Arduino reset itself? (READY seen again) -> reboot Jetson app to sync.
        if arduino_reset_request:
            restart_program("Arduino sent READY again (board reset button?)")

        # PONG only proves the serial link is alive. It cannot replace a yaw
        # sample. End an active run if headings stop arriving.
        if (not yaw_ready() and
                (state not in (STATE_WAIT_START, STATE_DONE, STATE_PARKED) or
                 pillar_active or preavoid_active or enc_stuck_active)):
            safe_write(b"STOP\n")
            print("[YAW] Valid yaw stopped arriving. Run stopped; check Arduino telemetry and restart.")
            if last_tlm_line:
                print("[YAW] Last TLM:", last_tlm_line[:250])
            break

        # initialize encoder-stuck reference once we have first enc read
        if not hasattr(main, "_enc_seeded"):
            enc_progress_cm_at = last_enc_cm
            enc_progress_t = time.time()
            main._enc_seeded = True

        # link watchdog
        if not link_alive(now) and (now - last_reset_t) >= TUNE["RESET_BACKOFF_SEC"]:
            print("[LINK] stale -> reset Arduino")
            reset_arduino()
            state = STATE_WAIT_START
            arduino_armed = False

        # autostart from Arduino ARMED signal
        if state == STATE_WAIT_START and arduino_armed and yaw_ready():
            start_yaw = float(last_yaw)
            heading_hold.reset()
            corner_line_rearm.reset()
            missed_corner_guard.reset()
            camera_failure_guard.reset()
            enc_back_cycles = 0
            enc_hold_reason = None
            enc_back_after_rear_seq = rear_range_clock.sequence
            enc_check_after_front_seq = front_range_clock.sequence
            enc_back_start_t = None
            corridor_idx = 0
            orientation_offset_deg = 0.0
            set_ref_to_corridor()
            run_start_time = time.time()
            run_end_time = None
            preavoid_available = True

            lastseq_active = False
            last_no_avoid_override = False
            pillar_suppress = False
            last_corner_sign = None
            last_corner_target = None
            last_parking_sign = None
            last_parking_target = None
            last_stage_start_t = -1.0
            last_center_target_cm = None
            last_turn_start_t = -1.0
            last_move_start_enc = None
            last_move_start_t = -1.0
            last_final_front_after_seq = front_range_clock.sequence
            last_final_stop_arm_until = -1.0
            last_wall_alignment_done = False
            wall_alignment.reset()

            if int(TUNE["ESCAPE_ENABLED"]):
                esc_seq_dir = decide_esc_seq_dir()
                build_escape_targets()
                esc_turn_start_t = time.time()
                esc_cmd_sent = False
                esc_waiting_dist = False
                state = STATE_ESC_S1_FWD_L
                dir_name = "RIGHT-first" if esc_seq_dir==+1 else "LEFT-first"
                print(f"[ESC] {dir_name}: S1->{esc_s1_target:.1f}°, S2->{esc_s2_target:.1f}°, "
                      f"S3->{esc_s3_target:.1f}°, S4->RTN {esc_s4_target:.1f}°")
            else:
                state = STATE_RUN
                print("[BUTTON] armed -> RUN; start_yaw=%.1f" % start_yaw)

        # frame grab
        ok, frame = cap.read()
        frame_ok = bool(ok)
        camera_motion_active = (
            state not in (STATE_WAIT_START, STATE_DONE, STATE_PARKED) or
            pillar_active or preavoid_active or enc_stuck_active)
        if camera_failure_guard.update(frame_ok, camera_motion_active):
            safe_write(b"STOP\n")
            print("[CAMERA] Repeated frame failures -> STOP and exit; "
                  "check the camera before restarting")
            break
        if not ok:
            frame = np.zeros((TUNE["FRAME_H"], TUNE["FRAME_W"], 3), dtype=np.uint8)
            cv.putText(frame, "Frame read failed", (10,30),
                       cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        h, w = frame.shape[:2]

        # DETECTION (RUN only)
        c_x0,c_y0,c_x1,c_y1 = color_roi_rect(w,h)
        roi_color = frame[c_y0:c_y1, c_x0:c_x1]
        k = normalized_odd_kernel(TUNE["BLUR_KSIZE"])
        roi_color_blur = (roi_color if k == 1 else
                          cv.GaussianBlur(roi_color, (k,k), 0))
        color_seen = None
        line_rect_blue = None
        line_rect_orange = None
        rejected_rect_blue = None
        rejected_rect_orange = None
        rejected_reason_blue = None
        rejected_reason_orange = None

        line_observation_valid = (
            frame_ok and now >= color_pause_until and detect_enabled and
            not pillar_active and not preavoid_active and
            state == STATE_RUN and not enc_stuck_active)

        if line_observation_valid:

            hsv = cv.cvtColor(roi_color_blur, cv.COLOR_BGR2HSV)
            if TUNE["SAT_BOOST"] != 1.0:
                hch, sch, vch = cv.split(hsv)
                sch = np.clip(sch.astype(np.float32) *
                              float(TUNE["SAT_BOOST"]), 0, 255).astype(np.uint8)
                hsv = cv.merge([hch, sch, vch])

            m_bl = mask_hsv_ranges(hsv, cfg.get("BLUE", {}))
            m_or = mask_hsv_ranges(hsv, cfg.get("ORANGE", {}))
            m_bl = apply_intersection_morphology(m_bl, cfg)
            m_or = apply_intersection_morphology(m_or, cfg)

            (rect_bl, pix_bl, rejected_rect_blue,
             rejected_reason_blue) = find_valid_intersection_rect(
                 m_bl, TUNE, diagnostics=True)
            (rect_or, pix_or, rejected_rect_orange,
             rejected_reason_orange) = find_valid_intersection_rect(
                 m_or, TUNE, diagnostics=True)
            marker_visible = (rect_bl is not None or rect_or is not None)
            detector_armed = update_corner_line_rearm(
                corner_line_rearm, frame_ok=line_observation_valid,
                marker_visible=marker_visible, now=now)
            if marker_visible:
                if pix_bl >= pix_or:
                    color_seen = "BLUE"
                    line_rect_blue = rect_bl
                else:
                    color_seen = "ORANGE"
                    line_rect_orange = rect_or
                if detector_armed:
                    color_q.append(color_seen)
                else:
                    color_q.clear()
            else:
                color_q.clear()
        else:
            # A dropout or maneuver breaks a partial clear-frame streak; it
            # cannot be used as evidence that the old line left the camera.
            update_corner_line_rearm(
                corner_line_rearm, frame_ok=line_observation_valid,
                marker_visible=False, now=now)
            color_q.clear()

        # PILLARS (RUN only)
        p_x0,p_y0,p_x1,p_y1 = pillar_roi_rect(w,h)
        roi_pillar = frame[p_y0:p_y1, p_x0:p_x1]

        red_gate_x   = int(w * TUNE["RED_LINE_X_FRAC"])
        green_gate_x = int(w * TUNE["GREEN_LINE_X_FRAC"])
        if red_gate_x > green_gate_x:
            red_gate_x, green_gate_x = green_gate_x, red_gate_x

        (pillar_seen, pillar_bbox, pillar_cx_frame, pillar_dist_cm,
         red_blob, grn_blob, m_red, m_grn) = detect_pillar(
            roi_bgr=roi_pillar,
            roi_full_x0=p_x0,
            gate_red_x=red_gate_x,
            gate_green_x=green_gate_x
        )
        gate_ok = (pillar_seen is not None)
        pillar_fsm_state_ok = pillar_fsm_state_allowed(state, STATE_RUN)

        # ALWAYS-ON: magnet walls
        hsv_p = cv.cvtColor(cv.GaussianBlur(roi_pillar, (5,5), 0), cv.COLOR_BGR2HSV)
        mag_mask_all = mask_hsv_ranges(hsv_p, cfg.get("MAGNET", {}))
        di_mag_all = max(0, int(TUNE.get("MAG_AVOID_DILATE_ITER", 2)))
        if di_mag_all > 0:
            mag_mask_all = cv.dilate(mag_mask_all, np.ones((3,3), np.uint8), iterations=di_mag_all)
        roi_w = (p_x1 - p_x0)
        band_w = max(2, int(roi_w * float(TUNE.get("MAG_AVOID_SAFE_FRAC", 0.08))))
        left_band_mask  = mag_mask_all[:, :band_w]
        right_band_mask = mag_mask_all[:, -band_w:]

        def band_blob_h(mask):
            cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            if not cnts:
                return None
            c = max(cnts, key=cv.contourArea)
            x,y,wc,hc = cv.boundingRect(c)
            area = wc*hc
            if area < int(TUNE.get("MAG_AVOID_MIN_AREA", 1800)):
                return None
            return hc

        lh = band_blob_h(left_band_mask)
        rh = band_blob_h(right_band_mask)
        mag_wall_left_cm  = (magnet_distance_cm_from_bbox(lh) if lh else None)
        mag_wall_right_cm = (magnet_distance_cm_from_bbox(rh) if rh else None)

        # PREAVOID TRIGGER
        if (
            int(TUNE["PREAVOID_ENABLED"]) and
            preavoid_available and
            (not pillar_active) and (not preavoid_active) and (not pillar_suppress) and
            (not enc_stuck_active) and
            state == STATE_RUN and gate_ok and (pillar_seen is not None) and (pillar_dist_cm is not None)
        ):
            if pillar_dist_cm < float(TUNE["PREAVOID_ON_CM"]):
                preavoid_active = True
                preavoid_state = "ACTIVE"
                preavoid_started_t = time.time()
                pillar_timer_start = preavoid_started_t if int(TUNE["PREAVOID_HUD_TIMER"]) else None
                last_seen_pillar_t = preavoid_started_t
                preavoid_color = pillar_seen
                preavoid_lost_frames = 0
                preavoid_last_bias_deg = 0.0
                pillar_suppress = True
                preavoid_available = False
                preavoid_enc_start_cm = last_enc_cm
                print(f"[PREAVOID] {pillar_seen} ~{pillar_dist_cm:.1f}cm -> dynamic steer-away ACTIVE")
                if turn_count >= 4 and first_pillar_after_first4 is None:
                    first_pillar_after_first4 = preavoid_color

        if preavoid_active and preavoid_state == "ACTIVE":
            pillar_visible_now = (pillar_seen is not None) and gate_ok
            steer_deg = compute_center_steer_deg(last_yaw)

            # direction away from pillar
            dir_sign = (+1 if preavoid_color == "RED" else -1)

            if pillar_visible_now and pillar_cx_frame is not None and pillar_dist_cm is not None:
                if preavoid_color == "RED":
                    pen = max(0.0, min(1.0,
                        (pillar_cx_frame - red_gate_x) /
                        max(1.0, (p_x1 - red_gate_x))
                    ))
                else:
                    pen = max(0.0, min(1.0,
                        (green_gate_x - pillar_cx_frame) /
                        max(1.0, (green_gate_x - p_x0))
                    ))
                dist_gain = max(0.25, min(1.0,
                              (float(TUNE["PREAVOID_ON_CM"]) /
                               max(1.0, pillar_dist_cm))))
                mag = (TUNE["PREAVOID_MIN_BIAS_DEG"] +
                       pen * (TUNE["PREAVOID_MAX_BIAS_DEG"] - TUNE["PREAVOID_MIN_BIAS_DEG"])) * dist_gain

                if int(TUNE["PREAVOID_USE_ENCODER"]) and (preavoid_enc_start_cm is not None):
                    travel = abs(last_enc_cm - preavoid_enc_start_cm)
                    ramp = max(0.40, min(1.0, travel /
                                 max(1.0, float(TUNE["PREAVOID_RAMP_CM"]))))
                    mag *= ramp
                bias = dir_sign * mag
                preavoid_last_bias_deg = bias
                preavoid_lost_frames = 0
            else:
                # fallback: keep last bias
                bias = preavoid_last_bias_deg if preavoid_last_bias_deg != 0.0 else (dir_sign * TUNE["PREAVOID_MAX_BIAS_DEG"])
                preavoid_lost_frames += 1

            # add wall emergency nudge if very close
            dL = sane_cm(last_US.get("dL", -1))
            dR = sane_cm(last_US.get("dR", -1))
            on_thr  = float(TUNE["WALL_AVOID_ON_CM"])
            nearL = (dL >= 0 and dL <= on_thr)
            nearR = (dR >= 0 and dR <= on_thr)
            if nearL or nearR:
                wall_sign = (+1 if nearL else -1)
                wall_mag = max(1.0, 0.5 * TUNE["WALL_AVOID_MIN_DEG"])
                bias += wall_sign * wall_mag

            steer_deg = int(max(LEFT_LIMIT, min(RIGHT_LIMIT, steer_deg + bias)))
            if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                send_center_deg(steer_deg, int(TUNE["PREAVOID_SPEED"]))
                last_tx = time.time()

            if int(TUNE["PREAVOID_USE_ENCODER"]) and (preavoid_enc_start_cm is not None):
                travel = abs(last_enc_cm - preavoid_enc_start_cm)
                if travel >= float(TUNE["PREAVOID_MAX_TRAVEL_CM"]):
                    preavoid_active = False
                    preavoid_state = "IDLE"
                    preavoid_post_suppress_until = time.time() + float(TUNE["PREAVOID_SUPPRESS_AFTER_S"])
                    pillar_timer_start = None
                    preavoid_color = None
                    preavoid_last_bias_deg = 0.0
                    print(f"[PREAVOID] Exit by encoder cap ({travel:.1f}cm) -> suppress briefly")
                else:
                    if travel < float(TUNE["PREAVOID_MIN_TRAVEL_CM_EXIT"]):
                        preavoid_lost_frames = 0

            if preavoid_active and preavoid_lost_frames >= int(TUNE["PREAVOID_LOST_CONFIRM"]):
                preavoid_active = False
                preavoid_state = "IDLE"
                preavoid_post_suppress_until = time.time() + float(TUNE["PREAVOID_SUPPRESS_AFTER_S"])
                pillar_timer_start = None
                preavoid_color = None
                preavoid_last_bias_deg = 0.0
                print("[PREAVOID] Pillar lost (confirmed) -> return; suppress briefly")

        if pillar_suppress and preavoid_post_suppress_until > 0 and time.time() >= preavoid_post_suppress_until:
            pillar_suppress = False
            preavoid_post_suppress_until = -1.0
            print("[PREAVOID] Post-suppress window elapsed -> pillar FSM re-enabled")

        # LEGACY PILLAR FSM TRIGGER
        if (not pillar_active) and (not preavoid_active) and (not pillar_suppress) and (not enc_stuck_active) and \
           (pillar_seen is not None) and (pillar_dist_cm is not None) and gate_ok and pillar_fsm_state_ok:
            pillar_q.append((pillar_seen, pillar_dist_cm, pillar_bbox))
        else:
            pillar_q.clear()

        # ENTER PILLAR FSM
        if (not pillar_active) and (not preavoid_active) and (not pillar_suppress) and \
           (not enc_stuck_active) and len(pillar_q) == TUNE["PILLAR_CONFIRM_FRAMES"] and pillar_fsm_state_ok:

            ps, pd, pb = pillar_q[0]
            if pd <= TUNE["PILLAR_ACCEPT_CM"]:
                pillar_active = True
                pillar_color = ps
                pillar_suppress = True
                corridor_yaw_before_pillar = wrap180(
                    start_yaw + corridor_idx * TUNE["TURN_DEG"] +
                    orientation_offset_deg
                )

                # latch lateral position at trigger (for dynamic forward distance)
                pillar_pre_cx = pillar_cx_frame
                if ps == "RED":
                    denom = max(1.0, (p_x1 - red_gate_x))
                    pillar_side_frac = max(0.0, min(1.0,
                        (pillar_pre_cx - red_gate_x) / denom))
                else:
                    denom = max(1.0, (green_gate_x - p_x0))
                    pillar_side_frac = max(0.0, min(1.0,
                        (green_gate_x - pillar_pre_cx) / denom))

                # Step 1: turn AWAY from pillar by PILLAR_TURN_DEG
                sign = +1 if ps == "RED" else -1
                target = wrap180(corridor_yaw_before_pillar + sign*TUNE["PILLAR_TURN_DEG"])
                pillar_turn_target = target
                pillar_turn_start_t = time.time()
                safe_write(f"TURN_ABS,{target:.1f},{TUNE['PILLAR_TURN_SPEED']}\n".encode("ascii"))
                pillar_state = "TURN"
                detect_enabled = False
                color_pause_until = now + 1.0

                pillar_fwd_cmd_sent = False
                pillar_fwd_waiting_dist = False

                print(f"[PILLAR] {ps} ~{pd:.1f}cm -> TURN "
                      f"{'RIGHT' if sign>0 else 'LEFT'} "
                      f"{TUNE['PILLAR_TURN_DEG']:.0f}° to {target:.1f}")
                if turn_count >= 4 and first_pillar_after_first4 is None:
                    first_pillar_after_first4 = pillar_color
                if turn_count == 4:
                    last_pillar_before_5th = pillar_color
                    if flip_policy_enable is None:
                        flip_policy_enable = 1 if pillar_color == "RED" else 0
                        print(f"[FLIP-POLICY] Last pillar before 5th (latched): {pillar_color} "
                              f"-> flip {'ENABLED' if flip_policy_enable else 'DISABLED'}")

        # WALL AVOID (RUN only)
        dF = sane_cm(last_US.get("dF", -1))
        dB = sane_cm(last_US.get("dB", -1))
        dL = sane_cm(last_US.get("dL", -1))
        dR = sane_cm(last_US.get("dR", -1))

        on_thr  = float(TUNE["WALL_AVOID_ON_CM"])
        off_thr = float(TUNE["WALL_AVOID_OFF_CM"])
        left_near  = (dL >= 0 and dL <= on_thr)
        right_near = (dR >= 0 and dR <= on_thr)

        if (not pillar_active) and (not preavoid_active) and (not enc_stuck_active) and state == STATE_RUN:
            if not avoid_active:
                if left_near and not right_near:
                    avoid_side = 'L'
                elif right_near and not left_near:
                    avoid_side = 'R'
                elif left_near and right_near:
                    avoid_side = 'L' if (dL <= dR) else 'R'
                else:
                    avoid_side = None
                if avoid_side is not None:
                    avoid_active = True
                    avoid_until = time.time() + float(TUNE["WALL_AVOID_HOLD_S"])
                    print(f"[AVOID] START side={avoid_side} (dL={dL:.1f}, dR={dR:.1f})")
            else:
                if (avoid_side == 'L' and left_near) or (avoid_side == 'R' and right_near):
                    avoid_until = time.time() + float(TUNE["WALL_AVOID_HOLD_S"])

                if left_near and right_near:
                    if dL + 1.0 < dR and avoid_side != 'L':
                        avoid_side = 'L'
                        avoid_until = time.time() + float(TUNE["WALL_AVOID_HOLD_S"])
                        print("[AVOID] FLIP -> L")
                    elif dR + 1.0 < dL and avoid_side != 'R':
                        avoid_side = 'R'
                        avoid_until = time.time() + float(TUNE["WALL_AVOID_HOLD_S"])
                        print("[AVOID] FLIP -> R")

                if time.time() >= avoid_until:
                    safe_again = (
                        (avoid_side == 'L' and (dL < 0 or dL >= off_thr)) or
                        (avoid_side == 'R' and (dR < 0 or dR >= off_thr))
                    )
                    if safe_again:
                        print("[AVOID] STOP (safe again)")
                        avoid_active = False
                        avoid_side = None

        # ENCODER STUCK DETECTION (GLOBAL FORWARD WATCH)
        watching_forward = should_watch_encoder(state)

        if watching_forward and (not pillar_active) and (not preavoid_active) and (not enc_stuck_active):
            # If we're supposed to be moving forward, check encoder progress.

            # Did encoder advance since last baseline? refresh baseline.
            if abs(last_enc_cm - enc_progress_cm_at) >= float(TUNE["STUCK_ENC_MIN_DELTA_CM"]):
                enc_progress_cm_at = last_enc_cm
                enc_progress_t = now

            if (now >= enc_cooldown_until):
                freeze_time = now - enc_progress_t
                if freeze_time >= float(TUNE["STUCK_ENC_FREEZE_S"]):
                    if abs(last_enc_cm - enc_progress_cm_at) < float(TUNE["STUCK_ENC_MIN_DELTA_CM"]):
                        # We are stuck -> trigger encoder-based wall escape
                        enc_stuck_active = True
                        enc_stuck_phase = "EBACK"
                        enc_cmd_sent = False
                        enc_waiting_dist = False
                        enc_back_cycles = 0
                        enc_hold_reason = None
                        enc_back_after_rear_seq = rear_range_clock.sequence
                        enc_check_after_front_seq = front_range_clock.sequence
                        enc_back_start_t = None
                        crash_recovery.latch(yaw_ref)
                        pillar_suppress = True
                        dist_done_flag = False  # clear any stale DIST_DONE so BACK_CM won't instantly exit
                        safe_write(b"STOP\n")
                        print(f"[E-WESC] TRIGGER (no encoder progress {freeze_time:.2f}s) -> "
                              f"BACK then return to saved yaw {crash_recovery.target(yaw_ref):.1f}°")

        else:
            # Not in a forward-driving state (or we're busy with pillar/preavoid/E-WESC):
            # reset the baseline timer so stale times don't false-trigger later.
            enc_progress_cm_at = last_enc_cm
            enc_progress_t = now

        # Refresh yaw after vision work, immediately before steering.
        read_tlm()
        # The second telemetry drain can contain a newer range packet. Use it
        # for recovery and the missed-corner guard in this same iteration.
        dF = sane_cm(last_US.get("dF", -1))
        dB = sane_cm(last_US.get("dB", -1))
        dL = sane_cm(last_US.get("dL", -1))
        dR = sane_cm(last_US.get("dR", -1))
        raw_front_cm = sane_cm(front_range_clock.value)
        raw_rear_cm = sane_cm(rear_range_clock.value)

        # CONTROL FSMs
        steer_deg = CENTER_DEG

        # ENCODER-BASED WALL ESCAPE (E-WESC)
        if enc_stuck_active:
            if enc_stuck_phase == "EBACK":
                if not enc_cmd_sent:
                    rear_status = recovery_rear_status(
                        raw_rear_cm, rear_range_clock.received_at,
                        time.monotonic(), TUNE,
                        sample_sequence=rear_range_clock.sequence,
                        after_sequence=enc_back_after_rear_seq)
                    if rear_status == "CLEAR":
                        sdeg = compute_center_steer_deg(last_yaw)
                        send_center_deg(sdeg, 0)
                        send_back_cm(float(TUNE["WESC_BACK_CM"]), int(TUNE["WESC_BACK_PWM"]))
                        enc_back_cycles += 1
                        enc_hold_reason = None
                        enc_cmd_sent = True
                        enc_waiting_dist = True
                        enc_back_start_t = time.monotonic()
                        print(f"[E-WESC] BACK_CM {TUNE['WESC_BACK_CM']}cm @ "
                              f"{TUNE['WESC_BACK_PWM']} (rear={raw_rear_cm:.1f}cm fresh)")
                    else:
                        hold_reason = f"rear={rear_status} ({raw_rear_cm:.1f}cm)"
                        if enc_hold_reason != hold_reason:
                            safe_write(b"STOP\n")
                            enc_hold_reason = hold_reason
                            print(f"[E-WESC] HOLD STOPPED before reverse: {hold_reason}")
                elif enc_waiting_dist:
                    if dist_done_flag:
                        dist_done_flag = False
                        enc_waiting_dist = False
                        enc_cmd_sent = False
                        enc_back_start_t = None
                        enc_stuck_phase = "ETURN"
                        print("[E-WESC] BACK done -> TURN to corridor yaw")
                    elif recovery_back_timed_out(
                            enc_back_start_t, time.monotonic(), TUNE):
                        safe_write(b"STOP\n")
                        enc_waiting_dist = False
                        enc_cmd_sent = False
                        enc_stuck_phase = "EFAULT"
                        enc_hold_reason = "reverse distance command timeout"
                        print("[E-WESC] FAULT STOPPED: reverse distance command timeout")

            elif enc_stuck_phase == "ETURN":
                target = crash_recovery.target(yaw_ref)
                if not enc_cmd_sent:
                    safe_write(f"TURN_ABS,{target:.1f},{int(TUNE['WESC_TURN_PWM'])}\n".encode("ascii"))
                    enc_cmd_sent = True
                    enc_turn_start_t = time.time()
                    print(f"[E-WESC] TURN_ABS to {target:.1f}° @ {TUNE['WESC_TURN_PWM']}")
                else:
                    done, hit, tmo = crash_recovery.status(
                        last_yaw, enc_turn_start_t, time.time(), yaw_ref)
                    if done:
                        safe_write(b"STOP\n")
                        enc_cmd_sent = False
                        enc_stuck_phase = "ECHECK"
                        enc_check_after_front_seq = front_range_clock.sequence
                        print(f"[E-WESC] TURN {'OK' if hit else 'TIMEOUT'} -> CHECK")

            elif enc_stuck_phase == "ECHECK":
                recovery_yaw = crash_recovery.target(yaw_ref)
                aligned = abs(shortest_err(recovery_yaw, last_yaw)) <= float(TUNE["WESC_EXIT_TOL_DEG"])
                front_status = recovery_front_status(
                    raw_front_cm, front_range_clock.received_at,
                    time.monotonic(), TUNE,
                    sample_sequence=front_range_clock.sequence,
                    after_sequence=enc_check_after_front_seq)
                rear_status = recovery_rear_status(
                    raw_rear_cm, rear_range_clock.received_at,
                    time.monotonic(), TUNE,
                    sample_sequence=rear_range_clock.sequence)
                recovery_action = choose_recovery_action(
                    front_status=front_status,
                    aligned=aligned,
                    back_cycles=enc_back_cycles,
                    rear_status=rear_status,
                    tune=TUNE,
                )

                if recovery_action == "RESUME":
                    yaw_ref = recovery_yaw
                    heading_hold.reset()
                    crash_recovery.clear()
                    enc_stuck_active = False
                    enc_stuck_phase = "EIDLE"
                    enc_back_cycles = 0
                    enc_hold_reason = None
                    pillar_suppress = False
                    enc_cooldown_until = time.time() + float(TUNE["WESC_COOLDOWN_S"])
                    enc_progress_cm_at = last_enc_cm
                    enc_progress_t = time.time()
                    safe_write(b"STOP\n")
                    fdeg = compute_center_steer_deg(last_yaw)
                    send_center_deg(fdeg, int(TUNE["DRIVE_SPEED"]))
                    last_tx = time.time()
                    if state != STATE_RUN:
                        state = STATE_RUN
                        print("[E-WESC] EXIT -> forced state RUN and sent forward CENTER to continue")
                    else:
                        print("[E-WESC] EXIT -> RUN (sent forward CENTER to continue)")
                elif recovery_action == "BACK":
                    enc_stuck_phase = "EBACK"
                    enc_cmd_sent = False
                    enc_waiting_dist = False
                    enc_hold_reason = None
                    # Require another rear packet after entering EBACK, so the
                    # command can never use a retained clearance value.
                    enc_back_after_rear_seq = rear_range_clock.sequence
                    print(f"[E-WESC] BOUNDED RETRY {enc_back_cycles + 1}/2 "
                          f"(front={raw_front_cm:.1f}cm rear={raw_rear_cm:.1f}cm)")
                else:
                    if not aligned:
                        hold_reason = "recovery yaw not aligned"
                    elif front_status in ("STALE", "INVALID"):
                        hold_reason = f"front sensor {front_status.lower()}"
                    elif enc_back_cycles >= min(2, max(
                            1, int(TUNE.get("WESC_MAX_BACK_CYCLES", 2)))):
                        hold_reason = "reverse retry limit reached"
                    else:
                        hold_reason = (f"rear clearance {rear_status.lower()} "
                                       f"({raw_rear_cm:.1f}cm)")
                    if enc_hold_reason != hold_reason:
                        safe_write(b"STOP\n")
                        enc_hold_reason = hold_reason
                        print(f"[E-WESC] HOLD STOPPED: {hold_reason}")

            elif enc_stuck_phase == "EFAULT":
                # STOP was already sent when the fault was latched. A manual
                # restart is required; never issue another motion command.
                pass

        # ESCAPE SEQUENCE (before main run)
        elif state == STATE_ESC_S1_FWD_L:
            if int(TUNE["ESC_USE_ENCODER"]):
                # encoder version with timeout fallback
                if not esc_cmd_sent:
                    send_center_abs_deg(esc_fwd_first_servo, 0)
                    send_fwd_cm(float(TUNE["ESC_S1_CM"]), int(TUNE["ESC_S1_PWM"]))
                    esc_cmd_sent = True
                    esc_waiting_dist = True
                    esc_turn_start_t = time.time()   # start timeout clock for S1
                    print(f"[ESC] S1 encoder: FWD {TUNE['ESC_S1_CM']}cm @ {TUNE['ESC_S1_PWM']}")
                else:
                    timed_out = (time.time() - esc_turn_start_t) >= float(TUNE["ESC_TURN_MAX_S"])
                    if esc_waiting_dist and (dist_done_flag or timed_out):
                        dist_done_flag = False
                        esc_waiting_dist = False
                        esc_cmd_sent = False
                        safe_write(b"STOP\n")
                        esc_turn_start_t = time.time()
                        state = STATE_ESC_S2_BACK_L
                        print("[ESC] S1 encoder %s -> S2" %
                              ("DONE" if not timed_out else "TIMEOUT"))
            else:
                tol = float(TUNE["ESC_TURN_TOL_DEG"])
                timed_out = (time.time() - esc_turn_start_t) >= float(TUNE["ESC_TURN_MAX_S"])
                if not at_yaw(esc_s1_target, last_yaw, tol) and not timed_out:
                    if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                        send_center_abs_deg(esc_fwd_first_servo,
                                            int(TUNE["ESC_TURN_SPEED_FWD"]))
                        last_tx = time.time()
                else:
                    safe_write(b"STOP\n")
                    esc_turn_start_t = time.time()
                    state = STATE_ESC_S2_BACK_L
                    print(f"[ESC] S1 {'TIMEOUT' if timed_out else 'OK'} -> S2 to {esc_s2_target:.1f}°")

        elif state == STATE_ESC_S2_BACK_L:
            if int(TUNE["ESC_USE_ENCODER"]):
                # encoder version with timeout fallback
                if not esc_cmd_sent:
                    send_center_abs_deg(esc_back_first_servo, 0)
                    send_back_cm(float(TUNE["ESC_S2_CM"]), int(TUNE["ESC_S2_PWM"]))
                    esc_cmd_sent = True
                    esc_waiting_dist = True
                    esc_turn_start_t = time.time()   # timeout clock for S2
                    print(f"[ESC] S2 encoder: BACK {TUNE['ESC_S2_CM']}cm @ {TUNE['ESC_S2_PWM']}")
                else:
                    timed_out = (time.time() - esc_turn_start_t) >= float(TUNE["ESC_TURN_MAX_S"])
                    if esc_waiting_dist and (dist_done_flag or timed_out):
                        dist_done_flag = False
                        esc_waiting_dist = False
                        esc_cmd_sent = False
                        safe_write(b"STOP\n")
                        esc_turn_start_t = time.time()
                        state = STATE_ESC_S3_FWD_L
                        print("[ESC] S2 encoder %s -> S3" %
                              ("DONE" if not timed_out else "TIMEOUT"))
            else:
                tol = float(TUNE["ESC_TURN_TOL_DEG"])
                timed_out = (time.time() - esc_turn_start_t) >= float(TUNE["ESC_TURN_MAX_S"])
                if not at_yaw(esc_s2_target, last_yaw, tol) and not timed_out:
                    if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                        send_backc_deg(esc_back_first_servo,
                                       int(TUNE["ESC_TURN_SPEED_BACK"]))
                        last_tx = time.time()
                else:
                    safe_write(b"STOP\n")
                    esc_turn_start_t = time.time()
                    state = STATE_ESC_S3_FWD_L
                    print(f"[ESC] S2 {'TIMEOUT' if timed_out else 'OK'} -> S3 to {esc_s3_target:.1f}°")

        elif state == STATE_ESC_S3_FWD_L:
            if int(TUNE["ESC_USE_ENCODER"]):
                # encoder version with timeout fallback
                if not esc_cmd_sent:
                    send_center_abs_deg(esc_fwd_first_servo, 0)
                    send_fwd_cm(float(TUNE["ESC_S3_CM"]), int(TUNE["ESC_S3_PWM"]))
                    esc_cmd_sent = True
                    esc_waiting_dist = True
                    esc_turn_start_t = time.time()   # timeout clock for S3
                    print(f"[ESC] S3 encoder: FWD {TUNE['ESC_S3_CM']}cm @ {TUNE['ESC_S3_PWM']}")
                else:
                    timed_out = (time.time() - esc_turn_start_t) >= float(TUNE["ESC_TURN_MAX_S"])
                    if esc_waiting_dist and (dist_done_flag or timed_out):
                        dist_done_flag = False
                        esc_waiting_dist = False
                        esc_cmd_sent = False
                        safe_write(b"STOP\n")
                        esc_s3_hold_until = time.time() + float(TUNE["ESC_S3_HOLD_S"])
                        state = STATE_ESC_S3_FWD_HOLD
                        print(f"[ESC] S3 encoder %s -> HOLD fwd {TUNE['ESC_S3_HOLD_S']:.2f}s then S4"
                              % ("DONE" if not timed_out else "TIMEOUT"))
            else:
                tol = float(TUNE["ESC_TURN_TOL_DEG"])
                timed_out = (time.time() - esc_turn_start_t) >= float(TUNE["ESC_TURN_MAX_S"])
                if not at_yaw(esc_s3_target, last_yaw, tol) and not timed_out:
                    if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                        send_center_abs_deg(esc_fwd_first_servo,
                                            int(TUNE["ESC_TURN_SPEED_FWD"]))
                        last_tx = time.time()
                else:
                    safe_write(b"STOP\n")
                    esc_s3_hold_until = time.time() + float(TUNE["ESC_S3_HOLD_S"])
                    state = STATE_ESC_S3_FWD_HOLD
                    print(f"[ESC] S3 {'TIMEOUT' if timed_out else 'OK'} -> HOLD fwd {TUNE['ESC_S3_HOLD_S']:.2f}s then S4 RTN to {esc_s4_target:.1f}°")

        elif state == STATE_ESC_S3_FWD_HOLD:
            steer_deg = compute_center_steer_deg(last_yaw)
            if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                send_center_deg(steer_deg, int(TUNE["ESC_S3_HOLD_SPEED"]))
                last_tx = time.time()
            if time.time() >= esc_s3_hold_until:
                safe_write(b"STOP\n")
                esc_turn_start_t = time.time()
                esc_cmd_sent = False
                esc_waiting_dist = False
                state = STATE_ESC_S4_FWD_RTN
                print(f"[ESC] S3 HOLD done -> S4 RTN to {esc_s4_target:.1f}°")

        elif state == STATE_ESC_S4_FWD_RTN:
            if int(TUNE["ESC_USE_ENCODER"]):
                # encoder version with timeout fallback
                if not esc_cmd_sent:
                    send_center_abs_deg(esc_fwd_return_servo, 0)
                    send_fwd_cm(float(TUNE["ESC_S4_CM"]), int(TUNE["ESC_S4_PWM"]))
                    esc_cmd_sent = True
                    esc_waiting_dist = True
                    esc_turn_start_t = time.time()   # timeout clock for S4
                    print(f"[ESC] S4 encoder: FWD {TUNE['ESC_S4_CM']}cm @ {TUNE['ESC_S4_PWM']}")
                else:
                    timed_out = (time.time() - esc_turn_start_t) >= float(TUNE["ESC_TURN_MAX_S"])
                    if esc_waiting_dist and (dist_done_flag or timed_out):
                        dist_done_flag = False
                        esc_waiting_dist = False
                        esc_cmd_sent = False
                        safe_write(b"STOP\n")
                        detect_enabled = True
                        color_pause_until = time.time() + 0.5
                        state = STATE_RUN
                        preavoid_available = True
                        print("[ESC] S4 encoder %s -> START RUN (preavoid armed)" %
                              ("DONE" if not timed_out else "TIMEOUT"))
            else:
                tol = float(TUNE["ESC_RTN_TOL_DEG"])
                timed_out = (time.time() - esc_turn_start_t) >= float(TUNE["ESC_TURN_MAX_S"])
                if not at_yaw(esc_s4_target, last_yaw, tol) and not timed_out:
                    if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                        send_center_abs_deg(esc_fwd_return_servo,
                                            int(TUNE["ESC_TURN_SPEED_FWD"]))
                        last_tx = time.time()
                else:
                    safe_write(b"STOP\n")
                    detect_enabled = True
                    color_pause_until = time.time() + 0.5
                    state = STATE_RUN
                    preavoid_available = True
                    print("[ESC] S4 complete -> START RUN (preavoid armed)")

        # PILLAR FLOW
        elif pillar_active:
            if pillar_state == "TURN":
                tol = float(TUNE["PILLAR_TURN_TOL_DEG"])
                done, hit, tmo = reached_or_timeout(
                    pillar_turn_target, pillar_turn_start_t, tol,
                    TUNE["PILLAR_TURN_MAX_S"], last_yaw
                )
                if done:
                    yaw_ref = (pillar_turn_target if pillar_turn_target is not None else
                               wrap180(corridor_yaw_before_pillar +
                                       (+1 if pillar_color=="RED" else -1) *
                                       TUNE["PILLAR_TURN_DEG"]))
                    safe_write(b"STOP\n")
                    pillar_state = "FWD"
                    pillar_fwd_cmd_sent = False
                    pillar_fwd_waiting_dist = False
                    print(f"[PILLAR] turn {'TIMEOUT' if tmo else 'DONE'} -> FWD encoder "
                          f"(yaw_ref={yaw_ref:.1f})")

            elif pillar_state == "FWD":
                if int(TUNE["PILLAR_FWD_USE_ENCODER"]):
                    if not pillar_fwd_cmd_sent:
                        raw_cm = float(TUNE["PILLAR_FWD_BASE_CM"]) + \
                                 float(TUNE["PILLAR_FWD_GAIN_CM"]) * float(pillar_side_frac)
                        fwd_cm = max(float(TUNE["PILLAR_FWD_MIN_CM"]),
                                     min(float(TUNE["PILLAR_FWD_MAX_CM"]), raw_cm))
                        sdeg = compute_center_steer_deg(last_yaw)
                        send_center_deg(sdeg, 0)
                        send_fwd_cm(fwd_cm, int(TUNE["PILLAR_FWD_SPEED"]))
                        pillar_fwd_cmd_sent = True
                        pillar_fwd_waiting_dist = True
                        print(f"[PILLAR] FWD_CM {fwd_cm:.1f}cm (side_frac={pillar_side_frac:.2f}, "
                              f"base={TUNE['PILLAR_FWD_BASE_CM']}, "
                              f"gain={TUNE['PILLAR_FWD_GAIN_CM']})")
                    elif pillar_fwd_waiting_dist and dist_done_flag:
                        dist_done_flag = False
                        pillar_fwd_waiting_dist = False
                        pillar_state = "RETURN"
                        target = wrap180(corridor_yaw_before_pillar)
                        pillar_turn_target = target
                        pillar_turn_start_t = time.time()
                        safe_write(f"TURN_ABS,{target:.1f},{TUNE['PILLAR_TURN_SPEED']}\n".encode("ascii"))
                        print(f"[PILLAR] FWD done -> RETURN turn to corridor {target:.1f}")
                else:
                    steer_deg = compute_center_steer_deg(last_yaw)
                    if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                        send_center_deg(steer_deg, TUNE["PILLAR_FWD_SPEED"])
                        last_tx = time.time()
                    if not hasattr(main, "_pfwd_t0"):
                        main._pfwd_t0 = time.time()
                    if time.time() - main._pfwd_t0 >= 0.60:
                        main._pfwd_t0 = None
                        pillar_state = "RETURN"
                        target = wrap180(corridor_yaw_before_pillar)
                        pillar_turn_target = target
                        pillar_turn_start_t = time.time()
                        safe_write(f"TURN_ABS,{target:.1f},{TUNE['PILLAR_TURN_SPEED']}\n".encode("ascii"))
                        print(f"[PILLAR] FWD timeout -> RETURN turn to corridor {target:.1f}")

            elif pillar_state == "RETURN":
                tol = float(TUNE["PILLAR_TURN_TOL_DEG"])
                done, hit, tmo = reached_or_timeout(
                    pillar_turn_target, pillar_turn_start_t, tol,
                    TUNE["PILLAR_TURN_MAX_S"], last_yaw
                )
                if done:
                    safe_write(b"STOP\n")
                    set_ref_to_corridor()
                    pillar_state = "IDLE"
                    pillar_active = False
                    detect_enabled = True
                    see_color_slow = False
                    pillar_timer_start = None
                    pillar_suppress = False
                    print("[PILLAR] sequence complete -> resume state (yaw_ref=%.1f)" % yaw_ref)

                    if waiting_first_pillar_after_turn8 and flip_enabled_now():
                        waiting_first_pillar_after_turn8 = False
                        flip_scheduled_color = pillar_color
                        flip_wait_until = time.time() + float(TUNE["FLIP_FWD_HOLD_S"])
                        state = STATE_FLIP_WAIT
                        print(f"[FLIP] Arm flip. First pillar after turn8 was {flip_scheduled_color}. "
                              f"Hold fwd {TUNE['FLIP_FWD_HOLD_S']:.1f}s.")

        # CORNER FLOW
        elif state == STATE_RUN and corner_color is not None:
            next_corner_num = turn_count + 1
            FRONT_THR = front_turn_cm_for(next_corner_num)
            front_status = corner_front_status(
                raw_front_cm,
                front_range_clock.received_at,
                time.monotonic(),
                FRONT_THR,
                TUNE["FRONT_TURN_MAX_AGE_S"],
            )
            front_confirmed = corner_distance_checker.update(
                raw_front_cm,
                front_range_clock.received_at,
                time.monotonic(),
                FRONT_THR,
                TUNE["FRONT_TURN_MAX_AGE_S"],
                sample_seq=front_range_clock.sequence,
            )
            if corner_distance_checker.last_update_was_new:
                print(f"[CORNER-CHECK] turn#{next_corner_num} front={raw_front_cm:.1f}cm "
                      f"threshold={FRONT_THR:.1f}cm status={corner_distance_checker.last_status} "
                      f"confirm={corner_distance_checker.confirmed_samples}/"
                      f"{corner_distance_checker.required_samples}")
            if front_confirmed:
                # last-corner custom path or normal corner path
                if int(TUNE["LAST_ENABLED"]) and next_corner_num == int(TUNE["MAX_TURNS"]):
                    last_plan = build_last_turn_plan(
                        yaw_ref,
                        corner_color if corner_source == "LINE" else None,
                        TUNE["LAST_CORNER_TURN_DEG"],
                        fallback_sign=current_corner_dir,
                        parking_turn_deg=TUNE["LAST_PARK_TURN_DEG"],
                    )
                    if last_plan is None:
                        safe_write(b"STOP\n")
                        state = STATE_DONE
                        run_end_time = time.time()
                        print("[LAST] Cannot determine last intersection direction "
                              "-> safe STOP (NOT PARKED)")
                    else:
                        (last_corner_sign, last_corner_target,
                         last_parking_sign, last_parking_target) = last_plan
                        current_corner_dir = last_corner_sign
                        target_idx = corridor_idx + last_corner_sign
                        intended_corner_yaw = last_corner_target
                        lastseq_active = True
                        last_wall_alignment_done = False
                        last_no_avoid_override = True
                        preavoid_post_suppress_until = -1.0
                        pillar_suppress = True
                        last_stage_start_t = time.time()
                        state = STATE_LAST_APPROACH_5
                        print(f"[LAST] {corner_color} intersection -> approach wall until "
                              f"front < {TUNE['LAST_PRETURN_FRONT_CM']:.1f}cm; "
                              f"corner will turn {'LEFT' if last_corner_sign < 0 else 'RIGHT'} "
                              f"to {last_corner_target:.1f}°")
                else:
                    state = STATE_CORNER_TURNING
                    corner_turn_start_t = time.time()
                    print(f"[CORNER] start {current_corner_mode} turn toward yaw={intended_corner_yaw:.1f} "
                          f"(front={raw_front_cm:.1f}cm < {FRONT_THR:.1f}cm, "
                          f"turn#{next_corner_num})")
            elif front_status in ("WAIT", "REACHED"):
                steer_deg = compute_center_steer_deg(last_yaw)

                if (avoid_active and not pillar_active and not preavoid_active) and (not last_no_avoid_override):
                    bias_sign = (-1 if avoid_side == 'L' else +1)
                    if int(TUNE["AVOID_INVERT_DIR"]):
                        bias_sign *= -1
                    near_d = dL if avoid_side=='L' else dR
                    if near_d < 0:
                        near_d = on_thr
                    prox = max(0.0, min(1.0, (on_thr - near_d) / max(1e-3, on_thr)))
                    mag = (prox ** float(TUNE["WALL_AVOID_EXP"])) * \
                          float(TUNE["WALL_AVOID_KP_DEG_PER_CM"]) * \
                          float(TUNE["WALL_AVOID_GAIN"])
                    mag = max(float(TUNE["WALL_AVOID_MIN_DEG"]),
                              min(float(TUNE["WALL_AVOID_MAX_BIAS_DEG"]), mag))
                    steer_deg = max(LEFT_LIMIT, min(RIGHT_LIMIT, steer_deg + bias_sign * mag))
                    spd = int(TUNE["WALL_AVOID_SPEED"])
                else:
                    spd = TUNE["DRIVE_SPEED"]
                    min_side = min(x for x in (dL, dR) if x >= 0) if (dL >= 0 or dR >= 0) else -1
                    if not hasattr(main, "_side_slow"):
                        main._side_slow = False
                    if min_side >= 0:
                        if main._side_slow:
                            if min_side > TUNE["WALL_SLOW_SIDE_OFF_CM"]:
                                main._side_slow = False
                        else:
                            if min_side < TUNE["WALL_SLOW_SIDE_ON_CM"]:
                                main._side_slow = True
                    if main._side_slow and (not last_no_avoid_override):
                        spd = max(int(spd * TUNE["SIDE_SLOWDOWN_FACTOR"]),
                                  int(TUNE["WALL_SLOW_MIN_SPEED"]))
                    if dF >= 0 and dF < TUNE["WALL_SLOW_FRONT_ON_CM"] and (not last_no_avoid_override):
                        spd = max(int(spd * TUNE["FRONT_SLOWDOWN_FACTOR"]),
                                  int(TUNE["WALL_SLOW_MIN_SPEED"]))

                # magnet wall nudge (if not last seq override)
                if not last_no_avoid_override:
                    mag_thr = float(TUNE["MAG_AVOID_ON_CM"])
                    left_mag_near  = (mag_wall_left_cm  is not None and mag_wall_left_cm  <= mag_thr)
                    right_mag_near = (mag_wall_right_cm is not None and mag_wall_right_cm <= mag_thr)
                    if left_mag_near or right_mag_near:
                        if left_mag_near and right_mag_near:
                            side = 'L' if mag_wall_left_cm <= mag_wall_right_cm else 'R'
                            near_d_mag = min(mag_wall_left_cm, mag_wall_right_cm)
                        elif left_mag_near:
                            side, near_d_mag = 'L', mag_wall_left_cm
                        else:
                            side, near_d_mag = 'R', mag_wall_right_cm
                        bias_sign = (-1 if side == 'L' else +1)
                        if int(TUNE["AVOID_INVERT_DIR"]):
                            bias_sign *= -1
                        prox = max(0.0, min(1.0, (mag_thr - near_d_mag) / max(1e-3, mag_thr)))
                        mag = (prox ** float(TUNE["MAG_AVOID_EXP"])) * \
                              float(TUNE["MAG_AVOID_KP_DEG_PER_CM"]) * \
                              float(TUNE["MAG_AVOID_GAIN"])
                        mag = max(float(TUNE["MAG_AVOID_MIN_DEG"]),
                                  min(float(TUNE["MAG_AVOID_MAX_BIAS_DEG"]), mag))
                        steer_deg = max(LEFT_LIMIT,
                                        min(RIGHT_LIMIT, steer_deg + bias_sign * mag))

                if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    send_center_deg(steer_deg, spd)
                    last_tx = time.time()
            else:
                # A latched corner may only continue forward with a fresh,
                # valid raw front range. Hold position until telemetry returns.
                if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    safe_write(b"STOP\n")
                    last_tx = time.time()

        elif state == STATE_RUN:
            steer_deg = compute_center_steer_deg(last_yaw)

            if (avoid_active and not pillar_active and not preavoid_active) and (not last_no_avoid_override):
                bias_sign = (-1 if avoid_side == 'L' else +1)
                if int(TUNE["AVOID_INVERT_DIR"]):
                    bias_sign *= -1
                near_d = dL if avoid_side=='L' else dR
                if near_d < 0:
                    near_d = on_thr
                prox = max(0.0, min(1.0, (on_thr - near_d) / max(1e-3, on_thr)))
                mag = (prox ** float(TUNE["WALL_AVOID_EXP"])) * \
                      float(TUNE["WALL_AVOID_KP_DEG_PER_CM"]) * \
                      float(TUNE["WALL_AVOID_GAIN"])
                mag = max(float(TUNE["WALL_AVOID_MIN_DEG"]),
                          min(float(TUNE["WALL_AVOID_MAX_BIAS_DEG"]), mag))
                steer_deg = max(LEFT_LIMIT, min(RIGHT_LIMIT, steer_deg + bias_sign * mag))
                spd = int(TUNE["WALL_AVOID_SPEED"])
            else:
                spd = TUNE["DRIVE_SPEED"]
                min_side = min(x for x in (dL, dR) if x >= 0) if (dL >= 0 or dR >= 0) else -1
                if not hasattr(main, "_side_slow"):
                    main._side_slow = False
                if min_side >= 0:
                    if main._side_slow:
                        if min_side > TUNE["WALL_SLOW_SIDE_OFF_CM"]:
                            main._side_slow = False
                    else:
                        if min_side < TUNE["WALL_SLOW_SIDE_ON_CM"]:
                            main._side_slow = True
                if main._side_slow and (not last_no_avoid_override):
                    spd = max(int(spd * TUNE["SIDE_SLOWDOWN_FACTOR"]),
                              int(TUNE["WALL_SLOW_MIN_SPEED"]))
                if dF >= 0 and dF < TUNE["WALL_SLOW_FRONT_ON_CM"] and (not last_no_avoid_override):
                    spd = max(int(spd * TUNE["FRONT_SLOWDOWN_FACTOR"]),
                              int(TUNE["WALL_SLOW_MIN_SPEED"]))

            # magnet wall nudge
            if not last_no_avoid_override:
                mag_thr = float(TUNE["MAG_AVOID_ON_CM"])
                left_mag_near  = (mag_wall_left_cm  is not None and mag_wall_left_cm  <= mag_thr)
                right_mag_near = (mag_wall_right_cm is not None and mag_wall_right_cm <= mag_thr)
                if left_mag_near or right_mag_near:
                    if left_mag_near and right_mag_near:
                        side = 'L' if mag_wall_left_cm <= mag_wall_right_cm else 'R'
                        near_d_mag = min(mag_wall_left_cm, mag_wall_right_cm)
                    elif left_mag_near:
                        side, near_d_mag = 'L', mag_wall_left_cm
                    else:
                        side, near_d_mag = 'R', mag_wall_right_cm
                    bias_sign = (-1 if side == 'L' else +1)
                    if int(TUNE["AVOID_INVERT_DIR"]):
                        bias_sign *= -1
                    prox = max(0.0, min(1.0, (mag_thr - near_d_mag) / max(1e-3, mag_thr)))
                    mag = (prox ** float(TUNE["MAG_AVOID_EXP"])) * \
                          float(TUNE["MAG_AVOID_KP_DEG_PER_CM"]) * \
                          float(TUNE["MAG_AVOID_GAIN"])
                    mag = max(float(TUNE["MAG_AVOID_MIN_DEG"]),
                              min(float(TUNE["MAG_AVOID_MAX_BIAS_DEG"]), mag))
                    steer_deg = max(LEFT_LIMIT,
                                    min(RIGHT_LIMIT, steer_deg + bias_sign * mag))

            if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                send_center_deg(steer_deg, spd)
                last_tx = time.time()

        elif state == STATE_CORNER_TURNING:
            tol = float(TUNE["TURN_TOL_DEG"])
            done, hit, tmo = reached_or_timeout(
                intended_corner_yaw,
                corner_turn_start_t,
                tol,
                TUNE["CORNER_TURN_MAX_S"],
                last_yaw
            )
            if not done:
                fixed_deg = select_servo_deg(current_corner_mode, current_corner_dir)
                if current_corner_mode == "BACKWARD":
                    if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                        send_backc_deg(fixed_deg, TUNE["TURN_SPEED"])
                        last_tx = time.time()
                else:
                    if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                        send_center_abs_deg(fixed_deg, TUNE["TURN_SPEED"])
                        last_tx = time.time()
            else:
                safe_write(b"STOP\n")
                old_idx = corridor_idx
                corridor_idx = target_idx
                set_ref_to_corridor()

                # After first corner, latch direction etc.
                if int(TUNE["LOCK_ALL_12_TURNS_ONE_DIRECTION"]) and not dir_locked:
                    if force_all_dir is None:
                        used_dir = +1 if (corridor_idx - old_idx) > 0 else -1
                        force_all_dir = used_dir
                    dir_locked = True
                    print(f"[DIR] LOCKED all 12: {'RIGHT(+1)' if force_all_dir==+1 else 'LEFT(-1)'}")

                if hit and current_corner_mode == "BACKWARD":
                    state = STATE_CORNER_BACKCENTER
                    print(f"[CORNER] reached target (tol {tol:.1f}) -> BACKWARD centering "
                          f"(yaw_ref={yaw_ref:.1f})")
                else:
                    corner_color = None
                    corner_source = None
                    see_color_slow = False
                    color_pause_until = -1.0
                    missed_corner_guard.reset()
                    pillar_suppress = False
                    state = STATE_RUN
                    turn_count += 1
                    preavoid_available = True
                    reason = "TIMEOUT" if tmo else "OK"
                    print(f"[CORNER] Commit ({reason}) -> RUN (turns={turn_count})")

                    if turn_count >= TUNE["MAX_TURNS"] and run_end_time is None:
                        run_end_time = time.time()

                    if turn_count == 8 and flip_enabled_now():
                        waiting_first_pillar_after_turn8 = True
                        print(f"[SEQ] 8 turns done. Wait first pillar then FLIP by "
                              f"{TUNE['FLIP_TURN_DEG']:.0f}°.")

                    if turn_count >= TUNE["MAX_TURNS"]:
                        final_hold_until = time.time() + float(TUNE["FINAL_HOLD_S"])
                        state = STATE_FINAL_HOLD
                        print(f"[FINAL] Starting forward-centering hold for "
                              f"{TUNE['FINAL_HOLD_S']:.1f}s")

        elif state == STATE_CORNER_BACKCENTER:
            steer_deg = compute_center_steer_deg(last_yaw)
            if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                send_backc_deg(steer_deg, TUNE["BACK_STOP_SPEED"])
                last_tx = time.time()
            if dB >= 0 and dB < TUNE["BACK_STOP_CM"]:
                safe_write(b"STOP\n")
                corner_color = None
                corner_source = None
                see_color_slow = False
                color_pause_until = -1.0
                missed_corner_guard.reset()
                pillar_suppress = False
                state = STATE_RUN
                turn_count += 1
                preavoid_available = True
                print(f"[CORNER] Back-center stop. turn_count={turn_count}")

                if turn_count >= TUNE["MAX_TURNS"] and run_end_time is None:
                    run_end_time = time.time()

                if turn_count == 8 and flip_enabled_now():
                    waiting_first_pillar_after_turn8 = True
                    print(f"[SEQ] 8 turns done. Wait first pillar then FLIP by "
                          f"{TUNE['FLIP_TURN_DEG']:.0f}°.")

                if turn_count >= TUNE["MAX_TURNS"]:
                    final_hold_until = time.time() + float(TUNE["FINAL_HOLD_S"])
                    state = STATE_FINAL_HOLD
                    print(f"[FINAL] Starting forward-centering hold for "
                          f"{TUNE['FINAL_HOLD_S']:.1f}s")

        # Last-turn parking sequence
        elif state == STATE_LAST_APPROACH_5:
            front_status = last_front_stage_status(
                front_range_clock.value,
                front_range_clock.received_at,
                time.monotonic(),
                TUNE["LAST_PRETURN_FRONT_CM"],
                TUNE["FRONT_TURN_MAX_AGE_S"],
                False,
            )
            front_action = last_front_stage_action(
                front_status,
                time.time() - last_stage_start_t,
                TUNE["LAST_APPROACH_5_MAX_S"],
            )
            if front_action == "FAIL":
                finish_last_sequence(False, "approach to 5cm timed out")
            elif front_action == "COMPLETE":
                safe_write(b"STOP\n")
                last_move_start_enc = None
                last_move_start_t = -1.0
                last_stage_start_t = time.time()
                heading_hold.reset()
                state = STATE_LAST_BACK_CENTER
                print(f"[LAST] Front < {TUNE['LAST_PRETURN_FRONT_CM']:.1f}cm -> "
                      f"yaw-centred reverse {TUNE['LAST_BACK_ENC_CM']:.1f}cm")
            elif front_action == "MOVE":
                steer_deg = compute_center_steer_deg(last_yaw)
                if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    send_center_deg(steer_deg, int(TUNE["LAST_APPROACH_5_PWM"]))
                    last_tx = time.time()
            elif (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                safe_write(b"STOP\n")
                last_tx = time.time()

        elif state == STATE_LAST_BACK_CENTER:
            if last_move_start_enc is None:
                if time.time() - last_stage_start_t >= float(TUNE["LAST_BACK_MAX_S"]):
                    finish_last_sequence(False, "reverse encoder telemetry unavailable")
                elif encoder_ready():
                    last_move_start_enc = float(last_enc_cm)
                    last_move_start_t = time.time()
                    heading_hold.reset()
                elif (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    safe_write(b"STOP\n")
                    last_tx = time.time()
            else:
                move_action = last_encoder_action(
                    last_move_start_enc, last_enc_cm,
                    TUNE["LAST_BACK_ENC_CM"],
                    time.time() - last_move_start_t,
                    TUNE["LAST_BACK_MAX_S"],
                    telemetry_fresh=encoder_ready(),
                )
                if move_action == "COMPLETE":
                    safe_write(b"STOP\n")
                    last_turn_start_t = time.time()
                    state = STATE_LAST_CORNER_TURN
                    print(f"[LAST] Reverse complete -> turn "
                          f"{'LEFT' if last_corner_sign < 0 else 'RIGHT'} "
                          f"to {last_corner_target:.1f}°")
                elif move_action == "FAIL":
                    finish_last_sequence(False, "yaw-centred reverse failed or timed out")
                elif (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    steer_deg = compute_center_steer_deg(last_yaw)
                    send_backc_deg(steer_deg, int(TUNE["LAST_BACK_PWM"]))
                    last_tx = time.time()

        elif state == STATE_LAST_CORNER_TURN:
            turn_elapsed = time.time() - last_turn_start_t
            yaw_fresh = yaw_ready()
            yaw_hit = yaw_fresh and at_yaw(
                last_corner_target, last_yaw, TUNE["LAST_CORNER_TURN_TOL_DEG"]
            )
            turn_action = last_turn_action(
                yaw_hit,
                turn_elapsed >= float(TUNE["LAST_CORNER_TURN_MAX_S"]),
                yaw_fresh,
            )
            if turn_action == "COMPLETE":
                safe_write(b"STOP\n")
                corridor_idx = target_idx
                turn_count += 1
                yaw_ref = last_corner_target
                heading_hold.reset()
                corner_color = None
                corner_source = None
                see_color_slow = False
                preavoid_post_suppress_until = -1.0
                pillar_suppress = True
                pillar_q.clear()
                wall_alignment.start(time.monotonic(), rear_range_clock.sequence)
                last_wall_action = None
                state = STATE_LAST_WALL_ALIGN
                # Remove the turning servo command now. Reverse starts as
                # soon as the next fresh rear sample arrives, without a
                # forward centering move between the turn and the wall step.
                send_center_abs_deg(CENTER_DEG, 0)
                last_tx = 0.0
                print(f"[LAST-WALL] Corridor turn reached {yaw_ref:.1f}° -> "
                      f"straight reverse until rear < {TUNE['LAST_WALL_REAR_CM']:.1f}cm, "
                      f"then push {TUNE['LAST_WALL_PRESS_S']:.2f}s")
            elif turn_action == "FAIL":
                finish_last_sequence(False, "last corridor turn timed out before target yaw")
            elif turn_action == "WAIT_SENSOR":
                if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    safe_write(b"STOP\n")
                    last_tx = time.time()
            elif (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                fixed_deg = select_servo_deg("FORWARD", last_corner_sign)
                send_center_abs_deg(fixed_deg, TUNE["LAST_CORNER_TURN_SPEED"])
                last_tx = time.time()

        elif state == STATE_LAST_WALL_ALIGN:
            previous_phase = wall_alignment.phase
            wall_action = wall_alignment.step(
                rear_range_clock.value, rear_range_clock.received_at,
                rear_range_clock.sequence, last_yaw, last_yaw_t,
                time.monotonic(),
            )
            if wall_alignment.phase != previous_phase:
                print(f"[LAST-WALL] {previous_phase} -> {wall_alignment.phase}; "
                      f"rear={rear_range_clock.value}, yaw={last_yaw:.2f}°")
            if wall_action == "FAIL":
                finish_last_sequence(False, wall_alignment.failure_reason)
            elif wall_action == "COMPLETE":
                safe_write(b"STOP\n")
                # Clear steering history only. Keep yaw_ref, the corridor
                # target and the planned parking turn exactly as they were.
                heading_hold.reset()
                avoid_active = False
                avoid_side = None
                avoid_until = -1.0
                preavoid_last_bias_deg = 0.0
                last_wall_alignment_done = True
                last_move_start_enc = None
                last_move_start_t = -1.0
                last_stage_start_t = time.time()
                last_center_after_enc_t = time.monotonic()
                # The marker is cleared after the corridor turn; its saved
                # turn sign still identifies BLUE (-1) or ORANGE (+1).
                last_center_target_cm = float(
                    TUNE["LAST_CENTER_YAW_ORANGE_CM"]
                    if last_corner_sign == +1 else TUNE["LAST_CENTER_YAW_CM"]
                )
                state = STATE_LAST_CENTER_ENCODER
                last_tx = 0.0
                print(f"[LAST-WALL] Alignment complete; keeping reference yaw {yaw_ref:.2f}°; "
                      f"center forward for {last_center_target_cm:.1f}cm by encoder, "
                      f"next parking target {last_parking_target:.2f}°")
            elif wall_action == "REVERSE":
                if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    # Physical wall alignment owns the wheels. Yaw correction
                    # here would fight the wall and leave the rear skewed.
                    send_backc_deg(CENTER_DEG, int(TUNE["LAST_WALL_BACK_PWM"]))
                    last_tx = time.time()
            else:
                # Stop on the FIRST settle/sensor-wait frame, even if the
                # ordinary motion update period has not elapsed yet.
                if wall_action != last_wall_action or previous_phase != wall_alignment.phase or (
                        time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    safe_write(b"STOP\n")
                    last_tx = time.time()
            last_wall_action = wall_action

        elif state == STATE_LAST_CENTER_ENCODER:
            if last_move_start_enc is None:
                if time.time() - last_stage_start_t >= float(TUNE["LAST_CENTER_YAW_MAX_S"]):
                    finish_last_sequence(False, "centering encoder telemetry unavailable")
                elif encoder_ready() and last_enc_t >= last_center_after_enc_t:
                    # Take a new baseline after wall alignment, so reverse
                    # travel and the wall push do not count toward this stage.
                    last_move_start_enc = float(last_enc_cm)
                    last_move_start_t = time.time()
                    heading_hold.reset()
                else:
                    safe_write(b"STOP\n")
                    last_tx = time.time()
            else:
                move_action = last_encoder_action(
                    last_move_start_enc, last_enc_cm,
                    last_center_target_cm,
                    time.time() - last_stage_start_t,
                    TUNE["LAST_CENTER_YAW_MAX_S"],
                    telemetry_fresh=encoder_ready(),
                )
                if move_action == "COMPLETE":
                    safe_write(b"STOP\n")
                    pillar_suppress = False
                    pillar_q.clear()
                    last_stage_start_t = time.time()
                    state = STATE_LAST_APPROACH_100
                    print(f"[LAST] Encoder centering complete ({last_center_target_cm:.1f}cm) "
                          f"-> forward until front <= {TUNE['LAST_FRONT_TRIGGER_CM']:.1f}cm")
                elif move_action == "FAIL":
                    finish_last_sequence(False, "encoder centering failed or timed out")
                elif (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    steer_deg = compute_center_steer_deg(last_yaw)
                    send_center_deg(steer_deg, int(TUNE["LAST_CENTER_YAW_PWM"]))
                    last_tx = time.time()

        elif state == STATE_LAST_APPROACH_100:
            front_status = last_front_stage_status(
                front_range_clock.value,
                front_range_clock.received_at,
                time.monotonic(),
                TUNE["LAST_FRONT_TRIGGER_CM"],
                TUNE["FRONT_TURN_MAX_AGE_S"],
                True,
            )
            front_action = last_front_stage_action(
                front_status,
                time.time() - last_stage_start_t,
                TUNE["LAST_APPROACH_100_MAX_S"],
            )
            if front_action == "FAIL":
                finish_last_sequence(False, "approach to 100cm timed out")
            elif front_action == "COMPLETE":
                safe_write(b"STOP\n")
                pillar_suppress = True
                last_move_start_enc = None
                last_move_start_t = -1.0
                last_stage_start_t = time.time()
                heading_hold.reset()
                state = STATE_LAST_ENCODER_FWD
                print(f"[LAST] Front <= {TUNE['LAST_FRONT_TRIGGER_CM']:.1f}cm -> "
                      f"yaw-centred encoder forward {TUNE['LAST_ENCODER_FWD_CM']:.1f}cm")
            elif front_action == "MOVE":
                steer_deg = compute_center_steer_deg(last_yaw)
                if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    send_center_deg(steer_deg, int(TUNE["LAST_APPROACH_100_PWM"]))
                    last_tx = time.time()
            elif (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                safe_write(b"STOP\n")
                last_tx = time.time()

        elif state == STATE_LAST_ENCODER_FWD:
            if last_move_start_enc is None:
                if time.time() - last_stage_start_t >= float(TUNE["LAST_ENCODER_FWD_MAX_S"]):
                    finish_last_sequence(False, "forward encoder telemetry unavailable")
                elif encoder_ready():
                    last_move_start_enc = float(last_enc_cm)
                    last_move_start_t = time.time()
                    heading_hold.reset()
                elif (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    safe_write(b"STOP\n")
                    last_tx = time.time()
            else:
                move_action = last_encoder_action(
                    last_move_start_enc, last_enc_cm,
                    TUNE["LAST_ENCODER_FWD_CM"],
                    time.time() - last_move_start_t,
                    TUNE["LAST_ENCODER_FWD_MAX_S"],
                    telemetry_fresh=encoder_ready(),
                )
                if move_action == "COMPLETE":
                    safe_write(b"STOP\n")
                    last_turn_start_t = time.time()
                    state = STATE_LAST_PARK_TURN
                    print(f"[LAST] Encoder forward complete -> turn "
                          f"{'LEFT' if last_parking_sign < 0 else 'RIGHT'} "
                          f"to {last_parking_target:.1f}° (opposite corner)")
                elif move_action == "FAIL":
                    finish_last_sequence(False, "encoder forward failed or timed out")
                elif (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    steer_deg = compute_center_steer_deg(last_yaw)
                    send_center_deg(steer_deg, int(TUNE["LAST_ENCODER_FWD_PWM"]))
                    last_tx = time.time()

        elif state == STATE_LAST_PARK_TURN:
            turn_elapsed = time.time() - last_turn_start_t
            yaw_fresh = yaw_ready()
            yaw_hit = yaw_fresh and at_yaw(
                last_parking_target, last_yaw, TUNE["LAST_PARK_TURN_TOL_DEG"]
            )
            turn_action = last_turn_action(
                yaw_hit,
                turn_elapsed >= float(TUNE["LAST_PARK_TURN_MAX_S"]),
                yaw_fresh,
            )
            if turn_action == "COMPLETE":
                safe_write(b"STOP\n")
                yaw_ref = last_parking_target
                heading_hold.reset()
                last_final_front_after_seq = front_range_clock.sequence
                last_final_stop_arm_until = (
                    time.monotonic() + float(TUNE["LAST_PARK_STOP_ARM_S"])
                )
                last_stage_start_t = time.time()
                state = STATE_LAST_PARK_APPROACH
                print(f"[LAST] Opposite parking turn reached {yaw_ref:.1f}° -> "
                      f"forward until front < {TUNE['LAST_PARK_STOP_CM']:.1f}cm")
            elif turn_action == "FAIL":
                finish_last_sequence(False, "parking turn timed out before target yaw")
            elif turn_action == "WAIT_SENSOR":
                if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    safe_write(b"STOP\n")
                    last_tx = time.time()
            elif (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                fixed_deg = select_servo_deg("FORWARD", last_parking_sign)
                send_center_abs_deg(fixed_deg, TUNE["LAST_PARK_TURN_SPEED"])
                last_tx = time.time()

        elif state == STATE_LAST_PARK_APPROACH:
            now_mono = time.monotonic()
            front_status = last_front_stage_status(
                front_range_clock.value,
                front_range_clock.received_at,
                now_mono,
                TUNE["LAST_PARK_STOP_CM"],
                TUNE["FRONT_TURN_MAX_AGE_S"],
                False,
            )
            stop_allowed = last_final_stop_allowed(
                front_range_clock.value,
                TUNE["LAST_PARK_STOP_CM"],
                front_range_clock.received_at,
                now_mono,
                TUNE["FRONT_TURN_MAX_AGE_S"],
                front_range_clock.sequence,
                last_final_front_after_seq,
                last_final_stop_arm_until,
            )
            final_front_status = (
                "REACHED" if stop_allowed else
                "WAIT" if front_status == "REACHED" else front_status
            )
            front_action = last_front_stage_action(
                final_front_status,
                time.time() - last_stage_start_t,
                TUNE["LAST_PARK_APPROACH_MAX_S"],
            )
            if front_action == "FAIL":
                finish_last_sequence(False, "final parking approach timed out")
            elif front_action == "COMPLETE":
                finish_last_sequence(
                    True,
                    f"front {float(front_range_clock.value):.1f}cm is inside parking limit",
                )
            elif front_action == "MOVE":
                steer_deg = compute_center_steer_deg(last_yaw)
                if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    send_center_deg(steer_deg, int(TUNE["LAST_PARK_APPROACH_PWM"]))
                    last_tx = time.time()
            elif (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                safe_write(b"STOP\n")
                last_tx = time.time()

        # FLIP after turn 8
        elif state == STATE_FLIP_WAIT:
            steer_deg = compute_center_steer_deg(last_yaw)
            if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                send_center_deg(steer_deg, TUNE["PILLAR_FWD_SPEED"])
                last_tx = time.time()
            if time.time() >= flip_wait_until:
                rotate_target = flip_target_from(yaw_ref, flip_scheduled_color)
                rotate_start_t = time.time()
                safe_write(
                    f"TURN_ABS,{rotate_target:.1f},{TUNE['PILLAR_TURN_SPEED']}\n".encode("ascii")
                )
                state = STATE_FLIP_TURN
                lr = 'LEFT' if flip_delta_deg(flip_scheduled_color) > 0 else 'RIGHT'
                print(f"[FLIP] Begin {lr} turn by {TUNE['FLIP_TURN_DEG']:.0f}° -> target {rotate_target:.1f}")

        elif state == STATE_FLIP_TURN:
            tol = float(TUNE["PILLAR_TURN_TOL_DEG"])
            done, hit, tmo = reached_or_timeout(
                rotate_target,
                rotate_start_t,
                tol,
                TUNE["PILLAR_TURN_MAX_S"],
                last_yaw
            )
            if done:
                safe_write(b"STOP\n")
                set_ref_to_corridor()
                pillar_active = False
                pillar_state = "IDLE"
                pillar_turn_target = None
                detect_enabled = True
                color_pause_until = time.time() + 0.5
                post_flip_soft_until = time.time() + float(TUNE["POST_FLIP_KP_HOLD_S"])
                state = STATE_RUN
                print(f"[FLIP] Complete (offset unchanged). yaw_ref={yaw_ref:.1f}")

        # FINAL_HOLD -> POST12 / DONE
        elif state == STATE_FINAL_HOLD:
            steer_deg = compute_center_steer_deg(last_yaw)
            if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                send_center_deg(steer_deg, TUNE["DRIVE_SPEED"])
                last_tx = time.time()
            if time.time() >= final_hold_until:
                safe_write(b"STOP\n")
                if int(TUNE["POST12_ENABLED"]):
                    sgn = dir_sign_from_string(TUNE["POST12_DIR"])
                    post12_turn_target = wrap180(yaw_ref + sgn * float(TUNE["POST12_TURN_DEG"]))
                    post12_turn_start_t = time.time()
                    state = STATE_POST12_TURN
                    print(f"[POST12] Begin FORWARD {TUNE['POST12_DIR']} turn by "
                          f"{TUNE['POST12_TURN_DEG']:.1f}° -> target {post12_turn_target:.1f}")
                else:
                    finish_run("final hold complete")

        elif state == STATE_POST12_TURN:
            tol = float(TUNE["POST12_TURN_TOL_DEG"])
            done, hit, tmo = reached_or_timeout(
                post12_turn_target,
                post12_turn_start_t,
                tol,
                TUNE["POST12_TURN_MAX_S"],
                last_yaw
            )
            dir_sign = dir_sign_from_string(TUNE["POST12_DIR"])
            fixed_deg = select_servo_deg("FORWARD", dir_sign)
            if not done:
                if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                    send_center_abs_deg(fixed_deg, int(TUNE["POST12_TURN_SPEED"]))
                    last_tx = time.time()
            else:
                safe_write(b"STOP\n")
                yaw_ref = post12_turn_target
                state = STATE_POST12_FWD
                print(f"[POST12] Turn {'TIMEOUT' if tmo else 'DONE'} -> forward-centering to "
                      f"F<{TUNE['POST12_FRONT_STOP_CM']:.1f}cm")

        elif state == STATE_POST12_FWD:
            steer_deg = compute_center_steer_deg(last_yaw)
            if (time.time()-last_tx) >= TUNE["CONTROL_PERIOD_S"]:
                send_center_deg(steer_deg, int(TUNE["POST12_FWD_SPEED"]))
                last_tx = time.time()
            thr_cm = float(TUNE["POST12_FRONT_STOP_CM"])
            if dF >= 0 and dF < thr_cm:
                safe_write(b"STOP\n")
                finish_run(f"POST12 front {dF:.1f}cm < {thr_cm:.1f}cm")

        elif state in (STATE_PARK_SEEK, STATE_PARK_APPROACH, STATE_PARK_ALIGN,
                       STATE_PARK_BACK_IN, STATE_PARKED,
                       STATE_PARK_TO_30, STATE_PARK_EXTRA_FWD, STATE_PARK_TURN_IN):
            # legacy slot, not used in this challenge
            pass

        # CORNER LATCH
        confirmed_line_color = confirmed_intersection_color(
            color_q, TUNE["COLOR_CONFIRM_FRAMES"])
        guard_dir = (force_all_dir if force_all_dir in (-1, +1)
                     else corner_dir_latch if corner_dir_latch in (-1, +1)
                     else None)
        missed_corner_confirmed = missed_corner_guard.update(
            front_cm=raw_front_cm,
            direction_locked=(loop_started_in_run and state == STATE_RUN and
                              dir_locked and
                              guard_dir in (-1, +1)),
            corner_pending=(corner_color is not None),
            pillar_active=(pillar_active_at_loop_start or pillar_active),
            preavoid_active=(preavoid_active_at_loop_start or preavoid_active),
            recovery_active=(enc_stuck_active_at_loop_start or enc_stuck_active),
            sample_seq=front_range_clock.sequence,
        )
        observed_corner_source, observed_corner_event, guard_event_dir = choose_corner_event(
            confirmed_line_color, missed_corner_confirmed, guard_dir)
        if guard_event_dir is not None:
            guard_dir = guard_event_dir

        if (not pillar_active) and (not preavoid_active) and (not enc_stuck_active) and \
           state == STATE_RUN:
            corner_source, corner_color, new_corner_event = latch_corner_event(
                corner_source, corner_color,
                observed_corner_source, observed_corner_event,
            )
        else:
            new_corner_event = False

        if new_corner_event:
            corner_distance_checker.reset(sample_seq=front_range_clock.sequence)
            color_q.clear()
            see_color_slow = True
            color_pause_until = -1.0
            corner_line_rearm.lock(now)
            missed_corner_guard.reset()
            yaw_at_corner_start = float(last_yaw)
            pillar_suppress = True

            if corner_source == "FRONT_GUARD":
                # Brake before the next loop begins the normal corner turn.
                safe_write(b"STOP\n")
                print(f"[CORNER-GUARD] Front {raw_front_cm:.1f}cm with no line latch -> "
                      f"use locked {'LEFT' if guard_dir == -1 else 'RIGHT'} direction")

            # Latch global dir from first seen line (BLUE => LEFT, ORANGE => RIGHT)
            if (corner_source == "LINE" and not dir_locked and
                    dir_first_base is None and turn_count == 0):
                latched = (-1 if corner_color == "BLUE" else +1)
                force_all_dir = latched
                dir_locked = True
                print(f"[DIR] LATCH: first line {corner_color} -> "
                      f"{'LEFT(-1)' if latched==-1 else 'RIGHT(+1)'} for all 12 turns")

            # pick base_dir THIS corner
            if corner_source == "FRONT_GUARD":
                base_dir = guard_dir
            elif force_all_dir is not None:
                base_dir = force_all_dir
            else:
                if dir_locked and (corner_dir_latch is not None):
                    base_dir = corner_dir_latch
                else:
                    base_dir = (-1 if corner_color == "BLUE" else +1)
                    if dir_first_base is None and turn_count == 0:
                        dir_first_base = base_dir
                        print(f"[DIR] Provisional from first line: {corner_color} -> "
                              f"{'LEFT(-1)' if base_dir==-1 else 'RIGHT(+1)'}")

            current_corner_dir = base_dir             # +1 right / -1 left
            target_idx = corridor_idx + base_dir       # which corridor index we're entering
            next_corner_num = turn_count + 1
            current_corner_mode = corner_turn_type_for(next_corner_num)

            # custom yaw for last corner vs normal grid yaw
            if int(TUNE["LAST_ENABLED"]) and next_corner_num == int(TUNE["MAX_TURNS"]):
                turn_deg = float(TUNE["LAST_CORNER_TURN_DEG"])
                intended_corner_yaw = wrap180(yaw_ref + current_corner_dir * turn_deg)
            else:
                intended_corner_yaw = wrap180(start_yaw + target_idx * TUNE["TURN_DEG"] + orientation_offset_deg)

            source_text = (f"line={corner_color}" if corner_source == "LINE"
                           else f"front-guard={raw_front_cm:.1f}cm")
            print(f"[CORNER] {source_text}  dir={'LEFT' if base_dir==-1 else 'RIGHT'}  "
                  f"mode={current_corner_mode}  -> idx={target_idx}, yaw={intended_corner_yaw:.1f}")

        # HUD
        vis = frame.copy()

        # Line ROI
        cv.rectangle(vis, (c_x0,c_y0),(c_x1,c_y1),(0,165,255), 2)
        for detected_rect, detected_color in (
                (line_rect_blue, "BLUE"),
                (line_rect_orange, "ORANGE")):
            overlay = intersection_line_overlay(
                detected_rect, c_x0, c_y0, detected_color)
            if overlay is None:
                continue
            line_top_left, line_bottom_right, line_text, line_bgr = overlay
            cv.rectangle(vis, line_top_left, line_bottom_right, line_bgr, 4)
            text_y = max(20, line_top_left[1] - 8)
            cv.putText(vis, line_text, (line_top_left[0], text_y),
                       cv.FONT_HERSHEY_SIMPLEX, 0.60, line_bgr, 2)

        # Show the strongest raw HSV match when geometry rejects it. This
        # makes AutoTune/main-code mismatches visible without triggering a turn.
        for rejected_rect, rejected_color, rejected_reason in (
                (rejected_rect_blue, "BLUE", rejected_reason_blue),
                (rejected_rect_orange, "ORANGE", rejected_reason_orange)):
            if rejected_rect is None or rejected_reason is None:
                continue
            rx, ry, rw, rh = rejected_rect
            rejected_top_left = (c_x0 + rx, c_y0 + ry)
            rejected_bottom_right = (c_x0 + rx + rw, c_y0 + ry + rh)
            rejected_bgr = ((180, 100, 0) if rejected_color == "BLUE"
                            else (0, 100, 180))
            cv.rectangle(vis, rejected_top_left, rejected_bottom_right,
                         rejected_bgr, 2)
            rejected_text_y = min(h - 8, rejected_bottom_right[1] + 18)
            cv.putText(
                vis, f"{rejected_color} REJECTED: {rejected_reason}",
                (rejected_top_left[0], rejected_text_y),
                cv.FONT_HERSHEY_SIMPLEX, 0.45, rejected_bgr, 1)

        # Pillar gate lines + ROI
        red_gate_x   = int(w * TUNE["RED_LINE_X_FRAC"])
        green_gate_x = int(w * TUNE["GREEN_LINE_X_FRAC"])
        if red_gate_x > green_gate_x:
            red_gate_x, green_gate_x = green_gate_x, red_gate_x
        cv.line(vis, (red_gate_x, 0),   (red_gate_x, h),   (0,0,255), 2)
        cv.line(vis, (green_gate_x, 0), (green_gate_x, h), (0,255,0), 2)
        p_x0,p_y0,p_x1,p_y1 = pillar_roi_rect(w,h)
        cv.rectangle(vis, (p_x0,p_y0),(p_x1,p_y1),(255,255,0), 2)

        # HUD pillar boxes + dist text
        def draw_pillar(vis, blob, color_name, color_bgr, xoff=p_x0, yoff=p_y0):
            if not blob:
                return None
            x,y,ww,hh,area = blob
            cv.rectangle(vis,
                         (xoff+x, yoff+y),
                         (xoff+x+ww, yoff+y+hh),
                         color_bgr, 2)
            dist_cm = pillar_distance_cm_from_bbox(hh)
            cv.putText(vis, f"{color_name}:{int(dist_cm)}cm",
                       (xoff+x, max(20, yoff+y-6)),
                       cv.FONT_HERSHEY_SIMPLEX, 0.55, color_bgr, 2)
            return dist_cm

        d_red = draw_pillar(vis, red_blob,  "RED",   (0,0,255))
        d_grn = draw_pillar(vis, grn_blob , "GREEN", (0,255,0))

        # Magnet ROI preview + bands
        m_x0,m_y0,m_x1,m_y1 = magnet_roi_rect(w,h)
        cv.rectangle(vis, (m_x0,m_y0),(m_x1,m_y1),(200,100,255), 1)
        band_w = max(2, int((p_x1 - p_x0) * float(TUNE.get("MAG_AVOID_SAFE_FRAC", 0.08))))
        cv.rectangle(vis, (p_x0, p_y0), (p_x0 + band_w, p_y1), (200,100,255), 1)
        cv.rectangle(vis, (p_x1 - band_w, p_y0), (p_x1, p_y1), (200,100,255), 1)

        # HUD text overlays
        st_name = {
            STATE_WAIT_START:"WAIT", STATE_ESC_S1_FWD_L:"ESC_S1",
            STATE_ESC_S2_BACK_L:"ESC_S2", STATE_ESC_S3_FWD_L:"ESC_S3",
            STATE_ESC_S3_FWD_HOLD:"ESC_S3_HOLD",
            STATE_ESC_S4_FWD_RTN:"ESC_RTN", STATE_RUN:"RUN",
            STATE_CORNER_TURNING:"CORNER_TURN", STATE_CORNER_BACKCENTER:"CORNER_BACKC",
            STATE_FLIP_WAIT:"FLIP_WAIT", STATE_FLIP_TURN:"FLIP_TURN",
            STATE_FINAL_HOLD:"FINAL_HOLD", STATE_POST12_TURN:"POST12_TURN",
            STATE_POST12_FWD:"POST12_FWD", STATE_DONE:"DONE",
            STATE_PARK_SEEK:"PARK_SEEK", STATE_PARK_APPROACH:"PARK_APP",
            STATE_PARK_ALIGN:"PARK_ALIGN", STATE_PARK_BACK_IN:"PARK_BACK", STATE_PARKED:"PARKED",
            STATE_PARK_TO_30:"PARK_TO_30", STATE_PARK_EXTRA_FWD:"PARK_EXTRA", STATE_PARK_TURN_IN:"PARK_TURN",
            STATE_LAST_APPROACH_5:"LAST_TO_5CM",
            STATE_LAST_BACK_CENTER:"LAST_BACK_CENTER",
            STATE_LAST_CORNER_TURN:"LAST_CORNER_TURN",
            STATE_LAST_CENTER_ENCODER:"LAST_CENTER_ENCODER",
            STATE_LAST_APPROACH_100:"LAST_TO_100CM",
            STATE_LAST_ENCODER_FWD:"LAST_ENCODER_FWD",
            STATE_LAST_PARK_TURN:"LAST_PARK_TURN",
            STATE_LAST_PARK_APPROACH:"LAST_PARK_IN",
            STATE_LAST_WALL_ALIGN:"LAST_WALL_" + wall_alignment.phase,
        }[state]
        esc_flag = " E-WESC" if enc_stuck_active else ""
        yaw_text = f"{last_yaw:5.1f}" if yaw_ready() else "   --"

        if continuous_heading_active() and time.monotonic() - last_yaw_debug_t >= 0.5:
            # Lets a track test separate yaw correction from wall intervention.
            print("[STRAIGHT] state=%s yaw=%.1f target=%.1f error=%+.1f rate=%+.1f cmd=%.2f wall=%s magL=%s magR=%s" %
                  (st_name, last_yaw, yaw_ref, shortest_err(yaw_ref, last_yaw),
                   heading_hold.rate, heading_hold.command,
                   avoid_side if avoid_active else '-',
                   '-' if mag_wall_left_cm is None else '%.1f' % mag_wall_left_cm,
                   '-' if mag_wall_right_cm is None else '%.1f' % mag_wall_right_cm))
            last_yaw_debug_t = time.monotonic()

        cv.putText(vis,
                   f"STATE:{st_name}{esc_flag}  yaw:{yaw_text}  ref:{yaw_ref:5.1f}  "
                   f"idx:{corridor_idx}  off:{orientation_offset_deg:+.0f}",
                   (10,22), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2)

        dF_i = last_US['dF']; dL_i = last_US['dL']; dR_i = last_US['dR']; dB_i = last_US['dB']
        cv.putText(vis,
                   f"US F:{dF_i}  L:{dL_i}  R:{dR_i}  B:{dB_i}",
                   (10,44), cv.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)

        mwL = '-' if mag_wall_left_cm  is None else int(mag_wall_left_cm)
        mwR = '-' if mag_wall_right_cm is not None else '-'
        if mag_wall_right_cm is not None:
            mwR = int(mag_wall_right_cm)
        cv.putText(vis,
                   f"MAG walls  L:{mwL}cm  R:{mwR}cm",
                   (10, 308),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, (210,170,255), 2)

        turns_txt = f"Turns: {turn_count}/{TUNE['MAX_TURNS']}"
        if run_start_time is not None:
            elapsed = (run_end_time if run_end_time is not None else time.time()) - run_start_time
            turns_txt += f"  Timer: {elapsed:5.2f}s"
        cv.putText(vis,
                   turns_txt,
                   (10,66), cv.FONT_HERSHEY_SIMPLEX, 0.55, (180,220,255), 2)

        dir_txt = ('LEFT(-1)' if (force_all_dir == -1)
                   else ('RIGHT(+1)' if (force_all_dir == +1) else 'AUTO'))
        cv.putText(vis,
                   f"All-12 dir: {dir_txt}  Locked:{'Y' if dir_locked else 'N'}",
                   (10,88), cv.FONT_HERSHEY_SIMPLEX, 0.55, (180,220,255), 2)

        seq = "RIGHT-first" if esc_seq_dir==+1 else "LEFT-first"
        if state in (STATE_ESC_S1_FWD_L, STATE_ESC_S2_BACK_L, STATE_ESC_S3_FWD_L,
                     STATE_ESC_S3_FWD_HOLD, STATE_ESC_S4_FWD_RTN):
            cv.putText(vis,
                       f"ESCAPE MODE ({seq})",
                       (10,110),
                       cv.FONT_HERSHEY_SIMPLEX, 0.60, (0,255,255), 2)

        flip_label = "ENABLED" if flip_enabled_now() else "DISABLED"
        cv.putText(vis,
                   f"FLIP: {flip_label}",
                   (10,132),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)

        intersection_style = intersection_hud_style(corner_color)
        if intersection_style is not None:
            intersection_text, intersection_color = intersection_style
            cv.putText(vis, intersection_text,
                       (10,154), cv.FONT_HERSHEY_SIMPLEX, 0.65,
                       intersection_color, 2)
            front_text = ("--" if raw_front_cm < 0
                          else f"{raw_front_cm:.1f}")
            cv.putText(vis,
                       f"FRONT: {front_text}cm  TURN BELOW: "
                       f"{front_turn_cm_for(turn_count + 1):.1f}cm",
                       (10,176), cv.FONT_HERSHEY_SIMPLEX, 0.55,
                       intersection_color, 2)

        if int(TUNE["PREAVOID_HUD_TIMER"]) and (pillar_timer_start is not None):
            elapsed = time.time() - pillar_timer_start
            cv.putText(vis,
                       f"Avoid Timer: {elapsed:5.2f}s",
                       (10, 176),
                       cv.FONT_HERSHEY_SIMPLEX, 0.65, (255,220,0), 2)

        cv.putText(vis,
                   f"Last pillar before 5th: {last_pillar_before_5th or '-'}",
                   (10, 198),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, (255,200,150), 2)
        cv.putText(vis,
                   f"First pillar after 4th: {first_pillar_after_first4 or '-'}",
                   (10, 220),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, (255,200,150), 2)

        cv.putText(vis,
                   f"ENC: {last_enc_cm:5.1f} cm",
                   (10, 286),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, (150,220,255), 2)

        if not yaw_ready():
            cv.putText(vis, "YAW: waiting for Arduino yaw=... in degrees",
                       (10,330), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0,180,255), 2)

        cv.imshow("robot", vis)

        # KEYS
        key = cv.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q')):
            safe_write(b"STOP\n")
            break

        elif key in (ord('s'), ord('S')):
            if not manual_start_state_allowed(state, STATE_WAIT_START):
                safe_write(b"STOP\n")
                print("[START] A run is already active or complete. "
                      "Press X for a clean restart before another run.")
                continue
            if not manual_start_allowed(enc_stuck_active, enc_stuck_phase):
                safe_write(b"STOP\n")
                print("[E-WESC] START blocked while recovery/fault is active. "
                      "Inspect the robot and sensors, then press X to restart.")
                continue
            if not yaw_ready():
                safe_write(b"STOP\n")
                print("[YAW] Start blocked: waiting for a valid Arduino yaw reading in degrees.")
                if last_tlm_line:
                    print("[YAW] Last TLM:", last_tlm_line[:250])
                continue
            if not safe_write(b"START\n"):
                print("[YAW] START not sent; wait for fresh telemetry and press S again.")
                continue
            start_yaw = float(last_yaw)
            heading_hold.reset()
            corner_line_rearm.reset()
            missed_corner_guard.reset()
            camera_failure_guard.reset()
            corridor_idx = 0
            orientation_offset_deg = 0.0
            set_ref_to_corridor()
            run_start_time = time.time()
            run_end_time = None

            first_pillar_after_first4 = None
            last_pillar_before_5th = None
            flip_policy_enable = None

            preavoid_available = True
            esc_cmd_sent = False
            esc_waiting_dist = False
            pillar_back_cmd_sent = False
            pillar_back_waiting_dist = False
            pillar_fwd_cmd_sent = False
            pillar_fwd_waiting_dist = False

            enc_stuck_active = False
            enc_back_cycles = 0
            enc_hold_reason = None
            enc_back_after_rear_seq = rear_range_clock.sequence
            enc_check_after_front_seq = front_range_clock.sequence
            enc_back_start_t = None
            crash_recovery.clear()
            enc_cooldown_until = -1.0
            enc_progress_cm_at = last_enc_cm
            enc_progress_t = time.time()

            lastseq_active = False
            last_no_avoid_override = False
            pillar_suppress = False
            last_corner_sign = None
            last_corner_target = None
            last_parking_sign = None
            last_parking_target = None
            last_stage_start_t = -1.0
            last_center_target_cm = None
            last_turn_start_t = -1.0
            last_move_start_enc = None
            last_move_start_t = -1.0
            last_final_front_after_seq = front_range_clock.sequence
            last_final_stop_arm_until = -1.0
            last_wall_alignment_done = False
            wall_alignment.reset()

            if int(TUNE["ESCAPE_ENABLED"]):
                esc_seq_dir = decide_esc_seq_dir()
                build_escape_targets()
                esc_turn_start_t = time.time()
                state = STATE_ESC_S1_FWD_L
                dir_name = "RIGHT-first" if esc_seq_dir==+1 else "LEFT-first"
                print("START -> ESC (%s) S1 %.1f  S2 %.1f  S3 %.1f  RTN %.1f" %
                      (dir_name, esc_s1_target, esc_s2_target,
                       esc_s3_target, esc_s4_target))
            else:
                state = STATE_RUN
                print("START sent. start_yaw=%.1f" % start_yaw)

        elif key in (ord('x'), ord('X')):
            restart_program("manual keypress (X)")

    # cleanup
    try:
        if ser:
            ser.close()
    except Exception:
        pass
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: exiting.")
        sys.exit(0)

