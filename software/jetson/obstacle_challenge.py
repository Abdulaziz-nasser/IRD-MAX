#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# file: robot_runner.py

import cv2 as cv
import numpy as np
import yaml, glob, time, os, re, math, sys
from collections import deque

try:
    import serial
except Exception:
    serial = None

# ==========================================================
# =================== TUNING PANEL =========================
# ==========================================================
TUNE = {
    # ----- Corridor turns (BLUE/ORANGE lines) -----
    "TURN_DEG": float(os.environ.get("TURN_DEG", "90.0")),
    "TURN_TOL_DEG": float(os.environ.get("TURN_TURN_TOL_DEG", os.environ.get("TURN_TOL_DEG", "45.0"))),

    # ----- Pillar sidestep turns (RED/GREEN pillars) -----
    "PILLAR_TURN_DEG": float(os.environ.get("PILLAR_TURN_DEG", "90.0")),
    "PILLAR_TURN_TOL_DEG": float(os.environ.get("PILLAR_TURN_TOL_DEG", "30.0")),

    # ----- Flip (optional) -----
    "FLIP_ENABLED": int(os.environ.get("FLIP_ENABLED", "0")),
    "FLIP_TURN_DEG": float(os.environ.get("FLIP_TURN_DEG", "180.0")),
    "FLIP_FWD_HOLD_S": float(os.environ.get("FLIP_FWD_HOLD_S", "1.50")),
    "FLIP_LEFT_SIGN": float(os.environ.get("FLIP_LEFT_SIGN", "1.0")),

    # ----- ESCAPE MODE (pre-run tiny alignment wiggle) -----
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
    "ESC_S1_CM": float(os.environ.get("ESC_S1_CM", "5.0")),
    "ESC_S2_CM": float(os.environ.get("ESC_S2_CM", "4.0")),
    "ESC_S3_CM": float(os.environ.get("ESC_S3_CM", "15.0")),
    "ESC_S4_CM": float(os.environ.get("ESC_S4_CM", "23.0")),
    "ESC_S1_PWM": int(os.environ.get("ESC_S1_PWM", "40")),
    "ESC_S2_PWM": int(os.environ.get("ESC_S2_PWM", "42")),
    "ESC_S3_PWM": int(os.environ.get("ESC_S3_PWM", "42")),
    "ESC_S4_PWM": int(os.environ.get("ESC_S4_PWM", "42")),

    # ----- Direction lock across all 12 corners -----
    "LOCK_ALL_12_TURNS_ONE_DIRECTION": int(os.environ.get("LOCK_ALL_12_TURNS_ONE_DIRECTION", "1")),
    "ALL_TURNS_DIR": os.environ.get("ALL_TURNS_DIR", "").strip().upper(),

    # ----- Turn watchdogs -----
    "CORNER_TURN_MAX_S": float(os.environ.get("CORNER_TURN_MAX_S", "1.5")),
    "PILLAR_TURN_MAX_S": float(os.environ.get("PILLAR_TURN_MAX_S", "1.0")),

    # ----- Run cap -----
    "MAX_TURNS": int(os.environ.get("MAX_TURNS", "12")),

    # ----- Final forward-centering hold after last turn -----
    "FINAL_HOLD_S": float(os.environ.get("FINAL_HOLD_S", "0.0")),

    # ----- Servo geometry (deg) -----
    "CENTER_DEG": int(os.environ.get("CENTER_DEG", "90")),
    "LEFT_LIMIT": int(os.environ.get("LEFT_LIMIT", "60")),
    "RIGHT_LIMIT": int(os.environ.get("RIGHT_LIMIT", "115")),
    "CENTER_STEER_MIN": 60,
    "CENTER_STEER_MAX": 115,

    # ----- General speeds (PWM 0..255) -----
    "DRIVE_SPEED": int(os.environ.get("DRIVE_SPEED", "41")),
    "SEE_COLOR_SPEED": int(os.environ.get("SEE_COLOR_SP5EED", "41")),
    "TURN_SPEED": int(os.environ.get("TURN_SPEED", "42")),

    # ----- Pillar pass speeds -----
    "PILLAR_TURN_SPEED": int(os.environ.get("PILLAR_TURN_SPEED", "40")),
    "PILLAR_FWD_SPEED": int(os.environ.get("PILLAR_FWD_SPEED", "41")),
    "PILLAR_BACK_SPEED": int(os.environ.get("PILLAR_BACK_SPEED", "41")),  # legacy

    # ----- Back-center stop speed -----
    "BACK_STOP_SPEED": int(os.environ.get("BACK_STOP_SPEED", "38")),

    "PILLAR_BACK_USE_ENCODER": int(os.environ.get("PILLAR_BACK_USE_ENCODER", "1")),
    "PILLAR_BACK_ENC_CM": float(os.environ.get("PILLAR_BACK_ENC_CM", "16.0")),
    "PILLAR_BACK_ENC_SPEED": int(os.environ.get("PILLAR_BACK_ENC_SPEED", "45")),

    # ----- Pillar forward distance (encoder) -----
    "PILLAR_FWD_USE_ENCODER": int(os.environ.get("PILLAR_FWD_USE_ENCODER", "1")),
    "PILLAR_FWD_BASE_CM": float(os.environ.get("PILLAR_FWD_BASE_CM", "0.0")),
    "PILLAR_FWD_GAIN_CM": float(os.environ.get("PILLAR_FWD_GAIN_CM", "14.0")),
    "PILLAR_FWD_MIN_CM": float(os.environ.get("PILLAR_FWD_MIN_CM", "0.0")),
    "PILLAR_FWD_MAX_CM": float(os.environ.get("PILLAR_FWD_MAX_CM", "20.0")),

    # ----- Corner fixed servo angles (hardware mapping) -----
    "BACK_LEFT_SERVO_DEG": int(os.environ.get("BACK_LEFT_SERVO_DEG", "65")),
    "BACK_RIGHT_SERVO_DEG": int(os.environ.get("BACK_RIGHT_SERVO_DEG", "120")),
    "FWD_LEFT_SERVO_DEG": int(os.environ.get("FWD_LEFT_SERVO_DEG", "115")),
    "FWD_RIGHT_SERVO_DEG": int(os.environ.get("FWD_RIGHT_SERVO_DEG", "60")),

    # ----- Distances (cm) -----
    "FRONT_TURN_CM": float(os.environ.get("FRONT_TURN_CM", "42.0")),
    "FRONT_TURN_CM_1": float(os.environ.get("FRONT_TURN_CM_1", "36.0")),
    "FRONT_TURN_CM_2": float(os.environ.get("FRONT_TURN_CM_2", "36.0")),
    "FRONT_TURN_CM_3": float(os.environ.get("FRONT_TURN_CM_3", "36.0")),
    "FRONT_TURN_CM_4": float(os.environ.get("FRONT_TURN_CM_4", "36.0")),
    "BACK_STOP_CM": float(os.environ.get("BACK_STOP_CM", "55.0")),
    "PILLAR_ACCEPT_CM": float(os.environ.get("PILLAR_ACCEPT_CM", "75.0")),
    "PILLAR_APPROACH_FRONT_CM": float(os.environ.get("PILLAR_APPROACH_FRONT_CM", "12.0")),  # legacy
    "PILLAR_BACK_TO_FRONT_CM": float(os.environ.get("PILLAR_BACK_TO_FRONT_CM", "14.0")),    # legacy

    # ----- Per-corner turn types -----
    "CORNER_TURN_TYPE_1": os.environ.get("CORNER_TURN_TYPE_1", "BACKWARD"),
    "CORNER_TURN_TYPE_2": os.environ.get("CORNER_TURN_TYPE_2", "BACKWARD"),
    "CORNER_TURN_TYPE_3": os.environ.get("CORNER_TURN_TYPE_3", "BACKWARD"),
    "CORNER_TURN_TYPE_4": os.environ.get("CORNER_TURN_TYPE_4", "BACKWARD"),

    # ----- Color detection / ROI -----
    "COLOR_CONFIRM_FRAMES": int(os.environ.get("COLOR_CONFIRM_FRAMES", "2")),
    "MIN_PIX_COLOR": int(os.environ.get("MIN_PIX_COLOR", "100")),
    "BLUR_KSIZE": int(os.environ.get("BLUR_KSIZE", "3")),
    "COLOR_ROI_WIDTH_FRAC": float(os.environ.get("COLOR_ROI_WIDTH_FRAC", "0.25")),
    "COLOR_ROI_HEIGHT_FRAC": float(os.environ.get("COLOR_ROI_HEIGHT_FRAC", "0.350")),
    "ROI_SCALE_AFTER_FIRST_TURN": float(os.environ.get("ROI_SCALE_AFTER_FIRST_TURN", "1.3")),
    "SAT_BOOST": float(os.environ.get("SAT_BOOST", "1.10")),
    "COLOR_SUSPEND_SEC": float(os.environ.get("COLOR_SUSPEND_SEC", "7.0")),

    # ----- Pillar detection (RED/GREEN) -----
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

    # ----- Pillar gates (side validation) -----
    "RED_LINE_X_FRAC": float(os.environ.get("RED_LINE_X_FRAC", "0.3")),
    "GREEN_LINE_X_FRAC": float(os.environ.get("GREEN_LINE_X_FRAC", "0.85")),

    # ----- Yaw centering controller -----
    "STEER_SIGN": float(os.environ.get("STEER_SIGN", "1.0")),
    "STEER_KP_DEG_PER_DEG": float(os.environ.get("STEER_KP_DEG_PER_DEG", ".32")),
    "STEER_MAX_DEG": float(os.environ.get("STEER_MAX_DEG", "5.0")),
    "STEER_DEAD_ERR_DEG": float(os.environ.get("STEER_DEAD_ERR_DEG", ".0001")),
    "ERR_FILTER_ALPHA": float(os.environ.get("ERR_FILTER_ALPHA", "0.35")),
    "STEER_SLEW_DEG": int(os.environ.get("STEER_SLEW_DEG", "200")),

    # ----- Post-flip centering softener -----
    "POST_FLIP_KP_GAIN": float(os.environ.get("POST_FLIP_KP_GAIN", "4.825")),
    "POST_FLIP_MAX_DEG": float(os.environ.get("POST_FLIP_MAX_DEG", "4.0")),
    "POST_FLIP_KP_HOLD_S": float(os.environ.get("POST_FLIP_KP_HOLD_S", ".0000005")),

    # ----- Wall slowdowns -----
    "WALL_SLOW_SIDE_ON_CM": float(os.environ.get("WALL_SLOW_SIDE_ON_CM", "15.5")),
    "WALL_SLOW_SIDE_OFF_CM": float(os.environ.get("WALL_SLOW_SIDE_OFF_CM", "16.5")),
    "SIDE_SLOWDOWN_FACTOR": float(os.environ.get("SIDE_SLOWDOWN_FACTOR", "0.55")),
    "WALL_SLOW_FRONT_ON_CM": float(os.environ.get("WALL_SLOW_FRONT_ON_CM", "45.0")),
    "FRONT_SLOWDOWN_FACTOR": float(os.environ.get("FRONT_SLOWDOWN_FACTOR", "0.55")),
    "WALL_SLOW_MIN_SPEED": int(os.environ.get("WALL_SLOW_MIN_SPEED", "35")),

    # ----- Wall avoid -----
    "WALL_AVOID_ON_CM": float(os.environ.get("WALL_AVOID_ON_CM", "15.5")),
    "WALL_AVOID_OFF_CM": float(os.environ.get("WALL_AVOID_OFF_CM", "16.5")),
    "WALL_AVOID_HOLD_S": float(os.environ.get("WALL_AVOID_HOLD_S", "0.35")),
    "WALL_AVOID_KP_DEG_PER_CM": float(os.environ.get("WALL_AVOID_KP_DEG_PER_CM", "3.8")),
    "WALL_AVOID_EXP": float(os.environ.get("WALL_AVOID_EXP", "1.0")),
    "WALL_AVOID_MIN_DEG": float(os.environ.get("WALL_AVOID_MIN_DEG", "2.0")),
    "WALL_AVOID_MAX_BIAS_DEG": float(os.environ.get("WALL_AVOID_MAX_BIAS_DEG", "5.0")),
    "WALL_AVOID_GAIN": float(os.environ.get("WALL_AVOID_GAIN", "1.0")),
    "AVOID_INVERT_DIR": int(os.environ.get("AVOID_INVERT_DIR", "-1")),
    "WALL_AVOID_SPEED": int(os.environ.get("WALL_AVOID_SPEED", "35")),

    # ----- First-Step Pillar Avoid -----
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

    # ----- Camera / serial -----
    "ALLOW_NO_CAMERA": int(os.environ.get("ALLOW_NO_CAMERA", "0")),
    "FRAME_W": int(os.environ.get("FRAME_W", "640")),
    "FRAME_H": int(os.environ.get("FRAME_H", "480")),
    "ROBOT_PORT": os.environ.get("ROBOT_PORT", "/dev/ttyUSB0"),
    "ROBOT_BAUD": int(os.environ.get("ROBOT_BAUD", "115200")),

    # ----- Link watchdogs -----
    "STALE_TLM_SEC": float(os.environ.get("STALE_TLM_SEC", "1.2")),
    "STALE_LINK_SEC": float(os.environ.get("STALE_LINK_SEC", "2.5")),
    "PING_PERIOD_SEC": float(os.environ.get("PING_PERIOD_SEC", "0.7")),
    "RESET_BACKOFF_SEC": float(os.environ.get("RESET_BACKOFF_SEC", "80.0")),

    # ----- Jetson <-> Arduino reset sync -----
    "JETSON_RESET_ON_ARDUINO_READY": int(os.environ.get("JETSON_RESET_ON_ARDUINO_READY", "1")),
    "JETSON_RESET_GRACE_S": float(os.environ.get("JETSON_RESET_GRACE_S", "1.0")),

    # ----- Misc -----
    "US_MAX_CM": float(os.environ.get("US_MAX_CM", "300.0")),

    # ----- Post-12 extra maneuver (optional) -----
    "POST12_ENABLED": int(os.environ.get("POST12_ENABLED", "0")),
    "POST12_DIR": os.environ.get("POST12_DIR", "RIGHT").strip().upper(),
    "POST12_TURN_DEG": float(os.environ.get("POST12_TURN_DEG", "90.0")),
    "POST12_TURN_TOL_DEG": float(os.environ.get("POST12_TURN_TOL_DEG", "20.0")),
    "POST12_TURN_MAX_S": float(os.environ.get("POST12_TURN_MAX_S", "2.5")),
    "POST12_TURN_SPEED": int(os.environ.get("POST12_TURN_SPEED", "31")),
    "POST12_FWD_SPEED": int(os.environ.get("POST12_FWD_SPEED", "28")),
    "POST12_FRONT_STOP_CM": float(os.environ.get("POST12_FRONT_STOP_CM", "8.0")),

    # ----- STUCK / WALL ESCAPE (legacy thresholds kept for harmony) -----
    "STUCK_FRONT_CM": -20.0,
    "STUCK_YAW_ERR_DEG": 0.00,
    "STUCK_MIN_S": .0,
    "WESC_BACK_S": 0.0,
    "WESC_BACK_SPEED": 0,
    "WESC_SPIN_DEG": 22.0,
    "WESC_SPIN_SPEED": 0,
    "WESC_MAX_SPIN_S": .90,
    "WESC_EXIT_FRONT_CM": 0.0,
    "WESC_EXIT_YAW_DEG": 180.0,  # kept for compatibility
    "WESC_COOLDOWN_S": .8,      # was 0.0, now nonzero to prevent retrigger spam

    # ----- Encoder-based stuck detection -----
    "STUCK_ENC_FREEZE_S": float(os.environ.get("STUCK_ENC_FREEZE_S", "1.850")),
    "STUCK_ENC_MIN_DELTA_CM": float(os.environ.get("STUCK_ENC_MIN_DELTA_CM", "0.9")),
    "WESC_BACK_CM": float(os.environ.get("WESC_BACK_CM", "15.0")),
    "WESC_BACK_PWM": int(os.environ.get("WESC_BACK_PWM", "38")),
    "WESC_TURN_PWM": int(os.environ.get("WESC_TURN_PWM", "41")),
    "WESC_EXIT_TOL_DEG": float(os.environ.get("WESC_EXIT_TOL_DEG", "45.0")),

    # ===== magnet-wall avoid (corridor) =====
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

    # ===== LAST CORNER SEQUENCE =====
    "LAST_ENABLED": int(os.environ.get("LAST_ENABLED", "1")),
    "LAST_PRETURN_FRONT_CM": float(os.environ.get("LAST_PRETURN_FRONT_CM", "5.0")),
    "LAST_BACK_ENC_CM": float(os.environ.get("LAST_BACK_ENC_CM", "16.0")),
    "LAST_BACK_PWM": int(os.environ.get("LAST_BACK_PWM", "40")),
    "LAST_NO_AVOID_HOLD_S": float(os.environ.get("LAST_NO_AVOID_HOLD_S", ".5")),
    "LAST_WAIT_FRONT_STOP_CM": float(os.environ.get("LAST_WAIT_FRONT_STOP_CM", "35.0")),

    # T1: first 90° (choice turn)
    "LAST_T1_DEG": float(os.environ.get("LAST_T1_DEG", "90.0")),
    "LAST_T1_TOL_DEG": float(os.environ.get("LAST_T1_TOL_DEG", "4.0")),
    "LAST_T1_SPEED": int(os.environ.get("LAST_T1_SPEED", "36")),
    "LAST_T1_MAX_S": float(os.environ.get("LAST_T1_MAX_S", "1.5")),

    # RET1 uses T1 tunables and SAME direction as T1 (fixed)

    # T2: opposite 90°
    "LAST_T2_DEG": float(os.environ.get("LAST_T2_DEG", "90.0")),
    "LAST_T2_TOL_DEG": float(os.environ.get("LAST_T2_TOL_DEG", "4.0")),
    "LAST_T2_SPEED": int(os.environ.get("LAST_T2_SPEED", "36")),
    "LAST_T2_MAX_S": float(os.environ.get("LAST_T2_MAX_S", "1.5")),

    # FINAL_RET: return to main yaw
    "LAST_FINAL_RET_DEG": float(os.environ.get("LAST_FINAL_RET_DEG", "0.0")),
    "LAST_FINAL_RET_TOL_DEG": float(os.environ.get("LAST_FINAL_RET_TOL_DEG", "2.0")),
    "LAST_FINAL_RET_SPEED": int(os.environ.get("LAST_FINAL_RET_SPEED", "35")),
    "LAST_FINAL_RET_MAX_S": float(os.environ.get("LAST_FINAL_RET_MAX_S", "2.7")),

    # Distances between the 4 final turns (cm)
    "LAST_FWD1_CM": float(os.environ.get("LAST_FWD1_CM", ".50")),
    "LAST_FWD2_CM": float(os.environ.get("LAST_FWD2_CM", "18.0")),
    "LAST_FWD3_CM": float(os.environ.get("LAST_FWD3_CM", "35.0")),

    # Timeout safety for LAST_FWD3
    "LAST_FWD3_TIMEOUT_S": float(os.environ.get("LAST_FWD3_TIMEOUT_S", "2.0")),

    # ----- Specific tuning for the corner turn AT the last corner -----
    "LAST_CORNER_TURN_DEG": float(os.environ.get("LAST_CORNER_TURN_DEG", os.environ.get("TURN_DEG", "90.0"))),
    "LAST_CORNER_TURN_TOL_DEG": float(os.environ.get("LAST_CO5RNER_TURN_TOL_DEG", os.environ.get("TURN_TOL_DEG", "45.0"))),
    "LAST_CORNER_TURN_SPEED": int(os.environ.get("LAST_CORNER_TURN_SPEED", os.environ.get("TURN_SPEED", "34"))),
    "LAST_CORNER_TURN_MAX_S": float(os.environ.get("LAST_CORNER_TURN_MAX_S", os.environ.get("CORNER_TURN_MAX_S", ".8"))),
}

# ================ CONSTANTS / HELPERS ================
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
    angle_deg = float(max(LEFT_LIMIT, min(RIGHT_LIMIT, int(angle_deg))))
    if angle_deg >= CENTER_DEG:
        return (angle_deg - CENTER_DEG) / float(max(1.0, SPAN_RIGHT))
    else:
        return (angle_deg - CENTER_DEG) / float(max(1.0, SPAN_LEFT))

def slew_limit(prev, target, max_delta):
    if prev is None: return target
    if target > prev + max_delta: return prev + max_delta
    if target < prev - max_delta: return prev - max_delta
    return target

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

# ===== Flip helpers =====
def flip_delta_deg(color):
    s = float(TUNE["FLIP_LEFT_SIGN"])
    ang = float(TUNE["FLIP_TURN_DEG"])
    return (s*ang) if color == "RED" else (-s*ang)

def flip_target_from(base_yaw, color):
    return wrap180(base_yaw + flip_delta_deg(color))

# ===== Per-corner FRONT_TURN_CM (repeats every 4 turns) =====
def front_turn_cm_for(next_turn_number: int) -> float:
    idx = ((int(next_turn_number) - 1) % 4) + 1
    return float(TUNE.get(f"FRONT_TURN_CM_{idx}", TUNE["FRONT_TURN_CM"]))

# ===== Per-corner turn type =====
def corner_turn_type_for(next_turn_number: int) -> str:
    idx = ((int(next_turn_number) - 1) % 4) + 1
    val = str(TUNE.get(f"CORNER_TURN_TYPE_{idx}", "BACKWARD")).strip().upper()
    return "FORWARD" if val.startswith("FOR") else "BACKWARD"

# ===== Servo chooser =====
def select_servo_deg(mode: str, dir_sign: int) -> int:
    if mode == "FORWARD":
        return (TUNE["FWD_LEFT_SERVO_DEG"] if dir_sign == +1 else TUNE["FWD_RIGHT_SERVO_DEG"])
    else:
        return (TUNE["BACK_RIGHT_SERVO_DEG"] if dir_sign == +1 else TUNE["BACK_LEFT_SERVO_DEG"])

def dir_sign_from_string(s: str) -> int:
    s = (s or "").strip().upper()
    return +1 if s == "RIGHT" else -1

# ---- Helpers for wall-escape ----
def yaw_err_deg(yaw_ref, yaw_now): return abs(wrap180(yaw_ref - yaw_now))

def choose_escape_dir(dL, dR):
    # +1 RIGHT, -1 LEFT favor more open side
    if dL >= 0 and dR >= 0: return +1 if dR >= dL else -1
    if dL >= 0 and dR < 0:  return -1 if dL > 0 else +1
    if dR >= 0 and dL < 0:  return +1
    return +1

# =================== MAIN ===================
def main():
    cfg, cfg_path = load_latest_yaml()
    print("Loaded vision config:", cfg_path)

    # ---- Serial & parsing ----
    ser = None
    device = TUNE["ROBOT_PORT"]
    baud = TUNE["ROBOT_BAUD"]

    last_tlm_t = 0.0
    last_pong_t = 0.0
    last_ping_t = 0.0
    last_reset_t = -1e9

    rx_buf = b""
    dist_done_re = re.compile(r"^DIST_DONE")
    dist_prog_re = re.compile(r"^DIST,cm=([-0-9.]+)")

    # --- Arduino reset sync ---
    program_start_t = time.time()
    had_ready_once = False
    arduino_reset_request = False

    def open_serial():
        nonlocal ser, rx_buf, last_yaw_t, arduino_armed
        rx_buf = b''
        last_yaw_t = None
        arduino_armed = False
        if serial is None: return
        try:
            ser = serial.Serial(device, baudrate=baud, timeout=0.01, write_timeout=0.2, dsrdtr=False)
            time.sleep(0.30)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print("Serial OK on", device)
        except Exception as e:
            print("WARNING: serial not open ->", e)
            ser = None

    def safe_write(line_bytes):
        nonlocal ser
        motion = line_bytes.strip().split(b',', 1)[0].upper() in (
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
                return False
            ser.write(line_bytes)
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

    # --- Telemetry ---
    last_yaw = 0.0  # Display placeholder only; last_yaw_t marks a real sample.
    last_yaw_t = None
    last_tlm_line = ''
    last_US = {"dF": -1, "dL": -1, "dR": -1, "dB": -1}
    us_ema  = {"dF": -1.0, "dL": -1.0, "dR": -1.0, "dB": -1.0}
    arduino_armed = False

    # encoder & distance move flags
    last_enc_cm = 0.0
    dist_done_flag = False
    last_dist_progress_cm = None

    def yaw_ready():
        return (last_yaw_t is not None and
                0.0 <= time.monotonic() - last_yaw_t < TUNE['STALE_TLM_SEC'])

    def sane_cm(x):
        if x is None or x < 0:
            return -1
        return float(min(x, TUNE["US_MAX_CM"]))

    def read_tlm():
        nonlocal rx_buf, last_yaw, last_tlm_t, last_US, us_ema, last_pong_t, arduino_armed
        nonlocal last_enc_cm, dist_done_flag, last_dist_progress_cm
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
                    arduino_armed = False
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

        except Exception as e:
            print("Serial read error:", e)
            try:
                ser.close()
            except Exception:
                pass
            open_serial()

    # ---- Camera ----
    cap, using_dummy = open_camera()
    if cap is None:
        return

    # we can now define restart_program safely
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

    # ============= State machine IDs =============
    (STATE_WAIT_START,
     STATE_ESC_S1_FWD_L, STATE_ESC_S2_BACK_L, STATE_ESC_S3_FWD_L, STATE_ESC_S3_FWD_HOLD, STATE_ESC_S4_FWD_RTN,
     STATE_RUN, STATE_CORNER_TURNING, STATE_CORNER_BACKCENTER,
     STATE_FLIP_WAIT, STATE_FLIP_TURN, STATE_FINAL_HOLD,
     STATE_POST12_TURN, STATE_POST12_FWD, STATE_DONE,
     STATE_PARK_SEEK, STATE_PARK_APPROACH, STATE_PARK_ALIGN, STATE_PARK_BACK_IN, STATE_PARKED,
     STATE_PARK_TO_30, STATE_PARK_EXTRA_FWD, STATE_PARK_TURN_IN,
     # ===== LAST CORNER SEQUENCE =====
     STATE_LAST_PREP_FWD, STATE_LAST_BACK, STATE_LAST_CORNER_TURN,
     STATE_LAST_NOAVOID_HOLD, STATE_LAST_WAIT_FRONT_STOP,
     STATE_LAST_CHOICE_TURN, STATE_LAST_FWD1, STATE_LAST_RET1,
     STATE_LAST_FWD2, STATE_LAST_TURN_OPP, STATE_LAST_FWD3, STATE_LAST_FINAL_RET
     ) = range(35)
    state = STATE_WAIT_START

    # ======== Main run bookkeeping ========
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

    # HUD
    line_rect_blue = None
    line_rect_orange = None

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

    # ---------- First-Step Pillar AVOID ----------
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

    # ---------- Pillar suppression ----------
    pillar_suppress = False

    # Steering vars
    err_ema = 0.0
    last_steer_cmd = None
    last_tx = 0.0

    # Wall avoid FSM
    avoid_active = False
    avoid_side = None
    avoid_until = -1.0

    # --- Encoder-stuck escape ---
    enc_progress_cm_at = 0.0
    enc_progress_t = time.time()
    enc_stuck_active = False
    enc_stuck_phase = "EIDLE"  # EIDLE, EBACK, ETURN, ECHECK
    enc_cmd_sent = False
    enc_waiting_dist = False
    enc_cooldown_until = -1.0
    enc_turn_start_t = -1.0

    # --- Legacy wall-escape (inactive but kept) ---
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

    # ===== LAST CORNER SEQUENCE VARS =====
    lastseq_active = False
    lastseq_main_yaw = None
    lastseq_hold_until = -1.0
    last_first_turn_sign = 0   # -1 left, +1 right (T1 direction)
    last_turn_target = None
    last_turn_start_t = -1.0
    last_fwd_cmd_sent = False
    last_fwd_waiting_dist = False
    last_fwd3_start_t = None   # for timeout in LAST_FWD3
    last_no_avoid_override = False  # disable wall/mag avoid during last seq

    # Pillar lateral position snapshot at trigger
    pillar_pre_cx = None
    pillar_side_frac = 0.0     # 0..1 normalize how close pillar is to its wall
    pillar_fwd_cmd_sent = False
    pillar_fwd_waiting_dist = False

    # HUD window
    cv.namedWindow("robot", cv.WINDOW_AUTOSIZE)

    # --- helpers to send ---
    def send_center_abs_deg(steer_deg, speed_pwm):
        steer_deg = int(max(LEFT_LIMIT, min(RIGHT_LIMIT, int(steer_deg))))
        steer_norm = deg_to_norm(steer_deg)
        spd = max(MIN_SPEED, min(255, int(speed_pwm)))
        safe_write(f"CENTER,{steer_norm:.3f},{spd}\n".encode("ascii"))

    def send_center_deg(steer_deg, speed_pwm):
        send_center_abs_deg(steer_deg, speed_pwm)

    def send_back(speed_pwm):
        spd = max(0, min(255, int(speed_pwm)))
        safe_write(f"BACK,{spd}\n".encode("ascii"))

    def send_backc_norm(norm, speed_pwm):
        spd = max(0, min(255, int(speed_pwm)))
        safe_write(f"BACKC,{norm:.3f},{spd}\n".encode("ascii"))

    def send_backc_deg(steer_deg, speed_pwm):
        u = deg_to_norm(steer_deg)
        send_backc_norm(u, speed_pwm)

    # Encoder move helpers
    def send_fwd_cm(cm, pwm):
        safe_write(f"FWD_CM,{float(cm):.2f},{int(pwm)}\n".encode("ascii"))

    def send_back_cm(cm, pwm):
        safe_write(f"BACK_CM,{float(cm):.2f},{int(pwm)}\n".encode("ascii"))

    # --- ROI helpers ---
    def color_roi_rect(w,h):
        wf, hf = TUNE["COLOR_ROI_WIDTH_FRAC"], TUNE["COLOR_ROI_HEIGHT_FRAC"]
        scale = (1.0 if corridor_idx==0 else TUNE["ROI_SCALE_AFTER_FIRST_TURN"])
        wf = max(0.06, min(0.80, wf*scale))
        hf = max(0.05, min(0.40, hf*scale))
        rw, rh = int(w*wf), int(h*hf)
        x0 = (w-rw)//2; x1 = x0+rw
        y1 = h; y0 = y1-rh
        return (x0,y0,x1,y1)

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

    # --------- steering for yaw centering ----------
    def compute_center_steer_deg(yaw_now):
        nonlocal err_ema, last_steer_cmd
        e = wrap180(yaw_ref - float(yaw_now))
        if abs(e) < TUNE["STEER_DEAD_ERR_DEG"]:
            e = 0.0
        alpha = float(TUNE["ERR_FILTER_ALPHA"])
        alpha = max(1e-3, min(1.0, alpha))
        err_ema = (1.0 - alpha) * err_ema + alpha * e
        kp = float(TUNE["STEER_KP_DEG_PER_DEG"])
        max_off = float(TUNE["STEER_MAX_DEG"])
        if time.time() < post_flip_soft_until:
            kp *= float(TUNE["POST_FLIP_KP_GAIN"])
            max_off = min(max_off, float(TUNE["POST_FLIP_MAX_DEG"]))
        u_deg = float(TUNE["STEER_SIGN"]) * kp * err_ema
        u_deg = max(-max_off, min(max_off, u_deg))
        target = CENTER_DEG + u_deg
        target = int(max(TUNE["CENTER_STEER_MIN"], min(TUNE["CENTER_STEER_MAX"], int(target))))
        target = slew_limit(last_steer_cmd, target, TUNE["STEER_SLEW_DEG"])
        last_steer_cmd = target
        return target

    def at_yaw(target_deg, now_deg, tol_deg):
        return abs(shortest_err(target_deg, now_deg)) <= float(tol_deg)

    def reached_or_timeout(target_deg, start_t, tol_deg, max_s, now_yaw):
        hit = at_yaw(target_deg, now_yaw, tol_deg)
        tmo = (time.time() - start_t) >= float(max_s)
        return (hit or tmo), hit, tmo

    print("READY. Keys: S=start, X=reset Arduino + restart Python, Q=quit.")

    # If user forced a global direction, lock it now
    if TUNE["ALL_TURNS_DIR"] in ("LEFT", "RIGHT") and int(TUNE["LOCK_ALL_12_TURNS_ONE_DIRECTION"]):
        force_all_dir = (-1 if TUNE["ALL_TURNS_DIR"] == "LEFT" else +1)
        dir_locked = True
        print(f"[DIR] Forced all-12 direction from TUNE: {TUNE['ALL_TURNS_DIR']} ({force_all_dir:+d})")

    # --- helper to choose ESC sequence direction ---
    def decide_esc_seq_dir():
        L = sane_cm(last_US.get("dL", -1))
        R = sane_cm(last_US.get("dR", -1))
        raw_dir = (+1 if L < R else -1)   # left<right -> RIGHT-first
        if int(TUNE["ESC_SIDE_INVERT"]):
            raw_dir *= -1
        if float(TUNE["ESC_DIR_SIGN"]) < 0:
            raw_dir *= -1
        return +1 if raw_dir >= 0 else -1

    # --- ESC targets & servos ---
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

    # -------------------- robust pillar detector --------------------
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
    # ----------------------------------------------------------------

    # --- helper: when should we watch encoder for stuck? ---
    def should_watch_encoder(st):
        """
        Return True if robot is supposed to be moving FORWARD under continuous drive,
        so if encoders stop changing we call E-WESC.
        We include normal run, escape forward steps, flip wait crawl, final hold,
        post12 forward push, and last-sequence forward pushes.
        We do NOT include pillar FSM FWD (that's encoder FWD_CM already)
        and do NOT include BACK or TURN phases.
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
            STATE_LAST_PREP_FWD,
            STATE_LAST_NOAVOID_HOLD,
            STATE_LAST_WAIT_FRONT_STOP,
            STATE_LAST_FWD1,
            STATE_LAST_FWD2,
            STATE_LAST_FWD3,
        }
        return (st in forward_states)

    # --- MAIN LOOP ---
    while True:
        now = time.time()
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
            err_ema = 0.0
            last_steer_cmd = None
            corridor_idx = 0
            orientation_offset_deg = 0.0
            set_ref_to_corridor()
            run_start_time = time.time()
            run_end_time = None
            preavoid_available = True

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
        if not ok:
            frame = np.zeros((TUNE["FRAME_H"], TUNE["FRAME_W"], 3), dtype=np.uint8)
            cv.putText(frame, "Frame read failed", (10,30),
                       cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        h, w = frame.shape[:2]

        # ================== DETECTION (RUN only) ==================
        c_x0,c_y0,c_x1,c_y1 = color_roi_rect(w,h)
        roi_color = frame[c_y0:c_y1, c_x0:c_x1]
        k = max(3, TUNE["BLUR_KSIZE"] | 1)
        roi_color_blur = cv.GaussianBlur(roi_color, (k,k), 0)
        color_seen = None
        line_rect_blue = None
        line_rect_orange = None

        if (now >= color_pause_until and detect_enabled
            and not pillar_active and not preavoid_active
            and state == STATE_RUN and not enc_stuck_active):

            hsv = cv.cvtColor(roi_color_blur, cv.COLOR_BGR2HSV)
            if TUNE["SAT_BOOST"] != 1.0:
                hch, sch, vch = cv.split(hsv)
                sch = np.clip(sch.astype(np.float32) *
                              float(TUNE["SAT_BOOST"]), 0, 255).astype(np.uint8)
                hsv = cv.merge([hch, sch, vch])

            m_bl = mask_hsv_ranges(hsv, cfg.get("BLUE", {}))
            m_or = mask_hsv_ranges(hsv, cfg.get("ORANGE", {}))

            def max_rect(mask):
                cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                if not cnts:
                    return None
                c = max(cnts, key=cv.contourArea)
                return cv.boundingRect(c)

            pix_bl = int(np.count_nonzero(m_bl))
            pix_or = int(np.count_nonzero(m_or))
            if pix_bl >= TUNE["MIN_PIX_COLOR"] or pix_or >= TUNE["MIN_PIX_COLOR"]:
                if pix_bl >= pix_or:
                    color_seen = "BLUE"
                    rect = max_rect(m_bl)
                    if rect:
                        line_rect_blue = rect
                else:
                    color_seen = "ORANGE"
                    rect = max_rect(m_or)
                    if rect:
                        line_rect_orange = rect
                color_q.append(color_seen)
            else:
                color_q.clear()
        else:
            color_q.clear()

        # -------------------- PILLARS (RUN only) --------------------
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

        # ===== ALWAYS-ON: magnet walls =====
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

        # ---------- PREAVOID TRIGGER ----------
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
            if (time.time()-last_tx) > 0.008:
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

        # ================== LEGACY PILLAR FSM TRIGGER ==================
        if (not pillar_active) and (not preavoid_active) and (not pillar_suppress) and (not enc_stuck_active) and \
           (pillar_seen is not None) and (pillar_dist_cm is not None) and gate_ok and state in (STATE_RUN,):
            pillar_q.append((pillar_seen, pillar_dist_cm, pillar_bbox))
        else:
            pillar_q.clear()

        # ======== ENTER PILLAR FSM ========
        if (not pillar_active) and (not preavoid_active) and (not pillar_suppress) and \
           (not enc_stuck_active) and len(pillar_q) == TUNE["PILLAR_CONFIRM_FRAMES"] and state == STATE_RUN:

            ps, pd, pb = pillar_q[0]
            if pd <= TUNE["PILLAR_ACCEPT_CM"]:
                pillar_active = True
                pillar_color = ps
                pillar_suppress = True
                corridor_yaw_before_pillar = wrap180(start_yaw + corridor_idx * TUNE["TURN_DEG"] + orientation_offset_deg)

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

        # ================== WALL AVOID (RUN only) ==================
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

        # ================== ENCODER STUCK DETECTION (GLOBAL FORWARD WATCH) ==================
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
                        pillar_suppress = True
                        dist_done_flag = False  # clear any stale DIST_DONE so BACK_CM won't instantly exit
                        safe_write(b"STOP\n")
                        print(f"[E-WESC] TRIGGER (no encoder progress {freeze_time:.2f}s) -> BACK then TURN")

        else:
            # Not in a forward-driving state (or we're busy with pillar/preavoid/E-WESC):
            # reset the baseline timer so stale times don't false-trigger later.
            enc_progress_cm_at = last_enc_cm
            enc_progress_t = now

        # ================== CONTROL FSMs ==================
        steer_deg = CENTER_DEG

        # ---- ENCODER-BASED WALL ESCAPE (E-WESC) ----
        if enc_stuck_active:
            if enc_stuck_phase == "EBACK":
                if not enc_cmd_sent:
                    sdeg = compute_center_steer_deg(last_yaw)
                    send_center_deg(sdeg, 0)
                    send_back_cm(float(TUNE["WESC_BACK_CM"]), int(TUNE["WESC_BACK_PWM"]))
                    enc_cmd_sent = True
                    enc_waiting_dist = True
                    print(f"[E-WESC] BACK_CM {TUNE['WESC_BACK_CM']}cm @ {TUNE['WESC_BACK_PWM']}")
                elif enc_waiting_dist and dist_done_flag:
                    dist_done_flag = False
                    enc_waiting_dist = False
                    enc_cmd_sent = False
                    enc_stuck_phase = "ETURN"
                    print("[E-WESC] BACK done -> TURN to corridor yaw")

            elif enc_stuck_phase == "ETURN":
                target = wrap180(yaw_ref)
                tol = float(TUNE["TURN_TOL_DEG"])
                if not enc_cmd_sent:
                    safe_write(f"TURN_ABS,{target:.1f},{int(TUNE['WESC_TURN_PWM'])}\n".encode("ascii"))
                    enc_cmd_sent = True
                    enc_turn_start_t = time.time()
                    print(f"[E-WESC] TURN_ABS to {target:.1f}° @ {TUNE['WESC_TURN_PWM']}")
                else:
                    done, hit, tmo = reached_or_timeout(
                        target, enc_turn_start_t, tol,
                        TUNE["WESC_MAX_SPIN_S"], last_yaw
                    )
                    if done:
                        safe_write(b"STOP\n")
                        enc_cmd_sent = False
                        enc_stuck_phase = "ECHECK"
                        print(f"[E-WESC] TURN {'OK' if hit else 'TIMEOUT'} -> CHECK")

            elif enc_stuck_phase == "ECHECK":
                aligned = abs(shortest_err(yaw_ref, last_yaw)) <= float(TUNE["WESC_EXIT_TOL_DEG"])
                front_clear = (dF < 0) or (dF >= float(TUNE["WESC_EXIT_FRONT_CM"]))

                if aligned and front_clear:
                    enc_stuck_active = False
                    enc_stuck_phase = "EIDLE"
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
                else:
                    enc_stuck_phase = "EBACK"
                    enc_cmd_sent = False
                    enc_waiting_dist = False
                    print("[E-WESC] REPEAT cycle")

        # ---- ESCAPE SEQUENCE (before main run) ----
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
                    if (time.time()-last_tx) > 0.006:
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
                    if (time.time()-last_tx) > 0.006:
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
                    if (time.time()-last_tx) > 0.006:
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
            if (time.time()-last_tx) > 0.008:
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
                    if (time.time()-last_tx) > 0.006:
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

        # ---- PILLAR FLOW ----
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
                    if (time.time()-last_tx) > 0.008:
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

        # ---- CORNER FLOW ----
        elif state == STATE_RUN and corner_color is not None:
            next_corner_num = turn_count + 1
            FRONT_THR = front_turn_cm_for(next_corner_num)
            if dF >= 0 and dF < FRONT_THR:
                # last-corner custom path or normal corner path
                if int(TUNE["LAST_ENABLED"]) and next_corner_num == int(TUNE["MAX_TURNS"]):
                    lastseq_active = True
                    last_no_avoid_override = True
                    lastseq_main_yaw = wrap180(start_yaw + corridor_idx * TUNE["TURN_DEG"] + orientation_offset_deg)
                    state = STATE_LAST_PREP_FWD
                    print(f"[LAST] Begin last-corner sequence. Main yaw {lastseq_main_yaw:.1f}°")
                else:
                    state = STATE_CORNER_TURNING
                    corner_turn_start_t = time.time()
                    print(f"[CORNER] start {current_corner_mode} turn toward yaw={intended_corner_yaw:.1f} "
                          f"(thr={FRONT_THR:.1f}cm, turn#{next_corner_num})")
            else:
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
                    steer_deg = int(max(LEFT_LIMIT, min(RIGHT_LIMIT, steer_deg + bias_sign * mag)))
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
                        steer_deg = int(max(LEFT_LIMIT,
                                            min(RIGHT_LIMIT, steer_deg + bias_sign * mag)))

                if (time.time()-last_tx) > 0.008:
                    send_center_deg(steer_deg, spd)
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
                steer_deg = int(max(LEFT_LIMIT, min(RIGHT_LIMIT, steer_deg + bias_sign * mag)))
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
                    steer_deg = int(max(LEFT_LIMIT,
                                        min(RIGHT_LIMIT, steer_deg + bias_sign * mag)))

            if (time.time()-last_tx) > 0.008:
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
                    if (time.time()-last_tx) > 0.006:
                        send_backc_deg(fixed_deg, TUNE["TURN_SPEED"])
                        last_tx = time.time()
                else:
                    if (time.time()-last_tx) > 0.006:
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
                    see_color_slow = False
                    color_pause_until = time.time() + TUNE["COLOR_SUSPEND_SEC"]
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
            if (time.time()-last_tx) > 0.010:
                send_backc_deg(steer_deg, TUNE["BACK_STOP_SPEED"])
                last_tx = time.time()
            if dB >= 0 and dB < TUNE["BACK_STOP_CM"]:
                safe_write(b"STOP\n")
                corner_color = None
                see_color_slow = False
                color_pause_until = time.time() + TUNE["COLOR_SUSPEND_SEC"]
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

        # ====================== LAST CORNER SEQUENCE ======================
        elif state == STATE_LAST_PREP_FWD:
            steer_deg = compute_center_steer_deg(last_yaw)
            if (time.time()-last_tx) > 0.008:
                send_center_deg(steer_deg, int(TUNE["DRIVE_SPEED"]))
                last_tx = time.time()
            if dF >= 0 and dF <= float(TUNE["LAST_PRETURN_FRONT_CM"]):
                safe_write(b"STOP\n")
                last_fwd_cmd_sent = False
                last_fwd_waiting_dist = False
                state = STATE_LAST_BACK
                print(f"[LAST] Front {dF:.1f}cm <= {TUNE['LAST_PRETURN_FRONT_CM']:.1f} "
                      f"-> BACK {TUNE['LAST_BACK_ENC_CM']}cm")

        elif state == STATE_LAST_BACK:
            if not last_fwd_cmd_sent:
                send_center_deg(compute_center_steer_deg(last_yaw), 0)
                send_back_cm(float(TUNE["LAST_BACK_ENC_CM"]), int(TUNE["LAST_BACK_PWM"]))
                last_fwd_cmd_sent = True
                last_fwd_waiting_dist = True
            elif last_fwd_waiting_dist and dist_done_flag:
                dist_done_flag = False
                last_fwd_waiting_dist = False
                safe_write(b"STOP\n")
                corner_turn_start_t = time.time()
                state = STATE_LAST_CORNER_TURN
                print(f"[LAST] BACK done -> FORWARD corner turn toward "
                      f"{intended_corner_yaw:.1f}°")

        elif state == STATE_LAST_CORNER_TURN:
            tol = float(TUNE["LAST_CORNER_TURN_TOL_DEG"])
            done, hit, tmo = reached_or_timeout(
                intended_corner_yaw,
                corner_turn_start_t,
                tol,
                TUNE["LAST_CORNER_TURN_MAX_S"],
                last_yaw
            )
            if not done:
                fixed_deg = select_servo_deg("FORWARD", current_corner_dir)  # forward fixed
                if (time.time()-last_tx) > 0.006:
                    send_center_abs_deg(fixed_deg, TUNE["LAST_CORNER_TURN_SPEED"])
                    last_tx = time.time()
            else:
                safe_write(b"STOP\n")
                old_idx = corridor_idx
                corridor_idx = target_idx

                # keep yaw_ref at intended custom yaw for last corner if it's last turn,
                # else use normal corridor yaw
                if int(TUNE["LAST_ENABLED"]) and (turn_count + 1) == int(TUNE["MAX_TURNS"]):
                    yaw_ref = intended_corner_yaw
                else:
                    set_ref_to_corridor()

                last_no_avoid_override = True
                lastseq_hold_until = time.time() + float(TUNE["LAST_NO_AVOID_HOLD_S"])
                reason = "TIMEOUT" if tmo else "OK"
                print(f"[LAST] Corner {reason} -> HOLD forward-centering "
                      f"{TUNE['LAST_NO_AVOID_HOLD_S']:.2f}s")
                state = STATE_LAST_NOAVOID_HOLD

        elif state == STATE_LAST_NOAVOID_HOLD:
            steer_deg = compute_center_steer_deg(last_yaw)
            if (time.time()-last_tx) > 0.008:
                send_center_deg(steer_deg, int(TUNE["DRIVE_SPEED"]))
                last_tx = time.time()
            if time.time() >= lastseq_hold_until:
                safe_write(b"STOP\n")
                state = STATE_LAST_WAIT_FRONT_STOP
                print(f"[LAST] Hold done -> continue forward until F < "
                      f"{TUNE['LAST_WAIT_FRONT_STOP_CM']:.1f}cm")

        elif state == STATE_LAST_WAIT_FRONT_STOP:
            steer_deg = compute_center_steer_deg(last_yaw)
            if (time.time()-last_tx) > 0.008:
                send_center_deg(steer_deg, int(TUNE["DRIVE_SPEED"]))
                last_tx = time.time()
            thr = float(TUNE["LAST_WAIT_FRONT_STOP_CM"])
            if dF >= 0 and dF < thr:
                safe_write(b"STOP\n")

                # decide first direction: +1 right, -1 left based on which side is more open
                last_first_turn_sign = (
                    -1 if ( (dL >= 0 and dR >= 0 and dL > dR) or (dR < 0 and dL >= 0) )
                    else +1
                )

                # Turn T1 (LAST_T1_DEG) in that direction
                last_turn_target = wrap180(
                    yaw_ref + last_first_turn_sign * float(TUNE["LAST_T1_DEG"])
                )
                last_turn_start_t = time.time()
                safe_write(
                    f"TURN_ABS,{last_turn_target:.1f},{int(TUNE['LAST_T1_SPEED'])}\n".encode("ascii")
                )
                state = STATE_LAST_CHOICE_TURN
                print(f"[LAST] F<{thr:.1f}cm -> FIRST {TUNE['LAST_T1_DEG']:.0f}° "
                      f"{'LEFT' if last_first_turn_sign<0 else 'RIGHT'} "
                      f"to {last_turn_target:.1f}°")

        elif state == STATE_LAST_CHOICE_TURN:
            tol = float(TUNE["LAST_T1_TOL_DEG"])
            done, hit, tmo = reached_or_timeout(
                last_turn_target,
                last_turn_start_t,
                tol,
                TUNE["LAST_T1_MAX_S"],
                last_yaw
            )
            if done:
                safe_write(b"STOP\n")
                last_fwd_cmd_sent = False
                last_fwd_waiting_dist = False
                state = STATE_LAST_FWD1
                print(f"[LAST] First turn {'OK' if hit else 'TIMEOUT'} -> FWD "
                      f"{TUNE['LAST_FWD1_CM']:.1f}cm")

        elif state == STATE_LAST_FWD1:
            if not last_fwd_cmd_sent:
                send_center_deg(compute_center_steer_deg(last_yaw), 0)
                send_fwd_cm(float(TUNE["LAST_FWD1_CM"]), int(TUNE["DRIVE_SPEED"]))
                last_fwd_cmd_sent = True
                last_fwd_waiting_dist = True
            elif last_fwd_waiting_dist and dist_done_flag:
                dist_done_flag = False
                last_fwd_waiting_dist = False
                safe_write(b"STOP\n")

                # RET1: SAME direction as first turn (fix: not mirrored)
                ret1_sign = last_first_turn_sign
                ret_deg = float(TUNE["LAST_T1_DEG"])
                last_turn_target = wrap180(lastseq_main_yaw + ret1_sign * ret_deg)
                last_turn_start_t = time.time()
                safe_write(
                    f"TURN_ABS,{last_turn_target:.1f},{int(TUNE['LAST_T1_SPEED'])}\n".encode("ascii")
                )
                state = STATE_LAST_RET1
                print(f"[LAST] FWD1 done -> RET1 {ret_deg:.0f}° "
                      f"{'LEFT' if ret1_sign<0 else 'RIGHT'} "
                      f"to {last_turn_target:.1f}° (same dir as T1)")

        elif state == STATE_LAST_RET1:
            tol = float(TUNE["LAST_T1_TOL_DEG"])
            done, hit, tmo = reached_or_timeout(
                last_turn_target,
                last_turn_start_t,
                tol,
                TUNE["LAST_T1_MAX_S"],
                last_yaw
            )
            if done:
                safe_write(b"STOP\n")
                last_fwd_cmd_sent = False
                last_fwd_waiting_dist = False
                state = STATE_LAST_FWD2
                print(f"[LAST] RET1 {'OK' if hit else 'TIMEOUT'} -> FWD "
                      f"{TUNE['LAST_FWD2_CM']:.1f}cm")

        elif state == STATE_LAST_FWD2:
            if not last_fwd_cmd_sent:
                send_center_deg(compute_center_steer_deg(last_yaw), 0)
                send_fwd_cm(float(TUNE["LAST_FWD2_CM"]), int(TUNE["DRIVE_SPEED"]))
                last_fwd_cmd_sent = True
                last_fwd_waiting_dist = True
            elif last_fwd_waiting_dist and dist_done_flag:
                dist_done_flag = False
                last_fwd_waiting_dist = False
                safe_write(b"STOP\n")

                opp_sign = -last_first_turn_sign  # second turn is opposite sign of first
                last_turn_target = wrap180(
                    yaw_ref + opp_sign * float(TUNE["LAST_T2_DEG"])
                )
                last_turn_start_t = time.time()
                safe_write(
                    f"TURN_ABS,{last_turn_target:.1f},{int(TUNE['LAST_T2_SPEED'])}\n".encode("ascii")
                )
                state = STATE_LAST_TURN_OPP
                print(f"[LAST] FWD2 done -> SECOND {TUNE['LAST_T2_DEG']:.0f}° "
                      f"{'LEFT' if opp_sign<0 else 'RIGHT'} "
                      f"to {last_turn_target:.1f}°")

        elif state == STATE_LAST_TURN_OPP:
            tol = float(TUNE["LAST_T2_TOL_DEG"])
            done, hit, tmo = reached_or_timeout(
                last_turn_target,
                last_turn_start_t,
                tol,
                TUNE["LAST_T2_MAX_S"],
                last_yaw
            )
            if done:
                safe_write(b"STOP\n")
                last_fwd_cmd_sent = False
                last_fwd_waiting_dist = False
                last_fwd3_start_t = None
                state = STATE_LAST_FWD3
                print(f"[LAST] Second turn {'OK' if hit else 'TIMEOUT'} -> FWD "
                      f"{TUNE['LAST_FWD3_CM']:.1f}cm")

        elif state == STATE_LAST_FWD3:
            # FWD3 with timeout safety
            if not last_fwd_cmd_sent:
                send_center_deg(compute_center_steer_deg(last_yaw), 0)
                send_fwd_cm(float(TUNE["LAST_FWD3_CM"]), int(TUNE["DRIVE_SPEED"]))
                last_fwd_cmd_sent = True
                last_fwd_waiting_dist = True
                last_fwd3_start_t = time.time()
            elif last_fwd_waiting_dist:
                # normal completion
                if dist_done_flag:
                    dist_done_flag = False
                    last_fwd_waiting_dist = False
                    safe_write(b"STOP\n")

                    ret_deg = float(TUNE["LAST_FINAL_RET_DEG"])
                    final_target = wrap180(lastseq_main_yaw + ret_deg)
                    last_turn_target = final_target
                    last_turn_start_t = time.time()
                    safe_write(
                        f"TURN_ABS,{final_target:.1f},{int(TUNE['LAST_FINAL_RET_SPEED'])}\n".encode("ascii")
                    )
                    state = STATE_LAST_FINAL_RET
                    print(f"[LAST] FWD3 done -> FINAL return to main yaw "
                          f"{final_target:.1f}°")

                # timeout safety
                elif last_fwd3_start_t is not None and \
                     (time.time() - last_fwd3_start_t) >= float(TUNE["LAST_FWD3_TIMEOUT_S"]):
                    last_fwd_waiting_dist = False
                    safe_write(b"STOP\n")

                    ret_deg = float(TUNE["LAST_FINAL_RET_DEG"])
                    final_target = wrap180(lastseq_main_yaw + ret_deg)
                    last_turn_target = final_target
                    last_turn_start_t = time.time()
                    safe_write(
                        f"TURN_ABS,{final_target:.1f},{int(TUNE['LAST_FINAL_RET_SPEED'])}\n".encode("ascii")
                    )
                    state = STATE_LAST_FINAL_RET
                    print(f"[LAST] FWD3 TIMEOUT ({TUNE['LAST_FWD3_TIMEOUT_S']:.2f}s) "
                          f"-> FINAL return to main yaw {final_target:.1f}°")

        elif state == STATE_LAST_FINAL_RET:
            tol = float(TUNE["LAST_FINAL_RET_TOL_DEG"])
            done, hit, tmo = reached_or_timeout(
                last_turn_target,
                last_turn_start_t,
                tol,
                TUNE["LAST_FINAL_RET_MAX_S"],
                last_yaw
            )
            if done:
                safe_write(b"STOP\n")
                last_no_avoid_override = False
                lastseq_active = False
                state = STATE_DONE
                run_end_time = time.time()
                print("[LAST] Sequence complete -> STOP (DONE)")

        # ---- FLIP after turn 8 ----
        elif state == STATE_FLIP_WAIT:
            steer_deg = compute_center_steer_deg(last_yaw)
            if (time.time()-last_tx) > 0.008:
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

        # ---- FINAL_HOLD -> POST12 / DONE ----
        elif state == STATE_FINAL_HOLD:
            steer_deg = compute_center_steer_deg(last_yaw)
            if (time.time()-last_tx) > 0.008:
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
                    state = STATE_DONE
                    print("[FINAL] Hold complete -> STOP (DONE)")

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
                if (time.time()-last_tx) > 0.006:
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
            if (time.time()-last_tx) > 0.008:
                send_center_deg(steer_deg, int(TUNE["POST12_FWD_SPEED"]))
                last_tx = time.time()
            thr_cm = float(TUNE["POST12_FRONT_STOP_CM"])
            if dF >= 0 and dF < thr_cm:
                safe_write(b"STOP\n")
                state = STATE_DONE
                print(f"[POST12] Front {dF:.1f}cm < {thr_cm:.1f}cm -> STOP (DONE)")

        elif state in (STATE_PARK_SEEK, STATE_PARK_APPROACH, STATE_PARK_ALIGN,
                       STATE_PARK_BACK_IN, STATE_PARKED,
                       STATE_PARK_TO_30, STATE_PARK_EXTRA_FWD, STATE_PARK_TURN_IN):
            # legacy slot, not used in this challenge
            pass

        # ================== CORNER LATCH (when line color seen) ==================
        if (not pillar_active) and (not preavoid_active) and (not enc_stuck_active) and \
           len(color_q) == TUNE["COLOR_CONFIRM_FRAMES"] and state == STATE_RUN:

            corner_color = color_q[0]
            color_q.clear()
            see_color_slow = True
            color_pause_until = now + TUNE["COLOR_SUSPEND_SEC"]
            yaw_at_corner_start = float(last_yaw)
            pillar_suppress = True

            # Latch global dir from first seen line (BLUE => LEFT, ORANGE => RIGHT)
            if not dir_locked and dir_first_base is None and turn_count == 0:
                latched = (-1 if corner_color == "BLUE" else +1)
                force_all_dir = latched
                dir_locked = True
                print(f"[DIR] LATCH: first line {corner_color} -> "
                      f"{'LEFT(-1)' if latched==-1 else 'RIGHT(+1)'} for all 12 turns")

            # pick base_dir THIS corner
            if force_all_dir is not None:
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

            print(f"[CORNER] line={corner_color}  dir={'LEFT' if base_dir==-1 else 'RIGHT'}  "
                  f"mode={current_corner_mode}  -> idx={target_idx}, yaw={intended_corner_yaw:.1f}")

        # ================== HUD ==================
        vis = frame.copy()

        # Line ROI
        cv.rectangle(vis, (c_x0,c_y0),(c_x1,c_y1),(0,165,255), 2)
        if line_rect_blue is not None:
            x,y,ww,hh = line_rect_blue
            cv.rectangle(vis, (c_x0+x, c_y0+y),
                         (c_x0+x+ww, c_y0+y+hh), (255,128,0), 2)
        if line_rect_orange is not None:
            x,y,ww,hh = line_rect_orange
            cv.rectangle(vis, (c_x0+x, c_y0+y),
                         (c_x0+x+ww, c_y0+y+hh), (0,128,255), 2)

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
            STATE_LAST_PREP_FWD:"LAST_PREP_FWD", STATE_LAST_BACK:"LAST_BACK", STATE_LAST_CORNER_TURN:"LAST_TURN",
            STATE_LAST_NOAVOID_HOLD:"LAST_HOLD", STATE_LAST_WAIT_FRONT_STOP:"LAST_WAITF",
            STATE_LAST_CHOICE_TURN:"LAST_T1", STATE_LAST_FWD1:"LAST_FWD1", STATE_LAST_RET1:"LAST_RET1",
            STATE_LAST_FWD2:"LAST_FWD2", STATE_LAST_TURN_OPP:"LAST_T2", STATE_LAST_FWD3:"LAST_FWD3",
            STATE_LAST_FINAL_RET:"LAST_FINAL_RET"
        }[state]
        esc_flag = " E-WESC" if enc_stuck_active else ""
        yaw_text = f"{last_yaw:5.1f}" if yaw_ready() else "   --"

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

        # ================== KEYS ==================
        key = cv.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q')):
            safe_write(b"STOP\n")
            break

        elif key in (ord('s'), ord('S')):
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
            err_ema = 0.0
            last_steer_cmd = None
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
            enc_cooldown_until = -1.0
            enc_progress_cm_at = last_enc_cm
            enc_progress_t = time.time()

            lastseq_active = False
            last_no_avoid_override = False

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
