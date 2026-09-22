#!/usr/bin/env python3
"""Sample track colours and save the flat HSV YAML used by the current programs.

Adapted from tools/autotune_colors.py in one_camera_robot_rebuild_source.zip.
This tool does not open serial, run a mission or control the motors.
"""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import tempfile

import numpy as np
import yaml

COLORS = ("BLUE", "ORANGE", "RED", "GREEN")


def circular_hue_ranges(hues, coverage=0.98, padding=3):
    values = np.sort(np.asarray(hues, dtype=np.int16).reshape(-1))
    if not values.size:
        raise ValueError("Select a coloured area first.")
    count = max(1, min(values.size, int(round(values.size * coverage))))
    extended = np.concatenate([values, values + 180])
    best_start, best_width = 0, 999
    for index in range(values.size):
        width = int(extended[index + count - 1] - extended[index])
        if width < best_width:
            best_start, best_width = int(extended[index]), width
    if best_width + 2 * padding >= 179:
        return [(0, 179)]
    start = (best_start - padding) % 180
    end = (best_start + best_width + padding) % 180
    return [(start, end)] if start <= end else [(0, end), (start, 179)]


def learn_hsv_ranges(sample_hsv):
    pixels = np.asarray(sample_hsv).reshape(-1, 3)
    pixels = pixels[(pixels[:, 1] >= 35) & (pixels[:, 2] >= 25)]
    if len(pixels) < 16:
        raise ValueError("Not enough coloured pixels. Select the target, avoiding shadows and background.")
    hues = circular_hue_ranges(pixels[:, 0])
    s_low = max(0, int(np.percentile(pixels[:, 1], 2)) - 12)
    # Floor-line detection in the current programs increases saturation.
    # Keep the ceiling open so that boost does not reject the sampled target.
    s_high = 255
    v_low = max(0, int(np.percentile(pixels[:, 2], 2)) - 15)
    v_high = min(255, int(np.percentile(pixels[:, 2], 99)) + 15)
    return [{"low": [int(lo), s_low, v_low], "high": [int(hi), s_high, v_high]}
            for lo, hi in hues]


def clip_rectangle(start, end, shape):
    height, width = shape[:2]
    x0, x1 = sorted([max(0, min(width, int(p[0]))) for p in (start, end)])
    y0, y1 = sorted([max(0, min(height, int(p[1]))) for p in (start, end)])
    return (x0, y0, x1, y1) if x1 - x0 >= 4 and y1 - y0 >= 4 else None


def save_config(colors, output_dir):
    if any(not colors.get(color) for color in COLORS):
        raise ValueError("Sample BLUE, ORANGE, RED and GREEN before saving.")
    target_dir = Path(output_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    prefix = datetime.now(timezone.utc).strftime("vision_%Y%m%d_%H%M%S_%f_")
    descriptor, name = tempfile.mkstemp(prefix=prefix, suffix=".yaml", dir=str(target_dir))
    with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
        handle.write("# HSV samples; check detection under the track lighting before driving.\n")
        yaml.safe_dump(colors, handle, default_flow_style=False)
    return Path(name)


def csi_pipeline(sensor_id):
    return (
        "nvarguscamerasrc sensor-id={} ! ".format(sensor_id)
        + "video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=30/1 ! "
        + "nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! "
        + "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=1"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=("csi", "usb"), default="csi")
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--device", type=int, default=0, help="USB camera index")
    parser.add_argument("--image", help="Use a saved image instead of a camera")
    parser.add_argument("--output-dir",
                        default=str(Path(__file__).resolve().parents[1] / "jetson" / "config"))
    args = parser.parse_args()
    import cv2 as cv

    print("Disconnect drive-motor power and close the driving program.")
    print("1=BLUE 2=ORANGE 3=RED 4=GREEN: select colour, then drag its rectangle and press Enter.")
    print("S saves a new file after all four colours are sampled. Q quits.")
    print("The preview is an HSV mask, not a full driving/obstacle detector.")
    capture = None
    still = None
    colors = {}
    active_color = None
    try:
        if args.image:
            still = cv.imread(args.image)
            if still is None:
                raise ValueError("Could not read the image.")
        else:
            source = csi_pipeline(args.sensor_id) if args.camera == "csi" else args.device
            backend = cv.CAP_GSTREAMER if args.camera == "csi" else cv.CAP_V4L2
            capture = cv.VideoCapture(source, backend)
            if not capture.isOpened():
                raise RuntimeError("Camera did not open. Check its driver and OpenCV/GStreamer support.")
        while True:
            ok, frame = (True, still.copy()) if still is not None else capture.read()
            if not ok or frame is None:
                raise RuntimeError("Camera opened but no frame arrived.")
            preview = frame.copy()
            sampled = ", ".join(color for color in COLORS if color in colors) or "none"
            cv.putText(preview, "Sampled: " + sampled, (10, 24),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv.imshow("calibration", preview)
            if active_color is not None:
                hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
                mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                for band in colors[active_color]:
                    mask = cv.bitwise_or(mask, cv.inRange(
                        hsv, np.array(band["low"], dtype=np.uint8),
                        np.array(band["high"], dtype=np.uint8)))
                cv.imshow("HSV preview", mask)
            key = cv.waitKey(20) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("1"), ord("2"), ord("3"), ord("4")):
                color = COLORS[key - ord("1")]
                x, y, width, height = cv.selectROI("sample", frame, showCrosshair=True, fromCenter=False)
                cv.destroyWindow("sample")
                rectangle = clip_rectangle((x, y), (x + width, y + height), frame.shape)
                if rectangle is None:
                    print("Selection cancelled or too small.")
                    continue
                x0, y0, x1, y1 = rectangle
                try:
                    sample = cv.cvtColor(frame[y0:y1, x0:x1], cv.COLOR_BGR2HSV)
                    colors[color] = learn_hsv_ranges(sample)
                    active_color = color
                    print("Sampled", color, colors[color])
                except ValueError as exc:
                    print(exc)
            elif key in (ord("s"), ord("S")):
                try:
                    print("Saved:", save_config(colors, args.output_dir))
                except ValueError as exc:
                    print(exc)
    finally:
        if capture is not None:
            capture.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled.")
    except Exception as exc:
        raise SystemExit(str(exc))
