# Calibration Tools

These tools are separate from the driving programs. Disconnect drive-motor power and close any driving program before checking the camera or sensors. Run the commands from the repository root after completing [setup](../SETUP.md).

## Colour calibration

```bash
python3 software/tools/autotune_colors.py
```

The default is CSI sensor 0, with a 1280 x 720 capture resized to 640 x 480. For USB:

```bash
python3 software/tools/autotune_colors.py --camera usb --device 0
```

For a saved photo, without opening a camera:

```bash
python3 software/tools/autotune_colors.py --image track_photo.jpg
```

1. Put the real floor lines and pillars in view under the track lighting.
2. Press **1** for blue, **2** for orange, **3** for red or **4** for green.
3. Drag a tight rectangle around that target and press Enter. Avoid background, glare and deep shadows.
4. Check the black-and-white HSV preview. White pixels are inside the sampled range. Repeat a selection if it includes unrelated objects.
5. Sample all four colours, then press **S**. The tool prints the path to a new file in `software/jetson/config/`.
6. Press **Q** to close.

Each save creates a new file without overwriting existing calibration. It uses top-level BLUE, ORANGE, RED and GREEN keys, matching the uploaded programs. Red may need two hue bands because OpenCV hue wraps between 179 and 0; the sampler handles that wrap.

The saturation ceiling is kept at 255 because the existing floor-line code increases saturation before applying these ranges. Hue, minimum saturation and brightness still limit the mask. This avoids rejecting the same target solely because of the boost, but does not replace a check under real track lighting.

The preview checks only the HSV ranges. The driving programs also use regions of interest, filters, saturation adjustments and thresholds. Their idle camera view does not test the full detection pipeline, so follow the [test procedure](../../testing/PROCEDURE.md) before a track run. This tool does not calibrate optional MAGNET settings or change driving parameters.

## Yaw and ultrasonic checks

```bash
python3 software/tools/sensor_check.py --port /dev/ttyACM0
```

This displays the fields reported by the selected controller. Open supplies yaw and front, left and right distances; Obstacle also supplies rear and encoder distances. Missing `dB` or `enc_cm` fields appear as `not reported` rather than zero. The examples use `/dev/ttyACM0` for our replacement Uno R3; check [serial-port discovery](../SETUP.md#4-find-the-serial-port) first.

- Rotate the car gently by hand and observe yaw. Record the physical rotation direction and sign rather than assuming the IMU mounting.
- Put a flat target at known distances from each ultrasonic sensor and compare with a ruler.
- With the Obstacle or older development controller installed, roll the car by hand and check that `enc_cm` changes. Open firmware has no encoder telemetry.
- Press Ctrl+C to exit.

To save readings:

```bash
python3 software/tools/sensor_check.py --port /dev/ttyACM0 --log sensor_check.csv
```

The log must be a new filename. Missing fields stay blank, not zero. Opening serial can reset the Uno even though this tool sends no commands.

## Encoder calibration

Use the scale from the firmware actually installed: the Obstacle controller has `COUNTS_PER_10CM = 140`; the older `ird_max_controller.ino` has 624. Neither value is a substitute for measurement on the current drivetrain. The Open controller has no encoder support, so this tool cannot be used with it.

```bash
python3 software/tools/encoder_calibrate.py --port /dev/ttyACM0 --counts-per-10cm 140
```

The command above is for the Obstacle controller. For the older development firmware, pass 624 instead.

Keep drive-motor power disconnected and USB/logic power on. Follow the prompts: place the car at a start mark, enter a measured distance, then roll it there by hand. Keep wheels on the surface and do not reset or unplug the Uno between readings.

```text
new counts per 10 cm = current counts per 10 cm x |reported distance| / measured distance
```

This corrects the existing scale using `enc_cm`, not raw ticks. It cannot compensate for lost counts or wheel slip. Repeat the measurement before deciding on a constant. The tool prints a proposed value but does not edit or upload firmware.

## Offline checks

```bash
python3 -m unittest discover -s software/tests -v
```

The tests cover HSV sampling, red wrap, file saving, telemetry and encoder maths. They run without opening the camera or serial port, so physical sensor calibration is checked separately.

## Source and compatibility

The sampler and manual encoder procedure were adapted from `one_camera_robot_rebuild_source.zip`.

| Archive item | What was kept or changed |
| --- | --- |
| `tools/autotune_colors.py` | Colour sampling; outputs this repository's flat HSV format, without rebuild mission/vision dependencies. |
| `tools/encoder_calibrate.py` | Measured hand-rolled distance; uses Uno enc_cm instead of unsupported raw-tick/reset commands. |
| IMU and ultrasonic checks | Combined into the read-only sensor checker. |
| `config/vision.yaml` | HSV starter ranges retained as a labelled example, not field measurements. |

The tools use Python 3.6-compatible syntax. They do not need the rebuild's Mega firmware, motion library, background service or OS installer. We left out its steering tool because the commands do not match our Uno code.

The helpers were checked offline. Camera capture, GUI interaction, serial timing and physical measurements require checks on the robot.
