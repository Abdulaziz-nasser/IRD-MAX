# Software

The Jetson reads the camera and decides what to do. The Uno reads the sensors and controls the motor and steering. They communicate over USB serial at 115200 baud.

## Start here

1. Follow [SETUP.md](SETUP.md) to check the environment, upload the Uno code and connect serial.
2. Use the [calibration tools](tools/README.md) with drive-motor power disconnected.
3. Follow the [test procedure](../testing/PROCEDURE.md) before running on the track.

## Files

| File | What it does |
| --- | --- |
| [jetson/o1.py](jetson/o1.py) | Existing camera and driving program, with colour detection, corners, pillar manoeuvres and final-movement states. |
| [jetson/slow_with_yaw.py](jetson/slow_with_yaw.py) | Slow open-track comparison with straight-heading correction. |
| [jetson/slow_without_yaw.py](jetson/slow_without_yaw.py) | Matching comparison with centered steering between corners. It still uses yaw for turns. |
| [controller/ird_max_controller.ino](controller/ird_max_controller.ino) | Uno motor, servo, IMU, encoder, three ultrasonic inputs and start-button code. |
| [tools/autotune_colors.py](tools/autotune_colors.py) | Samples four track colours and writes HSV settings in the format these programs read. |
| [tools/encoder_calibrate.py](tools/encoder_calibrate.py) | Calculates an encoder-scale correction from a measured, hand-rolled distance. |
| [tools/sensor_check.py](tools/sensor_check.py) | Displays yaw, distances and encoder readings; can save them to CSV. |
| [jetson/config/](jetson/config/) | Colour-file format and a labelled example. |
| [tests/test_calibration.py](tests/test_calibration.py) | Offline checks for calibration maths, file saving and telemetry parsing. |

The calibration tools were adapted from the supplied one-camera rebuild source. The rebuild's Mega firmware, mission code and OS installer are not part of this setup. See [the source notes](tools/README.md#source-and-compatibility).

## What runs where

The camera connects to the Jetson. A Python program opens the camera, loads its colour file, receives sensor readings and sends movement commands. The Uno handles the motor outputs and steering signal. It also reports heading, three wall distances and encoder distance.

The slow comparison programs differ in one setting: `STRAIGHT_YAW_ENABLED`. Their matching speeds and turn settings let us compare straight driving without changing both variables at once. The recordings are in [the yaw journal entry](../Journal.md#problem-3--yaw-and-straight-driving).

## Controller compatibility

The Uno code understands `CENTER`, `BACK`, `BACKC`, `TURN_ABS`, `STOP` and the setup commands in [PROTOCOL.md](PROTOCOL.md). It reports front, left and right ultrasonic distances, not a rear distance.

There is a version difference in the existing files: `o1.py` has paths that send `FWD_CM` and `BACK_CM`, while the uploaded Uno code does not implement those distance commands. Its prefix-based parser can mistake `BACK_CM` for `BACK`. Do not assume this file pair supports every manoeuvre. The calibration tools do not use those commands.

The tools leave the driving programs and Uno code unchanged. Their offline tests check file format and calculations, not performance on the robot.
