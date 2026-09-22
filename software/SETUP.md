# Setup

These instructions are for our original Jetson Nano, Arduino Uno R3 and the code in this repository. The Orin Nano OS image and Arduino Mega pin map belong to a different setup.

Keep drive-motor power disconnected during installation and calibration. Close all driving programs before opening the camera or Arduino serial port from another tool.

## 1. Get the files

On the Jetson:

```bash
git clone https://github.com/Abdulaziz-nasser/IRD-MAX.git
cd IRD-MAX
git rev-parse HEAD
```

Keep the commit ID with a test result. It identifies the files used. For a ZIP copy, choose GitHub's **Code > Download ZIP** and record the commit ID shown when downloading. Extract the archive before opening individual files.

## 2. Check Python and the camera environment

The calibration tools use Python 3.6-compatible syntax. OpenCV needs GUI support, while CSI capture also needs GStreamer and NVIDIA's camera driver. Keep the working Jetson OpenCV build when installing these tools.

```bash
python3 --version
python3 -c "import cv2; print(cv2.__version__); print(cv2.__file__); print(cv2.getBuildInformation())"
```

Check that GStreamer is enabled for CSI use. A successful import alone does not prove the camera can deliver frames.

Imports are listed in [requirements.txt](requirements.txt). On a Jetson with working OpenCV, install missing supporting packages through its Ubuntu package manager:

```bash
sudo apt-get update
sudo apt-get install python3-numpy python3-yaml python3-serial
python3 -c "import numpy, yaml, serial; print(numpy.__version__, yaml.__version__, serial.VERSION)"
```

These commands install the dependencies but do not record the versions used in a test. Save the actual environment with:

```bash
cat /etc/os-release
cat /etc/nv_tegra_release
python3 --version
python3 -c "import cv2, numpy, yaml, serial; print('OpenCV', cv2.__version__); print('NumPy', numpy.__version__); print('PyYAML', yaml.__version__); print('pyserial', serial.VERSION)"
```

The driving programs use OpenCV's two-result `findContours` interface. Check this when rebuilding the Nano image because a helper test alone does not confirm that a fresh image can run the driving programs.

## 3. Upload the Uno code

Use a computer with Arduino IDE and the **Arduino AVR Boards** package.

1. Install the [DFRobot BNO055 library](https://github.com/DFRobot/DFRobot_BNO055). The code includes `DFRobot_BNO055.h`, not the Adafruit library.
2. Make sure Servo is installed. Wire is supplied with the board package.
3. Select the controller code for your challenge from the table below. Copy only that `.ino` file into a folder with the same name, then open it in Arduino IDE. Do not open all three controller files together because each one defines its own setup and loop.
4. Select **Arduino Uno**, choose its USB port, then Verify and Upload.
5. Open Serial Monitor at **115200 baud**. After reset, expect `READY` and lines beginning with `TLM,`.
6. Close Serial Monitor before starting a Python tool.

| Challenge | Repository code | Local code folder name |
| --- | --- | --- |
| Open | [open_challenge_controller.ino](controller/open_challenge_controller.ino) | `open_challenge_controller` |
| Obstacle | [obstacle_challenge_controller.ino](controller/obstacle_challenge_controller.ino) | `obstacle_challenge_controller` |

The older [ird_max_controller.ino](controller/ird_max_controller.ino) is kept as a development reference and cannot replace the Obstacle controller. Check connections against the [version-specific Uno pin map](../hardware/PINOUT.md), not the Mega pinout from the archive.

## 4. Find the serial port

```bash
python3 -m serial.tools.list_ports
```

The replacement Uno R3 was identified as `/dev/ttyACM0`. The source files still default to `/dev/ttyUSB0`; the commands below override that default without editing code. Use the actual port from the listing if its number differs. Keep Serial Monitor and other serial programs closed.

For a permissions error, check the device's group and your account's groups. On systems where access belongs to `dialout`:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in before retrying. Do not run the driving program with sudo as a workaround.

Read sensors without sending movement commands:

```bash
python3 software/tools/sensor_check.py --port /dev/ttyACM0
```

Opening serial can reset the Uno. Keep motor power disconnected even for a read-only tool. Open firmware has no `enc_cm` or `dB`, so seeing those fields as `not reported` is expected. Obstacle firmware reports both.

## 5. Calibrate colours and distance

Follow [the calibration guide](tools/README.md). The colour tool saves four top-level colour names and HSV ranges inside `software/jetson/config/`.

The [example file](jetson/config/vision.example.yaml) contains starter values from the archive, not measurements from our field. It does not match `vision_*.yaml`, so the driving programs do not automatically select it.

The scripts select the alphabetically last `config/vision_*.yaml` relative to the working directory, unless `VISION_CFG` points to an existing file. Use an explicit path when comparing runs. A legacy error in the Open program mentions `tuner_multi.py`; the colour tool supplied here is [autotune_colors.py](tools/autotune_colors.py).

Encoder calibration applies to the Obstacle controller (source scale 140 counts per 10 cm) or the older development controller (624), not Open. Pass the value from the firmware actually installed; do not copy the other version's scale.

## 6. Check the existing programs

From the repository root, these commands check syntax and the calibration helpers without operating the robot:

```bash
python3 -m py_compile software/jetson/open_challenge.py software/jetson/obstacle_challenge.py software/jetson/slow_with_yaw.py software/jetson/slow_without_yaw.py
python3 -m unittest discover -s software/tests -v
```

For a stationary camera check, use the autotuner with drive-motor power disconnected. It never opens the Arduino port.

For a bench launch, change into `software/jetson` and enter the colour-file path printed by the autotuner. Use an absolute path or a path relative to this directory, without adding quote characters when answering the prompt.

```bash
cd software/jetson
read -r -p "Calibrated colour YAML path: " VISION_CFG
export VISION_CFG
```

Choose **one** launch command and upload its matching Arduino controller first. The file check prevents a mistyped calibration path from silently selecting another file. `ALLOW_NO_CAMERA=0` prevents a camera-open failure from intentionally falling back to a dummy frame; still confirm live images before starting.

Open Challenge:

```bash
if [ -f "$VISION_CFG" ]; then
    ROBOT_PORT=/dev/ttyACM0 ALLOW_NO_CAMERA=0 python3 open_challenge.py
else
    printf '%s\n' 'Calibration file not found; program not started.'
fi
```

Obstacle Challenge:

```bash
if [ -f "$VISION_CFG" ]; then
    ROBOT_PORT=/dev/ttyACM0 ALLOW_NO_CAMERA=0 python3 obstacle_challenge.py
else
    printf '%s\n' 'Calibration file not found; program not started.'
fi
```

Check the printed configuration path and serial connection before pressing start. To repeat the historical yaw comparison, use `slow_with_yaw.py` or `slow_without_yaw.py` with the controller and settings recorded for that test; do not treat a comparison as the competition release.

The existing programs use **S** to start and **Q** to send STOP and quit while their OpenCV window is focused. The physical start button connects to the Uno. Keyboard controls are for bench testing, not a replacement for the competition start procedure.

Do not treat closing a terminal, Ctrl+C or unplugging USB as an emergency stop. The uploaded Uno code has no general serial-command-loss watchdog. Keep a physical drive-power disconnect available and verify stopping with the wheels raised before floor testing.

## Verification limits

The syntax and helper tests run without the robot. Arduino Verify/Upload, camera frames, serial permissions and powered start/stop checks still need the actual equipment. The terminal commands are for bench testing and do not install an automatic competition start service.

## References

- [Arduino code-folder requirements](https://docs.arduino.cc/arduino-cli/sketch-specification/)
- [DFRobot BNO055 library](https://github.com/DFRobot/DFRobot_BNO055)
- [OpenCV video backends](https://docs.opencv.org/4.x/d0/da7/videoio_overview.html)
- [NVIDIA JetPack archive](https://developer.nvidia.com/embedded/jetpack-archive)
