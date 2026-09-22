# Arduino Uno Pin Map

This map follows the three Arduino controller files in this repository. Our current board is an Uno R3. Select the [controller for the challenge](../software/README.md#competition-programs); the older development code is not the Obstacle controller.

Disconnect power before checking wiring. Have the coach or an experienced adult check battery and regulator wiring before powering the robot.

## Common connections

| Connection | Uno pin | Detail |
| --- | --- | --- |
| Buzzer | A0 | Buzzer signal. |
| Start button | A1 | INPUT_PULLUP; press connects input to ground. |
| Right ultrasonic | D2 | Single trigger/echo signal. |
| Left ultrasonic | D3 | Single trigger/echo signal. |
| Front ultrasonic | D4 | Single trigger/echo signal. |
| BTS7960 RPWM | D5 | Motor PWM. |
| BTS7960 LPWM | D6 | Motor PWM. |
| BTS7960 R_EN | D7 | Driver enable. |
| BTS7960 L_EN | D8 | Driver enable. |
| Steering servo signal | D9 | Signal only, not servo power. |
| BNO055 SDA | A4 / SDA | Uno I2C data; address 0x28. |
| BNO055 SCL | A5 / SCL | Uno I2C clock. |
| Jetson | USB | Serial at 115200 baud. |

## Connections and settings by controller

| Detail | [Open](../software/controller/open_challenge_controller.ino) | [Obstacle](../software/controller/obstacle_challenge_controller.ino) | [Older development](../software/controller/ird_max_controller.ino) |
| --- | --- | --- | --- |
| Ultrasonic readings | Front, left, right | Front, left, right, rear | Front, left, right |
| Rear ultrasonic | Not used | D10; telemetry field `dB` | Not used |
| Encoder A / CLK | Not used | A2 | A2 |
| Encoder B / DT | Not used | A3 | A3 |
| Encoder telemetry | None | `enc_cm` | `enc_cm` |
| Counts per 10 cm in source | Not applicable | 140 | 624 |
| Servo centre / minimum / maximum | 90 / 65 / 112 degrees | 90 / 60 / 115 degrees | 90 / 60 / 115 degrees |
| Arduino steering sign | Direct normalized mapping | `SERVO_DIR = -1` | `SERVO_DIR = -1` |

These are the values written in the code, not measured wheel angles or a finished encoder calibration. Jetson steering mapping and trim also affect the result. Keep each challenge's Jetson and Arduino files together.

On the Uno R3, the encoder code uses pin-change interrupts for A2/A3. D2/D3 are already assigned to ultrasonic sensors, so generic interrupt examples using those pins do not match our wiring.

The Obstacle controller uses all four ultrasonic sensors, while Open uses three. A missing sensor field does not mean a zero-distance reading.

## Drawing and power references

The [schematics](../schematic/) show how we connect the components in our robot. This table gives the pin assignments in each uploaded controller version. See the [power notes](power/) for supply details and electrical checks.
