# Arduino Uno Pin Map

This comes from [the uploaded Uno code](../software/controller/ird_max_controller.ino). Use it with the [circuit diagram](../schematic/circuit-diagram.png), not the Mega pinout in the rebuild archive.

Disconnect power before checking wiring. Have the coach or an experienced adult check battery and regulator wiring before powering the robot.

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
| Encoder A / CLK | A2 | Quadrature input. |
| Encoder B / DT | A3 | Quadrature input. |
| BNO055 SDA | A4 / SDA | Uno I2C data. |
| BNO055 SCL | A5 / SCL | Uno I2C clock. |
| Jetson | USB | Serial at 115200 baud. |

The IMU address is 0x28. Encoder scale is 624 counts per 10 cm. Servo settings are center 90, minimum 60, maximum 115 degrees, with SERVO_DIR = -1. These are code settings, not measured wheel angles.

The hardware inventory lists four ultrasonic sensors. This firmware reads only front, left and right. No rear pin is defined and no dB value is sent. Do not guess a rear connection from the Mega archive.

See [power notes](power/) for the documented supply arrangement. Check actual regulator outputs and device ratings; a signal-pin map is not a complete power-wiring guide.
