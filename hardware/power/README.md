# Power Notes

The car uses a holder with four lithium cells labelled 5000 mAh. We measured approximately 15–18 V at the holder output. The installed computer is the original Jetson Nano, and the RC380 drive motor is marked 7.2 V.

We still need the exact cell and regulator model numbers to confirm the charging limit and current ratings. We check those labels and datasheets with our coach before wiring or charging the pack.

## Battery Values

| Item | Recorded information |
| --- | --- |
| Cells | Four, reported as connected in series. |
| Capacity label | 5000 mAh per cell; a 4S1P pack retains 5000 mAh capacity. |
| Nominal voltage | About 14.8 V if each cell is nominally 3.7 V. |
| Measured holder output | Approximately 15–18 V; this was not a logged load test. |
| Charging limit | Must come from the exact cell manufacturer's specification. Do not assume a 4.4 V charge limit. |

The voltage needs to be checked again with a meter under load. It does not confirm the charging limit or whether the cells are balanced.

## Power Layout and Checks

A DC-DC step-down regulator supplies the Jetson and low-power electronics. The drive branch was described as connecting without a step-down regulator, but the measured pack voltage is higher than the motor's 7.2 V marking. This part of the wiring needs to be checked with our coach before powered testing.

We have not listed separate servo or motor regulators, fuses or pack protection because their exact models have not been confirmed. The table below is a checklist, not a completed load test.

| Branch | Supply requirement to check |
| --- | --- |
| Jetson Nano | Regulated 5 V at the correct input. NVIDIA's barrel-input guidance specifies up to 4 A for demanding loads on the applicable developer kit. |
| Uno and sensors | USB/logic supply with the correct sensor-board voltages and enough current for the connected devices. |
| MG996R servo | Supply matched to the exact servo's voltage and peak-current requirements; do not load the Uno 5 V pin with the steering motor. |
| RC380 drive motor | Supply matched to the motor's 7.2 V rating, through its driver. A motor driver's voltage rating does not make battery voltage suitable for the motor. |

A step-down regulator reduces voltage but does not isolate the circuits. The grounds and high-current wiring must be planned together.

## Sensors and Placement

| Device | Role and check |
| --- | --- |
| BNO055 IMU | Provides yaw. Secure its mounting, note axis direction and keep it away from motor magnetic fields where practical. Check readings while rotating the car by hand. |
| Ultrasonic sensors | Measure space around the car. Compare readings with known distances and make sure the body does not block the sensing faces. |
| Encoder | Measures drivetrain movement. The Obstacle controller uses 140 counts per 10 cm; the older development controller uses 624. The Open controller does not read the encoder. The [manual calibration tool](../../software/tools/#encoder-calibration) checks the scale in the firmware actually installed. |
| IMX477 camera | Connects to the Jetson through CSI. Check its view with the cover fitted and sample colours under the track lighting. |

The inventory lists four ultrasonic sensors. Open publishes front, left and right distances; Obstacle also publishes rear distance. See [the pin map](../PINOUT.md) for connections by controller version.

## Checks to Record

With qualified supervision, record the actual regulator model and output for each connected load. Check the output before attaching electronics and again under the intended load. Keep measured values separate from intended setpoints.

For each power test, record the supply voltage, current where measurable, resets or shutdowns, and any heating. Fuse, protection and switch details must match the hardware installed on the car.

We have not added a measured current budget or regulator load-test table, so this page records the layout and checks rather than a finished electrical validation.

[NVIDIA Jetson Nano Developer Kit User Guide](https://developer.nvidia.com/embedded/dlc/jetson_nano_developer_kit_user_guide)
