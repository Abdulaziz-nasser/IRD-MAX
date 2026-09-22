# Hardware

The Jetson processes camera images and sends driving commands to the Arduino Uno over USB. The Arduino controls the RC380 motor and MG996R steering servo and reads the sensors.

Use the [Uno pin map](PINOUT.md) to check signal connections and the [power notes](power/) for supply information.

| Component | Quantity | Photo or reference |
|---|---|---|
| NVIDIA Jetson Nano | 1 | <a href="https://developer.nvidia.com/embedded/dlc/jetson_nano_developer_kit_user_guide"><img src="images/jetson-nano.png" width="180" alt="NVIDIA Jetson Nano"></a> |
| Arduino Uno R3 with shield | 1 | <a href="https://store.arduino.cc/products/arduino-uno-rev3"><img src="https://cdn.shopify.com/s/files/1/0438/4735/2471/files/A000066_03.front.jpg?v=1727098250" width="130" alt="Arduino Uno with shield — product reference photo"></a> <a href="https://thinkrobotics.com/products/io-sensor-shield-online"><img src="https://cdn.shopify.com/s/files/1/0014/4313/5560/products/H22ce1d6591f647658c827f17694af887r.jpg?v=1667075719" width="130" alt="Arduino Uno with shield — product reference photo"></a> |
| HW-039 module | 1 | <a href="https://kitsguru.com/products/double-bts7960b-dc-43a-stepper-motor-driver-h-bridge-pwm-for-arduino-smart-car"><img src="https://cdn.shopify.com/s/files/1/0587/2130/4757/files/DoubleBTS796043AH-BridgeHigh-PowerStepperMotorDriverModule.jpg?v=1775714838" width="180" alt="HW-039 module — product reference photo"></a> |
| IMX477 camera | 1 | <a href="https://thinkrobotics.com/products/waveshare-imx477-160-12-3mp-camera-160-fov"><img src="https://cdn.shopify.com/s/files/1/0014/4313/5560/products/H3683a4868aba4ffa9f039c9733f8eda8c.jpg?v=1631119910" width="180" alt="IMX477 camera — product reference photo"></a> |
| DFRobot ultrasonic sensors | 4 | <a href="https://www.dfrobot.com/product-2172.html"><img src="https://dfimg.dfrobot.com/enshop/image/data/SEN0388/SEN0388_251022%20%281%29.jpg" width="180" alt="DFRobot ultrasonic sensors — product reference photo"></a> |
| DFRobot BNO055 9-axis IMU | 1 | <a href="https://www.dfrobot.com/product-2142.html"><img src="https://dfimg.dfrobot.com/enshop/SEN0374/SEN0374_Main_01.jpg" width="180" alt="DFRobot 9-axis IMU — product reference photo"></a> |
| RC380 brushed drive motor, marked 7.2 V | 1 | [Installed vehicle views](../README.md#vehicle-photos). Use the actual motor label; a generic RS380 listing is not its datasheet. |
| **MG996R steering servo** | 1 | <a href="https://towerpro.com.tw/product/mg996r/"><img src="https://towerpro.com.tw/wp-content/uploads/2014/08/MG996R-3b.jpg" width="180" alt="MG996R steering servo — product reference photo"></a> |
| **Rotary encoder** | 1 | <a href="https://kitsguru.com/products/m274-360-degree-rotary-encoder-brick-sensor-module"><img src="https://cdn.shopify.com/s/files/1/0587/2130/4757/products/Rotary-Encoder-Brick-Sensor-Module-1.jpg?v=1737971857" width="180" alt="Rotary encoder — product reference photo"></a> |
| Buzzer | 1 | <a href="https://www.dfrobot.com/product-84.html"><img src="https://dfimg.dfrobot.com/enshop/DFR0032/DFR0032_Main_01.jpg" width="180" alt="Buzzer — product reference photo"></a> |
| Battery cell holder | 1 | <a href="https://quartzcomponents.com/products/4-cell-18650-battery-holder"><img src="https://cdn.shopify.com/s/files/1/0300/6424/6919/products/4-Cell-18650-Battery-Holder.jpg?v=1649937983" width="180" alt="Battery cell holder — product reference photo"></a> |
| DC-DC step-down regulator | See power notes | <img src="images/dc-dc-step-down-regulator.png" width="180" alt="Adjustable DC-DC step-down regulator"> |
| Start button | 1 | <a href="https://www.dfrobot.com/product-1097.html"><img src="https://dfimg.dfrobot.com/enshop/image/data/DFR0029-W/DFR0029-W_260526%20%281%29.jpg" width="180" alt="Start button — product reference photo"></a> |

Most photos are product references, not identification photos of the installed parts. Check the label on the actual component before using a linked product's voltage or current rating.

Wiring: [circuit diagram](../schematic/circuit-diagram.png) and [machine schematic](../schematic/machine-schematic.png).


The inventory lists four ultrasonic sensors. The Open controller reads front, left and right; the Obstacle controller also reads rear distance on D10. Encoder inputs A2/A3 are used by the Obstacle and older development controllers, not the Open controller. See the [version-specific pin map](PINOUT.md).

The documented computer is the original Jetson Nano, not the Orin Nano. Product-reference images illustrate component families; the vehicle photos and actual part labels identify the installed build.

1