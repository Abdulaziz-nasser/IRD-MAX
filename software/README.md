# Software

Our car uses a Jetson for camera processing and driving decisions, and an Arduino Uno for movement and sensor readings. They communicate through USB serial at 115200 baud.

## Jetson

[`jetson/o1.py`](jetson/o1.py) detects floor lines and red/green pillars. It uses yaw and distance readings to control straight driving, corners, obstacle avoidance, and parking.

## Arduino Uno

[`controller/ird_max_controller.ino`](controller/ird_max_controller.ino) controls the motor and steering servo. It reads the IMU, encoder, and ultrasonic sensors, then sends those measurements to the Jetson.
