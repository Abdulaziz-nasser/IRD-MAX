# IRD MAX Software

The Arduino Uno controls movement and reads the sensors. The Jetson reads those measurements and is the platform for camera processing and autonomous driving.

|Folder|Contents|Current use|
|-|-|-|
|[controller/](controller/)|The latest Uno code, pin map, upload instructions, and command reference|Motor, steering, and sensor control|
|[jetson/](jetson/)|The telemetry reader, dependencies, tests, and setup guide|Reading the current Uno's measurements|

Start with the controller upload guide and the Jetson telemetry reader. This checks that the two boards exchange the measurements we expect before we connect camera decisions to movement.

The telemetry reader accepts the current Uno's `TLM` messages. The autonomous package expects a different set of commands and replies, described in its guide. It is not yet a complete driving program for the Uno code in this repository.

