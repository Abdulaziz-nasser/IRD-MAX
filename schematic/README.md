# Reference Drawings

These supplied drawings document a reference layout, not the as-built wiring of the current car. The [Uno pin map](../hardware/PINOUT.md) is the code-based signal reference; the [power notes](../hardware/power/) distinguish reported wiring from supply requirements.

## Circuit Diagram

![Reference circuit diagram](circuit-diagram.png)

The drawing uses HC-SR04 symbols and a two-cell battery-holder symbol. The hardware inventory instead reports DFRobot ultrasonic modules and four cells. The uploaded firmware uses single-pin ultrasonic signalling, so do not copy the drawing's separate trigger/echo connections as the current pinout.

## Machine Schematic

![Reference machine schematic](machine-schematic.png)

This schematic uses a Raspberry Pi symbol labelled Jetson Nano, an OV7670 camera symbol and a different motor-driver symbol. These are not the installed parts: the documented build uses the original Jetson Nano, IMX477 camera and BTS7960 driver.

Both images are retained as reference material, not verified power-wiring instructions. Have the coach or an experienced adult check the actual hardware before reproducing a connection.

The [CAD page](../CAD/) shows the vehicle layout from six directions. The [hardware list](../hardware/) identifies the reported components.
