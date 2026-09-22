# Troubleshooting

These are the main problems we faced while setting up and testing the car, along with what we tried and what fixed them.

## Problem 1 — Faulty USB Drive
<img width="640" height="480" alt="maybe_a_problem" src="https://github.com/user-attachments/assets/881410ae-a910-4c82-b6fe-b376b6668296" />


### Problem

The JetPack installation did not finish when we used the first USB drive.

### What We Tried

- Tried commands during setup.
- Waited about 30 minutes.
- Checked the USB drive again.

### Solution

We used another USB drive and repeated the JetPack installation.

### Result
<img width="640" height="480" alt="worked" src="https://github.com/user-attachments/assets/e0a43b2a-5aad-4f6c-968b-c6432be90d0d" />

The installation worked with the replacement USB drive.


## Problem 2 — NoMachine Connection
<img width="640" height="480" alt="photo_no_machine_not_done_#crying" src="https://github.com/user-attachments/assets/20470a50-e19c-4336-aa9e-58a59ea4540b" />


### Problem

The laptop had NoMachine 7.8 and the Jetson had version 10. We could not connect, and NoMachine showed “The connection with the server was lost.”

### What We Tried

- Tried connecting with the existing versions.
- Tried downgrading the Jetson to version 9.8, but the connection still failed.

### Solution

We downloaded and installed the same NoMachine version on both devices.

### Result
<img width="640" height="480" alt="photo_no_machine_done" src="https://github.com/user-attachments/assets/07575dfe-7652-491f-a7ad-9f5fa959bb84" />

The connection worked, and we could access the Jetson desktop from the laptop.

## Problem 3 — Yaw and Straight Driving

### Problem

During track testing, the car kept steering left and right when it should have been driving straight.

### What We Tried

- Tested straight driving at low speed.
- Adjusted the yaw filter and steering limits.
- Checked the yaw readings and steering commands. We found that corrections were calculated faster than new yaw readings arrived. The steering values were also rounded to whole degrees, which made the corrections uneven.

### Solution

We added `[STRAIGHT]` to hold one target heading on each straight section. It uses fresh yaw readings, makes small, smooth corrections, and resets its correction history after turns.

### Result

We made two versions with the same speeds and turns to compare them:

**Without the yaw system:** [slow_without_yaw.py](software/jetson/slow_without_yaw.py) keeps steering centered between turns.


https://github.com/user-attachments/assets/1d59722b-05b5-4f70-8cae-8201ca815710


**With the yaw system:** [slow_with_yaw.py](software/jetson/slow_with_yaw.py) uses `[STRAIGHT]` to correct the heading between turns.


https://github.com/user-attachments/assets/0a3b3312-4361-43b0-a533-556fb77a1832


Both versions still use the IMU for corner turns.

## Problem 4 — Inconsistent Distance

### Problem

The programmed distance was inconsistent. The robot travelled a different distance after each full lap around the field, which includes four corner turns.

### Solution

We created a separate distance-checker program and repeated the same movement several times. This allowed us to verify the distance values and adjust the calculation until the robot travelled a consistent and accurate distance.

### Result

The distance became more reliable and repeatable during full-field runs.

## Problem 5 — Worn Encoder

### Problem

The encoder had been used heavily during testing. Over time, its readings became lower and unreliable, even when the robot travelled the same distance.

### Solution

We replaced the worn encoder with a new one and checked its readings using the distance-checker program.

### Result

The new encoder produced stable readings and restored accurate distance measurement.


[Recorded tests](testing/) · [Test procedure](testing/PROCEDURE.md) · [Setup and calibration](software/SETUP.md)
