# Current Uno Serial Commands

This follows [ird_max_controller.ino](controller/ird_max_controller.ino), not the firmware in the rebuild archive. USB serial runs at 115200 baud. Each command ends with a newline.

## Messages from the Uno

- `READY`: setup finished.
- `ARMED`: the button or START armed the controller.
- `PONG`: reply to PING.
- `TLM,...`: heading, state, steering/speed requests, distances and encoder distance.
- `ENC_CM,...`: reply to ENC_GET.
- `OK,ENC_ZERO`: encoder count reset.

Telemetry field order:

```text
TLM,yaw=<degrees>,state=<0|1|2>,steer=<normalized>,speed=<PWM>,dF=<cm>,dL=<cm>,dR=<cm>,enc_cm=<cm>
```

States: 0 is idle, 1 is continuous drive and 2 is a turn. Speed is commanded PWM, not measured vehicle speed. Steer is the normalized request, not a measured wheel angle. This firmware does not publish rear distance `dB`, raw encoder ticks or an `armed=` field.

## Accepted commands

| Command | Use |
| --- | --- |
| `PING` | Check the link. |
| `ENC_GET` / `ENC_ZERO` | Read scaled encoder distance / reset the count. |
| `START` | Arm the controller; does not itself request movement. |
| `STOP` | Stop the motor and center steering. It does not disarm. |
| `CENTER,<norm>,<PWM>` | Continuous forward drive and steering. |
| `BACK,<PWM>` / `BACKC,<norm>,<PWM>` | Reverse drive / reverse with steering. |
| `TURN_ABS,<yaw>[,<PWM>[,REV]]` | Turn toward an absolute heading; optional reverse mode. |
| `STEER_DEG,<degrees>` | Direct servo command; the active state can overwrite it next loop. |
| `SET_CENTER,<degrees>` / `TRIM_NORM,<value>` | Change steering setup in memory. |
| `SET_TURN_PWM,<PWM>` | Change the default turn PWM in memory. |
| `CLEAR_TURN_PWM` | Clear the per-turn override; does not reset `TURN_PWM_DEFAULT`. |
| `SET_TURN_TIMEOUT,<ms>` | Set the turn timeout within the code's limits. |
| `SET_US_TIMEOUT,<microseconds>` / `SET_US_FAR_CM,<cm>` | Change ultrasonic reading limits. |

Before arming, the parser accepts PING, START, encoder commands and ultrasonic settings. Most steering/movement settings require arming. Serial settings are not saved to EEPROM.

## Safety and version differences

The controller has a turn timeout and a stale-IMU stop during turns. It has no general timeout that stops continuous drive when serial commands disappear. A lost USB connection can leave a previous drive command active.

The uploaded `o1.py` uses `FWD_CM` and `BACK_CM` in some paths. Those distance commands and their completion replies are not implemented here. `BACK_CM` also matches the broader `BACK` prefix. Do not send it as a distance command to this firmware.

The archive's ARM, DRIVE, MOVE, RESET_ENCODER and boot/session handshake belong to a different controller. The added sensor and encoder tools only read telemetry.
