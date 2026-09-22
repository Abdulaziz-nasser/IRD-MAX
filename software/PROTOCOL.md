# Arduino Serial Reference

USB serial runs at 115200 baud, and every command ends with a newline. This page covers [Open](controller/open_challenge_controller.ino), [Obstacle](controller/obstacle_challenge_controller.ino) and the [older development controller](controller/ird_max_controller.ino). It does not cover the Mega firmware from the rebuild archive.

## Telemetry

All three controllers send `READY` after setup, `ARMED` when armed by the button or START, `PONG` in reply to PING, and lines beginning with `TLM,`.

Open:

```text
TLM,yaw=<degrees>,state=<0|1|2>,steer=<normalized>,speed=<PWM>,dF=<cm>,dL=<cm>,dR=<cm>
```

Obstacle:

```text
TLM,yaw=<degrees>,state=<0|1|2|3>,steer=<normalized>,speed=<PWM>,dF=<cm>,dL=<cm>,dR=<cm>,dB=<cm>,enc_cm=<cm>
```

The older development controller uses the Open field order followed by `enc_cm`, without `dB`.

States are 0 idle, 1 continuous drive, 2 turning and 3 encoder-distance movement (Obstacle only). The speed and steering fields show stored commands rather than measured vehicle speed or wheel angle. During turns and distance moves, they may differ from the active output. Telemetry timing depends on sensor-read delays.

## Command support

| Command | Open | Obstacle | Older development |
| --- | --- | --- | --- |
| `PING`, `START`, `STOP` | Yes | Yes | Yes |
| `CENTER,<norm>,<PWM>` | Yes | Yes | Yes |
| `BACK,<PWM>` | Yes | Yes | Yes |
| `BACKC,<norm>,<PWM>` | No | Yes | Yes |
| `TURN_ABS,<yaw>[,<PWM>]` | Yes | Yes | Yes |
| `TURN_ABS,<yaw>,<PWM>,REV` | No | Yes | Yes |
| `SET_CENTER,<degrees>`, `TRIM_NORM,<value>` | Yes | Yes | Yes |
| `SET_TURN_PWM,<PWM>`, `CLEAR_TURN_PWM` | Yes | Yes | Yes |
| `STEER_DEG,<degrees>` | No | Yes | Yes |
| `SET_TURN_TIMEOUT,<ms>` | Yes | No | Yes |
| `SET_US_TIMEOUT,<microseconds>`, `SET_US_FAR_CM,<cm>` | No | Yes | Yes |
| `ENC_GET`, `ENC_ZERO` | No | Yes | Yes |
| `FWD_CM,<cm>[,<PWM>]`, `BACK_CM,<cm>[,<PWM>]` | No | Yes | No |

Before arming, Open accepts PING and START. Obstacle and the older development controller also accept encoder queries/reset and ultrasonic settings. Other settings and motion commands require arming. Settings are held in memory, not saved to EEPROM.

`START` arms the controller but does not move the car. `STOP` stops the motor and centres the steering without clearing the armed flag. An active state can overwrite a direct `STEER_DEG` request. `CLEAR_TURN_PWM` clears the per-turn override, not the default PWM.

## Obstacle distance and turn replies

- `ENC_CM,<cm>`: reply to ENC_GET.
- `OK,ENC_ZERO`: encoder reset.
- `DIST_BEGIN,cm=<cm>`: distance movement started.
- `DIST,cm=<cm>`: signed progress.
- `DIST_DONE`: the encoder target was reached and the distance move stopped.
- `TURN_DONE`: the turn ended, either within heading tolerance or on timeout; this message alone does not prove the target heading was reached.

Obstacle uses a fixed `TURN_MAX_MS = 2500` and does not implement `SET_TURN_TIMEOUT`. Its Arduino distance state has no time limit if encoder counts stop arriving. Parking decisions and sequence timing remain in the Jetson program.

Open uses its own turn timeout and resumes continuous drive after a completed or timed-out turn. It reports `INFO,TURN_FINISH,TIMEOUT` on timeout rather than Obstacle's TURN_DONE message.

## Safety and compatibility

None of these Arduino programs has a general serial-loss watchdog for continuous driving. They check for stale IMU data during turns, but a lost USB connection can still leave the previous drive command active. Keep drive-motor power disconnected during serial setup. For powered checks, use a physical power disconnect and appropriate supervision.

Use the [matched challenge files](README.md#competition-programs). Do not send BACKC or BACK_CM to Open: its broad BACK prefix can misinterpret them. Do not send FWD_CM/BACK_CM to the older development controller; it lacks distance movement, and BACK_CM can match BACK.

The sensor and encoder calibration tools read telemetry without sending movement commands.
