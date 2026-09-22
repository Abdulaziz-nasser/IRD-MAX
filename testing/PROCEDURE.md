# Test Procedure

This page explains how we repeat our tests. The completed recordings and observations are in [README.md](README.md).

## Record the setup

Keep the date, program filename, Git commit ID, Uno code version and colour YAML filename with each test. Note the track direction, wall/pillar layout and lighting.

Write down the setting changed, its old and new values, and what counts as a pass. Change one group of settings at a time so the result can be linked to that change.

## Bench checks

Keep drive-motor power disconnected for the first three stages.

| Stage | Check | Record |
| --- | --- | --- |
| Setup | Imports, camera frames and serial connection. | Versions, port and any errors. |
| Sensors | Read IMU, ultrasonic and encoder data with [sensor_check.py](../software/tools/sensor_check.py). | CSV readings and physical reference measurements. |
| Calibration | Sample real colours; repeat hand-rolled encoder measurements. | Saved YAML, measured distance and calculated scale. |
| Raised wheels | Verify start, steering direction, low drive output and stopping with a matched controller/program pair. | Actual settings and response. |

Keep a physical drive-power disconnect available. The uploaded Uno firmware does not automatically stop all continuous-drive commands if USB is lost. Do not test this by unplugging USB while driving on the floor.

## Track checks

Start with short straight sections and corners, then test pillars and the final movements. Run in both directions. Record failures and successes, including time, distance or completed laps, contacts and manual intervention.

For the yaw comparison, keep speed, track section, starting position and colour file the same. Only straight-heading correction differs; both programs still use the IMU for turns.

Link each video to the test it shows. If a recording stops early, report only what is visible. A printing video shows a part being made, not a strength measurement or a successful track run.

## Compare results

Record what happened, the suspected cause of a failure, the change made and the result of the next test. Keep measured values separate from estimates and visual observations.

Useful measures include completed laps, time, interventions, wall contacts, heading error and measured-versus-reported distance. Include only values actually recorded.

## Offline helper checks

```bash
python3 -m unittest discover -s software/tests -v
```

These checks run without hardware. Arduino compilation, live sensor checks and track testing are separate.
