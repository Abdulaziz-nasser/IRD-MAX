# Change Notes

## 2026-09-22 - Challenge files and documentation consistency

- Uploaded the supplied Open Jetson/Arduino files and Obstacle Arduino file; retained the existing Obstacle Jetson source.
- Renamed the four challenge files and documented their matched pairs.
- Updated setup paths and Uno R3 serial-port examples without changing source defaults.
- Separated Open, Obstacle and older-controller pinouts, telemetry and encoder scales.
- Identified substitute symbols in the reference drawings and removed the mismatched motor-product reference.
- Kept reported power details separate from unverified protection and regulator arrangements.
- Left robot programs, tuning, original images and recorded test observations unchanged during the documentation cleanup.

## 2026-09-17 - Setup and calibration documentation

- Added Nano/Uno setup, dependency information, a code-based pin map and serial reference.
- Added a colour sampler, read-only sensor logger and manual encoder-scale calculator, adapted from the supplied rebuild source.
- Added a labelled HSV example and offline calibration tests.
- Added a repeatable test procedure and clearer navigation.
- Reworded documentation while preserving recorded observations and media.
- Left all existing driving programs and Uno code unchanged.

This records a documentation/helper-tool update, not a new tested competition release. Exact versions are in the Git commit history.
