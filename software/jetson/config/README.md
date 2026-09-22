# Colour Settings

The existing programs read HSV ranges from YAML. The main keys are BLUE, ORANGE, RED and GREEN. Each can hold one low/high mapping or a list of mappings.

[vision.example.yaml](vision.example.yaml) contains starter ranges from the supplied rebuild archive, not measured field settings. Its name intentionally does not match `vision_*.yaml`, so it is not selected automatically.

Use [autotune_colors.py](../../tools/autotune_colors.py) to sample the real field. It writes a new `vision_*.yaml` here. OpenCV hue ranges from 0 to 179; saturation and value from 0 to 255.

The environment variable `VISION_CFG` can select a file explicitly. Otherwise the programs sort `config/vision_*.yaml` relative to their working directory and load the final filename. Check the printed path before a run.

Keep the exact colour file with the code version used in a recorded test. Do not replace a working file with starter values.

1