"""Offline tests: no camera, serial connection, or motor commands."""
import importlib
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))


class CalibrationTests(unittest.TestCase):
    def tool(self, name):
        try:
            return importlib.import_module(name)
        except ImportError as exc:
            self.fail("Calibration tool is not implemented: {} ({})".format(name, exc))

    def test_red_wraps_without_selecting_green(self):
        tool = self.tool("autotune_colors")
        self.assertEqual(tool.circular_hue_ranges(np.array([178, 179, 0, 1])),
                         [(0, 4), (175, 179)])

    def test_broad_hue_sample_does_not_collapse_at_wrap(self):
        tool = self.tool("autotune_colors")
        self.assertEqual(tool.circular_hue_ranges(np.arange(180)), [(0, 179)])

    def test_learned_hsv_has_legacy_low_high_keys(self):
        tool = self.tool("autotune_colors")
        sample = np.tile([60, 150, 200], (100, 1))
        self.assertEqual(tool.learn_hsv_ranges(sample),
                         [{"low": [57, 138, 185], "high": [63, 255, 215]}])

    def test_grey_selection_is_rejected(self):
        tool = self.tool("autotune_colors")
        with self.assertRaises(ValueError):
            tool.learn_hsv_ranges(np.tile([0, 0, 150], (100, 1)))

    def test_sample_still_matches_after_existing_driver_saturation_boost(self):
        tool = self.tool("autotune_colors")
        bands = tool.learn_hsv_ranges(np.tile([110, 150, 200], (100, 1)))
        # The existing drivers boost floor-line saturation by 1.10 or 1.15.
        # Literal processed samples: 150 -> 165 or 172 after uint8 conversion.
        for saturation in (150, 165, 172):
            with self.subTest(saturation=saturation):
                self.assertTrue(any(
                    band["low"][0] <= 110 <= band["high"][0]
                    and band["low"][1] <= saturation <= band["high"][1]
                    and band["low"][2] <= 200 <= band["high"][2]
                    for band in bands))

    def test_empty_selection_is_rejected(self):
        tool = self.tool("autotune_colors")
        with self.assertRaises(ValueError):
            tool.learn_hsv_ranges(np.empty((0, 3)))

    def test_selection_bounds_work_when_dragging_backwards(self):
        tool = self.tool("autotune_colors")
        self.assertEqual(tool.clip_rectangle((120, 90), (-10, -5), (80, 100, 3)),
                         (0, 0, 100, 80))

    def test_zero_area_selection_is_rejected(self):
        tool = self.tool("autotune_colors")
        self.assertIsNone(tool.clip_rectangle((10, 10), (10, 30), (80, 100, 3)))

    def test_saved_config_matches_current_driver_format(self):
        tool = self.tool("autotune_colors")
        colors = {name: [{"low": [1, 40, 50], "high": [4, 200, 220]}]
                  for name in ("BLUE", "ORANGE", "RED", "GREEN")}
        with tempfile.TemporaryDirectory() as tmp:
            saved = tool.save_config(colors, tmp)
            data = yaml.safe_load(saved.read_text())
            self.assertEqual(set(data), {"BLUE", "ORANGE", "RED", "GREEN"})
            self.assertEqual(data["RED"][0]["low"], [1, 40, 50])
            self.assertTrue(saved.name.startswith("vision_"))
            second = tool.save_config(colors, tmp)
            self.assertNotEqual(saved, second)
            self.assertTrue(saved.is_file())

    def test_save_requires_all_four_colors(self):
        tool = self.tool("autotune_colors")
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                tool.save_config({"BLUE": []}, tmp)
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_telemetry_accepts_uno_fields_without_inventing_rear(self):
        tool = self.tool("calibration_common")
        packet = tool.parse_telemetry(
            "TLM,yaw=-20.5,state=0,steer=0.00,speed=0,dF=120,dL=30,dR=-1,enc_cm=12.34")
        self.assertEqual(packet["yaw"], -20.5)
        self.assertEqual(packet["dR"], -1)
        self.assertEqual(packet["enc_cm"], 12.34)
        self.assertNotIn("dB", packet)
        self.assertNotIn("enc_ticks", packet)

    def test_bad_telemetry_is_not_mistaken_for_zero(self):
        tool = self.tool("calibration_common")
        self.assertIsNone(tool.parse_telemetry("READY"))
        packet = tool.parse_telemetry("TLM,yaw=nan,enc_cm=bad,dF=inf,dL=12")
        self.assertNotIn("yaw", packet)
        self.assertNotIn("enc_cm", packet)
        self.assertNotIn("dF", packet)
        self.assertEqual(packet["dL"], 12)

    def test_packet_reader_never_sends_motor_commands(self):
        tool = self.tool("calibration_common")
        class Receiver:
            def __init__(self):
                self.lines = iter([b"READY\n", b"TLM,yaw=5,enc_cm=2\n"])
            def readline(self, size=2048):
                return next(self.lines, b"")
            def write(self, data):
                raise AssertionError("Read-only calibration must not send commands")
        self.assertEqual(tool.read_packet(Receiver(), timeout=0.1)["enc_cm"], 2)

    def test_missing_sensor_field_fails_instead_of_using_zero(self):
        tool = self.tool("calibration_common")
        class Receiver:
            def readline(self, size=2048):
                return b"TLM,yaw=5\n"
        with self.assertRaises(TimeoutError):
            tool.read_packet(Receiver(), required=("enc_cm",), timeout=0.001)

    def test_encoder_scale_is_corrected_from_reported_distance(self):
        tool = self.tool("encoder_calibrate")
        self.assertAlmostEqual(tool.corrected_counts_per_10cm(624, 90, 100), 561.6)
        self.assertAlmostEqual(tool.corrected_counts_per_10cm(624, -90, 100), 561.6)

    def test_encoder_rejects_invalid_measurements(self):
        tool = self.tool("encoder_calibrate")
        for args in [(624, 0, 100), (624, 90, 0), (0, 90, 100),
                     (624, float("nan"), 100), (624, 90, float("inf"))]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                tool.corrected_counts_per_10cm(*args)

    def test_sensor_display_keeps_missing_and_invalid_distances_distinct(self):
        tool = self.tool("sensor_check")
        self.assertEqual(tool.display_distance(None), "not reported")
        self.assertEqual(tool.display_distance(-1), "invalid/no echo")
        self.assertEqual(tool.display_distance(23), "23.0 cm")


if __name__ == "__main__":
    unittest.main()

1