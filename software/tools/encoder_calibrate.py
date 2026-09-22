#!/usr/bin/env python3
"""Check encoder scale by rolling the car by hand, with drive-motor power off."""
import argparse
import math

from calibration_common import add_serial_arguments, open_link, read_packet


def corrected_counts_per_10cm(current_scale, reported_distance, measured_distance):
    values = (current_scale, reported_distance, measured_distance)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Measurements must be finite numbers.")
    if current_scale <= 0 or measured_distance <= 0 or reported_distance == 0:
        raise ValueError("Use a positive scale and measured distance, with nonzero encoder movement.")
    return current_scale * abs(reported_distance) / measured_distance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_serial_arguments(parser)
    parser.add_argument("--counts-per-10cm", type=float, required=True,
                        help="The COUNTS_PER_10CM value in the code actually on the Uno")
    args = parser.parse_args()
    if not math.isfinite(args.counts_per_10cm) or args.counts_per_10cm <= 0:
        parser.error("--counts-per-10cm must be positive and finite")
    print("Disconnect drive-motor power. Keep USB/logic power on. Close driving programs.")
    if input("Type MOTOR OFF to confirm: ").strip() != "MOTOR OFF":
        return
    link = open_link(args.port, args.baud)
    try:
        input("Place the car at a marked start point; press Enter without moving it: ")
        link.reset_input_buffer()
        start = read_packet(link, required=("enc_cm",))["enc_cm"]
        actual = float(input("Enter the distance you will roll the car, in cm: "))
        if not math.isfinite(actual) or actual <= 0:
            raise ValueError("Distance must be positive and finite.")
        input("Roll it that distance by hand, stop, then press Enter: ")
        link.reset_input_buffer()
        end = read_packet(link, required=("enc_cm",))["enc_cm"]
        reported = end - start
        scale = corrected_counts_per_10cm(args.counts_per_10cm, reported, actual)
        print("Reported distance: {:.2f} cm; measured distance: {:.2f} cm".format(reported, actual))
        print("Calculated COUNTS_PER_10CM: {:.3f} (nearest integer: {})".format(scale, int(round(scale))))
        print("This uses enc_cm, not raw ticks. Repeat the measurement before changing the Uno code.")
        print("No file was changed and no motor command was sent.")
    finally:
        link.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled.")
    except Exception as exc:
        raise SystemExit(str(exc))

1