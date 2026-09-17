#!/usr/bin/env python3
"""Display yaw, ultrasonic distances and encoder distance; send no commands."""
import argparse
import csv
import time

from calibration_common import add_serial_arguments, open_link, read_packet


def display_distance(value):
    if value is None:
        return "not reported"
    return "invalid/no echo" if value < 0 else "{:.1f} cm".format(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_serial_arguments(parser)
    parser.add_argument("--log", help="New CSV file; an existing file is never overwritten")
    args = parser.parse_args()
    print("Disconnect drive-motor power and close all driving programs first.")
    print("Opening serial may reset the Uno. This tool sends no commands. Ctrl+C exits.")
    log = None
    link = None
    try:
        if args.log:
            log = open(args.log, "x", newline="")
            fields = ["time_unix", "yaw", "dF", "dL", "dR", "dB", "enc_cm", "state", "speed"]
            writer = csv.DictWriter(log, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
        link = open_link(args.port, args.baud)
        while True:
            packet = read_packet(link)
            print("yaw={} deg | F={} | L={} | R={} | rear={} | enc={} cm".format(
                packet.get("yaw", "not reported"),
                display_distance(packet.get("dF")), display_distance(packet.get("dL")),
                display_distance(packet.get("dR")), display_distance(packet.get("dB")),
                packet.get("enc_cm", "not reported")))
            if log:
                writer.writerow(dict(packet, time_unix=time.time()))
                log.flush()
    except KeyboardInterrupt:
        print("\nStopped reading.")
    finally:
        if link is not None:
            link.close()
        if log is not None:
            log.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        raise SystemExit(str(exc))
