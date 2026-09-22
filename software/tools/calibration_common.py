"""Read the current Uno telemetry without sending control commands."""
import math
import os
import time


def add_serial_arguments(parser):
    parser.add_argument("--port", default=os.environ.get("ROBOT_PORT", "/dev/ttyUSB0"))
    parser.add_argument("--baud", type=int, default=115200)


def open_link(port, baud):
    import serial
    # Opening USB serial can reset an Uno. Keep drive-motor power disconnected.
    link = serial.Serial(port, baudrate=baud, timeout=0.2)
    time.sleep(2.0)
    link.reset_input_buffer()
    return link


def parse_telemetry(line):
    parts = line.strip().split(",")
    if not parts or parts[0] != "TLM":
        return None
    result = {}
    for part in parts[1:]:
        key, separator, value = part.partition("=")
        if not separator:
            continue
        try:
            number = float(value)
        except ValueError:
            continue
        if math.isfinite(number):
            result[key.strip()] = number
    return result


def read_packet(link, required=(), timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = link.readline(2048).decode("ascii", errors="replace")
        packet = parse_telemetry(line)
        if packet is not None and all(key in packet for key in required):
            return packet
    raise TimeoutError("No fresh TLM packet with the required fields. Check the Uno code, port and baud.")

1