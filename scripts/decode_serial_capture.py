#!/usr/bin/env python3
"""Capture one CAM:BEGIN/CAM:END base64 JPEG dump from serial and save it.

Usage: python3 decode_serial_capture.py <serial-port> [output.jpg]
Press the shutter button on the board after this is running and listening.
"""
import base64
import sys

import serial

def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <serial-port> [output.jpg]", file=sys.stderr)
        sys.exit(1)

    port = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else "capture.jpg"

    with serial.Serial(port, 115200, timeout=5) as ser:
        print(f"listening on {port}, waiting for CAM:BEGIN ...")
        payload_lines = []
        capturing = False
        while True:
            line = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
            if not line:
                continue
            print(line if not capturing else f"(base64 line, {len(line)} chars)")
            if line.startswith("CAM:BEGIN"):
                capturing = True
                payload_lines = []
                continue
            if line == "CAM:END":
                break
            if capturing:
                payload_lines.append(line)

    data = base64.b64decode("".join(payload_lines))
    with open(out_path, "wb") as f:
        f.write(data)
    print(f"wrote {len(data)} bytes to {out_path}")

if __name__ == "__main__":
    main()
