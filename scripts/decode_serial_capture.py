#!/usr/bin/env python3
"""Capture one CAM:BEGIN/CAM:END base64 JPEG dump from serial and save it.

Usage: python3 decode_serial_capture.py <serial-port> [output.jpg]
Press the shutter button on the board after this is running and listening.

Rotates 90 degrees left (CCW) to correct for the camera's mounting
orientation — the OV2640 only has vflip/hmirror in hardware, so this
software rotation stands in until step 4 does it during TFT blit instead.
"""
import base64
import io
import sys

import serial
from PIL import Image

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
    image = Image.open(io.BytesIO(data)).rotate(90, expand=True)
    image.save(out_path)
    print(f"wrote {out_path} ({image.width}x{image.height}, rotated 90 CCW)")

if __name__ == "__main__":
    main()
