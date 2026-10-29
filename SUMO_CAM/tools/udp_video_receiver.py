#!/usr/bin/env python3
import os
import socket
import struct
import time
from pathlib import Path

import cv2
import numpy as np


HEADER_FORMAT = "<HBBIHHIHH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
MAGIC = 0xCAFE
VERSION = 1


def load_env(path):
    values = {}
    if not path.exists():
        return values
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


class UdpVideoReceiver:
    def __init__(self, host, port, frame_timeout=0.2):
        self.frame_timeout = frame_timeout
        self.frames = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind((host, port))
        self.sock.settimeout(0.5)

    def read(self):
        while True:
            try:
                packet, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            now = time.monotonic()

            for frame_id, state in list(self.frames.items()):
                if now - state["updated_at"] > self.frame_timeout:
                    del self.frames[frame_id]

            if len(packet) < HEADER_SIZE:
                continue

            header = packet[:HEADER_SIZE]
            payload = packet[HEADER_SIZE:]
            (
                magic,
                version,
                _flags,
                frame_id,
                packet_id,
                packet_count,
                frame_size,
                payload_size,
                _reserved,
            ) = struct.unpack(HEADER_FORMAT, header)

            if magic != MAGIC or version != VERSION:
                continue
            if packet_id >= packet_count or payload_size != len(payload):
                continue
            if packet_count == 0 or frame_size == 0:
                continue

            state = self.frames.get(frame_id)
            if state is None:
                state = {
                    "parts": [None] * packet_count,
                    "received": 0,
                    "frame_size": frame_size,
                    "updated_at": now,
                }
                self.frames[frame_id] = state

            if len(state["parts"]) != packet_count or state["frame_size"] != frame_size:
                del self.frames[frame_id]
                continue

            if state["parts"][packet_id] is None:
                state["parts"][packet_id] = payload
                state["received"] += 1
                state["updated_at"] = now

            if state["received"] != packet_count:
                continue

            jpeg = b"".join(state["parts"])
            del self.frames[frame_id]
            if len(jpeg) != frame_size:
                continue

            image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is not None:
                return image


def main():
    project_dir = Path(__file__).resolve().parents[1]
    config = load_env(project_dir / ".env")
    port = int(os.environ.get("LAPTOP_PORT", config.get("LAPTOP_PORT", "5005")))

    receiver = UdpVideoReceiver("0.0.0.0", port)
    print(f"Listening for ESP32-CAM UDP video on 0.0.0.0:{port}")

    last_report = time.monotonic()
    frames = 0

    while True:
        frame = receiver.read()

        cv2.imshow("ESP32-CAM UDP", frame)

        frames += 1
        now = time.monotonic()
        if now - last_report >= 2.0:
            print(f"RX FPS: {frames / (now - last_report):.1f}")
            frames = 0
            last_report = now

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
