#!/usr/bin/env python3
"""Stiffness commands for SlaveTeleImpedance without the sensors (hardware tests of the stiffness input and the tank).

python3 stiffness_sender.py               manual: type a level 0..1 and Enter (0 = 300 N/m, 20 Nm/rad; 1 = 700 N/m,
                                          40 Nm/rad), or "q" to quit
python3 stiffness_sender.py --steps 4     automatic: 0 and 1 alternating every 4 s until Ctrl+C

Sends the current level at 5 Hz to UDP 127.0.0.1:4500 (two doubles: N/m, Nm/rad), as MOSAIC does. When the sender
stops, the slave goes to its fallback (500 N/m, 30 Nm/rad) 0.5 s later.
"""
import argparse
import socket
import struct
import threading
import time

TRANSLATION = (300.0, 700.0)  # N/m, slave range
ROTATION = (20.0, 40.0)  # Nm/rad, effective

parser = argparse.ArgumentParser()
parser.add_argument("--steps", type=float, help="alternate 0 and 1 with this period (s)")
parser.add_argument("--port", type=int, default=4500)
args = parser.parse_args()

level = [0.0]
stop = threading.Event()


def send():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while not stop.is_set():
        u = level[0]
        sock.sendto(struct.pack("<2d", TRANSLATION[0] + u * (TRANSLATION[1] - TRANSLATION[0]),
                                ROTATION[0] + u * (ROTATION[1] - ROTATION[0])), ("127.0.0.1", args.port))
        time.sleep(0.2)


thread = threading.Thread(target=send)
thread.start()
try:
    if args.steps:
        t0 = time.monotonic()
        while True:
            level[0] = float(int((time.monotonic() - t0) / args.steps) % 2)
            time.sleep(0.05)
    else:
        print("level 0 = 300 N/m, 20 Nm/rad; 1 = 700 N/m, 40 Nm/rad. Sending level 0.")
        while (line := input("level 0..1 (q quits): ").strip().lower()) != "q":
            try:
                level[0] = min(max(float(line), 0.0), 1.0)
                print(f"sending {TRANSLATION[0] + level[0] * 400:.0f} N/m, {ROTATION[0] + level[0] * 20:.1f} Nm/rad")
            except ValueError:
                print("a number between 0 and 1, or q")
except (KeyboardInterrupt, EOFError):
    pass
finally:
    stop.set()
    thread.join()
    print("stopped: the slave goes to its fallback in 0.5 s")
