#!/usr/bin/env python3
"""Records the 100 Hz diagnostics of the feedback energy tank and the buzz-band damping of master_ti (UDP, logging only) to a CSV file until
Ctrl+C.

python3 master_tank_logger.py [out.csv] [--port 11062]

Fields: time since the master started, teleoperation active, tank energy, work of the feedback on the hand (total),
work of the hand against the feedback (total), energy above the tank maximum (dissipated, total), work the limiter
took out of the commands (total), command reduced in this cycle, power of the feedback on the hand in the last cycle
(without the damping part), force and torque taken out of the command in this cycle, damping the tank added
(stage 1: Ns/m, Nms/rad), power the feedback gives the hand in the buzz band (force, torque; averaged), buzz-band
damping (Ns/m, Nms/rad).
"""
import argparse
import socket
import struct

FIELDS = ("t_s", "active", "tank_j", "delivered_j", "received_j", "dissipated_j", "removed_j", "limited",
          "feedback_power_w", "force_cut_n", "torque_cut_nm", "tank_damping_ns_per_m", "tank_damping_nms_per_rad",
          "buzz_force_power_w", "buzz_torque_power_w", "buzz_damping_ns_per_m", "buzz_damping_nms_per_rad")

parser = argparse.ArgumentParser()
parser.add_argument("out", nargs="?", default="master_tank_log.csv")
parser.add_argument("--port", type=int, default=11062)
args = parser.parse_args()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", args.port))
with open(args.out, "w") as f:
    f.write(",".join(FIELDS) + "\n")
    try:
        while True:
            data = sock.recv(1024)
            if len(data) == 8 * len(FIELDS):
                f.write(",".join(f"{v:.6g}" for v in struct.unpack(f"<{len(FIELDS)}d", data)) + "\n")
                f.flush()
    except KeyboardInterrupt:
        pass
