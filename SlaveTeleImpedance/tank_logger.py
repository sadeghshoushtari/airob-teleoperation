#!/usr/bin/env python3
"""Records the 100 Hz tele-impedance diagnostics of slave_ti (UDP, logging only) to a CSV file until Ctrl+C.

python3 tank_logger.py [out.csv] [--port 11061]
"""
import argparse
import socket
import struct

FIELDS = ("t_s", "stiffness_live", "cmd_kt_n_per_m", "cmd_kr_nm_per_rad", "target_kt", "applied_kt",
          "target_kr_gain", "applied_kr_gain", "tank_j", "injected_j", "harvested_j", "spring_energy_j",
          "tank_limited", "active", "port_energy_j")

parser = argparse.ArgumentParser()
parser.add_argument("out", nargs="?", default="tele_impedance_log.csv")
parser.add_argument("--port", type=int, default=11061)
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
