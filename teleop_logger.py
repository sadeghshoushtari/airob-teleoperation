#!/usr/bin/env python3
"""Final-validation logger for the bilateral Virtuose--Franka controller.

Purpose
-------
Record enough information to freeze the bilateral controller before sensor
integration. The logger combines:

1. Passive AF_PACKET copies of the existing 1 kHz command UDP stream.
2. Passive AF_PACKET copies of the existing Franka feedback UDP stream.
3. A dedicated logging-only Master diagnostic UDP stream (default 100 Hz).

The diagnostic stream is transmitted by Master/masterv.cpp only for logging.
It is never received by the Slave and never feeds back into control.

Typical use
-----------
From the thesis repository root:

    sudo python3 teleop_logger.py \
        --label final_full_validation \
        --project .

Optional fixed duration:

    sudo python3 teleop_logger.py \
        --label final_full_validation \
        --project . \
        --duration 120

Output
------
Each run creates:
  * one 100 Hz CSV
  * one JSON metadata/provenance file

The CSV includes:
  * existing Master command packet
  * raw Franka feedback packet
  * bias, debiased wrench, compensation term
  * mapped force and compensated moment
  * sync, reflection, directional, desired, filtered wrench terms
  * baseline rendered wrench before contact-saturation hold
  * actual commanded Virtuose wrench after the hold
  * contact weight, effective torque reflection gain
  * force/torque saturation-hold flags
  * Master and mapped Franka velocities
  * controller status and timing fields

Important limits
----------------
  * The CSV is 100 Hz. It is suitable for controller validation and thesis
    plots, not rigorous >50 Hz spectral analysis or 1 kHz passivity proofs.
  * Command, feedback, and diagnostic packets are independently sampled latest
    values. A row is not a mathematically synchronized request/reply tuple.
  * AF_PACKET dequeue timestamps are logger times, not hardware timestamps.
  * The dedicated diagnostic stream is best effort. The controller deliberately
    ignores telemetry send failures so logging cannot stop teleoperation.
"""

import argparse
import csv
import ctypes
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import pwd
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time


PERIOD_NS = 10_000_000  # 100 Hz CSV
LOOPBACK_BYTES = socket.inet_aton("127.0.0.1")

COMMAND_FIELDS = [
    "cmd_id",
    "cmd_source_time_us",
    "master_qw",
    "master_qx",
    "master_qy",
    "master_qz",
    "master_x_m",
    "master_y_m",
    "master_z_m",
    "force_reflection_gain_metadata",
    "torque_reflection_gain_metadata",
    "slave_Kt_command_N_per_m",
    "deadman_raw",
    "estop_raw",
]

FEEDBACK_FIELDS = [
    "fb_id",
    "fb_source_time_us",
    "franka_Fx_N",
    "franka_Fy_N",
    "franka_Fz_N",
    "franka_Mx_Nm",
    "franka_My_Nm",
    "franka_Mz_Nm",
    "franka_x_m",
    "franka_y_m",
    "franka_z_m",
    "franka_qw",
    "franka_qx",
    "franka_qy",
    "franka_qz",
    "franka_vx_m_per_s",
    "franka_vy_m_per_s",
    "franka_vz_m_per_s",
    "franka_wx_rad_per_s",
    "franka_wy_rad_per_s",
    "franka_wz_rad_per_s",
    "slave_Kt_applied_N_per_m",
    "slave_Kr_applied_Nm_per_rad",
]

DIAGNOSTIC_FIELDS = [
    "diag_id",
    "diag_control_packet_id",
    "diag_source_time_us",
    "diag_backward_active",
    "diag_backward_reference_ready",
    "diag_bias_ready",
    "diag_bias_count",
    "diag_force_safe",
    "diag_torque_safe",
    "diag_api_ok",
    "diag_contact_active",
    "diag_contact_weight",
    "diag_effective_torque_reflection_gain",
    "diag_force_saturation_hold",
    "diag_torque_saturation_hold",
    "diag_virtuose_command_applied",
    "diag_analog_normalized",
    "diag_master_Kt_N_per_m",
    "diag_master_Kr_Nm_per_rad",
    "diag_master_Dt_Ns_per_m",
    "diag_master_Dr_Nms_per_rad",
    "diag_feedback_age_ms",

    "diag_raw_Fx_N",
    "diag_raw_Fy_N",
    "diag_raw_Fz_N",
    "diag_raw_Mx_Nm",
    "diag_raw_My_Nm",
    "diag_raw_Mz_Nm",

    "diag_bias_Fx_N",
    "diag_bias_Fy_N",
    "diag_bias_Fz_N",
    "diag_bias_Mx_Nm",
    "diag_bias_My_Nm",
    "diag_bias_Mz_Nm",

    "diag_debiased_Fx_N",
    "diag_debiased_Fy_N",
    "diag_debiased_Fz_N",
    "diag_debiased_Mx_Nm",
    "diag_debiased_My_Nm",
    "diag_debiased_Mz_Nm",

    "diag_est_force_induced_Mx_Nm",
    "diag_est_force_induced_My_Nm",
    "diag_est_force_induced_Mz_Nm",

    "diag_corrected_franka_Mx_Nm",
    "diag_corrected_franka_My_Nm",
    "diag_corrected_franka_Mz_Nm",

    "diag_mapped_Fx_N",
    "diag_mapped_Fy_N",
    "diag_mapped_Fz_N",
    "diag_mapped_Mx_Nm",
    "diag_mapped_My_Nm",
    "diag_mapped_Mz_Nm",

    "diag_sync_Fx_N",
    "diag_sync_Fy_N",
    "diag_sync_Fz_N",
    "diag_sync_Mx_Nm",
    "diag_sync_My_Nm",
    "diag_sync_Mz_Nm",

    "diag_reflection_Fx_N",
    "diag_reflection_Fy_N",
    "diag_reflection_Fz_N",
    "diag_reflection_Mx_Nm",
    "diag_reflection_My_Nm",
    "diag_reflection_Mz_Nm",

    "diag_directional_Fx_N",
    "diag_directional_Fy_N",
    "diag_directional_Fz_N",
    "diag_directional_Mx_Nm",
    "diag_directional_My_Nm",
    "diag_directional_Mz_Nm",

    "diag_desired_Fx_N",
    "diag_desired_Fy_N",
    "diag_desired_Fz_N",
    "diag_desired_Mx_Nm",
    "diag_desired_My_Nm",
    "diag_desired_Mz_Nm",

    "diag_filtered_Fx_N",
    "diag_filtered_Fy_N",
    "diag_filtered_Fz_N",
    "diag_filtered_Mx_Nm",
    "diag_filtered_My_Nm",
    "diag_filtered_Mz_Nm",

    "diag_rendered_Fx_N",
    "diag_rendered_Fy_N",
    "diag_rendered_Fz_N",
    "diag_rendered_Mx_Nm",
    "diag_rendered_My_Nm",
    "diag_rendered_Mz_Nm",

    "diag_commanded_Fx_N",
    "diag_commanded_Fy_N",
    "diag_commanded_Fz_N",
    "diag_commanded_Mx_Nm",
    "diag_commanded_My_Nm",
    "diag_commanded_Mz_Nm",

    "diag_master_vx_m_per_s",
    "diag_master_vy_m_per_s",
    "diag_master_vz_m_per_s",
    "diag_master_wx_rad_per_s",
    "diag_master_wy_rad_per_s",
    "diag_master_wz_rad_per_s",

    "diag_franka_mapped_vx_m_per_s",
    "diag_franka_mapped_vy_m_per_s",
    "diag_franka_mapped_vz_m_per_s",
    "diag_franka_mapped_wx_rad_per_s",
    "diag_franka_mapped_wy_rad_per_s",
    "diag_franka_mapped_wz_rad_per_s",
]

CONFIG_PATHS = [
    "Common/teleop_common.hpp",
    "Master/master_config.hpp",
    "Slave/slave_config.hpp",
    "Master/masterv.cpp",
    "Slave/slavev.cpp",
    "teleop_logger.py",
]


def decode_ipv4(data, command_port, feedback_port):
    """Return (stream_name, UDP payload) for relevant loopback IPv4 UDP."""
    if len(data) < 28 or data[0] >> 4 != 4 or data[9] != 17:
        return None

    ihl = (data[0] & 15) * 4
    total = struct.unpack_from("!H", data, 2)[0]

    if ihl < 20 or total > len(data) or total < ihl + 8:
        return None

    if struct.unpack_from("!H", data, 6)[0] & 0x3FFF:
        return None

    if data[12:16] != LOOPBACK_BYTES or data[16:20] != LOOPBACK_BYTES:
        return None

    _, destination, udp_length = struct.unpack_from("!HHH", data, ihl)

    if udp_length < 8 or ihl + udp_length != total:
        return None

    stream_name = {
        command_port: "cmd",
        feedback_port: "fb",
    }.get(destination)

    if stream_name is None:
        return None

    return stream_name, data[ihl + 8:total]


def attach_filter(sock, command_port, feedback_port):
    """Classic BPF: loopback IPv4 UDP and either existing controller port."""
    class Filter(ctypes.Structure):
        _fields_ = [
            ("code", ctypes.c_ushort),
            ("jt", ctypes.c_ubyte),
            ("jf", ctypes.c_ubyte),
            ("k", ctypes.c_uint32),
        ]

    class Program(ctypes.Structure):
        _fields_ = [
            ("length", ctypes.c_ushort),
            ("filters", ctypes.POINTER(Filter)),
        ]

    instructions = [
        (0x30, 0, 0, 9),              # ldb IP protocol
        (0x15, 0, 10, 17),            # UDP, else reject
        (0x20, 0, 0, 12),             # source IPv4
        (0x15, 0, 8, 0x7F000001),
        (0x20, 0, 0, 16),             # destination IPv4
        (0x15, 0, 6, 0x7F000001),
        (0x28, 0, 0, 6),              # flags/fragment offset
        (0x45, 4, 0, 0x3FFF),         # reject fragmented IP
        (0xB1, 0, 0, 0),              # X = IP header length
        (0x48, 0, 0, 2),              # ldh [X+2]: UDP destination
        (0x15, 2, 0, command_port),
        (0x15, 1, 0, feedback_port),
        (0x06, 0, 0, 0),              # reject
        (0x06, 0, 0, 1024),           # accept
    ]

    filters = (Filter * len(instructions))(
        *[Filter(*instruction) for instruction in instructions]
    )
    program = Program(len(instructions), filters)

    libc = ctypes.CDLL(None, use_errno=True)
    result = libc.setsockopt(
        sock.fileno(),
        socket.SOL_SOCKET,
        26,  # SO_ATTACH_FILTER on Linux
        ctypes.byref(program),
        ctypes.sizeof(program),
    )

    if result != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


def norm3(values, indices):
    return math.sqrt(sum(values[index] * values[index] for index in indices))


class PacketStream:
    def __init__(self, name, fields, byte_order):
        self.name = name
        self.fields = fields
        self.index = {field: i for i, field in enumerate(fields)}
        self.decoder = struct.Struct(byte_order + str(len(fields)) + "d")

        self.latest = None
        self.received_ns = None
        self.advanced_ns = None

        self.total = 0
        self.invalid = 0

        self.reset_window()

    def reset_window(self):
        self.count = 0
        self.repeated = 0
        self.skipped = 0
        self.backward = 0
        self.max_gap_ms = None

        self.fb_peaks = [0.0] * 6
        self.fb_squares = [0.0] * 6
        self.fb_norm_peaks = [0.0, 0.0]

        self.diag_norm_peaks = {
            "rendered_force": 0.0,
            "rendered_torque": 0.0,
            "commanded_force": 0.0,
            "commanded_torque": 0.0,
        }

        self.hold_changes = 0

    def accept(self, payload, now_ns):
        if len(payload) != self.decoder.size:
            self.invalid += 1
            return

        values = self.decoder.unpack(payload)

        if not all(math.isfinite(value) for value in values):
            self.invalid += 1
            return

        if values[0] < 0 or not values[0].is_integer():
            self.invalid += 1
            return

        if self.latest is not None:
            gap_ms = (now_ns - self.received_ns) / 1e6
            self.max_gap_ms = max(self.max_gap_ms or 0.0, gap_ms)

            delta = int(values[0]) - int(self.latest[0])
            self.repeated += int(delta == 0)
            self.skipped += max(0, delta - 1)
            self.backward += int(delta < 0)

            if self.name == "diag":
                old_force_hold = self.latest[
                    self.index["diag_force_saturation_hold"]
                ]
                old_torque_hold = self.latest[
                    self.index["diag_torque_saturation_hold"]
                ]
                new_force_hold = values[
                    self.index["diag_force_saturation_hold"]
                ]
                new_torque_hold = values[
                    self.index["diag_torque_saturation_hold"]
                ]
                self.hold_changes += int(
                    old_force_hold != new_force_hold
                    or old_torque_hold != new_torque_hold
                )

        if (
            self.latest is None
            or values[0] != self.latest[0]
            or values[1] != self.latest[1]
        ):
            self.advanced_ns = now_ns

        self.latest = values
        self.received_ns = now_ns
        self.total += 1
        self.count += 1

        if self.name == "fb":
            for i, value in enumerate(values[2:8]):
                self.fb_peaks[i] = max(self.fb_peaks[i], abs(value))
                self.fb_squares[i] += value * value

            self.fb_norm_peaks[0] = max(
                self.fb_norm_peaks[0],
                math.hypot(*values[2:5]),
            )
            self.fb_norm_peaks[1] = max(
                self.fb_norm_peaks[1],
                math.hypot(*values[5:8]),
            )

        elif self.name == "diag":
            groups = {
                "rendered_force": [
                    self.index["diag_rendered_Fx_N"],
                    self.index["diag_rendered_Fy_N"],
                    self.index["diag_rendered_Fz_N"],
                ],
                "rendered_torque": [
                    self.index["diag_rendered_Mx_Nm"],
                    self.index["diag_rendered_My_Nm"],
                    self.index["diag_rendered_Mz_Nm"],
                ],
                "commanded_force": [
                    self.index["diag_commanded_Fx_N"],
                    self.index["diag_commanded_Fy_N"],
                    self.index["diag_commanded_Fz_N"],
                ],
                "commanded_torque": [
                    self.index["diag_commanded_Mx_Nm"],
                    self.index["diag_commanded_My_Nm"],
                    self.index["diag_commanded_Mz_Nm"],
                ],
            }

            for key, indices in groups.items():
                self.diag_norm_peaks[key] = max(
                    self.diag_norm_peaks[key],
                    norm3(values, indices),
                )

    def snapshot(self, now_ns):
        prefix = self.name + "_"

        age_ms = (
            None
            if self.received_ns is None
            else (now_ns - self.received_ns) / 1e6
        )
        progress_age_ms = (
            None
            if self.advanced_ns is None
            else (now_ns - self.advanced_ns) / 1e6
        )

        row = {
            prefix + "seen": int(self.latest is not None),
            prefix + "logger_receive_age_ms": age_ms,
            prefix + "id_or_timestamp_change_age_ms": progress_age_ms,
            prefix + "receive_stale_50ms": int(
                age_ms is None or age_ms >= 50.0
            ),
            prefix + "valid_packets_window": self.count,
            prefix + "valid_packets_total": self.total,
            prefix + "invalid_packets_total": self.invalid,
            prefix + "repeated_ids_window": self.repeated,
            prefix + "skipped_ids_window": self.skipped,
            prefix + "backward_ids_window": self.backward,
            prefix + "max_dequeue_gap_ms_window": self.max_gap_ms,
        }

        row.update(
            zip(
                self.fields,
                self.latest or [None] * len(self.fields),
            )
        )

        if self.name == "cmd":
            quaternion = (
                None if self.latest is None else self.latest[2:6]
            )
            row["cmd_quaternion_norm"] = (
                math.hypot(*quaternion) if quaternion else None
            )

        elif self.name == "fb":
            quaternion = (
                None if self.latest is None else self.latest[11:15]
            )
            row["fb_quaternion_norm"] = (
                math.hypot(*quaternion) if quaternion else None
            )

            for i, field in enumerate(FEEDBACK_FIELDS[2:8]):
                row[field + "_abs_peak_window"] = (
                    self.fb_peaks[i] if self.count else None
                )
                row[field + "_rms_window"] = (
                    math.sqrt(self.fb_squares[i] / self.count)
                    if self.count
                    else None
                )

            row["franka_force_norm_peak_N_window"] = (
                self.fb_norm_peaks[0] if self.count else None
            )
            row["franka_moment_norm_peak_Nm_window"] = (
                self.fb_norm_peaks[1] if self.count else None
            )

        elif self.name == "diag":
            if self.latest is None:
                for name in (
                    "diag_rendered_force_norm_N",
                    "diag_rendered_torque_norm_Nm",
                    "diag_commanded_force_norm_N",
                    "diag_commanded_torque_norm_Nm",
                    "diag_mapped_force_norm_N",
                    "diag_mapped_torque_norm_Nm",
                ):
                    row[name] = None
            else:
                groups = {
                    "diag_rendered_force_norm_N": (
                        "diag_rendered_Fx_N",
                        "diag_rendered_Fy_N",
                        "diag_rendered_Fz_N",
                    ),
                    "diag_rendered_torque_norm_Nm": (
                        "diag_rendered_Mx_Nm",
                        "diag_rendered_My_Nm",
                        "diag_rendered_Mz_Nm",
                    ),
                    "diag_commanded_force_norm_N": (
                        "diag_commanded_Fx_N",
                        "diag_commanded_Fy_N",
                        "diag_commanded_Fz_N",
                    ),
                    "diag_commanded_torque_norm_Nm": (
                        "diag_commanded_Mx_Nm",
                        "diag_commanded_My_Nm",
                        "diag_commanded_Mz_Nm",
                    ),
                    "diag_mapped_force_norm_N": (
                        "diag_mapped_Fx_N",
                        "diag_mapped_Fy_N",
                        "diag_mapped_Fz_N",
                    ),
                    "diag_mapped_torque_norm_Nm": (
                        "diag_mapped_Mx_Nm",
                        "diag_mapped_My_Nm",
                        "diag_mapped_Mz_Nm",
                    ),
                }

                for output_name, field_names in groups.items():
                    indices = [self.index[name] for name in field_names]
                    row[output_name] = norm3(self.latest, indices)

            row["diag_rendered_force_norm_peak_N_window"] = (
                self.diag_norm_peaks["rendered_force"]
                if self.count
                else None
            )
            row["diag_rendered_torque_norm_peak_Nm_window"] = (
                self.diag_norm_peaks["rendered_torque"]
                if self.count
                else None
            )
            row["diag_commanded_force_norm_peak_N_window"] = (
                self.diag_norm_peaks["commanded_force"]
                if self.count
                else None
            )
            row["diag_commanded_torque_norm_peak_Nm_window"] = (
                self.diag_norm_peaks["commanded_torque"]
                if self.count
                else None
            )
            row["diag_hold_state_changes_window"] = self.hold_changes

        self.reset_window()
        return row


class PassiveControllerCapture:
    def __init__(
        self,
        sock,
        command_port,
        feedback_port,
        byte_order,
        stop_event,
    ):
        self.sock = sock
        self.command_port = command_port
        self.feedback_port = feedback_port
        self.stop = stop_event

        self.streams = {
            "cmd": PacketStream("cmd", COMMAND_FIELDS, byte_order),
            "fb": PacketStream("fb", FEEDBACK_FIELDS, byte_order),
        }

        self.lock = threading.Lock()
        self.error = None
        self.thread = threading.Thread(
            target=self.run,
            name="controller-packet-capture",
            daemon=True,
        )

    def run(self):
        try:
            while not self.stop.is_set():
                try:
                    data, address = self.sock.recvfrom(2048)
                except socket.timeout:
                    continue

                # PACKET_OUTGOING: avoid the duplicate loopback copy.
                if address[2] == 4:
                    continue

                decoded = decode_ipv4(
                    data,
                    self.command_port,
                    self.feedback_port,
                )

                if decoded is None:
                    continue

                stream_name, payload = decoded

                with self.lock:
                    self.streams[stream_name].accept(
                        payload,
                        time.monotonic_ns(),
                    )

        except Exception as exc:
            self.error = repr(exc)
            self.stop.set()

    def snapshot(self, now_ns):
        row = {}

        with self.lock:
            for stream in self.streams.values():
                row.update(stream.snapshot(now_ns))

        return row


class DiagnosticCapture:
    def __init__(self, sock, byte_order, stop_event):
        self.sock = sock
        self.stop = stop_event
        self.stream = PacketStream(
            "diag",
            DIAGNOSTIC_FIELDS,
            byte_order,
        )

        self.lock = threading.Lock()
        self.error = None
        self.thread = threading.Thread(
            target=self.run,
            name="master-diagnostic-capture",
            daemon=True,
        )

    def run(self):
        try:
            while not self.stop.is_set():
                try:
                    payload, _ = self.sock.recvfrom(4096)
                except socket.timeout:
                    continue

                with self.lock:
                    self.stream.accept(
                        payload,
                        time.monotonic_ns(),
                    )

        except Exception as exc:
            self.error = repr(exc)
            self.stop.set()

    def snapshot(self, now_ns):
        with self.lock:
            return self.stream.snapshot(now_ns)


def git_value(root, arguments):
    try:
        process = subprocess.run(
            [
                "git",
                "--no-optional-locks",
                "-C",
                str(root),
                *arguments,
            ],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )

        if process.returncode == 0:
            return process.stdout.strip()

    except (OSError, subprocess.TimeoutExpired):
        pass

    return None


def project_metadata(project):
    if project is None:
        return None

    root = project.expanduser().resolve()

    result = {
        "path": str(root),
        "files": {},
        "head": git_value(root, ["rev-parse", "HEAD"]),
        "branch": git_value(root, ["branch", "--show-current"]),
        "status": git_value(root, ["status", "--short"]),
    }

    for relative_name in CONFIG_PATHS:
        path = root / relative_name

        if not path.is_file():
            continue

        content = path.read_bytes()

        entry = {
            "sha256": hashlib.sha256(content).hexdigest(),
            "size_bytes": len(content),
        }

        # Config text is useful when interpreting a thesis log months later.
        if relative_name.endswith(".hpp"):
            entry["text"] = content.decode(
                "utf-8",
                errors="replace",
            )

        result["files"][relative_name] = entry

    return result


def drop_sudo_privileges():
    if os.geteuid() != 0 or not os.environ.get("SUDO_UID"):
        return

    uid = int(os.environ["SUDO_UID"])
    gid = int(os.environ["SUDO_GID"])

    if uid == 0:
        return

    username = pwd.getpwuid(uid).pw_name
    os.initgroups(username, gid)
    os.setgid(gid)
    os.setuid(uid)


def run(args):
    if not sys.platform.startswith("linux"):
        raise RuntimeError(
            "This logger requires Linux for AF_PACKET capture."
        )

    stop_event = threading.Event()

    # Existing controller traffic: passive raw capture.
    raw_sock = socket.socket(
        socket.AF_PACKET,
        socket.SOCK_DGRAM,
        socket.htons(0x0800),
    )

    # Logging-only Master telemetry: ordinary UDP receive socket.
    diag_sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
        socket.IPPROTO_UDP,
    )

    try:
        raw_sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_RCVBUF,
            4 * 1024 * 1024,
        )
        attach_filter(
            raw_sock,
            args.command_port,
            args.feedback_port,
        )
        raw_sock.bind(("lo", 0))
        raw_sock.settimeout(0.1)

        diag_sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_RCVBUF,
            1 * 1024 * 1024,
        )
        diag_sock.bind(("127.0.0.1", args.diagnostic_port))
        diag_sock.settimeout(0.1)

        drop_sudo_privileges()

        try:
            os.nice(10)
        except OSError:
            pass

        byte_order = (
            "<" if args.byte_order == "little" else ">"
        )

        passive_capture = PassiveControllerCapture(
            raw_sock,
            args.command_port,
            args.feedback_port,
            byte_order,
            stop_event,
        )
        diagnostic_capture = DiagnosticCapture(
            diag_sock,
            byte_order,
            stop_event,
        )

        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(
                sig,
                lambda *_: stop_event.set(),
            )

        args.output.mkdir(
            parents=True,
            exist_ok=True,
        )

        stamp = dt.datetime.now(
            dt.timezone.utc
        ).strftime("%Y%m%dT%H%M%S_%fZ")

        label = re.sub(
            r"[^A-Za-z0-9_-]+",
            "_",
            args.label,
        ).strip("_") or "run"

        csv_path = args.output / (
            stamp + "_" + label + ".csv"
        )
        metadata_path = csv_path.with_suffix(".json")

        project_start = project_metadata(args.project)

        metadata = {
            "schema_version": 2,
            "reference_commit": (
                project_start.get("head")
                if project_start
                else None
            ),
            "logger_sha256": hashlib.sha256(
                Path(__file__).read_bytes()
            ).hexdigest(),
            "target_log_hz": 100,
            "interface": "lo",
            "ipv4_endpoints": "127.0.0.1",
            "command_port": args.command_port,
            "feedback_port": args.feedback_port,
            "diagnostic_port": args.diagnostic_port,
            "diagnostic_field_count": len(DIAGNOSTIC_FIELDS),
            "byte_order": args.byte_order,
            "label": args.label,
            "notes": args.notes,
            "project_at_start": project_start,
            "raw_socket_receive_buffer_bytes": raw_sock.getsockopt(
                socket.SOL_SOCKET,
                socket.SO_RCVBUF,
            ),
            "diagnostic_socket_receive_buffer_bytes": diag_sock.getsockopt(
                socket.SOL_SOCKET,
                socket.SO_RCVBUF,
            ),
            "diagnostic_fields": DIAGNOSTIC_FIELDS,
            "semantics": __doc__,
            "complete": False,
        }

        metadata_path.write_text(
            json.dumps(metadata, indent=2) + "\n"
        )

        start_ns = time.monotonic_ns()
        previous_ns = start_ns
        deadline_ns = start_ns + PERIOD_NS
        end_ns = (
            start_ns + int(args.duration * 1e9)
            if args.duration
            else None
        )

        metadata["start_utc"] = dt.datetime.now(
            dt.timezone.utc
        ).isoformat()

        rows = 0
        missed_ticks = 0
        failure = None

        passive_capture.thread.start()
        diagnostic_capture.thread.start()

        print(
            "Final-validation logger started at 100 CSV rows/s.",
            flush=True,
        )
        print(
            "Controller streams: %d / %d | diagnostic: %d"
            % (
                args.command_port,
                args.feedback_port,
                args.diagnostic_port,
            ),
            flush=True,
        )
        print(
            "CSV: " + str(csv_path.resolve()),
            flush=True,
        )

        try:
            with csv_path.open(
                "x",
                newline="",
                buffering=65536,
            ) as handle:
                writer = None

                while not stop_event.is_set():
                    now_ns = time.monotonic_ns()

                    wake_ns = (
                        min(deadline_ns, end_ns)
                        if end_ns
                        else deadline_ns
                    )

                    if stop_event.wait(
                        max(0.0, (wake_ns - now_ns) / 1e9)
                    ):
                        break

                    if (
                        end_ns
                        and time.monotonic_ns() >= end_ns
                        and deadline_ns > end_ns
                    ):
                        break

                    now_ns = time.monotonic_ns()

                    row = {}
                    row.update(
                        passive_capture.snapshot(now_ns)
                    )
                    row.update(
                        diagnostic_capture.snapshot(now_ns)
                    )

                    late_ns = max(
                        0,
                        now_ns - deadline_ns,
                    )

                    missed_ticks += (
                        late_ns // PERIOD_NS
                    )

                    row = {
                        "row": rows,
                        "logger_t_s": (
                            now_ns - start_ns
                        ) / 1e9,
                        "logger_window_ms": (
                            now_ns - previous_ns
                        ) / 1e6,
                        "logger_lateness_ms": late_ns / 1e6,
                        "logger_skipped_ticks_total": missed_ticks,
                        **row,
                    }

                    if writer is None:
                        metadata["columns"] = list(row)
                        writer = csv.DictWriter(
                            handle,
                            fieldnames=list(row),
                        )
                        writer.writeheader()

                    writer.writerow(row)

                    rows += 1
                    previous_ns = now_ns
                    deadline_ns += (
                        late_ns // PERIOD_NS + 1
                    ) * PERIOD_NS

                    if rows % 100 == 0:
                        handle.flush()

                    if rows % 500 == 0:
                        print(
                            "rows=%d cmd_age=%s ms fb_age=%s ms "
                            "diag_age=%s ms skipped=%d"
                            % (
                                rows,
                                row.get(
                                    "cmd_logger_receive_age_ms"
                                ),
                                row.get(
                                    "fb_logger_receive_age_ms"
                                ),
                                row.get(
                                    "diag_logger_receive_age_ms"
                                ),
                                missed_ticks,
                            ),
                            flush=True,
                        )

                        if not row.get("diag_seen"):
                            print(
                                "No Master diagnostic telemetry yet. "
                                "Check that the telemetry-enabled "
                                "Master is running.",
                                flush=True,
                            )

        except BaseException as exc:
            failure = repr(exc)
            raise

        finally:
            stop_event.set()

            passive_capture.thread.join(timeout=2)
            diagnostic_capture.thread.join(timeout=2)

            duration_s = (
                time.monotonic_ns() - start_ns
            ) / 1e9

            try:
                packets, drops = struct.unpack(
                    "=II",
                    raw_sock.getsockopt(
                        263,
                        6,
                        8,
                    ),
                )
                metadata["kernel_capture_packets"] = packets
                metadata["kernel_capture_drops"] = drops

            except OSError as exc:
                metadata[
                    "kernel_capture_statistics_error"
                ] = str(exc)

            metadata.update(
                {
                    "complete": (
                        failure is None
                        and passive_capture.error is None
                        and diagnostic_capture.error is None
                    ),
                    "error": (
                        failure
                        or passive_capture.error
                        or diagnostic_capture.error
                    ),
                    "rows": rows,
                    "wall_duration_s": duration_s,
                    "logger_skipped_ticks": missed_ticks,
                    "last_row_t_s": (
                        previous_ns - start_ns
                    ) / 1e9,
                    "valid_packets": {
                        "cmd": passive_capture.streams[
                            "cmd"
                        ].total,
                        "fb": passive_capture.streams[
                            "fb"
                        ].total,
                        "diag": diagnostic_capture.stream.total,
                    },
                    "invalid_packets": {
                        "cmd": passive_capture.streams[
                            "cmd"
                        ].invalid,
                        "fb": passive_capture.streams[
                            "fb"
                        ].invalid,
                        "diag": diagnostic_capture.stream.invalid,
                    },
                    "project_at_end": project_metadata(
                        args.project
                    ),
                }
            )

            metadata_path.write_text(
                json.dumps(
                    metadata,
                    indent=2,
                )
                + "\n"
            )

            print(
                "Saved %d rows. AF_PACKET drops: %s. Metadata: %s"
                % (
                    rows,
                    metadata.get(
                        "kernel_capture_drops",
                        "unknown",
                    ),
                    metadata_path,
                ),
                flush=True,
            )

        if passive_capture.error:
            raise RuntimeError(
                "Controller capture failed: "
                + passive_capture.error
            )

        if diagnostic_capture.error:
            raise RuntimeError(
                "Diagnostic capture failed: "
                + diagnostic_capture.error
            )

    finally:
        raw_sock.close()
        diag_sock.close()


def main():
    script_root = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=script_root / "logs",
        help="output directory; default: logs/ beside this script",
    )
    parser.add_argument(
        "--label",
        default="run",
        help="run label used in filenames",
    )
    parser.add_argument(
        "--notes",
        default="",
        help="test description stored in JSON metadata",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0,
        help="seconds; 0 means until Ctrl+C",
    )
    parser.add_argument(
        "--project",
        type=Path,
        default=script_root,
        help="repo path for provenance; default: script directory",
    )
    parser.add_argument(
        "--command-port",
        type=int,
        default=11056,
    )
    parser.add_argument(
        "--feedback-port",
        type=int,
        default=11055,
    )
    parser.add_argument(
        "--diagnostic-port",
        type=int,
        default=11057,
    )
    parser.add_argument(
        "--byte-order",
        choices=("little", "big"),
        default=sys.byteorder,
        help="sender native double byte order; x86 lab machines use little",
    )

    args = parser.parse_args()

    if (
        not math.isfinite(args.duration)
        or args.duration < 0
    ):
        parser.error(
            "duration must be finite and nonnegative"
        )

    ports = (
        args.command_port,
        args.feedback_port,
        args.diagnostic_port,
    )

    if (
        not all(
            1 <= port <= 65535
            for port in ports
        )
        or len(set(ports)) != len(ports)
    ):
        parser.error(
            "command, feedback, and diagnostic ports must "
            "be distinct and within 1..65535"
        )

    if len(DIAGNOSTIC_FIELDS) != 106:
        parser.error(
            "internal diagnostic schema mismatch: "
            "expected 106 doubles"
        )

    try:
        run(args)

    except PermissionError as exc:
        print(
            "Permission denied: %s. "
            "AF_PACKET capture needs sudo/CAP_NET_RAW."
            % exc,
            file=sys.stderr,
        )
        return 1

    except (OSError, RuntimeError) as exc:
        print(
            "Logger error: " + str(exc),
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
