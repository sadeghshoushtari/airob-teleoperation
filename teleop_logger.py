#!/usr/bin/env python3
"""Passive 100 Hz diagnostic logger for the local Virtuose--Franka UDP link.

Linux / Python 3.8+, standard library only; no compilation required.
Place this script in the thesis root, beside Master/, Slave/, and Common/.
Run in a separate terminal:
    sudo python3 teleop_logger.py --label rotation_xyz
By default, logs/ is created beside this script, regardless of the terminal's
working directory. --output overrides that location if explicitly supplied.
Stop with Ctrl+C. Optional: --duration 60. See --help for other options.

No controller edits, UDP port sharing, forwarding, or transmitted packets.
An AF_PACKET socket receives copies on lo. A kernel filter selects only
127.0.0.1 -> 127.0.0.1 UDP destination ports 11056 and 11055. Outgoing copies
are discarded. Capture requires root or CAP_NET_RAW. When invoked with sudo,
root privileges are dropped to the invoking user after opening the socket.

One CSV row per nominal 10 ms contains the latest command and feedback,
plus statistics over all accepted packets dequeued since the preceding row.
The existing 1 kHz traffic is inspected in memory; no full-rate file is made.
Capture and disk writing use separate threads. Scheduling is best-effort:
late intervals are recorded, skipped ticks are counted, and no catch-up rows
are invented. This process still uses CPU; lab timing impact is unvalidated.

Measurement limits:
  * Poses and raw Franka estimated external wrench are on-wire values.
  * Commanded slave Kt and feedback applied slave Kt/Kr are transmitted.
  * No master velocity, bias, rendered wrench, individual haptic terms,
    captured references, desired slave pose, alarm, or active flag is sent.
    Therefore this logger does not fabricate exact tracking errors, master
    power/passivity, saturation, trigger position, damping, or active state.
  * A command and feedback in one row are independently sampled latest
    values, not a matched request/reply. Their source clocks start separately.
  * Receive times/ages use logger dequeue time, not kernel/hardware arrival
    time or one-way latency. Backlog can make old packets appear recent.
  * Feedback is assembled from separate atomic fields in the controller;
    the wire packet is not guaranteed to be one coherent control-cycle state.
  * Feedback IDs are callback IDs, not send IDs. Skips/repeats are observations,
    not proof of network loss. Backward IDs can also mean restart/reordering.
  * 100 Hz snapshots are not anti-aliased. Do not use them for >50 Hz spectra
    or rigorous 1 kHz passivity analysis. Window RMS/absolute peaks preserve
    some brief wrench events, but not their full waveform. Duplicated feedback
    samples are included, so RMS is packet-weighted, not callback-weighted.

Each run creates a CSV and JSON with schema notes, capture diagnostics,
reference commit, and optional hashes/Git state of the local project.
No recorded configuration or Git state proves which executable is running.
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

REFERENCE_COMMIT = "085cbc9317c1a2484677f3ccfe0a5fd56d653faa"
PERIOD_NS = 10_000_000
LOOPBACK = socket.inet_aton("127.0.0.1")
COMMAND_FIELDS = [
    "cmd_id", "cmd_source_time_us", "master_qw", "master_qx", "master_qy",
    "master_qz", "master_x_m", "master_y_m", "master_z_m",
    "force_reflection_gain_metadata", "torque_reflection_gain_metadata",
    "slave_Kt_command_N_per_m", "deadman_raw", "estop_raw",
]
FEEDBACK_FIELDS = [
    "fb_id", "fb_source_time_us", "franka_Fx_N", "franka_Fy_N", "franka_Fz_N",
    "franka_Mx_Nm", "franka_My_Nm", "franka_Mz_Nm",
    "franka_x_m", "franka_y_m", "franka_z_m",
    "franka_qw", "franka_qx", "franka_qy", "franka_qz",
    "franka_vx_m_per_s", "franka_vy_m_per_s", "franka_vz_m_per_s",
    "franka_wx_rad_per_s", "franka_wy_rad_per_s", "franka_wz_rad_per_s",
    "slave_Kt_applied_N_per_m", "slave_Kr_applied_Nm_per_rad",
]
CONFIG_PATHS = [
    "Common/teleop_common.hpp", "Master/master_config.hpp",
    "Slave/slave_config.hpp", "Master/masterv.cpp", "Slave/slavev.cpp",
]


def decode_ipv4(data, command_port, feedback_port):
    """Return (stream, payload); reject fragmented/truncated/unrelated packets."""
    if len(data) < 28 or data[0] >> 4 != 4 or data[9] != 17:
        return None
    ihl = (data[0] & 15) * 4
    total = struct.unpack_from("!H", data, 2)[0]
    if ihl < 20 or total > len(data) or total < ihl + 8:
        return None
    if struct.unpack_from("!H", data, 6)[0] & 0x3fff:
        return None
    if data[12:16] != LOOPBACK or data[16:20] != LOOPBACK:
        return None
    _, destination, length = struct.unpack_from("!HHH", data, ihl)
    if length < 8 or ihl + length != total:
        return None
    name = {command_port: "cmd", feedback_port: "fb"}.get(destination)
    return (name, data[ihl + 8:total]) if name else None


def attach_filter(sock, command_port, feedback_port):
    """Classic BPF for cooked IPv4 packets (no Ethernet header)."""
    class Filter(ctypes.Structure):
        _fields_ = [("code", ctypes.c_ushort), ("jt", ctypes.c_ubyte),
                    ("jf", ctypes.c_ubyte), ("k", ctypes.c_uint32)]

    class Program(ctypes.Structure):
        _fields_ = [("length", ctypes.c_ushort), ("filters", ctypes.POINTER(Filter))]

    instructions = [
        (0x30, 0, 0, 9),              # ldb IP protocol
        (0x15, 0, 10, 17),            # UDP, else reject
        (0x20, 0, 0, 12),             # source IPv4
        (0x15, 0, 8, 0x7f000001),
        (0x20, 0, 0, 16),             # destination IPv4
        (0x15, 0, 6, 0x7f000001),
        (0x28, 0, 0, 6),              # flags/fragment offset
        (0x45, 4, 0, 0x3fff),         # reject fragmented IP
        (0xb1, 0, 0, 0),              # X = IP header length
        (0x48, 0, 0, 2),              # ldh [X+2]: UDP destination
        (0x15, 2, 0, command_port),
        (0x15, 1, 0, feedback_port),
        (0x06, 0, 0, 0),              # reject
        (0x06, 0, 0, 512),            # capture enough for either packet
    ]
    filters = (Filter * len(instructions))(*[Filter(*x) for x in instructions])
    program = Program(len(instructions), filters)
    libc = ctypes.CDLL(None, use_errno=True)
    result = libc.setsockopt(sock.fileno(), socket.SOL_SOCKET, 26,
                            ctypes.byref(program), ctypes.sizeof(program))
    if result != 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))


class Stream:
    def __init__(self, name, byte_order):
        self.name = name
        self.fields = COMMAND_FIELDS if name == "cmd" else FEEDBACK_FIELDS
        self.decoder = struct.Struct(byte_order + str(len(self.fields)) + "d")
        self.latest = None
        self.received_ns = None
        self.advanced_ns = None
        self.total = 0
        self.invalid = 0
        self.reset_window()

    def reset_window(self):
        self.count = self.repeated = self.skipped = self.backward = 0
        self.max_gap_ms = None
        self.peaks = [0.0] * 6
        self.squares = [0.0] * 6
        self.norm_peaks = [0.0, 0.0]
        self.state_changes = 0

    def accept(self, payload, now_ns):
        if len(payload) != self.decoder.size:
            self.invalid += 1
            return
        values = self.decoder.unpack(payload)
        if (not all(math.isfinite(x) for x in values) or
                values[0] < 0 or not values[0].is_integer()):
            self.invalid += 1
            return
        if self.latest is not None:
            gap = (now_ns - self.received_ns) / 1e6
            self.max_gap_ms = max(self.max_gap_ms or 0.0, gap)
            delta = int(values[0]) - int(self.latest[0])
            self.repeated += int(delta == 0)
            self.skipped += max(0, delta - 1)
            self.backward += int(delta < 0)
            if self.name == "cmd":
                self.state_changes += int(values[12:14] != self.latest[12:14])
        if (self.latest is None or values[0] != self.latest[0] or
                values[1] != self.latest[1]):
            self.advanced_ns = now_ns
        self.latest, self.received_ns = values, now_ns
        self.total += 1
        self.count += 1
        if self.name == "fb":
            for i, value in enumerate(values[2:8]):
                self.peaks[i] = max(self.peaks[i], abs(value))
                self.squares[i] += value * value
            for i in range(2):
                magnitude = math.hypot(*values[2 + 3*i:5 + 3*i])
                self.norm_peaks[i] = max(self.norm_peaks[i], magnitude)

    def snapshot(self, now_ns):
        p = self.name + "_"
        age = None if self.received_ns is None else (now_ns - self.received_ns) / 1e6
        progress_age = (None if self.advanced_ns is None else
                        (now_ns - self.advanced_ns) / 1e6)
        row = {
            p + "seen": int(self.latest is not None),
            p + "logger_receive_age_ms": age,
            p + "id_or_timestamp_change_age_ms": progress_age,
            p + "receive_stale_50ms": int(age is None or age >= 50),
            p + "valid_packets_window": self.count,
            p + "valid_packets_total": self.total,
            p + "invalid_packets_total": self.invalid,
            p + "repeated_ids_window": self.repeated,
            p + "skipped_ids_window": self.skipped,
            p + "backward_ids_window": self.backward,
            p + "max_dequeue_gap_ms_window": self.max_gap_ms,
        }
        row.update(zip(self.fields, self.latest or [None] * len(self.fields)))
        q = None if self.latest is None else (
            self.latest[2:6] if self.name == "cmd" else self.latest[11:15])
        row[p + "quaternion_norm"] = math.hypot(*q) if q else None
        if self.name == "cmd":
            row["deadman_or_estop_changes_window"] = self.state_changes
        else:
            for i, field in enumerate(FEEDBACK_FIELDS[2:8]):
                row[field + "_abs_peak_window"] = self.peaks[i] if self.count else None
                row[field + "_rms_window"] = (
                    math.sqrt(self.squares[i] / self.count) if self.count else None)
            row["franka_force_norm_peak_N_window"] = self.norm_peaks[0] if self.count else None
            row["franka_moment_norm_peak_Nm_window"] = self.norm_peaks[1] if self.count else None
        self.reset_window()
        return row


class Capture:
    def __init__(self, sock, command_port, feedback_port, byte_order):
        self.sock = sock
        self.ports = (command_port, feedback_port)
        self.streams = {name: Stream(name, byte_order) for name in ("cmd", "fb")}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.run, name="udp-capture", daemon=True)

    def run(self):
        try:
            while not self.stop.is_set():
                try:
                    data, address = self.sock.recvfrom(512)
                except socket.timeout:
                    continue
                if address[2] == 4:  # PACKET_OUTGOING: avoid loopback duplication
                    continue
                decoded = decode_ipv4(data, *self.ports)
                if decoded:
                    with self.lock:
                        self.streams[decoded[0]].accept(decoded[1], time.monotonic_ns())
        except Exception as exc:
            self.error = repr(exc)
            self.stop.set()

    def snapshot(self):
        with self.lock:
            now_ns = time.monotonic_ns()
            row = {}
            for stream in self.streams.values():
                row.update(stream.snapshot(now_ns))
        return now_ns, row


def project_metadata(project):
    if project is None:
        return None
    root = project.expanduser().resolve()
    result = {"path": str(root), "files": {}}
    for name in CONFIG_PATHS:
        path = root / name
        if path.is_file():
            content = path.read_bytes()
            result["files"][name] = {"sha256": hashlib.sha256(content).hexdigest()}
            if name.endswith(".hpp"):
                result["files"][name]["text"] = content.decode("utf-8", errors="replace")
    for key, arguments in [("head", ["rev-parse", "HEAD"]),
                           ("branch", ["branch", "--show-current"]),
                           ("status", ["status", "--short"])]:
        try:
            proc = subprocess.run(["git", "--no-optional-locks", "-C", str(root)] + arguments,
                                  capture_output=True, text=True, timeout=3, check=False)
            result[key] = proc.stdout.strip() if proc.returncode == 0 else None
        except (OSError, subprocess.TimeoutExpired):
            result[key] = None
    return result


def drop_sudo_privileges():
    if os.geteuid() == 0 and os.environ.get("SUDO_UID"):
        uid, gid = int(os.environ["SUDO_UID"]), int(os.environ["SUDO_GID"])
        if uid != 0:
            os.initgroups(pwd.getpwuid(uid).pw_name, gid)
            os.setgid(gid)
            os.setuid(uid)


def run(args):
    if not sys.platform.startswith("linux"):
        raise RuntimeError("This logger requires Linux AF_PACKET capture.")
    sock = socket.socket(socket.AF_PACKET, socket.SOCK_DGRAM, socket.htons(0x0800))
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        attach_filter(sock, args.command_port, args.feedback_port)
        sock.bind(("lo", 0))
        sock.settimeout(0.1)
        drop_sudo_privileges()
        os.nice(10)
        capture = Capture(sock, args.command_port, args.feedback_port,
                          "<" if args.byte_order == "little" else ">")
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: capture.stop.set())
        args.output.mkdir(parents=True, exist_ok=True)
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        label = re.sub(r"[^A-Za-z0-9_-]+", "_", args.label).strip("_") or "run"
        csv_path = args.output / (stamp + "_" + label + ".csv")
        meta_path = csv_path.with_suffix(".json")
        metadata = {
            "schema_version": 1, "reference_commit": REFERENCE_COMMIT,
            "logger_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "target_log_hz": 100, "interface": "lo", "ipv4_endpoints": "127.0.0.1",
            "command_port": args.command_port, "feedback_port": args.feedback_port,
            "byte_order": args.byte_order, "label": args.label,
            "notes": args.notes, "project_at_start": project_metadata(args.project),
            "socket_receive_buffer_bytes": sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
            "semantics": __doc__, "complete": False,
        }
        with meta_path.open("x") as handle:
            json.dump(metadata, handle, indent=2)
        rows = missed = 0
        failure = None
        start_ns = time.monotonic_ns()
        metadata["start_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        previous_ns = start_ns
        deadline = start_ns + PERIOD_NS
        end_ns = start_ns + int(args.duration * 1e9) if args.duration else None
        capture.thread.start()
        print("Passive capture started; target 100 CSV rows/s. Ctrl+C stops only the logger.", flush=True)
        print("CSV: " + str(csv_path.resolve()), flush=True)
        try:
            with csv_path.open("x", newline="", buffering=65536) as handle:
                writer = None
                while not capture.stop.is_set():
                    now_ns = time.monotonic_ns()
                    wake_ns = min(deadline, end_ns) if end_ns else deadline
                    if capture.stop.wait(max(0, (wake_ns - now_ns) / 1e9)):
                        break
                    if end_ns and time.monotonic_ns() >= end_ns and deadline > end_ns:
                        break
                    now_ns, row = capture.snapshot()
                    late_ns = max(0, now_ns - deadline)
                    missed += late_ns // PERIOD_NS
                    row = {
                        "row": rows, "logger_t_s": (now_ns - start_ns) / 1e9,
                        "logger_window_ms": (now_ns - previous_ns) / 1e6,
                        "logger_lateness_ms": late_ns / 1e6,
                        "logger_skipped_ticks_total": missed, **row,
                    }
                    if writer is None:
                        metadata["columns"] = list(row)
                        writer = csv.DictWriter(handle, fieldnames=list(row))
                        writer.writeheader()
                    writer.writerow(row)
                    rows += 1
                    previous_ns = now_ns
                    deadline += (late_ns // PERIOD_NS + 1) * PERIOD_NS
                    if rows % 100 == 0:
                        handle.flush()
                    if rows % 500 == 0:
                        print("rows=%d  cmd age=%s ms  fb age=%s ms  skipped ticks=%d" %
                              (rows, row["cmd_logger_receive_age_ms"],
                               row["fb_logger_receive_age_ms"], missed), flush=True)
                        if not row["cmd_seen"] or not row["fb_seen"]:
                            print("Waiting for both streams on lo; check controller startup and ports.", flush=True)
        except BaseException as exc:
            failure = repr(exc)
            raise
        finally:
            capture.stop.set()
            capture.thread.join(timeout=2)
            duration = (time.monotonic_ns() - start_ns) / 1e9
            try:
                packets, drops = struct.unpack("=II", sock.getsockopt(263, 6, 8))
                metadata["kernel_capture_packets"] = packets
                metadata["kernel_capture_drops"] = drops
            except OSError as exc:
                metadata["kernel_capture_statistics_error"] = str(exc)
            metadata.update({
                "complete": failure is None and capture.error is None,
                "error": failure or capture.error, "rows": rows,
                "wall_duration_s": duration, "logger_skipped_ticks": missed,
                "last_row_t_s": (previous_ns - start_ns) / 1e9,
                "valid_packets": {k: v.total for k, v in capture.streams.items()},
                "invalid_packets": {k: v.invalid for k, v in capture.streams.items()},
                "project_at_end": project_metadata(args.project),
            })
            meta_path.write_text(json.dumps(metadata, indent=2) + "\n")
            print("Saved %d rows. Kernel capture drops: %s. Metadata: %s" %
                  (rows, metadata.get("kernel_capture_drops", "unknown"), meta_path), flush=True)
        if capture.error:
            raise RuntimeError("Capture failed: " + capture.error)
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parent / "logs",
                        help="output directory (default: logs/ beside this script)")
    parser.add_argument("--label", default="run", help="run label in filenames")
    parser.add_argument("--notes", default="", help="test description saved in JSON")
    parser.add_argument("--duration", type=float, default=0, help="seconds; 0 means until Ctrl+C")
    parser.add_argument("--project", type=Path, help="optional local repo for read-only provenance")
    parser.add_argument("--command-port", type=int, default=11056)
    parser.add_argument("--feedback-port", type=int, default=11055)
    parser.add_argument("--byte-order", choices=("little", "big"), default=sys.byteorder,
                        help="sender's native double byte order; lab x86 uses little")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("duration must be finite and nonnegative")
    if (not all(1 <= p <= 65535 for p in (args.command_port, args.feedback_port)) or
            args.command_port == args.feedback_port):
        parser.error("ports must be distinct and within 1..65535")
    try:
        run(args)
    except PermissionError as exc:
        print("Permission denied: %s. Capture needs sudo/CAP_NET_RAW; output must be writable." % exc,
              file=sys.stderr)
        return 1
    except (OSError, RuntimeError) as exc:
        print("Logger error: " + str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
