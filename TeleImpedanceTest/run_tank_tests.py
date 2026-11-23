#!/usr/bin/env python3
"""Closed-loop simulation tests of MasterTeleImpedance + SlaveTeleImpedance with the energy tank.

python3 run_tank_tests.py [<work dir>]

Builds master_ti against the VIRTUOSE stub and slave_ti (tank on, and tank off) against the libfranka stub
(robot_stub: the end effector as a 3 kg body with identity Jacobian; FRANKA_STUB_HAND = environment spring around
the start pose with 20 Ns/m), runs the scenarios below with the real UDP links (master <-> slave on 11055/11056,
stiffness commands on 4500 at 5 Hz as MOSAIC sends them, diagnostics on 11061) and checks:

  free_steps    no contact, master still: commands 300 <-> 700 N/m (20 <-> 40 Nm/rad) reach the slave in real
                time, with the slave's rate limits; when the commands stop, fallback to 500 N/m, 30 Nm/rad
                0.5 s after the last packet.
  contact_tank  master pushes the slave 3 cm into a 3000 N/m environment, operator alternates relaxed / stiff
                every 2.5 s: each increase at a large error costs tank energy; the tank never drops below its
                minimum (passive) and holds increases it cannot pay for.
  contact_off   the same without the tank.
  small_tank    tank of 0.05 J (maximum 0.1 J), master pushes 4 cm into a 10000 N/m wall: a step 300 -> 700 N/m
                needs about 0.3 J, so the tank holds it (passive); without the tank (small_off) the integrated tank
                goes below zero: the energy the stiffness change creates, which the tank prevents.
  deep          master pushes 7 cm into the 10000 N/m wall, step 300 -> 700 N/m: the spring force rises to the
                limit (kMaxSpringForceN, 30 N, below the 35 N collision threshold) and stays there for the whole
                contact.
  moving        master moves 4 cm / 15 deg at 0.5 Hz while the commands jump between 300 and 700: stable, tank
                stays above its minimum.
Energy check in every scenario with a still master: storage (spring + tank) grows by at most the energy that
entered through the robot.

The passivity layer at the master (feedback tank and buzz-band damping) is tested with a simulated operator's hand
(VIRTUOSE stub, TI_STUB_HAND: a light grip of 0.5 kg, 8 Ns/m, 200 N/m around the intended pose; the hand presses 3 cm
towards a wall of 10000 N/m and 30 Nm/rad, holds, releases), slave at 700 N/m:
  hand_press    no delay, master with and without the layers: the same steady hold force, not dropped, the layers
                stay idle.
  delay_loop    the slave's external-wrench estimate reported 12 ms late (FRANKA_STUB_FORCE_DELAY_MS): without the
                layers the loop oscillates and the feedback gives the hand far more energy than it receives; with
                them the oscillation stays below a tenth.
  full_tank     the same with the tank full (1 J, as after much pressing): the tank does not act, the buzz-band
                damping stops the oscillation.
  gain_08       force reflection 0.8, torque reflection up to 0.75, no delay: without the layers the loop
                oscillates; with them it does not grow.
Master-port check in every scenario with the layers: the feedback gives the hand at most the energy it received
plus the initial tank.
"""
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
WORK = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/ti_test")
FIELDS = ("t_s", "live", "cmd_kt", "cmd_kr", "target_kt", "applied_kt", "target_kg", "applied_kg", "tank",
          "injected", "harvested", "spring", "limited", "active", "port")
MASTER_FIELDS = ("t_s", "active", "tank", "delivered", "received", "dissipated", "removed", "limited", "power",
                 "force_cut", "torque_cut", "damping_t", "damping_r", "buzz_pf", "buzz_pm", "buzz_dt", "buzz_dr")
TANK_MIN = 0.01
failures = []
master_rows = {}


def check(name, ok, detail):
    print(("PASS  " if ok else "FAIL  ") + name + ": " + detail, flush=True)
    if not ok:
        failures.append(name)


def build():
    WORK.mkdir(parents=True, exist_ok=True)
    common = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", f"-I{ROOT / 'Common'}", "-I/usr/include/eigen3",
              "-pthread"]
    s = ROOT / "SlaveTeleImpedance"
    small = ["-DTI_TANK_INITIAL_J=0.05", "-DTI_TANK_MAXIMUM_J=0.1"]
    for name, flags in (("slave_on", ["-DTI_ENERGY_TANK=1"]), ("slave_off", ["-DTI_ENERGY_TANK=0"]),
                        ("slave_small_on", ["-DTI_ENERGY_TANK=1"] + small),
                        ("slave_small_off", ["-DTI_ENERGY_TANK=0"] + small)):
        subprocess.run(common + flags + [str(s / "slave_ti.cpp"), str(s / "examples_common.cpp"), f"-I{s}",
                                 f"-I{HERE / 'robot_stub'}", "-o", str(WORK / name)], check=True)
    m = ROOT / "MasterTeleImpedance"
    off = ["-DTI_MASTER_TANK=0", "-DTI_BUZZ_DAMPER=0"]
    g08 = ["-DTI_FORCE_REFLECTION_GAIN=0.8", "-DTI_TORQUE_REFLECTION_GAIN_MAX=0.75"]
    for name, flags in (("master", []), ("master_layers_off", off), ("master_full_tank", ["-DTI_MASTER_TANK_INITIAL_J=1.0"]),
                        ("master_g08", g08), ("master_g08_off", g08 + off)):
        subprocess.run(common + flags + [str(m / "master_ti.cpp"), str(HERE / "virtuose_stub" / "virtuose_stub.cpp"),
                                         f"-I{m}", "-o", str(WORK / name)], check=True)


def quat_z(angle):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def run(name, slave, env, pose_of_t, command_of_t, seconds, master="master", extra_env=None):
    """Runs one scenario; returns the slave's diagnostic rows (list of dicts). The master tank's rows go to
    master_rows[name]."""
    pose_file, env_file = WORK / f"{name}_pose.txt", WORK / f"{name}_env.txt"
    env_file.write_text(" ".join(str(v) for v in env) + "\n" if env else "")
    pose_file.write_text("0 0 0 0 0 0 1\n")
    rows, mrows, stop = [], [], threading.Event()
    t0 = time.monotonic()

    def logger(port, fields, out):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", port))
        sock.settimeout(0.2)
        while not stop.is_set():
            try:
                data = sock.recv(1024)
            except socket.timeout:
                continue
            if len(data) == 8 * len(fields):
                row = dict(zip(fields, struct.unpack(f"<{len(fields)}d", data)))
                row["t_s"] = time.monotonic() - t0  # runner clock, the time base of the commands and poses
                out.append(row)
        sock.close()

    log_threads = [threading.Thread(target=logger, args=(11061, FIELDS, rows)),
                   threading.Thread(target=logger, args=(11062, MASTER_FIELDS, mrows))]
    for thread in log_threads:
        thread.start()
    env_vars = dict(os.environ, FRANKA_STUB_HAND=str(env_file), TI_STUB_POSE=str(pose_file),
                    TI_STUB_FORCE_LOG=str(WORK / f"{name}_master_force.txt"),
                    FRANKA_STUB_FORCE_LOG=str(WORK / f"{name}_slave_force.txt"),
                    FRANKA_STUB_MOMENT_COUPLING="0.405 0.071 0.205")  # as the real estimate (master compensates it)
    (WORK / f"{name}_master_force.txt").unlink(missing_ok=True)
    (WORK / f"{name}_slave_force.txt").unlink(missing_ok=True)
    env_vars.update(extra_env or {})
    slave_p = subprocess.Popen([str(WORK / slave), "0.0.0.0"], stdin=subprocess.PIPE, stdout=open(WORK / f"{name}_slave.txt", "w"),
                               stderr=subprocess.STDOUT, env=env_vars, text=True)
    slave_p.stdin.write("\n")
    slave_p.stdin.flush()
    master_p = subprocess.Popen([str(WORK / master)], stdout=open(WORK / f"{name}_master.txt", "w"),
                                stderr=subprocess.STDOUT, env=env_vars)
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_pose = last_cmd = -1.0
    while (t := time.monotonic() - t0) < seconds:
        cmd = command_of_t(t)
        if cmd is not None and t - last_cmd >= 0.2:  # 5 Hz, the MOSAIC output rate
            last_cmd = t
            out.sendto(struct.pack("<2d", *cmd), ("127.0.0.1", 4500))
        if t - last_pose >= 0.02:
            last_pose = t
            p, q = pose_of_t(t)
            tmp = pose_file.with_suffix(".tmp")
            tmp.write_text(" ".join(f"{v:.6f}" for v in (*p, *q)) + "\n")
            tmp.replace(pose_file)
        time.sleep(0.005)  # 200 Hz, as fast as the MOSAIC output or faster
    master_p.send_signal(2)
    slave_p.send_signal(2)
    for p in (master_p, slave_p):
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    stop.set()
    for thread in log_threads:
        thread.join()
    for path, fields, out in ((WORK / f"{name}_log.csv", FIELDS, rows), (WORK / f"{name}_master_tank.csv", MASTER_FIELDS, mrows)):
        with open(path, "w") as f:
            f.write(",".join(fields) + "\n")
            for r in out:
                f.write(",".join(f"{r[k]:.6g}" for k in fields) + "\n")
    master_rows[name] = mrows
    return rows


MASTER_TANK_INITIAL = 0.05
LIGHT_GRIP = "0.5 8 200 0.005 0.1 2"  # kg, Ns/m, N/m, kg m^2, Nms/rad, Nm/rad


def force_norms(path, t0, t1):
    """|F| (N) of a force log ("t fx fy fz ...") between t0 and t1."""
    out = []
    for line in path.read_text().splitlines():
        v = list(map(float, line.split()))
        if len(v) >= 4 and t0 <= v[0] < t1:
            out.append(math.sqrt(v[1] ** 2 + v[2] ** 2 + v[3] ** 2))
    return out


def hand_run(name, master, delay_ms):
    """Hand presses 3 cm towards the wall at 4-5 s, holds to 11 s, releases at 11-12 s. Returns (VIRTUOSE |F| and
    slave |F| over the hold 7-11 s, master tank rows while active)."""
    wall = [10000, 0, 0, 0, 10000, 0, 0, 0, 10000, 30, 0, 0, 0, 30, 0, 0, 0, 30]
    def press(t):
        a = min(max(t - 4.0, 0.0), 1.0) * (1.0 - min(max(t - 11.0, 0.0), 1.0))
        return (0.03 * a, 0.0, 0.0), quat_z(0.0)
    run(name, "slave_on", wall, press, lambda t: (700.0, 40.0), 14.0, master=master,
        extra_env={"TI_STUB_HAND": LIGHT_GRIP, "FRANKA_STUB_FORCE_DELAY_MS": str(delay_ms)})
    return (force_norms(WORK / f"{name}_master_force.txt", 7.0, 11.0),
            force_norms(WORK / f"{name}_slave_force.txt", 7.0, 11.0),
            [r for r in master_rows[name] if r["active"] > 0])


def swing(values):
    return max(values) - min(values) if values else float("nan")


def master_port_check(name, mrows, initial=MASTER_TANK_INITIAL):
    last = mrows[-1]
    excess = last["delivered"] - last["received"] - initial
    check(f"{name}: the feedback gives the hand at most what it received plus the initial tank (master port passive)",
          excess < 5e-3, f"gave {last['delivered']:.3f} J, received {last['received']:.3f} J, initial {initial:.2f} J, "
          f"lowest tank {min(r['tank'] for r in mrows):+.4f} J")


def window(rows, t0, t1):
    return [r for r in rows if t0 <= r["t_s"] < t1]


def energy_check(name, rows, after):
    rows = [r for r in rows if r["t_s"] >= after]
    h0 = rows[0]["spring"] + rows[0]["tank"]
    p0 = rows[0]["port"]
    excess = max((r["spring"] + r["tank"] - h0) - (r["port"] - p0) for r in rows)
    check(f"{name}: storage (spring + tank) grows at most by the energy entering through the robot",
          excess < 2e-3, f"largest excess {excess * 1000:.2f} mJ (tolerance 2 mJ, discretisation)")


def main():
    build()
    still = lambda t: ((0.0, 0.0, 0.0), quat_z(0.0))

    # --- free_steps
    def cmd_free(t):
        if t < 4: return 300.0, 20.0
        if t < 10: return 700.0, 40.0
        if t < 16: return 300.0, 20.0
        if t < 20: return None  # MOSAIC stopped
        return 600.0, 35.0
    rows = run("free_steps", "slave_on", None, still, cmd_free, 24.0)
    w = window(rows, 9.0, 10.0)
    check("free_steps: commands reach the slave (700 N/m, 40 Nm/rad = gain 80)",
          bool(w) and all(r["live"] == 1 for r in w) and abs(w[-1]["applied_kt"] - 700) < 1 and abs(w[-1]["applied_kg"] - 80) < 0.5,
          f"applied {w[-1]['applied_kt']:.1f} N/m, gain {w[-1]['applied_kg']:.1f}" if w else "no rows")
    ramp = window(rows, 4.0, 6.5)
    rates = [(b["applied_kt"] - a["applied_kt"]) / (b["t_s"] - a["t_s"]) for a, b in zip(ramp, ramp[1:]) if b["t_s"] > a["t_s"]]
    check("free_steps: translational change limited to 200 N/m/s (300 -> 700 in about 2 s)",
          bool(rates) and max(rates) < 200 * 1.1 and max(rates) > 150,
          f"largest rate {max(rates):.0f} N/m/s" if rates else "no rows")
    w = window(rows, 15.0, 16.0)
    check("free_steps: back to 300 N/m and 20 Nm/rad", bool(w) and abs(w[-1]["applied_kt"] - 300) < 1 and abs(w[-1]["applied_kg"] - 40) < 0.5,
          f"applied {w[-1]['applied_kt']:.1f} N/m, gain {w[-1]['applied_kg']:.1f}" if w else "no rows")
    w_live = window(rows, 4.0, 16.0)
    check("free_steps: input stays live between the 5 Hz packets",
          bool(w_live) and all(r["live"] == 1 for r in w_live),
          f"{sum(r['live'] for r in w_live):.0f} of {len(w_live)} samples live")
    w = window(rows, 16.0, 17.0)
    stale_at = next((r["t_s"] for r in w if r["live"] == 0), None)
    check("free_steps: commands stop -> fallback within 0.7 s (timeout 0.5 s after the last packet)",
          stale_at is not None and stale_at - 16.0 < 0.7,
          f"fallback flagged at +{(stale_at - 16.0) * 1000:.0f} ms" if stale_at else "never flagged")
    w = window(rows, 19.0, 20.0)
    check("free_steps: fallback 500 N/m, 30 Nm/rad (gain 60)", bool(w) and abs(w[-1]["applied_kt"] - 500) < 1 and abs(w[-1]["applied_kg"] - 60) < 0.5,
          f"applied {w[-1]['applied_kt']:.1f} N/m, gain {w[-1]['applied_kg']:.1f}" if w else "no rows")
    w = window(rows, 23.0, 24.0)
    check("free_steps: commands resume -> 600 N/m, 35 Nm/rad", bool(w) and w[-1]["live"] == 1 and abs(w[-1]["applied_kt"] - 600) < 1,
          f"applied {w[-1]['applied_kt']:.1f} N/m, gain {w[-1]['applied_kg']:.1f}" if w else "no rows")
    check("free_steps: tank stays above its minimum", min(r["tank"] for r in rows) > TANK_MIN,
          f"lowest {min(r['tank'] for r in rows):.4f} J")
    energy_check("free_steps", rows, 3.0)

    # --- contact: master pushes 3 cm into a 3000 N/m environment, operator alternates
    env = [3000, 0, 0, 0, 3000, 0, 0, 0, 3000, 5, 0, 0, 0, 5, 0, 0, 0, 5]
    def push(t):
        a = min(max((t - 2.0) / 1.0, 0.0), 1.0)
        return (0.03 * a, 0.0, 0.0), quat_z(math.radians(10) * a)
    def cmd_alt(t):
        if t < 4: return 300.0, 20.0
        return (700.0, 40.0) if int((t - 4) / 2.5) % 2 == 0 else (300.0, 20.0)
    results = {}
    for name, slave in (("contact_tank", "slave_on"), ("contact_off", "slave_off")):
        rows = run(name, slave, env, push, cmd_alt, 24.0)
        results[name] = rows
        energy_check(name, [r for r in rows if r["t_s"] >= 4.0], 4.0)
    on, off = results["contact_tank"], results["contact_off"]
    check("contact_tank: tank never below its minimum (passive)", min(r["tank"] for r in on) >= TANK_MIN - 1e-6,
          f"lowest {min(r['tank'] for r in on):.4f} J (minimum {TANK_MIN} J), increases held in "
          f"{100 * sum(r['limited'] for r in on) / len(on):.0f} % of the samples")
    check("contact: net energy the stiffness changes injected (tank / no tank)", True,
          f"{on[-1]['injected']:.3f} J / {off[-1]['injected']:.3f} J; lowest integrated tank without it "
          f"{min(r['tank'] for r in off):.3f} J")

    # --- small tank against a stiff wall
    wall = [10000, 0, 0, 0, 10000, 0, 0, 0, 10000, 5, 0, 0, 0, 5, 0, 0, 0, 5]
    def push4(t):
        a = min(max((t - 2.0) / 1.0, 0.0), 1.0)
        return (0.04 * a, 0.0, 0.0), quat_z(0.0)
    def cmd_step(t):
        return (300.0, 20.0) if t < 5 else (700.0, 40.0)
    small_on = run("small_tank", "slave_small_on", wall, push4, cmd_step, 12.0)
    small_off = run("small_off", "slave_small_off", wall, push4, cmd_step, 12.0)
    for name, rows in (("small_tank", small_on), ("small_off", small_off)):
        energy_check(name, [r for r in rows if r["t_s"] >= 4.0], 4.0)
    w = window(small_on, 10.0, 12.0)
    check("small_tank: the tank holds the increase it cannot pay for, stays at its minimum (passive)",
          min(r["tank"] for r in small_on) >= TANK_MIN - 1e-6 and w and w[-1]["applied_kt"] < 650 and any(r["limited"] for r in small_on),
          f"lowest tank {min(r['tank'] for r in small_on):.4f} J, stiffness held at {w[-1]['applied_kt']:.0f} N/m (target 700)" if w else "no rows")
    w = window(small_off, 10.0, 12.0)
    check("small_off: without the tank the step goes through and creates energy (integrated tank below zero)",
          min(r["tank"] for r in small_off) < 0.0 and w and w[-1]["applied_kt"] > 695,
          f"lowest integrated tank {min(r['tank'] for r in small_off):.3f} J, stiffness {w[-1]['applied_kt']:.0f} N/m" if w else "no rows")
    check("contact_tank: the tank lets the stiffness rise while it has energy",
          max(r["applied_kt"] for r in window(on, 4.0, 7.0)) > 650,
          f"first stiff phase reaches {max(r['applied_kt'] for r in window(on, 4.0, 7.0)):.0f} N/m")

    # --- deep contact: the spring force stays at its limit
    def push7(t):
        a = min(max(t - 2.0, 0.0), 1.0) * (1.0 - min(max(t - 14.0, 0.0), 1.0))
        return (0.07 * a, 0.0, 0.0), quat_z(0.0)
    rows = run("deep", "slave_on", wall, push7, cmd_step, 16.0)
    energy_check("deep", [r for r in rows if r["t_s"] >= 4.0], 4.0)
    force = [list(map(float, l.split())) for l in (WORK / "deep_slave_force.txt").read_text().splitlines() if l.strip()]
    force = [(f[0], math.sqrt(f[1] ** 2 + f[2] ** 2 + f[3] ** 2)) for f in force]
    held = [f for t, f in force if 7.0 <= t < 13.5]
    peak = max(f for _, f in force) if force else float("nan")
    check("deep: 7 cm into the wall at 300 -> 700 N/m, the slave force rises to 30 N and stays there in contact",
          bool(held) and peak < 30.5 and min(held) > 29.5,
          f"largest {peak:.1f} N, 7-13.5 s {min(held):.1f}-{max(held):.1f} N" if held else "no force log")

    # --- moving master
    def wave(t):
        a = min(max((t - 2.0) / 2.0, 0.0), 1.0)
        s = math.sin(2 * math.pi * 0.5 * t)
        return (0.04 * a * s, 0.0, 0.0), quat_z(math.radians(15) * a * s)
    def cmd_jump(t):
        return (700.0, 40.0) if int(t / 1.5) % 2 else (300.0, 20.0)
    rows = run("moving", "slave_on", None, wave, cmd_jump, 20.0)
    check("moving: tank stays above its minimum, stiffness follows the commands",
          min(r["tank"] for r in rows) >= TANK_MIN - 1e-6 and max(r["applied_kt"] for r in rows) > 600 and min(r["applied_kt"] for r in window(rows, 5, 20)) < 400,
          f"tank {min(r['tank'] for r in rows):.3f}-{max(r['tank'] for r in rows):.3f} J, applied "
          f"{min(r['applied_kt'] for r in window(rows, 5, 20)):.0f}-{max(r['applied_kt'] for r in rows):.0f} N/m")
    force = [list(map(float, l.split())) for l in (WORK / "moving_master_force.txt").read_text().splitlines() if l.strip()]
    fmax = max(math.sqrt(f[1] ** 2 + f[2] ** 2 + f[3] ** 2) for f in force) if force else float("nan")
    check("moving: master renders bounded forces (no instability)", force and fmax < 20.0, f"largest master force {fmax:.2f} N")
    master_port_check("moving", [r for r in master_rows["moving"] if r["active"] > 0])

    # --- passivity layer at the master, with a simulated hand
    f_on, s_on, m_on = hand_run("hand_press", "master", 0)
    f_off, s_off, m_off = hand_run("hand_press_off", "master_layers_off", 0)
    check("hand_press: no delay, the layers do not change a stable contact (same steady hold force, not dropped)",
          swing(f_on) < 0.1 and swing(f_off) < 0.1 and abs(sum(f_on) / len(f_on) - sum(f_off) / len(f_off)) < 0.05,
          f"VIRTUOSE hold force {sum(f_on) / len(f_on):.2f} N (swing {swing(f_on):.2f}) with, "
          f"{sum(f_off) / len(f_off):.2f} N (swing {swing(f_off):.2f}) without the layers")
    check("hand_press: the layers stay idle in a stable contact",
          max(r["buzz_dt"] for r in m_on) < 0.5 and max(r["damping_t"] for r in m_on) < 0.5 and not any(r["limited"] for r in m_on),
          f"buzz damping max {max(r['buzz_dt'] for r in m_on):.2f} Ns/m, tank damping max {max(r['damping_t'] for r in m_on):.2f} Ns/m")
    master_port_check("hand_press", m_on)

    f_off, s_off, m_off = hand_run("delay_loop_off", "master_layers_off", 12)
    f_on, s_on, m_on = hand_run("delay_loop", "master", 12)
    check("delay_loop_off: estimate 12 ms late, without the layers the loop oscillates and the feedback creates energy",
          swing(s_off) > 10.0 and m_off[-1]["delivered"] > m_off[-1]["received"] + 1.0,
          f"slave force swing {swing(s_off):.1f} N, feedback gave {m_off[-1]['delivered']:.2f} J, received {m_off[-1]['received']:.2f} J")
    check("delay_loop: with the layers the oscillation stays below a tenth of that",
          swing(f_on) < 1.0 and swing(s_on) < 0.1 * swing(s_off),
          f"VIRTUOSE force swing {swing(f_on):.2f} N, slave force swing {swing(s_on):.2f} N (without: {swing(s_off):.1f} N)")
    master_port_check("delay_loop", m_on)

    f_full, s_full, m_full = hand_run("full_tank", "master_full_tank", 12)
    check("full_tank: tank full (1 J), it does not act; the buzz-band damping stops the oscillation",
          swing(s_full) < 0.1 * swing(s_off) and min(r["tank"] for r in m_full) > 0.9 and max(r["buzz_dt"] for r in m_full) > 1.0,
          f"slave force swing {swing(s_full):.2f} N, lowest tank {min(r['tank'] for r in m_full):.3f} J, "
          f"buzz damping max {max(r['buzz_dt'] for r in m_full):.1f} Ns/m")
    master_port_check("full_tank", m_full, initial=1.0)

    f_off, s_off, m_off = hand_run("gain_08_off", "master_g08_off", 0)
    f_on, s_on, m_on = hand_run("gain_08", "master_g08", 0)
    check("gain_08_off: reflection 0.8 / 0.75 without the layers, the loop oscillates (light grip)",
          swing(s_off) > 10.0, f"slave force swing {swing(s_off):.1f} N, feedback gave {m_off[-1]['delivered']:.2f} J, "
          f"received {m_off[-1]['received']:.2f} J")
    check("gain_08: with the layers it does not grow (a residual vibration remains)",
          swing(s_on) < 0.2 * swing(s_off), f"VIRTUOSE force swing {swing(f_on):.2f} N, slave force swing {swing(s_on):.2f} N, "
          f"hold force {sum(f_on) / len(f_on):.2f} N, buzz damping max {max(r['buzz_dt'] for r in m_on):.1f} Ns/m")
    master_port_check("gain_08", m_on)
    print("ALL CHECKS PASSED" if not failures else f"{len(failures)} CHECK(S) FAILED")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
