# Tele-impedance from the operator's arm (branch `tele-impedance-sensors`)

This branch replaces the VIRTUOSE trigger with the stiffness of the operator's arm, measured by the wearable
elastography device and computed by the MOSAIC run pipeline of
[fr3-hand-stiffness-perturbation](https://github.com/sadeghshoushtari/fr3-hand-stiffness-perturbation)
(`mosaic/session_run.json`). `Master/` and `Slave/` are unchanged; the tele-impedance versions are separate copies.

```text
  board (biceps, triceps) -> MOSAIC run pipeline -> UDP 4500: Kt (N/m), Kr (Nm/rad) -> SlaveTeleImpedance
                                                                                         | energy tank
                                                                     feedback[21] = applied Kt
                                                                                         v
                                                       MasterTeleImpedance: backward coupling follows Kt
```

## SlaveTeleImpedance

`slave_ti.cpp` is `Slave/slavev.cpp` with four changes:

1. **Input.** Two doubles on UDP port 4500 from MOSAIC (block `ToSlave`, 5 Hz): translational stiffness (N/m) and
   rotational stiffness (Nm/rad, effective). The rotational gain of the slave's quaternion law is twice the
   effective stiffness (for small angles the law acts with half its gain), so 20-40 Nm/rad = gain 40-80. The
   translation and the rotation are set separately (the trigger version derived the rotation from the translation).
   Both are clamped to the slave's ranges (300-700 N/m, gain 40-80) and pass the existing rate limits
   (200 N/m/s, 20 gain/s).
2. **Fallback.** Without a valid command for 0.5 s (2.5 MOSAIC periods: MOSAIC stopped, sensors off), the slave
   goes to 500 N/m and gain 60 (30 Nm/rad), through the same rate limits and the tank.
3. **Energy tank** (`energy_tank.hpp`, Ferraguti, Secchi and Fantuzzi, ICRA 2013). The impedance stores
   V = 1/2 Kt |e|^2 + 2 Kg (1 - w). Raising a stiffness while the robot is away from its reference adds
   1/2 dKt |e|^2 + 2 dKg (1 - w) without energy entering through the robot: the only way a variable stiffness makes
   the controller non-passive. The damping of the impedance fills the tank with the energy it dissipates; each
   stiffness change takes its energy from the tank (a decrease returns it); an increase the tank cannot pay for is
   scaled down so that the tank stops at its minimum. The storage V + T then grows at most by the energy that enters
   through the robot. Tank: 0.5 J at start, 0.01-1.0 J. One step 300 -> 700 N/m at the largest error the 35 N
   collision threshold allows (5 cm) needs 0.5 J.

4. **Force limit.** The translational spring force is limited to `kMaxSpringForceN` (`slave_config.hpp`, 30 N, below
   the 35 N collision threshold): beyond the offset 30 N / Kt the spring keeps 30 N, so a stiffness increase deep in
   contact no longer stops the robot, and the force stays at the limit as long as the master holds the offset. The
   limited spring is the gradient of a potential (1/2 Kt |e|^2 up to the limit, then growing linearly with |e|), so it
   stays passive; the tank charges a stiffness change with the offset at the limit. The rotation is not limited
   (contact moments of about 6 Nm on 2026-11-23, threshold 35 Nm).

Switch: `kEnableEnergyTank` (build with `-DTI_ENERGY_TANK=0` to run without it; the log still integrates the tank,
so it shows the energy the stiffness changes inject).

Diagnostics: 100 Hz UDP packet to port 11061 (logging only). Record it with
`python3 SlaveTeleImpedance/tank_logger.py run.csv`: time, input live, MOSAIC commands, rate-limited targets, applied
stiffnesses, tank energy, energy injected and harvested, spring energy, tank holding an increase, teleoperation
active, energy that entered through the robot. The terminal prints the input state, the commanded and applied
stiffnesses and the tank once per second.

The energy tank is the passivity controller of the variable stiffness. The bilateral coupling itself (master and
slave on one workstation, UDP on the loopback at 1 kHz, no delay worth compensating) is the validated baseline of
`main` and is not changed; a time-domain passivity controller for the communication channel (Hannaford and Ryu
2002) would only be needed with a network delay between separate hosts.

## MasterTeleImpedance

`master_ti.cpp` is `Master/masterv.cpp` without the trigger: the backward coupling (the spring that lets the FR3 move
the VIRTUOSE) follows the translational stiffness the slave applies (feedback[21]) over the same ranges as the
trigger did: 300..700 N/m -> 5..15 N/m and 0.5..1.5 Nm/rad, with the existing rate limits. Slot 11 of the command
packet carries the slave's stiffness for the logs; SlaveTeleImpedance does not read it.

## Build (robot PC)

```bash
cd SlaveTeleImpedance
g++ -std=c++17 -O2 -Wall -Wextra slave_ti.cpp examples_common.cpp -I. -I../Common -I/usr/include/eigen3 \
    -o slave_ti -lfranka -pthread
cd ../MasterTeleImpedance
cp <baseline>/Master/virtuoseAPI.so .   # the VIRTUOSE library is not in git
g++ -std=c++17 -O2 -Wall -Wextra master_ti.cpp -I. -I../Common -I/usr/include/eigen3 ./virtuoseAPI.so \
    -Wl,-rpath,$(pwd) -o master_ti -pthread
```

## Run

1. MOSAIC with `session_run.json` (fr3-hand-stiffness-perturbation; sensors on, session reference set).
2. `./slave_ti <franka-ip>`, then `./master_ti`, as with the baseline.
3. Optional: `python3 SlaveTeleImpedance/tank_logger.py run.csv`.

Without MOSAIC the slave runs at the fallback stiffness (500 N/m, 30 Nm/rad) and prints `input FALLBACK`.

Hardware test of the stiffness input and the tank without the sensors (the trigger no longer sets the stiffness):
`python3 TeleImpedanceTest/stiffness_sender.py` sends a level typed by hand (0 = 300 N/m, 20 Nm/rad; 1 = 700 N/m,
40 Nm/rad) at 5 Hz to port 4500, as MOSAIC does; `--steps 4` alternates 0 and 1 every 4 s. When it stops, the slave
returns to the fallback 0.5 s later.

## Tests (no robot)

`python3 TeleImpedanceTest/run_tank_tests.py [work dir]` builds master_ti against a VIRTUOSE stub and slave_ti
against a libfranka stub (end effector as a 3 kg body, identity Jacobian, environment spring from a file) and runs
the real UDP links with the commands at 5 Hz:

| Scenario | Result (2026-11-23) |
|---|---|
| free space, commands 300 <-> 700 N/m, 20 <-> 40 Nm/rad | applied 700 N/m / gain 80 and back, rate 200 N/m/s, input live between packets |
| commands stop, then resume | fallback 500 N/m, gain 60, 0.4 s after the last packet; resumes at 600 N/m |
| contact 3 cm into 3000 N/m, operator alternates every 2.5 s | tank 0.47-0.5 J, never below its minimum |
| tank of 0.05 J, step 300 -> 700 N/m 4 cm into a 10000 N/m wall | tank holds the step at 353 N/m and stops at 0.01 J; without the tank the step goes through and the integrated tank goes to -0.24 J (energy created) |
| master 7 cm into a 10000 N/m wall, step 300 -> 700 N/m | slave force rises to 30.0 N and stays there for the whole contact (without the limit: 34.4 N, the tank holding the stiffness at 517 N/m) |
| master moving 4 cm / 15 deg at 0.5 Hz, commands jumping | stable, tank 0.5-1.0 J, master force below 0.1 N |
| every scenario | storage (spring + tank) grew at most by the energy entering through the robot (largest excess 0.02 mJ) |

Real data through the real pipeline (2026-11-21): the board recording of the calibration run of 2026-11-19 replayed
in real time into the MOSAIC run pipeline (with that run's model), its commands into slave_ti and master_ti in
simulation. The slave applied the commands continuously (no fallback in 258 s), always within 300-700 N/m and
20-40 Nm/rad; mean applied stiffness per trial: relaxed 318 / 301 N/m, medium 656 / 506 N/m, max 647 / 670 N/m
(rotation 20.9 / 20.0, 37.8 / 30.3, 37.3 / 38.5 Nm/rad).

Both slaves compile against the libfranka 0.13.3 headers; master_ti against the VIRTUOSE header of `Master/`.

Hardware test (2026-11-23, `stiffness_sender.py`, contact with foam): the commands reached the slave and the fallback
worked; in contact the tank paid 2.4 J of stiffness increases and held one at 596 N/m at its minimum. Two reflex
stops: a joint velocity violation during light, making-and-breaking contact, and a Cartesian reflex when the force
reached 35.5 N while the stiffness rose in deep contact (the force limit, change 4, came after this test).
