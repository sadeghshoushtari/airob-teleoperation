# Tele-impedance from the operator's arm (branch `tele-impedance-sensors`)

This branch replaces the VIRTUOSE trigger with the stiffness of the operator's arm, measured by the wearable
elastography device and computed by the MOSAIC run pipeline of
[fr3-hand-stiffness-perturbation](https://github.com/sadeghshoushtari/fr3-hand-stiffness-perturbation)
(`mosaic/session_run.json`). The baseline controller (`Master/`, `Slave/`) is on branch `main`; this branch holds only the tele-impedance versions.

```text
  board (biceps, triceps) -> MOSAIC run pipeline -> UDP 4500: Kt (N/m), Kr (Nm/rad) -> SlaveTeleImpedance
                                                                                         | energy tank
                                                                     feedback[21] = applied Kt
                                                                                         v
                                                       MasterTeleImpedance: backward coupling follows Kt,
                                                       feedback tank + buzz-band damping on the VIRTUOSE command
```

## SlaveTeleImpedance

`slave_ti.cpp` is the baseline `Slave/slavev.cpp` (branch `main`) with four changes:

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

The energy tank is the passivity controller of the variable stiffness. The force feedback has its own passivity layer
at the master (below): even with master and slave on one workstation and no network delay, the loop has lags (the
external-wrench estimate, the master's filter and slew limit, sampling) that can make the feedback give the operator
energy.

## MasterTeleImpedance

`master_ti.cpp` is the baseline `Master/masterv.cpp` (branch `main`) without the trigger: the backward coupling (the spring that lets the FR3 move
the VIRTUOSE) follows the translational stiffness the slave applies (feedback[21]) over the same ranges as the
trigger did: 300..700 N/m -> 5..15 N/m and 0.5..1.5 Nm/rad, with the existing rate limits. Slot 11 of the command
packet carries the slave's stiffness for the logs; SlaveTeleImpedance does not read it.

### Passivity layer of the force feedback

The baseline wrench (reflection, backward coupling, directional damping, filter, slew limit, saturation hold) is the
transparency layer and is unchanged. Two blocks act on the command sent to the VIRTUOSE:

1. **Feedback tank** (`feedback_tank.hpp`; the master tank of the two-layer approach, Franken, Stramigioli, Misra,
   Secchi and Macchelli, IEEE T-RO 2011). Every cycle it books the energy the command exchanged with the operator's
   hand, (W - D) . dX, with the pure damping part D of the command excluded. The feedback can give the hand at most
   the energy the hand gave it plus the initial tank (0.05 J; maximum 1 J). Below 0.02 J it adds damping on the
   VIRTUOSE's velocity (up to 30 Ns/m and 0.3 Nms/rad on an empty tank); if that is not enough, the command is
   reduced along the predicted motion so that it does no more work than the tank holds. At rest nothing is paid, so a
   steady contact force stays at its value (also when the saturation hold keeps it at the ceiling).
2. **Buzz-band damping** (`buzz_damper.hpp`). The edge-contact vibration (6-32 Hz in light contact, hardware test
   of 2026-11-23) does not empty the tank: during it the operator's push gave the feedback 26-408 mW while the
   vibration took 2.6-6.6 mW out of it (four vibration stretches of that test). The block measures the power the
   feedback gives the hand at 5-45 Hz (the same band-pass on the command and on the VIRTUOSE velocity). Above 1 mW
   (torque: 0.5 mW) it adds damping on the band-passed velocity that dissipates twice the excess (up to 20 Ns/m and
   0.3 Nms/rad), released with a time constant of 1 s. Slow pressing and releasing are below the band and feel almost
   no drag. The band-pass is positive real, so the damping is passive. In the log of that test the band power stayed
   below 1 mW whenever the band force was quiet (rms below 0.1 N: always; 0.1-0.2 N: 99 % of the time) and exceeded
   it in 55-88 % of the time during vibration with a band force rms above 0.3 N.

Switches: `-DTI_MASTER_TANK=0`, `-DTI_BUZZ_DAMPER=0` (both are still computed and logged). Parameters in
`master_config.hpp`. Diagnostics: 100 Hz UDP packet to port 11062 (logging only), recorded with
`python3 MasterTeleImpedance/master_tank_logger.py master_tank.csv`: tank energy, energy given to and received from
the hand, damping added by the tank, command reductions, buzz-band power and damping. The terminal line every 0.2 s
ends with the tank level, the time the command was reduced and the buzz damping. The 106-double diagnostics for the
teleop logger are unchanged.

## Build (robot PC)

In VS Code: File > Open Workspace from File > `thesis.code-workspace` (this repository next to
`fr3-hand-stiffness-perturbation`, `mosaic-2.0`, `mosaic_models` and `mosaic_logs` in the thesis folder);
Ctrl+Shift+B builds slave_ti, master_ti, perturbation and MOSAIC with our blocks. On the command line:

```bash
cd SlaveTeleImpedance
g++ -std=c++17 -O2 -Wall -Wextra slave_ti.cpp examples_common.cpp -I. -I../Common -I/usr/include/eigen3 \
    -o slave_ti -lfranka -pthread
cd ../MasterTeleImpedance
cp <path>/virtuoseAPI.so .   # the VIRTUOSE library is not in git
g++ -std=c++17 -O2 -Wall -Wextra master_ti.cpp -I. -I../Common -I/usr/include/eigen3 ./virtuoseAPI.so \
    -Wl,-rpath,$(pwd) -o master_ti -pthread
```

## Run

1. MOSAIC with `session_run.json` (fr3-hand-stiffness-perturbation; sensors on, session reference set).
2. `./slave_ti <franka-ip>`, then `./master_ti`, as with the baseline.
3. Optional: `python3 SlaveTeleImpedance/tank_logger.py run.csv` and
   `python3 MasterTeleImpedance/master_tank_logger.py master_tank.csv`.

Without MOSAIC the slave runs at the fallback stiffness (500 N/m, 30 Nm/rad) and prints `input FALLBACK`.

Hardware test of the stiffness input and the tank without the sensors (the trigger no longer sets the stiffness):
`python3 TeleImpedanceTest/stiffness_sender.py` sends a level typed by hand (0 = 300 N/m, 20 Nm/rad; 1 = 700 N/m,
40 Nm/rad) at 5 Hz to port 4500, as MOSAIC does; `--steps 4` alternates 0 and 1 every 4 s. When it stops, the slave
returns to the fallback 0.5 s later.

## Tests (no robot)

`python3 TeleImpedanceTest/run_tank_tests.py [work dir]` builds master_ti against a VIRTUOSE stub and slave_ti
against a libfranka stub (end effector as a 3 kg body, identity Jacobian, environment spring from a file) and runs
the real UDP links with the commands at 5 Hz. The stub's reported moment contains the force-to-moment coupling the
master compensates (`FRANKA_STUB_MOMENT_COUPLING`); for the master's passivity layer the VIRTUOSE stub simulates the
operator's hand (`TI_STUB_HAND`: a light grip of 0.5 kg, 8 Ns/m, 200 N/m around the intended pose) and the robot stub
can report its external wrench late (`FRANKA_STUB_FORCE_DELAY_MS`). It takes about 8 minutes:

| Scenario | Result (2026-11-23) |
|---|---|
| free space, commands 300 <-> 700 N/m, 20 <-> 40 Nm/rad | applied 700 N/m / gain 80 and back, rate 200 N/m/s, input live between packets |
| commands stop, then resume | fallback 500 N/m, gain 60, 0.4 s after the last packet; resumes at 600 N/m |
| contact 3 cm into 3000 N/m, operator alternates every 2.5 s | tank 0.47-0.5 J, never below its minimum |
| tank of 0.05 J, step 300 -> 700 N/m 4 cm into a 10000 N/m wall | tank holds the step at 353 N/m and stops at 0.01 J; without the tank the step goes through and the integrated tank goes to -0.24 J (energy created) |
| master 7 cm into a 10000 N/m wall, step 300 -> 700 N/m | slave force rises to 30.0 N and stays there for the whole contact (without the limit: 34.4 N, the tank holding the stiffness at 517 N/m) |
| master moving 4 cm / 15 deg at 0.5 Hz, commands jumping | stable, tank 0.5-1.0 J, master force below 0.1 N |
| every scenario | storage (spring + tank) grew at most by the energy entering through the robot (largest excess 0.02 mJ) |
| hand presses 3 cm (light grip), no delay, master with / without its passivity layer | the same steady hold force, 4.02 N, not dropped; the layer stays idle |
| slave's wrench estimate 12 ms late | without the layer the loop oscillates (slave force swing 24.7 N; the feedback gives the hand 11.4 J and receives 0.65 J); with it the swing stays at 1.3 N |
| the same with the feedback tank full (1 J) | the tank does not act (lowest 0.968 J); the buzz-band damping (up to 8.3 Ns/m) holds the swing at 1.3 N |
| force reflection 0.8, torque reflection up to 0.75, no delay | without the layer: swing 22.3 N (22.4 J given, 0.8 J received); with it: no growth, a residual swing of 0.94 N on the VIRTUOSE |
| every scenario with the master's passivity layer | the feedback gave the hand at most the energy it received plus the initial tank |

Higher reflection gains (simulation, the same press with a light grip; force / torque gain):

| Gains | Without the passivity layer | With it |
|---|---|---|
| 0.6 / 0.5 (current) | steady | steady, the same force; the layer stays idle |
| 0.8 / 0.75 | oscillates (slave force swing 22 N) | passive; residual swing 0.9-1.0 N at 8 Hz on the VIRTUOSE |
| 1.0 / 1.0 | oscillates (slave force swing 29 N) | passive; residual swing 0.9-1.6 N at 8 Hz |

With 5 ms extra delay the result is the same. With a firm grip (1 kg, 15 Ns/m, 500 N/m) all three are steady
without the layer, and the layer stays idle. The hand parameters are assumptions; on the robot the gains should be
raised in steps with `master_tank_logger.py` recording.

Real data through the real pipeline (2026-11-21): the board recording of the calibration run of 2026-11-19 replayed
in real time into the MOSAIC run pipeline (with that run's model), its commands into slave_ti and master_ti in
simulation. The slave applied the commands continuously (no fallback in 258 s), always within 300-700 N/m and
20-40 Nm/rad; mean applied stiffness per trial: relaxed 318 / 301 N/m, medium 656 / 506 N/m, max 647 / 670 N/m
(rotation 20.9 / 20.0, 37.8 / 30.3, 37.3 / 38.5 Nm/rad).

Both slaves compile against the libfranka 0.13.3 headers; master_ti against the VIRTUOSE header in `MasterTeleImpedance/`.

Hardware test (2026-11-23, `stiffness_sender.py`, contact with foam): the commands reached the slave and the fallback
worked; in contact the tank paid 2.4 J of stiffness increases and held one at 596 N/m at its minimum. Two reflex
stops: a joint velocity violation during light, making-and-breaking contact, and a Cartesian reflex when the force
reached 35.5 N while the stiffness rose in deep contact (the force limit, change 4, came after this test).
