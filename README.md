# AIROB Bilateral Tele-Impedance Control

Controller code of my Master's thesis on bilateral 6-DoF teleoperation and tele-impedance control. A Haption VIRTUOSE 6D TAO High Force haptic device is the master and a Franka Research 3 (FR3) arm is the slave. The operator moves the robot through the VIRTUOSE, feels the contact forces and torques measured at the robot, and changes the robot's Cartesian stiffness online.

`main` holds the controller as validated on the hardware. The sensor side of the thesis (measuring the operator's arm stiffness with a wearable elastography device) is in [fr3-hand-stiffness-perturbation](https://github.com/sadeghshoushtari/fr3-hand-stiffness-perturbation).

## Research objective

The project investigates bilateral teleoperation in which:

- the operator commands the remote robot's Cartesian pose through the VIRTUOSE,
- interaction forces and moments measured/estimated at the FR3 are rendered back to the operator,
- Cartesian impedance can be varied online,
- translational and rotational impedance are treated separately, and
- the manual stiffness input is later replaced by an estimate of the operator's arm stiffness from a wearable sensor.

## Hardware

| Role | Hardware |
|---|---|
| Master | Haption VIRTUOSE 6D TAO High Force |
| Slave | Franka Research 3 |
| Communication | UDP, nominal 1 kHz |
| Current network configuration | Local loopback (`127.0.0.1`) |
| Stiffness input | VIRTUOSE analogue finger trigger |
| Diagnostics | Passive Python UDP logger |

The controller currently runs the master and slave processes on the same workstation. The UDP addresses are centralized in `Common/teleop_common.hpp`, so moving the processes to separate hosts only requires changing the configured addresses rather than modifying the control code.

## Control architecture

```text
                         FORWARD CHANNEL

  Haption VIRTUOSE                                  Franka Research 3
  pose / orientation  --------------------------->  Cartesian pose reference
  analogue trigger   ---------------------------->  variable Cartesian impedance

                         BACKWARD CHANNEL

  rendered force/torque <--------------------------  estimated external wrench
  relative-pose coupling <-------------------------  measured FR3 pose
```

The controller runs at a nominal **1 kHz**.

### Forward path

The VIRTUOSE Cartesian position and orientation are sent to the FR3 as the desired Cartesian pose. The slave uses a Cartesian impedance controller of the form

```text
tau_task = J^T (-K e - D v)
```

with nullspace control and Coriolis compensation added at joint-torque level.

The present translational slave stiffness range is:

- minimum: **300 N/m**
- baseline: **500 N/m**
- maximum: **700 N/m**

The current rotational range is:

- minimum: **40 Nm/rad**
- baseline: **80 Nm/rad**
- maximum: **80 Nm/rad**

The slave applies rate limits to both translational and rotational stiffness commands.

### Backward path

The FR3 external wrench estimate is sent to the master and rendered through the VIRTUOSE. The backward path contains:

- direct force reflection,
- contact-aware torque reflection,
- translational relative-pose spring-damper coupling,
- rotational relative-pose spring-damper coupling,
- directional force damping,
- directional torque damping,
- force/torque filtering and slew limiting,
- stale-feedback gating,
- startup wrench bias removal, and
- empirical force-to-moment cross-coupling compensation.

The current reflection gains are:

- force reflection gain: **0.60**
- torque reflection gain: **0.05 to 0.50**, increased smoothly during contact.

VIRTUOSE output is limited to:

- **30 N** force
- **1.5 Nm** torque.

## Rotation cross-coupling compensation

A locked-orientation translation experiment showed strong force-to-moment coupling in the FR3 wrench estimate. The current master controller therefore applies an empirical compensation

```text
M_corrected = M_measured - r_fit x F
```

with

```text
r_fit = [0.405, 0.071, 0.205] m
```

expressed in the Franka O frame.

The vector is fitted from the data, not taken from the tool geometry. With the compensation, the spurious rotational feedback became much smaller; it is part of the validated controller on `main`.

## Repository structure

```text
airob-teleoperation/
├── Common/
│   ├── teleop_common.hpp       # UDP addresses, ports, packet sizes, frame map
│   ├── teleop_math.hpp         # shared pose / rotation mathematics
│   └── teleop_runtime.hpp      # shared runtime helpers
│
├── MasterTeleImpedance/
│   ├── master_ti.cpp           # VIRTUOSE master controller
│   ├── master_config.hpp       # master gains, thresholds and limits
│   ├── feedback_tank.hpp       # energy tank of the force feedback
│   ├── buzz_damper.hpp         # damping in the 5-45 Hz band
│   ├── master_tank_logger.py   # logger of the master tank
│   └── virtuoseAPI.h           # Haption API header
│
├── SlaveTeleImpedance/
│   ├── slave_ti.cpp            # FR3 Cartesian impedance controller, stiffness from MOSAIC
│   ├── slave_config.hpp        # slave gains, limits and startup pose
│   ├── energy_tank.hpp         # energy tank of the variable stiffness
│   ├── tank_logger.py          # logger of the slave tank
│   ├── examples_common.cpp
│   └── examples_common.h
│
├── TeleImpedanceTest/          # simulation stubs, tank tests, stiffness_sender.py
├── TELE_IMPEDANCE.md           # tele-impedance design and tests
├── teleop_logger.py            # passive ~100 Hz diagnostic logger
├── thesis_data/                # selected experimental data
└── .gitignore
```

The Haption runtime library `virtuoseAPI.so` is intentionally excluded from version control and must be available locally in `MasterTeleImpedance/`. The baseline controller (`Master/`, `Slave/`) is on branch `main`.

## Dependencies

The code is written for Linux and currently requires:

- C++17 compiler (tested with `g++`)
- Eigen3
- libfranka
- Haption VIRTUOSE SDK / runtime library
- Python 3.8+ for the diagnostic logger

The logger itself uses only the Python standard library.

## Build

From the repository root:

### Master

```bash
cd MasterTeleImpedance

g++ -std=c++17 -O2 -Wall -Wextra \
    master_ti.cpp \
    -I. -I../Common -I/usr/include/eigen3 \
    ./virtuoseAPI.so \
    -Wl,-rpath,"$PWD" \
    -o master_ti \
    -pthread
```

### Slave

```bash
cd SlaveTeleImpedance

g++ -std=c++17 -O2 -Wall -Wextra \
    slave_ti.cpp examples_common.cpp \
    -I. -I../Common -I/usr/include/eigen3 \
    -o slave_ti \
    -lfranka \
    -pthread
```

## Run

The exact hardware startup procedure depends on the local FR3 and VIRTUOSE installation. Once both devices are ready for external control:

### 1. Start the FR3 slave

```bash
cd SlaveTeleImpedance
./slave_ti <FRANKA_IP>
```

### 2. Start the VIRTUOSE master

```bash
cd MasterTeleImpedance
./master_ti
```

The master connects to the local VIRTUOSE service using:

```text
127.0.0.1#53210
```

### 3. Optional: start the passive logger

Run from the repository root in a separate terminal:

```bash
sudo python3 teleop_logger.py --label rotation_test
```

Optional fixed-duration recording:

```bash
sudo python3 teleop_logger.py --label rotation_test --duration 60
```

The logger observes copies of the existing loopback UDP packets and does **not** transmit controller commands.

## UDP interface

The shared packet definitions are in `Common/teleop_common.hpp`.

Current ports:

| Direction | UDP port |
|---|---:|
| Master -> Slave command | 11056 |
| Slave -> Master feedback | 11055 |

### Master -> Slave command packet

14 doubles:

```text
[0]      packet id
[1]      source timestamp
[2..5]   master quaternion
[6..8]   master Cartesian position
[9]      force-reflection gain metadata
[10]     torque-reflection gain metadata
[11]     requested translational stiffness
[12]     deadman state
[13]     emergency-stop state
```

### Slave -> Master feedback packet

23 doubles:

```text
[0]      packet id
[1]      source timestamp
[2..7]   external wrench: Fx Fy Fz Mx My Mz
[8..10]  FR3 Cartesian position
[11..14] FR3 quaternion
[15..17] FR3 linear velocity
[18..20] FR3 angular velocity
[21]     applied translational stiffness
[22]     applied rotational stiffness
```

## Safety and robustness mechanisms

This is experimental research software, not production or safety-certified control software.

Current safeguards include:

- VIRTUOSE deadman and emergency-stop gating,
- 50 ms communication-staleness checks,
- VIRTUOSE force/torque saturation,
- force/torque command slew limits,
- FR3 joint-velocity soft guard,
- FR3 joint-torque rate saturation,
- bounded Cartesian workspace increments,
- bounded relative orientation command,
- nullspace posture control,
- explicit startup wrench-bias settling period, and
- conservative stiffness rate limits.

Hardware experiments should only be performed by trained users with the physical emergency-stop mechanisms available.

## Diagnostic logging

`teleop_logger.py` provides a passive approximately **100 Hz** view of the 1 kHz UDP traffic.

Each run produces CSV/JSON metadata containing command and feedback snapshots, timing diagnostics, wrench statistics, stiffness values and repository/configuration information when available.

Important limitation: the logger observes the network interface, not every internal 1 kHz controller state. It is appropriate for experimental diagnostics and trend analysis, but it must not be treated as a full-rate passivity or energy observer.

## Branches

| Branch | Purpose |
|---|---|
| **`main`** | Current validated working controller; recommended branch for review |
| `development` | Stable bilateral rotational-feedback checkpoint before saturation-hold validation |
| `contact-saturation-hold` | Source branch for the validated contact saturation-hold implementation and final validation dataset |
| `baseline` | Earlier monolithic baseline retained for reference |

The controller state promoted to `main` originates from:

```text
748489769847bb9f19f0181b0f3873088aefdea1
Add final bilateral validation dataset
```

The final validation data are stored under:

```text
thesis_data/final_bilateral_validation/
```

The validated saturation-hold logic freezes the force/torque direction at the instant the relevant VIRTUOSE ceiling is first reached during contact and releases the hold when contact ends. This logic is applied only at the final rendered VIRTUOSE output; the underlying bilateral controller remains unchanged.

## Current research status

### Implemented

- 6-DoF master-to-slave Cartesian pose following
- bilateral force and torque feedback
- online variable translational and rotational impedance
- contact-aware torque-reflection gain
- directional translational and rotational damping
- force-to-moment cross-coupling compensation
- startup bias handling
- stale-packet / deadman / emergency-stop gating
- passive diagnostic logger
- refactored Master / Slave / Common structure

### In progress

The next research stage is **sensor-driven tele-impedance**.

The intended sensing pipeline is:

```text
mechanical muscle sensing (vibration / elastography)
                    +
                 IMUs
                    |
                    v
       muscle / joint stiffness estimate
                    |
                    v
      musculoskeletal projection model
                    |
                    v
      Cartesian endpoint stiffness
                    |
                    v
       robot impedance command
```

The sensor code is developed and tested separately ([fr3-hand-stiffness-perturbation](https://github.com/sadeghshoushtari/fr3-hand-stiffness-perturbation)) before it commands the robot's stiffness. Until then the analogue trigger stays the stiffness input and remains the fallback.

A later stage will investigate formal passivity supervision for time-varying impedance.

## Development approach

The controller was developed in small steps:

1. preserve a known working bilateral baseline,
2. change one mechanism at a time,
3. log the resulting behavior,
4. validate free-space behavior before contact,
5. keep translational and rotational effects distinguishable, and
6. move experimental additions to separate branches until validated.

## License / reuse

This repository contains thesis research code and vendor-dependent interfaces. No general-purpose reuse or safety guarantee is implied. Check the licensing conditions of libfranka and the Haption SDK before redistributing vendor-related components.
