#pragma once

#include <array>
#include <cstdint>

#include "teleop_common.hpp"

namespace {

// Communication
constexpr double kControlRateHz = 1000.0;
constexpr uint64_t kMasterStaleNs = 50ULL * 1000ULL * 1000ULL;
constexpr int kFeedbackSendPeriodUs = 1000;
constexpr int kReceiveIdleSleepUs = 100;

// Translational impedance
constexpr double kMinimumTranslationalStiffness = 300.0;   // N/m
constexpr double kBaselineTranslationalStiffness = 500.0;  // N/m
constexpr double kMaximumTranslationalStiffness = 700.0;   // N/m
constexpr double kBaselineTranslationalDamping = 20.0;     // Ns/m
constexpr double kMaximumStiffnessRate = 200.0;            // N/m/s

// Rotational impedance
constexpr double kMinimumRotationalStiffness = 40.0;       // Nm/rad
constexpr double kBaselineRotationalStiffness = 80.0;      // Nm/rad
constexpr double kMaximumRotationalStiffness = 80.0;       // Nm/rad
constexpr double kBaselineRotationalDamping = 0.5;         // Nms/rad
constexpr double kMaximumRotationalStiffnessRate = 20.0;   // Nm/rad/s

// Tele-impedance input: the stiffness commands of the operator's arm, from the MOSAIC run pipeline
// (fr3-hand-stiffness-perturbation, session_run.json, block ToSlave): 2 doubles, translational stiffness (N/m) and
// rotational stiffness (Nm/rad, effective: the quaternion law above acts with half its gain, so the gain is twice
// the command). They replace the VIRTUOSE trigger. Without a valid command for kStiffnessStaleNs (MOSAIC stopped,
// sensors off), the slave goes to the fallback stiffness, through the same rate limits and energy tank. MOSAIC sends
// at the board's update rate, 5 Hz (one packet every 200 ms), so the timeout is 2.5 packet periods.
constexpr int kStiffnessCommandPort = 4500;
constexpr uint64_t kStiffnessStaleNs = 500ULL * 1000ULL * 1000ULL;
constexpr double kFallbackTranslationalStiffness = kBaselineTranslationalStiffness;  // N/m
constexpr double kFallbackRotationalStiffness = 60.0;  // gain, middle of the range (30 Nm/rad effective)

// Energy tank (energy_tank.hpp): keeps the stiffness changes passive. The impedance's damping fills it; a
// stiffness increase while the robot is away from its reference is paid from it and held when it is empty.
// Scale: raising 300 -> 700 N/m at a 2 cm error needs 0.08 J.
// Switch: build with -DTI_ENERGY_TANK=0 to run without it (stiffness never held; the log still integrates the tank).
#ifndef TI_ENERGY_TANK
#define TI_ENERGY_TANK 1
#endif
constexpr bool kEnableEnergyTank = TI_ENERGY_TANK != 0;
// Capacity: one step 300 -> 700 N/m at the largest error the 35 N collision threshold allows (35 N / 700 N/m =
// 5 cm) needs 1/2 * 400 * 0.05^2 = 0.5 J. -DTI_TANK_INITIAL_J / -DTI_TANK_MAXIMUM_J override for tests.
#ifndef TI_TANK_INITIAL_J
#define TI_TANK_INITIAL_J 0.5
#endif
#ifndef TI_TANK_MAXIMUM_J
#define TI_TANK_MAXIMUM_J 1.0
#endif
constexpr double kTankInitialEnergyJ = TI_TANK_INITIAL_J;
constexpr double kTankMinimumEnergyJ = 0.01;
constexpr double kTankMaximumEnergyJ = TI_TANK_MAXIMUM_J;

// Largest translational spring force (N), below the 35 N collision threshold. Beyond the offset
// kMaxSpringForceN / Kt the spring force stays at this value (saturated spring, still passive).
constexpr double kMaxSpringForceN = 30.0;

// Diagnostics: 100 Hz UDP packet for SlaveTeleImpedance/tank_logger.py (logging only, never read by control).
constexpr int kTeleImpedanceDiagnosticPort = 11061;
constexpr int kTeleImpedanceDiagnosticEveryCycles = 10;
constexpr int kTeleImpedanceConsoleEveryCycles = 1000;

// Forward teleoperation
constexpr bool kEnableTranslationFollowing = true;
constexpr bool kEnableRotationFollowing = true;
constexpr double kPositionScaleX = 1.0;
constexpr double kPositionScaleY = 1.0;
constexpr double kPositionScaleZ = 1.0;
constexpr double kMaxTranslationPerAxisM = 0.80;
constexpr double kMaxRelativeRotationRad = 60.0 * kPi / 180.0;
constexpr double kMaxDesiredLinearSpeedMps = 1.50;
constexpr double kMaxDesiredAngularSpeedRadps = 120.0 * kPi / 180.0;

// Joint-velocity soft guard.
// These are Franka's suggested conservative rectangular FR3 velocity limits.
// The guard starts before those values are reached and smoothly removes the
// Cartesian tracking drive as measured joint speed approaches the stop ratio.
constexpr std::array<double, 7> kJointVelocityGuardLimitsRadps = {
    2.0,
    1.0,
    1.5,
    1.25,
    3.0,
    1.5,
    3.0};
constexpr double kJointVelocityGuardStartRatio = 0.70;
constexpr double kJointVelocityGuardStopRatio = 0.85;

// Nullspace / numerical regularization
constexpr double kNullspaceStiffness = 8.0;
constexpr double kNullspaceDamping = 2.5;
constexpr double kDampedInverseLambda = 0.05;

// Torque / callback limits
constexpr double kMaxDeltaTorquePerCycleNm = 1.0;
constexpr double kMinimumCallbackDtS = 0.0005;

// Franka startup
constexpr std::array<double, 7> kInitialJointConfiguration = {
    0.0,
    -kPi / 4.0,
    0.0,
    -3.0 * kPi / 4.0,
    0.0,
    kPi / 2.0,
    kPi / 4.0};

constexpr double kInitialMotionSpeedFactor = 0.35;
constexpr double kCollisionThreshold = 35.0;

}  // namespace