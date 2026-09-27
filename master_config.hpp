#pragma once

#include "teleop_common.hpp"

namespace {
// ------------------------------------------------------------
// CONTROL RATE / TIMING
// ------------------------------------------------------------

constexpr double kLoopHz =
    1000.0;

constexpr double kFeedbackStaleMs =
    50.0;

constexpr float kVirtuoseApiTimeoutS =
    0.05f;


// ============================================================
// BACKWARD CONTROL SWITCHES
// ============================================================

// Full 6-DoF backward pose coupling is now verified.

constexpr bool kEnableBackwardPositionCoupling =
    true;

constexpr bool kEnableBackwardRotationCoupling =
    true;


// ------------------------------------------------------------
// DIRECTIONAL DAMPING
//
// Independent from force/torque reflection.
//
// Translation:
//   project Virtuose linear velocity onto Franka external-force
//   direction, then oppose only that projected motion.
//
// Rotation:
//   project Virtuose angular velocity onto Franka external-torque
//   direction, then oppose only that projected motion.
//
// IMPORTANT:
//   Force [N] and torque [Nm] are NOT combined into one normalized
//   6-D wrench, unlike the reference implementation.
// ------------------------------------------------------------

constexpr bool kEnableDirectionalForceDamping =
    true;

constexpr bool kEnableDirectionalTorqueDamping =
    true;

// ------------------------------------------------------------
// WRENCH REFLECTION
//
// Reflection is a separate additive haptic term. These switches
// do not enable/disable directional damping.
// ------------------------------------------------------------

constexpr bool kEnableForceReflection =
    true;

constexpr bool kEnableTorqueReflection =
    true;

// ============================================================
// BACKWARD POSITION COUPLING
//
// Fsync = Kb * (x_target - x_V)
//       + Db * (v_target - v_V)
//
// ============================================================

// Variable master-side virtual translational impedance.
// Damping scales with sqrt(K/K0) around the previously used nominal point.
constexpr double kMinimumBackwardPositionStiffness =
    5.0;                         // N/m

constexpr double kBaselineBackwardPositionStiffness =
    10.0;                        // N/m

constexpr double kMaximumBackwardPositionStiffness =
    15.0;                        // N/m

constexpr double kBaselineBackwardPositionDamping =
    1.0;                         // Ns/m at K = 10 N/m

constexpr double kMaximumBackwardPositionStiffnessRate =
    5.0;                         // N/m/s -> 5...15 in ~2 s

// ============================================================
// BACKWARD ROTATION COUPLING
//
// Msync = Kr * rotation_error
//       + Dr * (w_target - w_V)
//
// ============================================================

constexpr double kMinimumBackwardRotationStiffness =
    0.5;                         // Nm/rad

constexpr double kBaselineBackwardRotationStiffness =
    1.0;                         // Nm/rad

constexpr double kMaximumBackwardRotationStiffness =
    1.5;                         // Nm/rad

constexpr double kBaselineBackwardRotationDamping =
    0.15;                        // Nms/rad at K = 1 Nm/rad

constexpr double kMaximumBackwardRotationStiffnessRate =
    0.5;                         // Nm/rad/s -> 0.5...1.5 in ~2 s

// ------------------------------------------------------------
// BACKWARD SYNCHRONIZATION ERROR LIMITS
// ------------------------------------------------------------

constexpr double kMaxBackwardPositionErrorM =
    0.10;                        // m

constexpr double kMaxBackwardRotationErrorDeg =
    30.0;                        // deg

constexpr double kMinBackwardRotationErrorDeg =
    2.0;     //was 5

constexpr double kMaxBackwardRotationErrorRad =

    kMaxBackwardRotationErrorDeg *
    kPi /
    180.0;

constexpr double kMinBackwardRotationErrorRad =

    kMinBackwardRotationErrorDeg *
    kPi /
    180.0;

// ============================================================
// SELECTIVE DIRECTIONAL DAMPING
//
// Translation:
//
//   nF        = F / ||F||
//   v_parallel = (v_V . nF) nF
//   Fd        = -bF v_parallel
//
//   Pd_lin = Fd . v_V
//          = -bF ||v_parallel||^2
//          <= 0
//
// Rotation:
//
//   nM         = M / ||M||
//   w_parallel = (w_V . nM) nM
//   Md         = -bM w_parallel
//
//   Pd_rot = Md . w_V
//          = -bM ||w_parallel||^2
//          <= 0
//
// Translation and rotation are kept dimensionally separate.
// The measured wrench is used only to define the damping axis;
// wrench reflection remains an independent additive term.
// ============================================================

constexpr double kDirectionalLinearDamping =
    1.0;                         // Ns/m

constexpr double kDirectionalAngularDamping =
    0.5;                        // Nms/rad

constexpr double kDirectionalForceThresholdN =
    0.5;                         // N

constexpr double kDirectionalTorqueThresholdNm =
    0.1;                        // Nm


// ============================================================
// WRENCH REFLECTION
//
// Currently disabled.
//
// F_V = -gF * F_F
// M_V = -gM * M_F
// ============================================================

constexpr double kForceReflectionGain =
    0.6;

constexpr double kTorqueReflectionGain =
    0.30;


// ------------------------------------------------------------
// VIRTUOSE SOFTWARE/API SATURATION
// ------------------------------------------------------------

// Haption VIRTUOSE 6D TAO HF published ratings.
// Peak ratings are documented here but are NOT held continuously.
constexpr double kDevicePeakForceN =
    70.0;

constexpr double kDevicePeakTorqueNm =
    5.0;

// Persistent software/API ceilings use the published continuous
// ratings, because the command may remain saturated indefinitely.
constexpr double kDeviceForceCeilingN =
    30.0;

constexpr double kDeviceTorqueCeilingNm =
    1.5;

constexpr double kNearCeilingFraction =
    0.90;


// ------------------------------------------------------------
// COMMAND SLEW LIMITS
// ------------------------------------------------------------

constexpr double kMaxForceStepN =
    0.075;                       // N/cycle @ 1 kHz

constexpr double kMaxTorqueStepNm =
    0.010;                       // Nm/cycle @ 1 kHz


// ------------------------------------------------------------
// COMMAND LOW-PASS FILTERS
// ------------------------------------------------------------

constexpr double kForceFilterAlpha =
    0.8;

constexpr double kTorqueFilterAlpha =
    0.5;


// ------------------------------------------------------------
// FRANKA WRENCH VALIDITY
//
// There is deliberately NO magnitude-triggered dropout here.
// A finite debiased wrench remains usable; the final command sent
// to the Virtuose is saturated to the continuous device ceiling.
// ------------------------------------------------------------


// ------------------------------------------------------------
// STARTUP WRENCH BIAS
// ------------------------------------------------------------

constexpr int kBiasSamplesRequired =
    500;


// ------------------------------------------------------------
// VIRTUOSE API SCALING
// ------------------------------------------------------------

constexpr float kVirtuoseForceFactor =
    1.0f;

constexpr float kVirtuoseSpeedFactor =
    1.0f;


// ------------------------------------------------------------
// ANALOGUE-GRIPPER 6-DOF STIFFNESS INPUT
// ------------------------------------------------------------
// Haption documents the finger trigger as a passive 0-100% analogue input.
// The API channel is kept as one explicit constant so it is trivial to change
// if this device exposes the trigger on a different analogue[] index.
//
// Released / very lightly pressed  -> minimum translational + rotational stiffness
// Fully pressed                    -> maximum translational + rotational stiffness
//
// The same normalized trigger command u in [0,1] drives both:
//   Ktrans = 300 .. 700 N/m
//   Krot   = 40  .. 80  Nm/rad
//
// The same trigger also drives the master-side virtual spring/damper
// through independent master gain ranges defined above.
// ------------------------------------------------------------

constexpr std::size_t kStiffnessAnalogChannel =
    0;

constexpr double kAnalogReleasedValue =
    0.0;

constexpr double kAnalogFullyPressedValue =
    1.0;

constexpr double kAnalogLowDeadzone =
    0.03;                        // first 3% stays at minimum K

constexpr double kAnalogFilterTimeConstantS =
    0.030;                       // 30 ms input smoothing

constexpr double kMinimumTranslationalStiffness =
    300.0;                       // N/m

constexpr double kMaximumTranslationalStiffness =
    700.0;                       // N/m

constexpr double kMinimumRotationalStiffness =
    40.0;                        // Nm/rad

constexpr double kMaximumRotationalStiffness =
    80.0;                        // Nm/rad


constexpr int kConsolePrintEveryCycles =
    200;

}  // namespace
