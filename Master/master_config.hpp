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

constexpr bool kEnableBackwardPositionCoupling =
    true;

constexpr bool kEnableBackwardRotationCoupling =
    true;


// ------------------------------------------------------------
// DIRECTIONAL DAMPING
// ------------------------------------------------------------

constexpr bool kEnableDirectionalForceDamping =
    true;

constexpr bool kEnableDirectionalTorqueDamping =
    true;

// ------------------------------------------------------------
// WRENCH REFLECTION
// ------------------------------------------------------------

constexpr bool kEnableForceReflection =
    true;

constexpr bool kEnableTorqueReflection =
    true;

// ============================================================
// BACKWARD POSITION COUPLING
// ============================================================

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
// ============================================================

constexpr double kForceReflectionGain =
    0.6;

constexpr double kTorqueReflectionGain =
    0.30;

// ------------------------------------------------------------
// VIRTUOSE SOFTWARE/API SATURATION
// ------------------------------------------------------------

constexpr double kDevicePeakForceN =
    70.0;

constexpr double kDevicePeakTorqueNm =
    5.0;

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
// ANALOGUE-GRIPPER STIFFNESS INPUT
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
