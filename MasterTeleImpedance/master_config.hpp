#pragma once

#include <array>
#include <cstddef>

#include "teleop_common.hpp"

namespace {

// Timing / communication
constexpr double kLoopHz = 1000.0;
constexpr double kFeedbackStaleMs = 50.0;
constexpr float kVirtuoseApiTimeoutS = 0.05f;

// Backward controller enables
constexpr bool kEnableBackwardPositionCoupling = true;
constexpr bool kEnableBackwardRotationCoupling = true;
constexpr bool kEnableDirectionalForceDamping = true;
constexpr bool kEnableDirectionalTorqueDamping = true;
constexpr bool kEnableForceReflection = true;
constexpr bool kEnableTorqueReflection = true;

// Backward translational coupling
constexpr double kMinimumBackwardPositionStiffness = 5.0;      // N/m
constexpr double kBaselineBackwardPositionStiffness = 10.0;    // N/m
constexpr double kMaximumBackwardPositionStiffness = 15.0;     // N/m
constexpr double kBaselineBackwardPositionDamping = 1.0;       // Ns/m
constexpr double kMaximumBackwardPositionStiffnessRate = 5.0;  // N/m/s
constexpr double kMaxBackwardPositionErrorM = 0.10;            // m

// Backward rotational coupling
constexpr double kMinimumBackwardRotationStiffness = 0.5;      // Nm/rad
constexpr double kBaselineBackwardRotationStiffness = 1.00;    // Nm/rad
constexpr double kMaximumBackwardRotationStiffness = 1.50;     // Nm/rad
constexpr double kBaselineBackwardRotationDamping = 0.15;      // Nms/rad
constexpr double kMaximumBackwardRotationStiffnessRate = 0.5;  // Nm/rad/s
constexpr double kMaxBackwardRotationErrorRad = 30.0 * kPi / 180.0;
constexpr double kMinBackwardRotationErrorRad = 2.0 * kPi / 180.0;

// Selective directional damping
constexpr double kDirectionalLinearDamping = 1.0;       // Ns/m
constexpr double kDirectionalAngularDamping = 0.25;      // Nms/rad
constexpr double kDirectionalForceThresholdN = 0.5;     // N
constexpr double kDirectionalTorqueThresholdNm = 1.5;   // Nm

// Wrench reflection
constexpr double kForceReflectionGain = 0.6;

// Empirical force-to-moment cross-coupling compensation.
// Identified from the locked-orientation translation test.
// The vector is expressed in the Franka O frame and is an empirical
// compensation model, not physical tool geometry.
constexpr bool kEnableMomentCompensation = true;
constexpr double kMomentCompensationScale = 1.0;
constexpr std::array<double, 3> kMomentCompensationVectorO = {
    0.405,
    0.071,
    0.205};

// Contact-aware rotational reflection.
// Free-space residuals receive only the minimum gain.
// Strong force or moment contact smoothly raises the gain to the maximum.
constexpr double kMinimumTorqueReflectionGain = 0.05;
constexpr double kMaximumTorqueReflectionGain = 0.50;

// Raised from the first experiment so free-space estimator excursions do not
// falsely look like contact. These thresholds are applied to the debiased
// Franka wrench estimate.
constexpr double kTorqueReflectionForceRampStartN = 3.5;    // N
constexpr double kTorqueReflectionForceRampEndN = 6.0;      // N
constexpr double kTorqueReflectionMomentRampStartNm = 2.3;  // Nm
constexpr double kTorqueReflectionMomentRampEndNm = 3.5;    // Nm

// Virtuose command limits
constexpr double kDeviceForceCeilingN = 30.0;
constexpr double kDeviceTorqueCeilingNm = 1.5;

// Contact saturation hold.
// This does not change the baseline wrench calculation. If the already
// rendered baseline command reaches a device ceiling during detected contact,
// only the command sent to the Virtuose is held at that ceiling. The force or
// torque direction is captured at the instant saturation is reached and kept
// fixed for that contact episode. The hold is released immediately when the
// existing contact detector returns to zero.
constexpr bool kEnableContactSaturationHold = true;
constexpr double kMaxForceStepN = 0.075;    // N/cycle @ 1 kHz
constexpr double kMaxTorqueStepNm = 0.010;  // Nm/cycle @ 1 kHz
constexpr double kForceFilterAlpha = 0.5;
constexpr double kTorqueFilterAlpha = 0.5;

// Startup wrench bias: fixed after startup; no online adaptation.
// Ignore the Slave's zero-filled startup feedback and allow the Franka
// external-wrench estimate to settle before collecting calibration samples.
constexpr double kBiasWarmupS = 2.0;
constexpr int kBiasSamplesRequired = 500;

// Virtuose API scaling
constexpr float kVirtuoseForceFactor = 1.0f;
constexpr float kVirtuoseSpeedFactor = 1.0f;

// Analogue trigger
constexpr std::size_t kStiffnessAnalogChannel = 0;
constexpr double kAnalogReleasedValue = 0.0;
constexpr double kAnalogFullyPressedValue = 1.0;
constexpr double kAnalogLowDeadzone = 0.03;
constexpr double kAnalogFilterTimeConstantS = 0.030;

// Slave translational stiffness command
constexpr double kMinimumTranslationalStiffness = 300.0;  // N/m
constexpr double kMaximumTranslationalStiffness = 700.0;  // N/m

// Diagnostic telemetry for the final validation logger.
// Logging-only path: never used by the Slave and never read back by control.
// 100 Hz at the current 1 kHz controller rate keeps timing overhead small.
constexpr bool kEnableDiagnosticTelemetry = true;
constexpr int kDiagnosticTelemetryPort = 11057;
constexpr int kDiagnosticTelemetryDecimation = 10;
constexpr std::size_t kDiagnosticTelemetryPacketSize = 106;

// Console
constexpr int kConsolePrintEveryCycles = 200;

}  // namespace