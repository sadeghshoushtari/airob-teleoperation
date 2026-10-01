#pragma once

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
constexpr double kBaselineBackwardRotationStiffness = 1.00;     // Nm/rad
constexpr double kMaximumBackwardRotationStiffness = 1.50;      // Nm/rad
constexpr double kBaselineBackwardRotationDamping = 0.15;      // Nms/rad
constexpr double kMaximumBackwardRotationStiffnessRate = 0.5;  // Nm/rad/s
constexpr double kMaxBackwardRotationErrorRad = 30.0 * kPi / 180.0;
constexpr double kMinBackwardRotationErrorRad = 2.0 * kPi / 180.0;

// Selective directional damping
constexpr double kDirectionalLinearDamping = 1.0;       // Ns/m
constexpr double kDirectionalAngularDamping = 0.5;      // Nms/rad
constexpr double kDirectionalForceThresholdN = 0.5;     // N
constexpr double kDirectionalTorqueThresholdNm = 0.1;   // Nm

// Wrench reflection
constexpr double kForceReflectionGain = 0.6;
constexpr double kTorqueReflectionGain = 0.3;

// Virtuose command limits
constexpr double kDeviceForceCeilingN = 30.0;
constexpr double kDeviceTorqueCeilingNm = 1.5;
constexpr double kMaxForceStepN = 0.075;    // N/cycle @ 1 kHz
constexpr double kMaxTorqueStepNm = 0.010;  // Nm/cycle @ 1 kHz
constexpr double kForceFilterAlpha = 0.5;
constexpr double kTorqueFilterAlpha = 0.5;

// Startup wrench bias
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

// Console
constexpr int kConsolePrintEveryCycles = 200;

}  // namespace