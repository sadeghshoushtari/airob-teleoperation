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

// Wrench reflection (-DTI_FORCE_REFLECTION_GAIN overrides for tests)
#ifndef TI_FORCE_REFLECTION_GAIN
#define TI_FORCE_REFLECTION_GAIN 0.6
#endif
constexpr double kForceReflectionGain = TI_FORCE_REFLECTION_GAIN;

// Energy tank of the force feedback (feedback_tank.hpp): the wrench sent to the VIRTUOSE gives the operator at most
// the energy the operator gave the feedback before, plus the initial tank. A press stores its energy for the
// release (a press of 18 N over 7 cm stores about 0.6 J), so the maximum is 1 J; the initial 0.05 J is all the
// feedback may give before the operator has put energy in. Build with -DTI_MASTER_TANK=0 to run without it (the tank
// is still booked, so the log shows the energy the feedback gave); -DTI_MASTER_TANK_INITIAL_J /
// -DTI_MASTER_TANK_MAXIMUM_J override for tests.
#ifndef TI_MASTER_TANK
#define TI_MASTER_TANK 1
#endif
#ifndef TI_MASTER_TANK_INITIAL_J
#define TI_MASTER_TANK_INITIAL_J 0.05
#endif
#ifndef TI_MASTER_TANK_MAXIMUM_J
#define TI_MASTER_TANK_MAXIMUM_J 1.0
#endif
constexpr bool kEnableFeedbackTank = TI_MASTER_TANK != 0;
constexpr double kFeedbackTankInitialJ = TI_MASTER_TANK_INITIAL_J;
constexpr double kFeedbackTankMinimumJ = 0.0;
constexpr double kFeedbackTankMaximumJ = TI_MASTER_TANK_MAXIMUM_J;
// Stage 1: below 0.02 J the tank adds damping on the VIRTUOSE, up to 30 Ns/m and 0.3 Nms/rad on an empty tank.
constexpr double kFeedbackTankDampingBandJ = 0.02;
constexpr double kFeedbackTankMaximumLinearDamping = 30.0;    // Ns/m
constexpr double kFeedbackTankMaximumAngularDamping = 0.30;   // Nms/rad
// Stage 2 predicts the motion from the displacement per cycle, smoothed with this factor per cycle (about 16 Hz).
constexpr double kFeedbackTankMotionSmoothing = 0.1;

// Damping in the buzz band (buzz_damper.hpp): while the feedback gives the hand more than 1 mW (torque: 0.5 mW) at
// 5-45 Hz (the edge contact vibration: 2.6-6.6 mW on 2026-11-23), damping on the band-passed VIRTUOSE velocity
// dissipates twice the excess, up to 20 Ns/m and 0.3 Nms/rad. Build with -DTI_BUZZ_DAMPER=0 to run without it (the
// band power is still logged).
#ifndef TI_BUZZ_DAMPER
#define TI_BUZZ_DAMPER 1
#endif
constexpr bool kEnableBuzzDamper = TI_BUZZ_DAMPER != 0;
constexpr double kBuzzBandLowHz = 5.0;
constexpr double kBuzzBandHighHz = 45.0;
constexpr double kBuzzPowerTimeConstantS = 0.2;   // averaging of the band power
constexpr double kBuzzReleaseTimeS = 1.0;         // damping falls back with this time constant
constexpr double kBuzzMargin = 2.0;               // dissipates this multiple of the band power
constexpr double kBuzzForcePowerThresholdW = 0.001;
constexpr double kBuzzTorquePowerThresholdW = 0.0005;
constexpr double kBuzzMaximumLinearDamping = 20.0;   // Ns/m
constexpr double kBuzzMaximumAngularDamping = 0.30;  // Nms/rad
constexpr double kBuzzLinearVelocityFloor = 0.005;   // m/s, below this the band velocity counts as noise
constexpr double kBuzzAngularVelocityFloor = 0.05;   // rad/s

// 100 Hz UDP packet of the tank and the buzz damping (logging only), recorded by master_tank_logger.py.
constexpr int kFeedbackTankDiagnosticPort = 11062;
constexpr int kFeedbackTankDiagnosticEveryCycles = 10;

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
#ifndef TI_TORQUE_REFLECTION_GAIN_MAX
#define TI_TORQUE_REFLECTION_GAIN_MAX 0.50
#endif
constexpr double kMaximumTorqueReflectionGain = TI_TORQUE_REFLECTION_GAIN_MAX;  // -D overrides for tests

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