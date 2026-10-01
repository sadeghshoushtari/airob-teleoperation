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