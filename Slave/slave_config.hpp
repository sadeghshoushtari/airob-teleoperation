#pragma once

#include <array>
#include <cstdint>

#include "teleop_common.hpp"

namespace {

// ------------------------------------------------------------
// COMMUNICATION
// ------------------------------------------------------------

constexpr double kControlRateHz =
    1000.0;

constexpr uint64_t kMasterStaleNs =
    50ULL *
    1000ULL *
    1000ULL;

constexpr int kFeedbackSendPeriodUs =
    1000;

constexpr int kReceiveIdleSleepUs =
    100;


// ------------------------------------------------------------
// VARIABLE CARTESIAN IMPEDANCE
// ------------------------------------------------------------

constexpr double kBaselineTranslationalStiffness =
    500.0;

constexpr double kBaselineTranslationalDamping =
    20.0; // was 45

constexpr double kMinimumTranslationalStiffness =
    300.0;

constexpr double kMaximumTranslationalStiffness =
    700.0;

constexpr double kMaximumStiffnessRate =
    200.0;                       // N/m/s

// ------------------------------------------------------------
// VARIABLE ROTATIONAL IMPEDANCE
// ------------------------------------------------------------

constexpr double kMinimumRotationalStiffness =
    40.0;                        // Nm/rad

constexpr double kBaselineRotationalStiffness =
    80.0;                        // Nm/rad

constexpr double kMaximumRotationalStiffness =
    80.0;                        // Nm/rad

constexpr double kBaselineRotationalDamping =
    0.5;                         // Nms/rad

constexpr double kMaximumRotationalStiffnessRate =
    20.0;                        // Nm/rad/s -> 40 Nm/rad sweep in 2 s

// ------------------------------------------------------------
// FORWARD ENABLES
// ------------------------------------------------------------

constexpr bool kEnableTranslationFollowing =
    true;

constexpr bool kEnableRotationFollowing =
    true;

// ------------------------------------------------------------
// FORWARD SCALE
// ------------------------------------------------------------

constexpr double kPositionScaleX =
    1.0;

constexpr double kPositionScaleY =
    1.0;

constexpr double kPositionScaleZ =
    1.0;

// ------------------------------------------------------------
// FORWARD WORKSPACE
// ------------------------------------------------------------

constexpr double kMaxTranslationPerAxisM =
    0.20;

constexpr double kMaxRelativeRotationDeg =
    60.0;

constexpr double kMaxRelativeRotationRad =
    kMaxRelativeRotationDeg *
    kPi /
    180.0;

// ------------------------------------------------------------
// FORWARD TARGET RATE
// ------------------------------------------------------------

constexpr double kMaxDesiredLinearSpeedMps =
    0.30;

constexpr double kMaxDesiredAngularSpeedDegps =
    120.0;

constexpr double kMaxDesiredAngularSpeedRadps =
    kMaxDesiredAngularSpeedDegps *
    kPi /
    180.0;

// ------------------------------------------------------------
// NULLSPACE
// ------------------------------------------------------------

constexpr double kNullspaceStiffness =
    8.0 ; //was 8

constexpr double kNullspaceDamping =
    2.5; //was 2.5

// ------------------------------------------------------------
// DAMPED LEAST SQUARES
// ------------------------------------------------------------

constexpr double kDampedInverseLambda =
    0.05;

// ------------------------------------------------------------
// JOINT TORQUE RATE
// ------------------------------------------------------------

constexpr double kMaxDeltaTorquePerCycleNm =
    1.0;

// ------------------------------------------------------------
// CALLBACK DT SANITY
// ------------------------------------------------------------

constexpr double kMinimumCallbackDtS =
    0.0005;

// ------------------------------------------------------------
// SLAVE FEEDBACK WRENCH LOW-PASS FILTER
// ------------------------------------------------------------

constexpr double kForceFilterAlpha =
    0.7;

constexpr double kTorqueFilterAlpha =
    0.7;

// ------------------------------------------------------------
// FRANKA INITIAL CONFIGURATION
// ------------------------------------------------------------

constexpr std::array<double, 7>
    kInitialJointConfiguration = {

        0.0,

        -kPi / 4.0,

        0.0,

        -3.0 * kPi / 4.0,

        0.0,

        kPi / 2.0,

        kPi / 4.0};


// ------------------------------------------------------------
// INITIAL MOTION
// ------------------------------------------------------------

constexpr double kInitialMotionSpeedFactor =
    0.35;

// ------------------------------------------------------------
// COLLISION THRESHOLD
// ------------------------------------------------------------

constexpr double kCollisionThreshold =
    35.0;

}  // namespace
