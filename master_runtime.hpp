#pragma once

#include <cmath>

#include "master_config.hpp"
#include "teleop_math.hpp"
#include "virtuoseAPI.h"

inline double normalize_analogue_trigger(
    double raw_value) {

  const double denominator =
      kAnalogFullyPressedValue -
      kAnalogReleasedValue;

  if (!std::isfinite(raw_value) ||
      std::abs(denominator) < 1e-12) {
    return 0.0;
  }

  double normalized =
      (raw_value - kAnalogReleasedValue) /
      denominator;

  normalized =
      clamp_scalar(normalized, 0.0, 1.0);

  if (normalized <= kAnalogLowDeadzone) {
    return 0.0;
  }

  normalized =
      (normalized - kAnalogLowDeadzone) /
      (1.0 - kAnalogLowDeadzone);

  return clamp_scalar(normalized, 0.0, 1.0);
}

inline void zero_wrench(
    VirtContext context) {

  float wrench[6] = {
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f};

  virtSetForce(
      context,
      wrench);
}

inline void shutdown_virtuose(
    VirtContext context) {

  if (!context) {
    return;
  }

  zero_wrench(
      context);

  virtEnableForceFeedback(
      context,
      0);

  virtSetPowerOn(
      context,
      0);

  virtClose(
      context);
}

inline const char* power_label(
    double power) {

  constexpr double kPowerDeadbandW =
      0.01;

  if (std::abs(power) <
      kPowerDeadbandW) {
    return "NEAR_ZERO";
  }

  if (power < 0.0) {
    return "RESISTING";
  }

  return "ASSISTING";
}
