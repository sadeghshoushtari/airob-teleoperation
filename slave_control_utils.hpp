#pragma once

#include <array>

#include <Eigen/Dense>

#include "slave_config.hpp"
#include "teleop_math.hpp"

inline std::array<double, 7>
saturate_torque_rate(

    const Eigen::Matrix<
        double,
        7,
        1>& calculated,

    const std::array<
        double,
        7>& previous_desired) {

  std::array<double, 7>
      result{};

  for (std::size_t i = 0;
       i < 7;
       ++i) {

    const double delta =
        calculated[
            static_cast<int>(
                i)] -
        previous_desired[i];

    result[i] =
        previous_desired[i] +
        clamp_scalar(
            delta,
            -kMaxDeltaTorquePerCycleNm,
            kMaxDeltaTorquePerCycleNm);
  }

  return result;
}
