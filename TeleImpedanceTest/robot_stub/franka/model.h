#pragma once
// Test stub of libfranka (not the real library): the Jacobian maps joints 1-6 one to one onto the Cartesian
// velocity, so the commanded torques 1-3 are the commanded force on the handle.
#include <array>
#include "robot_state.h"
namespace franka {
enum class Frame { kEndEffector };
struct Model {
  std::array<double, 42> zeroJacobian(Frame, const RobotState&) const {
    std::array<double, 42> j{};
    for (int i = 0; i < 6; ++i) j[i * 6 + i] = 1.0;  // column-major 6 x 7
    return j;
  }
  std::array<double, 7> coriolis(const RobotState&) const { return {}; }
};
}  // namespace franka
