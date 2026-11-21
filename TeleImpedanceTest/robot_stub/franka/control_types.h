#pragma once
#include <array>
namespace franka {
struct Finishable { bool motion_finished = false; };
struct Torques : Finishable { std::array<double,7> tau_J{}; Torques(const std::array<double,7>& t) : tau_J(t) {} };
struct JointPositions : Finishable { std::array<double,7> q{}; JointPositions(const std::array<double,7>& v) : q(v) {} };
template <typename T> T MotionFinished(T t) { t.motion_finished = true; return t; }
}
