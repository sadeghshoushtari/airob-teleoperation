#pragma once
#include <array>
namespace franka {
struct RobotState { std::array<double,7> q{}, dq{}, q_d{}, tau_J_d{}; std::array<double,16> O_T_EE{}; std::array<double,6> O_F_ext_hat_K{}; };
}
