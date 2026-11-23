#pragma once
// Test stub of libfranka (not the real library) for perturbation.cpp: a 3 kg handle at (0.3, 0, 0.5) held by the
// program's torques 1-3 (the stub Jacobian makes them the Cartesian force) and by a hand spring K_h with 20 Ns/m; its
// orientation (inertia 0.02 kg m^2, small-angle rotation vector) held by torques 4-6 and a rotational hand spring K_hr
// with 0.3 Nms/rad. K_h (9 numbers, N/m, row by row), optionally followed by K_hr (9 numbers, Nm/rad), is read every
// 50 ms from the file named by FRANKA_STUB_HAND (none: no hand). O_F_ext_hat_K = (K_h dx, K_hr dtheta); its force is
// appended at 100 Hz ("t fx fy fz") to the file named by FRANKA_STUB_FORCE_LOG (if set). FRANKA_STUB_FORCE_DELAY_MS
// (if set) reports O_F_ext_hat_K that many ms late (the lag of the real external-wrench estimate; the body still
// feels the hand springs at once). FRANKA_STUB_MOMENT_COUPLING = "rx ry rz" (m, if set) adds r x F to the reported
// moment, the force-to-moment cross-coupling of the real estimate that MasterTeleImpedance compensates
// (kMomentCompensationVectorO). Runs at 1 kHz wall-clock time until the program ends the control loop.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include "control_types.h"
#include "duration.h"
#include "model.h"
#include "robot_state.h"
namespace franka {
struct Robot {
  explicit Robot(const std::string&) {}
  void setCollisionBehavior(const std::array<double,7>&, const std::array<double,7>&, const std::array<double,6>&, const std::array<double,6>&) {}
  void setCollisionBehavior(const std::array<double,7>&, const std::array<double,7>&, const std::array<double,7>&, const std::array<double,7>&, const std::array<double,6>&, const std::array<double,6>&, const std::array<double,6>&, const std::array<double,6>&) {}
  void setJointImpedance(const std::array<double,7>&) {}
  void setCartesianImpedance(const std::array<double,6>&) {}
  void control(std::function<Torques(const RobotState&, Duration)> cb) {
    const double home[3] = {0.3, 0.0, 0.5}, mass = 3.0, hand_damping = 20.0, dt = 0.001;
    const std::array<double, 7> q_home = {0.0, -0.785398163397448, 0.0, -2.356194490192345, 0.0, 1.570796326794897,
                                          0.785398163397448};
    const char* hand_file = std::getenv("FRANKA_STUB_HAND");
    const char* delay_text = std::getenv("FRANKA_STUB_FORCE_DELAY_MS");
    const int delay = delay_text ? std::max(0, std::atoi(delay_text)) : 0;
    double coupling[3] = {0.0, 0.0, 0.0};
    if (const char* coupling_text = std::getenv("FRANKA_STUB_MOMENT_COUPLING")) {
      if (std::sscanf(coupling_text, "%lf %lf %lf", &coupling[0], &coupling[1], &coupling[2]) != 3) {
        coupling[0] = coupling[1] = coupling[2] = 0.0;
      }
    }
    std::vector<std::array<double, 6>> reported(delay + 1, std::array<double, 6>{});
    double k[18] = {0}, x[6] = {0}, v[6] = {0};
    const double inertia = 0.02, hand_rot_damping = 0.3;
    RobotState st{};
    st.q = q_home;
    st.q_d = q_home;
    auto next = std::chrono::steady_clock::now();
    for (long i = 0;; ++i) {
      if (hand_file && i % 50 == 0) {
        std::ifstream f(hand_file);
        double kn[18] = {0};
        int got = 0;
        while (got < 18 && f >> kn[got]) ++got;
        if (got == 9 || got == 18) for (int n = 0; n < 18; ++n) k[n] = kn[n];
      }
      // orientation: rotation vector x[3..5] (Rodrigues), column-major rotation matrix in O_T_EE
      const double th = std::sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
      double a[3] = {0, 0, 0}, R[9];
      if (th > 1e-12) for (int n = 0; n < 3; ++n) a[n] = x[3 + n] / th;
      const double c = std::cos(th), s1 = std::sin(th), c1 = 1.0 - c;
      R[0] = c + a[0] * a[0] * c1;        R[3] = a[0] * a[1] * c1 - a[2] * s1; R[6] = a[0] * a[2] * c1 + a[1] * s1;
      R[1] = a[1] * a[0] * c1 + a[2] * s1; R[4] = c + a[1] * a[1] * c1;        R[7] = a[1] * a[2] * c1 - a[0] * s1;
      R[2] = a[2] * a[0] * c1 - a[1] * s1; R[5] = a[2] * a[1] * c1 + a[0] * s1; R[8] = c + a[2] * a[2] * c1;
      st.O_T_EE = {R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0,
                   home[0] + x[0], home[1] + x[1], home[2] + x[2], 1};
      std::array<double, 6> wrench{};
      for (int r = 0; r < 6; ++r) {
        st.dq[r] = v[r];
        const int o = r < 3 ? 0 : 3, kb = r < 3 ? 3 * r : 9 + 3 * (r - 3);
        wrench[r] = k[kb] * x[o] + k[kb + 1] * x[o + 1] + k[kb + 2] * x[o + 2];
      }
      std::array<double, 6>& report = reported[i % (delay + 1)];
      report = wrench;
      report[3] += coupling[1] * wrench[2] - coupling[2] * wrench[1];
      report[4] += coupling[2] * wrench[0] - coupling[0] * wrench[2];
      report[5] += coupling[0] * wrench[1] - coupling[1] * wrench[0];
      st.O_F_ext_hat_K = reported[(i + 1) % (delay + 1)];
      static const char* force_log = std::getenv("FRANKA_STUB_FORCE_LOG");
      if (force_log && i % 10 == 0) {
        std::ofstream lf(force_log, std::ios::app);
        lf << i * dt << ' ' << wrench[0] << ' ' << wrench[1] << ' ' << wrench[2] << '\n';
      }
      const Torques out = cb(st, Duration{});
      st.tau_J_d = out.tau_J;
      if (out.motion_finished) return;
      for (int r = 0; r < 3; ++r) v[r] += dt * (out.tau_J[r] - wrench[r] - hand_damping * v[r]) / mass;
      for (int r = 3; r < 6; ++r)
        v[r] += dt * (out.tau_J[r] - wrench[r] - hand_rot_damping * v[r]) / inertia;
      for (int r = 0; r < 6; ++r) x[r] += dt * v[r];  // small angles: rotation vector rate = angular velocity
      next += std::chrono::microseconds(1000);
      std::this_thread::sleep_until(next);
    }
  }
  template <typename G> void control(G&) {}
  Model loadModel() { return {}; }
};
}  // namespace franka
