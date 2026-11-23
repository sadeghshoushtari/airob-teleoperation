#pragma once

// Damping in the buzz band of the force feedback. In light edge contact the reflected force vibrates at 6-32 Hz
// (hardware tests 2026-11-23). The master port stays passive while it does: the operator's push puts far more energy
// into the feedback than the vibration takes out, so the energy tank (feedback_tank.hpp) never acts on it. This
// block looks at the band only.
//
// The same band-pass filter (first-order high-pass at low_hz, then first-order low-pass at high_hz) is applied to the
// wrench sent to the VIRTUOSE and to its velocity; force . velocity of the filtered signals is the power the feedback
// delivers to the hand in the band. While that power, averaged over time_constant_s, exceeds a threshold (below it:
// the band content of ordinary pressing and releasing, a few hundredths of a mW in the simulation; the buzz on the
// hardware: 2.6-6.6 mW), the block adds damping on the band-passed velocity, sized to dissipate margin times the
// excess:
//   D = margin * (P_band - threshold) / mean |v_band|^2, at most maximum_damping,
// falling back with release_time_s once the band power is below the threshold. Below the band (pressing, releasing,
// moving) the filtered velocity is small, so slow motion feels almost no drag. The band-pass
// b s / ((s + a)(s + b)) is positive real, so damping through it is passive. Force and torque are handled
// separately (N and Nm are never mixed).

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

class BuzzDamper {
 public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  BuzzDamper(double low_hz, double high_hz, double time_constant_s, double release_time_s, double margin,
             double force_power_threshold, double torque_power_threshold, double linear_damping_max,
             double angular_damping_max, double linear_velocity_floor, double angular_velocity_floor, bool enable)
      : low_rc_(1.0 / (2.0 * M_PI * low_hz)), high_rc_(1.0 / (2.0 * M_PI * high_hz)),
        time_constant_(time_constant_s), release_time_(release_time_s), margin_(margin),
        threshold_{force_power_threshold, torque_power_threshold},
        maximum_damping_{linear_damping_max, angular_damping_max},
        velocity_floor_sq_{linear_velocity_floor * linear_velocity_floor,
                           angular_velocity_floor * angular_velocity_floor},
        enabled_(enable) {}

  // sent: the wrench sent in the last cycle without this block's own part; velocity: the VIRTUOSE velocity now.
  // Returns this block's damping wrench for the next command (zero when disabled).
  Vector6d update(const Vector6d& sent, const Vector6d& velocity, double dt) {
    if (!(dt > 0.0) || !sent.allFinite() || !velocity.allFinite()) {
      return Vector6d::Zero();
    }
    const Vector6d wrench_band = wrench_filter_.step(sent, dt, low_rc_, high_rc_);
    const Vector6d velocity_band = velocity_filter_.step(velocity, dt, low_rc_, high_rc_);
    const double average = std::min(dt / time_constant_, 1.0);
    const double release = std::min(dt / release_time_, 1.0);
    Vector6d output = Vector6d::Zero();
    for (int part = 0; part < 2; ++part) {
      const int start = 3 * part;
      const Eigen::Vector3d v = velocity_band.segment<3>(start);
      power_[part] += average * (wrench_band.segment<3>(start).dot(v) - power_[part]);
      velocity_sq_[part] += average * (v.squaredNorm() - velocity_sq_[part]);
      const double target = std::clamp(
          margin_ * (power_[part] - threshold_[part]) / std::max(velocity_sq_[part], velocity_floor_sq_[part]), 0.0,
          maximum_damping_[part]);
      damping_[part] = target > damping_[part] ? target : damping_[part] + release * (target - damping_[part]);
      if (enabled_) {
        output.segment<3>(start) = -damping_[part] * v;
      }
    }
    return output;
  }

  double linear_damping() const { return enabled_ ? damping_[0] : 0.0; }   // Ns/m
  double angular_damping() const { return enabled_ ? damping_[1] : 0.0; }  // Nms/rad
  double force_band_power() const { return power_[0]; }                    // W, averaged
  double torque_band_power() const { return power_[1]; }                   // W, averaged

 private:
  struct BandPass {
    Vector6d previous_input = Vector6d::Zero();
    Vector6d high = Vector6d::Zero();
    Vector6d output = Vector6d::Zero();
    bool started = false;

    Vector6d step(const Vector6d& input, double dt, double low_rc, double high_rc) {
      if (!started) {
        started = true;
        previous_input = input;
      }
      high = (low_rc / (low_rc + dt)) * (high + input - previous_input);
      previous_input = input;
      output += (dt / (high_rc + dt)) * (high - output);
      return output;
    }
  };

  double low_rc_;
  double high_rc_;
  double time_constant_;
  double release_time_;
  double margin_;
  double threshold_[2];
  double maximum_damping_[2];
  double velocity_floor_sq_[2];
  bool enabled_;
  BandPass wrench_filter_;
  BandPass velocity_filter_;
  double power_[2] = {0.0, 0.0};
  double velocity_sq_[2] = {0.0, 0.0};
  double damping_[2] = {0.0, 0.0};
};
