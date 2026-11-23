#pragma once

// Energy tank of the force feedback: the passivity layer at the master (two-layer approach, Franken, Stramigioli,
// Misra, Secchi and Macchelli, IEEE T-RO 2011). The slave has its own tank for the variable stiffness
// (SlaveTeleImpedance/energy_tank.hpp); this one covers the wrench sent to the VIRTUOSE.
//
// The transparency layer (the baseline wrench: reflection, backward coupling, damping, filter, saturation hold) is
// unchanged. Every cycle the tank books the energy the last command exchanged with the operator's hand,
//   dE = (W - D) . dX,
// W the wrench sent, D its pure damping part (-D v of the VIRTUOSE's own velocity, passive by itself, neither paid
// nor stored), dX the VIRTUOSE displacement (translation, world-frame rotation vector) over the cycle. dE > 0: the
// feedback did work on the hand and the tank pays; dE < 0: the hand did work against the feedback and the tank
// stores it (up to its maximum; above it the energy is dissipated). The feedback then never gives the operator more
// energy than the operator gave it (plus the initial tank), whatever creates the energy: sampling, filters, delays,
// the force estimate or the saturation hold. At rest (dX = 0) nothing is paid, so a steady contact force is rendered
// in full.
//
// The new command is limited in two stages:
//   1. Below minimum + band the tank adds damping on the VIRTUOSE's velocity, growing linearly to its maximum as the
//      tank empties. It is continuous in the velocity (an empty tank would otherwise switch the force on and off
//      with the direction of motion and make it chatter), it takes out the energy the loop creates, and its
//      negative work refills the tank.
//   2. If the predicted power of the command still exceeds what the tank holds, the command is reduced along the
//      motion so that it does no more work than the tank holds. The motion is predicted from the smoothed
//      displacement per cycle: a prediction from the last cycle alone flips with every reversal, so the cut would
//      switch on and off one cycle late and make the force chatter. The tank can therefore be overdrawn by the
//      prediction error; the booking stays exact, so any overdraft shows as a negative tank and stage 1 (at its
//      maximum) pays it back.

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

struct FeedbackTank {
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  double energy;                   // J
  double minimum;                  // J
  double maximum;                  // J
  double band;                     // J, stage 1 acts below minimum + band
  double maximum_linear_damping;   // Ns/m, stage 1 on an empty tank
  double maximum_angular_damping;  // Nms/rad
  double smoothing;                // 0..1 per cycle, filter of the predicted motion (stage 2)
  bool enabled;
  double delivered = 0.0;          // J, work of the feedback on the hand
  double received = 0.0;           // J, work of the hand against the feedback
  double dissipated = 0.0;         // J, received above the maximum
  double removed = 0.0;            // J, predicted work stage 2 took out of the commands
  double linear_damping = 0.0;     // Ns/m, stage 1 in the last command
  double angular_damping = 0.0;    // Nms/rad
  bool limited = false;            // stage 2 reduced the last command
  Vector6d predicted_velocity = Vector6d::Zero();

  FeedbackTank(double initial, double minimum_energy, double maximum_energy, double damping_band,
               double linear_damping_max, double angular_damping_max, double motion_smoothing, bool enable)
      : energy(initial), minimum(minimum_energy), maximum(maximum_energy), band(damping_band),
        maximum_linear_damping(linear_damping_max), maximum_angular_damping(angular_damping_max),
        smoothing(motion_smoothing), enabled(enable) {}

  // Books the energy exchanged over the last cycle: sent = wrench sent then, damping = its pure damping part,
  // displacement = VIRTUOSE motion since then.
  void account(const Vector6d& sent, const Vector6d& damping, const Vector6d& displacement) {
    const double exchanged = (sent - damping).dot(displacement);
    if (!std::isfinite(exchanged)) {
      return;
    }
    energy -= exchanged;
    if (exchanged > 0.0) {
      delivered += exchanged;
    } else {
      received -= exchanged;
    }
    if (energy > maximum) {
      dissipated += energy - maximum;
      energy = maximum;
    }
  }

  // Stage 1 damping on velocity (the VIRTUOSE's velocity signal, 6D: m/s, rad/s), then stage 2: reduces the wrench
  // along the predicted motion (smoothed displacement / dt) so that its predicted power without the baseline damping
  // part fits in the tank over dt. Force and torque are reduced in proportion to the power each would deliver; a
  // part is never turned past zero work.
  Vector6d limit(const Vector6d& wrench, const Vector6d& damping, const Vector6d& velocity,
                 const Vector6d& displacement, double dt) {
    limited = false;
    linear_damping = angular_damping = 0.0;
    if (!enabled || !(dt > 0.0) || !velocity.allFinite() || !displacement.allFinite()) {
      return wrench;
    }
    predicted_velocity += smoothing * (displacement / dt - predicted_velocity);
    const double level = band > 0.0 ? std::clamp((minimum + band - energy) / band, 0.0, 1.0) : 0.0;
    linear_damping = level * maximum_linear_damping;
    angular_damping = level * maximum_angular_damping;
    Vector6d damped = wrench;
    damped.head<3>() -= linear_damping * velocity.head<3>();
    damped.tail<3>() -= angular_damping * velocity.tail<3>();
    const Vector6d active = damped - damping;
    const double power_force = active.head<3>().dot(predicted_velocity.head<3>());
    const double power_torque = active.tail<3>().dot(predicted_velocity.tail<3>());
    const double power = power_force + power_torque;
    const double allowed = std::max(energy - minimum, 0.0) / dt;
    if (!(power > allowed)) {
      return damped;
    }
    const double excess = power - allowed;
    const double positive = std::max(power_force, 0.0) + std::max(power_torque, 0.0);
    Vector6d limited_wrench = damped;
    const auto reduce = [&](int start, double part_power) {
      const Eigen::Vector3d v = predicted_velocity.segment<3>(start);
      const double v_sq = v.squaredNorm();
      if (part_power <= 0.0 || v_sq < 1e-12) {
        return;
      }
      const double cut = std::min(excess * part_power / positive, part_power);
      limited_wrench.segment<3>(start) -= (cut / v_sq) * v;
    };
    reduce(0, power_force);
    reduce(3, power_torque);
    removed += excess * dt;
    limited = true;
    return limited_wrench;
  }
};
