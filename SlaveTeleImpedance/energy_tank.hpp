#pragma once

// Energy tank for the slave's variable Cartesian stiffness (tank-based variable impedance, Ferraguti, Secchi and
// Fantuzzi, ICRA 2013).
//
// The slave's impedance stores the potential energy
//   V = 1/2 Kt |e|^2 + 2 Kg (1 - w)
// (translation: spring Kt on the position error e; rotation: the quaternion law tau = -Kg vec(q_err), whose
// potential is 2 Kg (1 - w), w = |scalar part of q_err|). Raising a stiffness while the robot is away from its
// reference adds dV = 1/2 dKt |e|^2 + 2 dKg (1 - w) without any energy entering through the robot: the only way a
// variable stiffness can make the controller non-passive. The tank pays for it:
//   - the damping of the impedance fills the tank with the energy it dissipates, Dt |v|^2 + Dr |w_rel|^2, while the
//     tank is below its maximum;
//   - a stiffness change takes dV from the tank (a decrease, dV < 0, returns it); an increase that would empty the
//     tank below its minimum is scaled down so that the tank stops at the minimum.
// The storage V + T then grows by at most the energy that enters through the robot, as with a constant stiffness.
// With the tank disabled the stiffness is never limited and T is still integrated (it may go below the minimum),
// so the log shows the energy the stiffness changes injected.

#include <algorithm>
#include <cmath>

struct EnergyTank {
  double energy;
  double minimum;
  double maximum;
  bool enabled;
  double injected = 0.0;   // J, energy the stiffness changes took from the tank (net)
  double harvested = 0.0;  // J, damping energy put into the tank
  bool limited = false;    // the last update scaled a stiffness increase down

  EnergyTank(double initial, double minimum_energy, double maximum_energy, bool enable)
      : energy(initial), minimum(minimum_energy), maximum(maximum_energy), enabled(enable) {}

  // Damping power (W, >= 0) over one cycle.
  void harvest(double dissipated_power, double dt) {
    if (!(dissipated_power > 0.0) || !std::isfinite(dissipated_power) || energy >= maximum) {
      return;
    }
    const double add = std::min(dissipated_power * dt, maximum - energy);
    energy += add;
    harvested += add;
  }

  // Moves the stiffnesses (kt N/m, kg quaternion gain) from their applied values towards the targets as far as
  // the tank allows. position_error_sq = |e|^2 (m^2), rotation_potential = 2 (1 - w).
  void update(double kt_target, double kg_target, double position_error_sq, double rotation_potential,
              double* kt_applied, double* kg_applied) {
    double dkt = kt_target - *kt_applied;
    double dkg = kg_target - *kg_applied;
    double needed = 0.5 * dkt * position_error_sq + dkg * rotation_potential;
    limited = false;
    if (enabled && needed > 0.0 && energy - needed < minimum) {
      // scale only the increases: decreases return energy and are always allowed
      const double gain_t = dkt > 0.0 ? 0.5 * dkt * position_error_sq : 0.0;
      const double gain_r = dkg > 0.0 ? dkg * rotation_potential : 0.0;
      const double refund = needed - gain_t - gain_r;  // <= 0
      const double alpha = std::clamp((energy - minimum - refund) / std::max(gain_t + gain_r, 1e-12), 0.0, 1.0);
      if (dkt > 0.0) dkt *= alpha;
      if (dkg > 0.0) dkg *= alpha;
      needed = 0.5 * dkt * position_error_sq + dkg * rotation_potential;
      limited = true;
    }
    *kt_applied += dkt;
    *kg_applied += dkg;
    energy = std::min(energy - needed, maximum);
    injected += needed;
  }
};
