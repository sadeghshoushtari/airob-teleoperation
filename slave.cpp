// ============================================================
// slave_compensated_adaptive_retare_yonly.cpp
//
// FRANKA SLAVE
// 1 kHz FULL 6-DOF VARIABLE-IMPEDANCE TELEOPERATION
// PAPER-GUIDED WRENCH-ESTIMATOR BUILD:
//   - slave control law/gains remain at the stable 12/4 baseline
//   - Y-only rotational-reference diagnostic remains enabled
//   - backward wrench is no longer taken directly from O_F_ext_hat_K
//   - instead, tau_ext_hat_filtered is compensated using a no-contact
//     configuration/velocity disturbance model learned online with RLS
//   - the compensated joint residual is mapped to a K-frame Cartesian
//     wrench with a damped least-squares inverse of J_K^T
//   - a smooth vector deadband rejects the remaining low-level residual
//   - the joint-space RLS prediction is bounded by calibration data
//   - remaining rotational drift is removed by a 0.1 s force-gated adaptive re-tare
//   - rotational reflection is contact-gated using a filtered raw force norm
//     with latched hysteresis, dwell times, and a smooth ramp
//
// The calibration model is learned while the operator deliberately moves in
// free space.  There is NO timed automatic transition to bilateral control.
// Calibration freezes only after the operator RELEASES the deadman after the
// minimum accumulated calibration time.  The NEXT deadman press captures fresh
// forward/backward references and starts the bilateral test.  During calibration
// the feedback wrench is exactly zero.
//
// Raw Franka wrench, reconstructed wrench, model bias, corrected joint
// residual, and rotational contact-gate state are all logged for comparison.
//
// Forward path remains the validated baseline except for one
// isolated rotational tracking fix: rotational damping uses the
// velocity error to the exact accepted orientation-reference step.
//
// Startup safety addition:
//   teleoperation arms only after one deadman RELEASE has been
//   observed after startup; the next press captures fresh
//   relative master/slave references.
//
// MASTER -> SLAVE packet: 14 doubles
//
// [0]      packet id
// [1]      timestamp [us]
// [2..5]   qw qx qy qz
// [6..8]   x y z
// [9]      effective force reflection gain
// [10]     effective torque reflection gain
// [11]     alpha
// [12]     deadman
// [13]     emergency stop
//
// SLAVE -> MASTER feedback: 22 doubles
//
// [0]      packet id
// [1]      timestamp [us]
// [2..7]   Fx Fy Fz Mx My Mz
// [8..10]  Franka x y z
// [11..14] Franka qw qx qy qz
// [15..17] Franka vx vy vz
// [18..20] Franka wx wy wz
// [21]     estimator_ready (0 during free-space calibration, 1 afterwards)
//
// Forward:
//
// translation scale = 1:1
// rotation scale    = 1:1
//
// Workspace:
//
// translation = +/-0.20 m per axis
// rotation    = 60 deg relative
//
// Target-rate limits:
//
// 0.30 m/s
// 120 deg/s
//
// Variable translation:
//
// K = 300 + 400 alpha
// D = 45 * sqrt(K_applied / 500)
// |dK/dt| <= 200 N/m/s
//
// Diagonal rotational impedance structure (base-frame axes):
//
// Krot = diag(12, 12, 12) Nm/rad
// Drot = diag(4, 4, 4) Nms/rad
//
// Rotation-reference diagnostic: Y-only desired rotation.
//
// ============================================================

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"


namespace {

// ============================================================
// CONFIGURATION
// ============================================================


// ------------------------------------------------------------
// MATHEMATICAL CONSTANTS
// ------------------------------------------------------------

constexpr double kPi =
    3.14159265358979323846;


// ------------------------------------------------------------
// NETWORK
// ------------------------------------------------------------

constexpr int kCommandReceivePort =
    11056;

constexpr int kFeedbackSendPort =
    11055;

constexpr const char* kFeedbackDestinationIp =
    "127.0.0.1";

constexpr std::size_t kCommandPacketSize =
    14;

constexpr std::size_t kFeedbackPacketSize =
    22;


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
// VARIABLE TRANSLATIONAL IMPEDANCE
// ------------------------------------------------------------

constexpr double kMinimumTranslationalStiffness =
    300.0;

constexpr double kMaximumTranslationalStiffness =
    700.0;

constexpr double kBaselineTranslationalStiffness =
    500.0;

constexpr double kBaselineTranslationalDamping =
    45.0; // was 45

constexpr double kMaximumStiffnessRate =
    200.0;


// ------------------------------------------------------------
// DIAGONAL ROTATIONAL IMPEDANCE
// ------------------------------------------------------------
//
// Keep the anisotropic/axis-wise matrix structure so each axis can
// be tuned independently later.  For this diagnostic, however, all
// axes are deliberately returned to the known stable 12 / 4 values.
// ------------------------------------------------------------

constexpr double kRotationalStiffnessX =
    12.0;

constexpr double kRotationalStiffnessY =
    12.0;

constexpr double kRotationalStiffnessZ =
    12.0;

constexpr double kRotationalDampingX =
    4.0;

constexpr double kRotationalDampingY =
    4.0;

constexpr double kRotationalDampingZ =
    4.0;


// ------------------------------------------------------------
// ROTATION-REFERENCE DIAGNOSTIC MODE
// ------------------------------------------------------------
//
// Full3D reproduces the normal controller.
// XOnly/YOnly/ZOnly project the mapped master relative-rotation
// VECTOR onto one base-frame axis before constructing the target.
//
// IMPORTANT: this constrains only DESIRED MOTION.  The 3-axis
// Cartesian impedance remains active, so corrective torque about
// the two locked axes is still available to resist cross-axis drift.
// ------------------------------------------------------------

enum class RotationReferenceMode {
  Full3D,
  XOnly,
  YOnly,
  ZOnly
};

constexpr RotationReferenceMode kRotationReferenceMode =
    RotationReferenceMode::Full3D;


// ------------------------------------------------------------
// PAPER-GUIDED EXTERNAL-WRENCH ESTIMATOR
// ------------------------------------------------------------
//
// Motivation:
//   * Petrea/Bertoni/Oboe (IECON 2021): compensate disturbance torques
//     using no-contact data before using the Panda interaction-force estimate.
//   * Lee/Zelek/Mombaur (Sci Rep 2025): built-in Franka estimates drift as
//     configuration changes because model residuals are interpreted as contact.
//   * Lieftink (2020): use a threshold to reject low-level estimator offset/noise.
//
// Our software-only implementation learns a compact no-contact joint-torque
// residual model during an explicit free-space calibration phase.  The model
// uses bounded q/dq features, then reconstructs the wrench at the Franka
// stiffness frame K from the corrected joint residual.
// ------------------------------------------------------------

constexpr double kEstimatorMinimumCalibrationActiveSeconds =
    8.0;

constexpr int kEstimatorFeatureCount =
    29;  // 1 + 7 sin(q) + 7 cos(q) + 7 dq + 7 tanh(dq/vs)

constexpr double kEstimatorRlsForgettingFactor =
    1.0;  // batch-like no-contact fit; no recency bias during calibration

constexpr double kEstimatorRlsInitialCovariance =
    100.0;

constexpr double kEstimatorFrictionVelocityScaleRadps =
    0.05;

// Calibration samples are accepted only while the built-in raw wrench remains
// in a plausible free-space range.  This guard prevents an accidental contact
// from being learned as robot/model bias.  Calibration time accumulates only
// for accepted samples.
constexpr double kEstimatorCalibrationMaxRawForceNormN =
    5.0;

constexpr double kEstimatorCalibrationMaxRawTorqueNormNm =
    2.0;

constexpr double kEstimatorDlsLambda =
    0.03;

// Smooth vector deadbands.  They remove the residual noise floor without
// introducing a discontinuous on/off wrench step.
constexpr double kEstimatorForceDeadbandN =
    0.75;

constexpr double kEstimatorTorqueDeadbandNm =
    0.12;

// ------------------------------------------------------------
// ROTATIONAL CONTACT GATE
// ------------------------------------------------------------
//
// The compensated torque estimate is useful in contact but remains too dirty
// in free space.  Force feedback is NOT gated here.  Only the rotational
// torque sent to the master is gated.  Contact detection uses the RAW Franka
// force estimate because the previous log cleanly separated free motion from
// contact with this signal.
//
// Gate ON  : filtered raw |F| > 6 N continuously for 30 ms
// Gate OFF : filtered raw |F| < 3 N continuously for 120 ms
// Output   : torque gate ramps linearly over 80 ms
//
// The wider hysteresis and longer release dwell keep sustained contact latched
// through brief force dips instead of repeatedly switching the haptic torque.

constexpr double kTorqueContactGateOnForceN =
    6.0;

constexpr double kTorqueContactGateOffForceN =
    3.0;

constexpr double kTorqueContactGateOnDwellS =
    0.030;

constexpr double kTorqueContactGateOffDwellS =
    0.120;

constexpr double kTorqueContactGateRampS =
    0.080;

// Low-pass only the scalar force norm used for contact detection.  This is
// deliberately separate from the haptic force channel.
constexpr double kTorqueContactGateForceFilterTauS =
    0.020;

// Claude's force-gated adaptive re-tare: track the remaining reconstructed
// rotational offset quickly in free space and freeze it in contact.
constexpr double kAdaptiveRetareFreeForceNormN =
    2.0;

constexpr double kAdaptiveRetareTauS =
    0.10;

// Prevent the learned joint-space model from extrapolating beyond the
// residual range actually observed during the free-space calibration sweep.
constexpr double kJointBiasClampRangeFraction =
    0.10;

constexpr double kJointBiasClampMarginNm =
    0.05;

// Numerical safety only. Normal haptic saturation still happens on the master.
constexpr double kEstimatorMaxForceNormN =
    100.0;

constexpr double kEstimatorMaxTorqueNormNm =
    2.5;


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
    8.0;

constexpr double kNullspaceDamping =
    2.5;


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
// MASTER -> FRANKA FRAME MAP
//
// Experimentally verified identity mapping.
// ------------------------------------------------------------

constexpr std::array<double, 9>
    kVirtuoseToFranka = {

        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};


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
//
// Joint arrays contain 7 entries.
// Cartesian wrench arrays contain 6 entries.
//
// This matches the installed libfranka 4-argument overload:
//
// setCollisionBehavior(
//     array<double,7>,
//     array<double,7>,
//     array<double,6>,
//     array<double,6>)
// ------------------------------------------------------------

constexpr double kCollisionThreshold =
    35.0;


// ------------------------------------------------------------
// LOGGING
// ------------------------------------------------------------

constexpr std::size_t kLogQueueCapacity =
    65536;

constexpr uint64_t kLogFlushRows =
    1000;


// ============================================================
// STOP FLAG
// ============================================================

volatile std::sig_atomic_t
    g_stop_requested = 0;


void sigint_handler(int) {

  g_stop_requested =
      1;
}


// ============================================================
// HELPERS
// ============================================================

uint64_t now_ns() {

  return static_cast<uint64_t>(

      std::chrono::duration_cast<
          std::chrono::nanoseconds>(

          std::chrono::steady_clock::now()
              .time_since_epoch())

          .count());
}


double clamp_scalar(
    double x,
    double lo,
    double hi) {

  return std::max(
      lo,
      std::min(
          x,
          hi));
}


bool finite_vector3(
    const Eigen::Vector3d& v) {

  return

      std::isfinite(v.x()) &&
      std::isfinite(v.y()) &&
      std::isfinite(v.z());
}


bool finite_quaternion(
    const Eigen::Quaterniond& q) {

  return

      std::isfinite(q.w()) &&
      std::isfinite(q.x()) &&
      std::isfinite(q.y()) &&
      std::isfinite(q.z()) &&

      q.squaredNorm() >
          1e-10;
}


std::string make_timestamp() {

  const std::time_t now =
      std::time(nullptr);

  std::tm tm{};

  localtime_r(
      &now,
      &tm);

  char buffer[32]{};

  std::strftime(
      buffer,
      sizeof(buffer),
      "%Y%m%d_%H%M%S",
      &tm);

  return std::string(
      buffer);
}


Eigen::Vector3d soft_vector_deadband(
    const Eigen::Vector3d& value,
    double threshold) {

  const double magnitude = value.norm();

  if (!std::isfinite(magnitude) ||
      magnitude <= threshold ||
      magnitude <= 1e-12) {
    return Eigen::Vector3d::Zero();
  }

  return (1.0 - threshold / magnitude) * value;
}


Eigen::Vector3d clamp_vector_norm(
    const Eigen::Vector3d& value,
    double maximum_norm) {

  const double magnitude = value.norm();

  if (!std::isfinite(magnitude)) {
    return Eigen::Vector3d::Zero();
  }

  if (magnitude <= maximum_norm ||
      magnitude <= 1e-12) {
    return value;
  }

  return value * (maximum_norm / magnitude);
}


Eigen::Matrix<double, kEstimatorFeatureCount, 1>
make_estimator_features(
    const Eigen::Matrix<double, 7, 1>& q,
    const Eigen::Matrix<double, 7, 1>& dq) {

  Eigen::Matrix<double, kEstimatorFeatureCount, 1> phi;
  phi.setZero();

  int index = 0;
  phi[index++] = 1.0;

  for (int i = 0; i < 7; ++i) {
    phi[index++] = std::sin(q[i]);
  }

  for (int i = 0; i < 7; ++i) {
    phi[index++] = std::cos(q[i]);
  }

  for (int i = 0; i < 7; ++i) {
    phi[index++] = dq[i];
  }

  for (int i = 0; i < 7; ++i) {
    phi[index++] = std::tanh(
        dq[i] / kEstimatorFrictionVelocityScaleRadps);
  }

  return phi;
}


Eigen::Vector3d clamp_components(
    const Eigen::Vector3d& value,
    double limit) {

  return Eigen::Vector3d(

      clamp_scalar(
          value.x(),
          -limit,
          limit),

      clamp_scalar(
          value.y(),
          -limit,
          limit),

      clamp_scalar(
          value.z(),
          -limit,
          limit));
}


Eigen::Matrix3d create_frame_map() {

  Eigen::Matrix3d R;

  R <<
      kVirtuoseToFranka[0],
      kVirtuoseToFranka[1],
      kVirtuoseToFranka[2],

      kVirtuoseToFranka[3],
      kVirtuoseToFranka[4],
      kVirtuoseToFranka[5],

      kVirtuoseToFranka[6],
      kVirtuoseToFranka[7],
      kVirtuoseToFranka[8];

  return R;
}


// ============================================================
// RELATIVE ROTATION LIMIT
// ============================================================

Eigen::Matrix3d limit_relative_rotation(
    const Eigen::Matrix3d& rotation,
    double maximum_angle_rad,
    bool* limited) {

  if (limited) {

    *limited =
        false;
  }


  Eigen::AngleAxisd aa(
      rotation);


  if (!std::isfinite(
          aa.angle()) ||

      aa.angle() <
          1e-12) {

    return Eigen::Matrix3d::
        Identity();
  }


  double angle =
      aa.angle();


  if (angle >
      maximum_angle_rad) {

    angle =
        maximum_angle_rad;


    if (limited) {

      *limited =
          true;
    }
  }


  return Eigen::AngleAxisd(
             angle,
             aa.axis())
      .toRotationMatrix();
}


// ============================================================
// ANGULAR TARGET RATE LIMIT
// ============================================================

Eigen::Quaterniond step_quaternion_toward(
    const Eigen::Quaterniond& current_input,
    const Eigen::Quaterniond& target_input,
    double maximum_step_rad,
    bool* limited) {

  if (limited) {

    *limited =
        false;
  }


  Eigen::Quaterniond current =
      current_input.normalized();


  Eigen::Quaterniond target =
      target_input.normalized();


  if (current.coeffs().dot(
          target.coeffs()) <
      0.0) {

    target.coeffs() *=
        -1.0;
  }


  Eigen::Quaterniond relative =

      current.conjugate() *
      target;


  relative.normalize();


  Eigen::AngleAxisd aa(
      relative);


  if (!std::isfinite(
          aa.angle()) ||

      aa.angle() <
          1e-12) {

    return target;
  }


  double step_angle =
      aa.angle();


  if (step_angle >
      maximum_step_rad) {

    step_angle =
        maximum_step_rad;


    if (limited) {

      *limited =
          true;
    }
  }


  Eigen::Quaterniond result =

      current *

      Eigen::Quaterniond(

          Eigen::AngleAxisd(
              step_angle,
              aa.axis()));


  result.normalize();


  return result;
}


// ============================================================
// TORQUE RATE LIMIT
// ============================================================

std::array<double, 7>
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


// ============================================================
// LOCK-FREE LOGGER
// ============================================================

template <
    typename T,
    std::size_t Capacity>
class SpscRing {

 public:

  bool push(
      const T& value) {

    const std::size_t head =

        head_.load(
            std::memory_order_relaxed);


    const std::size_t next =

        (head + 1) %
        Capacity;


    if (next ==

        tail_.load(
            std::memory_order_acquire)) {

      return false;
    }


    data_[head] =
        value;


    head_.store(
        next,
        std::memory_order_release);


    return true;
  }


  bool pop(
      T& value) {

    const std::size_t tail =

        tail_.load(
            std::memory_order_relaxed);


    if (tail ==

        head_.load(
            std::memory_order_acquire)) {

      return false;
    }


    value =
        data_[tail];


    tail_.store(

        (tail + 1) %
            Capacity,

        std::memory_order_release);


    return true;
  }


  bool empty() const {

    return

        tail_.load(
            std::memory_order_acquire) ==

        head_.load(
            std::memory_order_acquire);
  }


 private:

  std::array<
      T,
      Capacity>
      data_{};


  std::atomic<std::size_t>
      head_{0};


  std::atomic<std::size_t>
      tail_{0};
};


// ============================================================
// SLAVE LOG SAMPLE
// ============================================================

struct SlaveLogSample {

  double time_s;
  double callback_dt_ms;

  double master_time_s;

  uint64_t master_packet_id;

  double packet_age_ms;

  int active;

  double alpha_received;

  double k_requested;
  double k_applied;
  double d_applied;

  double k_rot_x;
  double k_rot_y;
  double k_rot_z;
  double d_rot_x;
  double d_rot_y;
  double d_rot_z;

  double actual_x;
  double actual_y;
  double actual_z;

  double desired_x;
  double desired_y;
  double desired_z;

  double error_x;
  double error_y;
  double error_z;

  double actual_qw;
  double actual_qx;
  double actual_qy;
  double actual_qz;

  double desired_qw;
  double desired_qx;
  double desired_qy;
  double desired_qz;

  double orientation_error_x;
  double orientation_error_y;
  double orientation_error_z;

  double vx;
  double vy;
  double vz;

  double wx;
  double wy;
  double wz;

  double desired_wx;
  double desired_wy;
  double desired_wz;

  double fx;
  double fy;
  double fz;

  double mx;
  double my;
  double mz;

  double g_force;
  double g_torque;

  int deadman;
  int estop;

  int translation_workspace_limited;
  int rotation_workspace_limited;

  int linear_speed_limited;
  int angular_speed_limited;

  // ========================================================
  // DIAGNOSTIC-ONLY JOINT / CONTROL DECOMPOSITION
  //
  // None of these fields participates in the control law.
  // The full 6x7 Jacobian is logged so singular values and
  // condition number can be computed OFFLINE, avoiding SVD
  // work inside the 1 kHz real-time callback.
  // ========================================================

  double q[7]{};
  double dq[7]{};

  double jacobian[42]{};  // row-major J(r,c): index = 7*r + c

  // Joint-space inertia matrix M(q), logged row-major as M(r,c).
  // This is diagnostic only; operational-space inertia is computed offline.
  double mass_matrix[49]{};

  double task_wrench[6]{};

  double task_torque[7]{};
  double nullspace_torque[7]{};
  double coriolis[7]{};

  double tau_J[7]{};
  double tau_J_d[7]{};
  double tau_ext_hat_filtered[7]{};
  double commanded_torque[7]{};

  // Paper-guided wrench-estimator diagnostics.
  int estimator_ready{};
  double estimator_calibration_active_s{};
  double raw_franka_wrench[6]{};
  double reconstructed_wrench[6]{};
  double feedback_wrench[6]{};
  double estimated_joint_bias[7]{};
  double corrected_joint_external_torque[7]{};

  // Rotational contact-gate diagnostics.
  int torque_contact_gate_active{};
  double torque_contact_gate_scale{};
  double torque_contact_gate_on_dwell_s{};
  double torque_contact_gate_off_dwell_s{};
  double raw_force_norm{};
  double filtered_raw_force_norm{};
  double adaptive_retare_bias[3]{};
  int adaptive_retare_updating{};
};


}  // namespace


// ============================================================
// MAIN
// ============================================================

int main(
    int argc,
    char** argv) {

  if (argc != 2) {

    std::cerr
        << "Usage: "
        << argv[0]
        << " <franka-ip>\n";

    return 1;
  }


  std::signal(
      SIGINT,
      sigint_handler);


  const std::string robot_ip =
      argv[1];


  const Eigen::Matrix3d v_to_f =
      create_frame_map();


  // ==========================================================
  // LOG FILE
  // ==========================================================

  const std::string log_name =

      "slave_paper_compensated_wrench_" +
      make_timestamp() +
      ".csv";


  std::ofstream csv(
      log_name);


  if (!csv.is_open()) {

    std::cerr
        << "ERROR: cannot open "
        << log_name
        << '\n';

    return 1;
  }


  csv
      << "time_s,"
      << "callback_dt_ms,"
      << "master_time_s,"
      << "master_packet_id,"
      << "packet_age_ms,"
      << "active,"

      << "alpha_received,"
      << "k_requested,"
      << "k_applied,"
      << "d_applied,"
      << "k_rot_x,"
      << "k_rot_y,"
      << "k_rot_z,"
      << "d_rot_x,"
      << "d_rot_y,"
      << "d_rot_z,"

      << "actual_x,"
      << "actual_y,"
      << "actual_z,"

      << "desired_x,"
      << "desired_y,"
      << "desired_z,"

      << "error_x,"
      << "error_y,"
      << "error_z,"

      << "actual_qw,"
      << "actual_qx,"
      << "actual_qy,"
      << "actual_qz,"

      << "desired_qw,"
      << "desired_qx,"
      << "desired_qy,"
      << "desired_qz,"

      << "orientation_error_x,"
      << "orientation_error_y,"
      << "orientation_error_z,"

      << "vx,"
      << "vy,"
      << "vz,"
      << "wx,"
      << "wy,"
      << "wz,"

      << "desired_wx,"
      << "desired_wy,"
      << "desired_wz,"

      << "fx,"
      << "fy,"
      << "fz,"
      << "mx,"
      << "my,"
      << "mz,"

      << "g_force,"
      << "g_torque,"
      << "deadman,"
      << "estop,"

      << "translation_workspace_limited,"
      << "rotation_workspace_limited,"
      << "linear_speed_limited,"
      << "angular_speed_limited,"

      << "q1,q2,q3,q4,q5,q6,q7,"
      << "dq1,dq2,dq3,dq4,dq5,dq6,dq7,"

      << "J00,J01,J02,J03,J04,J05,J06,"
      << "J10,J11,J12,J13,J14,J15,J16,"
      << "J20,J21,J22,J23,J24,J25,J26,"
      << "J30,J31,J32,J33,J34,J35,J36,"
      << "J40,J41,J42,J43,J44,J45,J46,"
      << "J50,J51,J52,J53,J54,J55,J56,"

      << "M00,M01,M02,M03,M04,M05,M06,"
      << "M10,M11,M12,M13,M14,M15,M16,"
      << "M20,M21,M22,M23,M24,M25,M26,"
      << "M30,M31,M32,M33,M34,M35,M36,"
      << "M40,M41,M42,M43,M44,M45,M46,"
      << "M50,M51,M52,M53,M54,M55,M56,"
      << "M60,M61,M62,M63,M64,M65,M66,"

      << "task_fx,task_fy,task_fz,task_mx,task_my,task_mz,"

      << "task_tau1,task_tau2,task_tau3,task_tau4,task_tau5,task_tau6,task_tau7,"
      << "null_tau1,null_tau2,null_tau3,null_tau4,null_tau5,null_tau6,null_tau7,"
      << "coriolis1,coriolis2,coriolis3,coriolis4,coriolis5,coriolis6,coriolis7,"
      << "tau_J1,tau_J2,tau_J3,tau_J4,tau_J5,tau_J6,tau_J7,"
      << "tau_J_d1,tau_J_d2,tau_J_d3,tau_J_d4,tau_J_d5,tau_J_d6,tau_J_d7,"
      << "tau_ext1,tau_ext2,tau_ext3,tau_ext4,tau_ext5,tau_ext6,tau_ext7,"
      << "estimator_ready,estimator_calibration_active_s,"
      << "raw_fx,raw_fy,raw_fz,raw_mx,raw_my,raw_mz,"
      << "recon_fx,recon_fy,recon_fz,recon_mx,recon_my,recon_mz,"
      << "feedback_fx,feedback_fy,feedback_fz,feedback_mx,feedback_my,feedback_mz,"
      << "bias_tau1,bias_tau2,bias_tau3,bias_tau4,bias_tau5,bias_tau6,bias_tau7,"
      << "corrected_tau1,corrected_tau2,corrected_tau3,corrected_tau4,corrected_tau5,corrected_tau6,corrected_tau7,"
      << "torque_contact_gate_active,torque_contact_gate_scale,"
      << "torque_contact_gate_on_dwell_s,torque_contact_gate_off_dwell_s,raw_force_norm,"
      << "filtered_raw_force_norm,retare_bias_mx,retare_bias_my,retare_bias_mz,adaptive_retare_updating,"
      << "cmd_tau1,cmd_tau2,cmd_tau3,cmd_tau4,cmd_tau5,cmd_tau6,cmd_tau7\n";


  csv
      << std::fixed
      << std::setprecision(9);


  static SpscRing<
      SlaveLogSample,
      kLogQueueCapacity>
      log_queue;


  std::atomic<bool>
      logger_stop{
          false};


  std::atomic<uint64_t>
      dropped_samples{
          0};


  std::thread logger_thread(

      [&]() {

        SlaveLogSample s{};

        uint64_t rows_since_flush =
            0;


        while (

            !logger_stop.load(
                std::memory_order_relaxed) ||

            !log_queue.empty()) {


          bool wrote =
              false;


          while (
              log_queue.pop(
                  s)) {

            wrote =
                true;


            csv
                << s.time_s << ','
                << s.callback_dt_ms << ','
                << s.master_time_s << ','
                << s.master_packet_id << ','
                << s.packet_age_ms << ','
                << s.active << ','

                << s.alpha_received << ','
                << s.k_requested << ','
                << s.k_applied << ','
                << s.d_applied << ','
                << s.k_rot_x << ','
                << s.k_rot_y << ','
                << s.k_rot_z << ','
                << s.d_rot_x << ','
                << s.d_rot_y << ','
                << s.d_rot_z << ','

                << s.actual_x << ','
                << s.actual_y << ','
                << s.actual_z << ','

                << s.desired_x << ','
                << s.desired_y << ','
                << s.desired_z << ','

                << s.error_x << ','
                << s.error_y << ','
                << s.error_z << ','

                << s.actual_qw << ','
                << s.actual_qx << ','
                << s.actual_qy << ','
                << s.actual_qz << ','

                << s.desired_qw << ','
                << s.desired_qx << ','
                << s.desired_qy << ','
                << s.desired_qz << ','

                << s.orientation_error_x << ','
                << s.orientation_error_y << ','
                << s.orientation_error_z << ','

                << s.vx << ','
                << s.vy << ','
                << s.vz << ','

                << s.wx << ','
                << s.wy << ','
                << s.wz << ','

                << s.desired_wx << ','
                << s.desired_wy << ','
                << s.desired_wz << ','

                << s.fx << ','
                << s.fy << ','
                << s.fz << ','

                << s.mx << ','
                << s.my << ','
                << s.mz << ','

                << s.g_force << ','
                << s.g_torque << ','
                << s.deadman << ','
                << s.estop << ','

                << s.translation_workspace_limited << ','
                << s.rotation_workspace_limited << ','
                << s.linear_speed_limited << ','
                << s.angular_speed_limited << ',';

            for (int i = 0; i < 7; ++i) csv << s.q[i] << ',';
            for (int i = 0; i < 7; ++i) csv << s.dq[i] << ',';
            for (int i = 0; i < 42; ++i) csv << s.jacobian[i] << ',';
            for (int i = 0; i < 49; ++i) csv << s.mass_matrix[i] << ',';
            for (int i = 0; i < 6; ++i) csv << s.task_wrench[i] << ',';
            for (int i = 0; i < 7; ++i) csv << s.task_torque[i] << ',';
            for (int i = 0; i < 7; ++i) csv << s.nullspace_torque[i] << ',';
            for (int i = 0; i < 7; ++i) csv << s.coriolis[i] << ',';
            for (int i = 0; i < 7; ++i) csv << s.tau_J[i] << ',';
            for (int i = 0; i < 7; ++i) csv << s.tau_J_d[i] << ',';
            for (int i = 0; i < 7; ++i) csv << s.tau_ext_hat_filtered[i] << ',';

            csv << s.estimator_ready << ','
                << s.estimator_calibration_active_s << ',';

            for (int i = 0; i < 6; ++i) csv << s.raw_franka_wrench[i] << ',';
            for (int i = 0; i < 6; ++i) csv << s.reconstructed_wrench[i] << ',';
            for (int i = 0; i < 6; ++i) csv << s.feedback_wrench[i] << ',';
            for (int i = 0; i < 7; ++i) csv << s.estimated_joint_bias[i] << ',';
            for (int i = 0; i < 7; ++i) csv << s.corrected_joint_external_torque[i] << ',';

            csv << s.torque_contact_gate_active << ','
                << s.torque_contact_gate_scale << ','
                << s.torque_contact_gate_on_dwell_s << ','
                << s.torque_contact_gate_off_dwell_s << ','
                << s.raw_force_norm << ','
                << s.filtered_raw_force_norm << ','
                << s.adaptive_retare_bias[0] << ','
                << s.adaptive_retare_bias[1] << ','
                << s.adaptive_retare_bias[2] << ','
                << s.adaptive_retare_updating << ',';

            // Last diagnostic block has no trailing comma.
            for (int i = 0; i < 7; ++i) {
              csv << s.commanded_torque[i];
              csv << (i == 6 ? '\n' : ',');
            }


            ++rows_since_flush;


            if (rows_since_flush >=
                kLogFlushRows) {

              csv.flush();

              rows_since_flush =
                  0;
            }
          }


          if (!wrote) {

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    1));
          }
        }


        csv.flush();
      });


  // ==========================================================
  // SHARED UDP STATE
  // ==========================================================

  std::array<
      std::atomic<double>,
      kCommandPacketSize>
      master_packet{};


  std::array<
      std::atomic<double>,
      kFeedbackPacketSize>
      feedback_packet{};


  for (auto& value :
       master_packet) {

    value.store(
        0.0,
        std::memory_order_relaxed);
  }


  for (auto& value :
       feedback_packet) {

    value.store(
        0.0,
        std::memory_order_relaxed);
  }


  std::atomic<uint64_t>
      last_master_receive_ns{
          0};


  // ==========================================================
  // MASTER COMMAND SOCKET
  // ==========================================================

  int receive_socket =

      socket(
          AF_INET,
          SOCK_DGRAM,
          0);


  if (receive_socket < 0) {

    std::perror(
        "command socket");

    logger_stop.store(
        true);

    logger_thread.join();

    return 1;
  }


  int reuse =
      1;


  setsockopt(
      receive_socket,
      SOL_SOCKET,
      SO_REUSEADDR,
      &reuse,
      sizeof(reuse));


  sockaddr_in receive_address{};

  receive_address.sin_family =
      AF_INET;

  receive_address.sin_addr.s_addr =
      htonl(
          INADDR_ANY);

  receive_address.sin_port =
      htons(
          kCommandReceivePort);


  if (bind(
          receive_socket,
          reinterpret_cast<sockaddr*>(
              &receive_address),
          sizeof(receive_address)) <
      0) {

    std::perror(
        "command bind");

    close(
        receive_socket);

    logger_stop.store(
        true);

    logger_thread.join();

    return 1;
  }


  const int receive_flags =

      fcntl(
          receive_socket,
          F_GETFL,
          0);


  if (receive_flags < 0 ||

      fcntl(
          receive_socket,
          F_SETFL,
          receive_flags |
              O_NONBLOCK) <
          0) {

    std::perror(
        "command fcntl");

    close(
        receive_socket);

    logger_stop.store(
        true);

    logger_thread.join();

    return 1;
  }


  // ==========================================================
  // RECEIVE THREAD
  // ==========================================================

  std::thread receive_thread(

      [&]() {

        while (!g_stop_requested) {

          double buffer[
              kCommandPacketSize]{};


          const ssize_t received =

              recvfrom(
                  receive_socket,
                  reinterpret_cast<char*>(
                      buffer),
                  sizeof(buffer),
                  0,
                  nullptr,
                  nullptr);


          if (received ==
              static_cast<ssize_t>(
                  sizeof(buffer))) {


            for (std::size_t i = 0;
                 i < kCommandPacketSize;
                 ++i) {

              master_packet[i].store(
                  buffer[i],
                  std::memory_order_relaxed);
            }


            last_master_receive_ns.store(
                now_ns(),
                std::memory_order_relaxed);

          } else {

            std::this_thread::sleep_for(

                std::chrono::microseconds(
                    kReceiveIdleSleepUs));
          }
        }
      });


  // ==========================================================
  // FEEDBACK THREAD
  // ==========================================================

  std::thread feedback_thread(

      [&]() {

        int feedback_socket =

            socket(
                AF_INET,
                SOCK_DGRAM,
                0);


        if (feedback_socket < 0) {

          std::perror(
              "feedback socket");

          g_stop_requested =
              1;

          return;
        }


        sockaddr_in destination{};

        destination.sin_family =
            AF_INET;

        destination.sin_port =
            htons(
                kFeedbackSendPort);


        if (inet_pton(
                AF_INET,
                kFeedbackDestinationIp,
                &destination.sin_addr) !=
            1) {

          std::cerr
              << "ERROR: invalid feedback destination.\n";

          close(
              feedback_socket);

          g_stop_requested =
              1;

          return;
        }


        auto next_send =
            std::chrono::steady_clock::now();


        const auto send_period =
            std::chrono::microseconds(
                kFeedbackSendPeriodUs);


        while (!g_stop_requested) {

          next_send +=
              send_period;


          double packet[
              kFeedbackPacketSize]{};


          for (std::size_t i = 0;
               i < kFeedbackPacketSize;
               ++i) {

            packet[i] =
                feedback_packet[i].load(
                    std::memory_order_relaxed);
          }


          sendto(
              feedback_socket,
              packet,
              sizeof(packet),
              0,
              reinterpret_cast<sockaddr*>(
                  &destination),
              sizeof(destination));


          const auto now =
              std::chrono::steady_clock::now();


          if (now >
              next_send +
                  std::chrono::milliseconds(
                      5)) {

            next_send =
                now;
          }


          std::this_thread::sleep_until(
              next_send);
        }


        close(
            feedback_socket);
      });


  // ==========================================================
  // FRANKA
  // ==========================================================

  try {

    std::cerr
        << "[FRANKA] Connecting to "
        << robot_ip
        << "...\n";


    franka::Robot robot(
        robot_ip);


    std::cerr
        << "[FRANKA] Connected.\n";


    setDefaultBehavior(
        robot);


    // ========================================================
    // CONFIGURATION SUMMARY
    // ========================================================

    std::cout
        << "\n============================================\n"
        << "CONTROLLER CONFIGURATION: SLAVE\n"
        << "============================================\n"

        << "Control rate              : "
        << kControlRateHz
        << " Hz\n"

        << "Wrench estimator          : bounded RLS + DLS(J_K) + adaptive re-tare\n"
        << "Adaptive re-tare         : tau=" << kAdaptiveRetareTauS
        << " s, update when |Fraw|<" << kAdaptiveRetareFreeForceNormN << " N\n"
        << "Torque contact gate      : ON " << kTorqueContactGateOnForceN
        << " N / OFF " << kTorqueContactGateOffForceN << " N\n"
        << "Estimator calibration     : "
        << kEstimatorMinimumCalibrationActiveSeconds
        << " active free-space s\n"
        << "Estimator force deadband  : "
        << kEstimatorForceDeadbandN
        << " N\n"
        << "Estimator torque deadband : "
        << kEstimatorTorqueDeadbandNm
        << " Nm\n"

        << "Forward scale X/Y/Z       : "
        << kPositionScaleX
        << " / "
        << kPositionScaleY
        << " / "
        << kPositionScaleZ
        << "\n"

        << "Translation workspace     : +/-"
        << kMaxTranslationPerAxisM
        << " m / axis\n"

        << "Linear target-speed limit : "
        << kMaxDesiredLinearSpeedMps
        << " m/s\n"

        << "Rotation workspace        : "
        << kMaxRelativeRotationDeg
        << " deg\n"

        << "Angular target-speed limit: "
        << kMaxDesiredAngularSpeedDegps
        << " deg/s\n"

        << "K range                   : "
        << kMinimumTranslationalStiffness
        << " ... "
        << kMaximumTranslationalStiffness
        << " N/m\n"

        << "K baseline                : "
        << kBaselineTranslationalStiffness
        << " N/m\n"

        << "D baseline                : "
        << kBaselineTranslationalDamping
        << " Ns/m\n"

        << "K slew                    : "
        << kMaximumStiffnessRate
        << " N/m/s\n"

        << "Krot X/Y/Z                : "
        << kRotationalStiffnessX << " / "
        << kRotationalStiffnessY << " / "
        << kRotationalStiffnessZ
        << " Nm/rad\n"

        << "Drot X/Y/Z                : "
        << kRotationalDampingX << " / "
        << kRotationalDampingY << " / "
        << kRotationalDampingZ
        << " Nms/rad\n"

        << "Rotation reference mode    : "
        << (kRotationReferenceMode == RotationReferenceMode::Full3D ? "Full3D" :
            kRotationReferenceMode == RotationReferenceMode::XOnly ? "XOnly" :
            kRotationReferenceMode == RotationReferenceMode::YOnly ? "YOnly" :
                                                                      "ZOnly")
        << "\n"

        << "Nullspace K/D             : "
        << kNullspaceStiffness
        << " / "
        << kNullspaceDamping
        << "\n"

        << "Joint torque slew         : "
        << kMaxDeltaTorquePerCycleNm
        << " Nm/cycle\n"

        << "Feedback packet           : "
        << kFeedbackPacketSize
        << " doubles\n"

        << "Collision threshold       : "
        << kCollisionThreshold
        << "\n"

        << "============================================\n\n"

        << "WARNING: Franka will move to initial pose.\n"
        << "Keep emergency stop accessible.\n"
        << "Press Enter to continue..."
        << std::endl;


    std::cin.get();


    MotionGenerator motion_generator(

        kInitialMotionSpeedFactor,

        kInitialJointConfiguration);


    robot.control(
        motion_generator);


    std::cout
        << "\n[FRANKA] Initial pose reached.\n";


    franka::Model model =
        robot.loadModel();


    Eigen::Matrix<
        double,
        7,
        1>
        q_goal_eigen;


    for (int i = 0;
         i < 7;
         ++i) {

      q_goal_eigen[i] =

          kInitialJointConfiguration[
              static_cast<std::size_t>(
                  i)];
    }


    // ========================================================
    // COLLISION BEHAVIOR
    //
    // CORRECT FOR THIS INSTALLED LIBFRANKA:
    //
    // 7 joint torque thresholds
    // 7 joint torque thresholds
    // 6 Cartesian wrench thresholds
    // 6 Cartesian wrench thresholds
    // ========================================================

    robot.setCollisionBehavior(

        {{kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold}},

        {{kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold}},

        {{kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold}},

        {{kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold,
          kCollisionThreshold}});


    // ========================================================
    // IMPEDANCE MATRICES
    // ========================================================

    Eigen::Matrix<
        double,
        6,
        6>
        stiffness =

            Eigen::Matrix<
                double,
                6,
                6>::Zero();


    Eigen::Matrix<
        double,
        6,
        6>
        damping =

            Eigen::Matrix<
                double,
                6,
                6>::Zero();


    stiffness.topLeftCorner<
        3,
        3>() =

        kBaselineTranslationalStiffness *
        Eigen::Matrix3d::
            Identity();


    damping.topLeftCorner<
        3,
        3>() =

        kBaselineTranslationalDamping *
        Eigen::Matrix3d::
            Identity();


    stiffness.bottomRightCorner<
        3,
        3>() =

        (Eigen::Vector3d(
             kRotationalStiffnessX,
             kRotationalStiffnessY,
             kRotationalStiffnessZ))
            .asDiagonal();


    damping.bottomRightCorner<
        3,
        3>() =

        (Eigen::Vector3d(
             kRotationalDampingX,
             kRotationalDampingY,
             kRotationalDampingZ))
            .asDiagonal();


    const auto control_start =
        std::chrono::steady_clock::now();


    uint64_t feedback_id =
        0;


    // ========================================================
    // PAPER-GUIDED WRENCH ESTIMATOR STATE
    // ========================================================

    using EstimatorFeatureVector =
        Eigen::Matrix<double, kEstimatorFeatureCount, 1>;

    using EstimatorCovariance =
        Eigen::Matrix<double,
                      kEstimatorFeatureCount,
                      kEstimatorFeatureCount>;

    // Columns correspond to the 7 joint external-torque channels.
    Eigen::Matrix<double, kEstimatorFeatureCount, 7>
        estimator_weights =
            Eigen::Matrix<double, kEstimatorFeatureCount, 7>::Zero();

    EstimatorCovariance estimator_covariance =
        kEstimatorRlsInitialCovariance *
        EstimatorCovariance::Identity();

    double estimator_calibration_active_s =
        0.0;

    bool estimator_ready =
        false;

    Eigen::Matrix<double, 7, 1> calibration_tau_min =
        Eigen::Matrix<double, 7, 1>::Constant(
            std::numeric_limits<double>::infinity());

    Eigen::Matrix<double, 7, 1> calibration_tau_max =
        Eigen::Matrix<double, 7, 1>::Constant(
            -std::numeric_limits<double>::infinity());

    Eigen::Vector3d adaptive_retare_bias =
        Eigen::Vector3d::Zero();

    bool adaptive_retare_initialized =
        false;

    bool adaptive_retare_updating =
        false;

    double filtered_raw_force_norm =
        0.0;

    bool filtered_raw_force_norm_initialized =
        false;

    // Rotational contact-gate state.  This state is meaningful only after
    // calibration is frozen.  It is reset whenever the teleoperation is
    // inactive so every new activation starts with zero reflected torque.
    bool torque_contact_gate_active =
        false;

    double torque_contact_gate_scale =
        0.0;

    double torque_contact_gate_on_dwell_s =
        0.0;

    double torque_contact_gate_off_dwell_s =
        0.0;


    std::cout
        << "\n[WRENCH ESTIMATOR] First "
        << kEstimatorMinimumCalibrationActiveSeconds
        << " seconds of ACTIVE teleoperation are calibration.\n"
        << "[WRENCH ESTIMATOR] Keep the Franka in FREE SPACE and move "
           "through the intended rotational workspace.\n"
        << "[WRENCH ESTIMATOR] Calibration is operator-driven; estimator_ready stays 0 until deadman RELEASE after minimum calibration time.\n"
        << "[WRENCH ESTIMATOR] The matching master disables ALL backward haptics "
           "(pose sync + wrench reflection + directional damping).\n"
        << "[WRENCH ESTIMATOR] Forward teleoperation remains active: YOU move the "
           "Virtuose and the Franka follows in free space.\n\n";


    // ========================================================
    // FRANKA 1 kHz CALLBACK
    // ========================================================

    robot.control(

        [&](const franka::RobotState& state,
            franka::Duration period)
            -> franka::Torques {


          const double dt =

              std::max(
                  period.toSec(),
                  kMinimumCallbackDtS);


          // ==================================================
          // MODEL
          // ==================================================

          const std::array<double, 42>
              jacobian_array =

                  model.zeroJacobian(

                      franka::Frame::
                          kEndEffector,

                      state);


          const std::array<double, 42>
              stiffness_jacobian_array =

                  model.zeroJacobian(

                      franka::Frame::
                          kStiffness,

                      state);


          const std::array<double, 7>
              coriolis_array =

                  model.coriolis(
                      state);


          const std::array<double, 49>
              mass_array =

                  model.mass(
                      state);


          Eigen::Map<
              const Eigen::Matrix<
                  double,
                  6,
                  7>>
              jacobian(
                  jacobian_array.data());


          Eigen::Map<
              const Eigen::Matrix<
                  double,
                  6,
                  7>>
              stiffness_jacobian(
                  stiffness_jacobian_array.data());


          Eigen::Map<
              const Eigen::Matrix<
                  double,
                  7,
                  1>>
              coriolis(
                  coriolis_array.data());


          Eigen::Map<
              const Eigen::Matrix<
                  double,
                  7,
                  7>>
              mass_matrix(
                  mass_array.data());


          Eigen::Map<
              const Eigen::Matrix<
                  double,
                  7,
                  1>>
              q(
                  state.q.data());


          Eigen::Map<
              const Eigen::Matrix<
                  double,
                  7,
                  1>>
              dq(
                  state.dq.data());


          // ==================================================
          // CURRENT FRANKA POSE
          // ==================================================

          Eigen::Affine3d transform(

              Eigen::Matrix4d::Map(
                  state.O_T_EE.data()));


          const Eigen::Vector3d position =
              transform.translation();


          Eigen::Quaterniond orientation(
              transform.rotation());


          orientation.normalize();


          // ==================================================
          // CARTESIAN VELOCITY
          //
          // 0..2: linear
          // 3..5: angular
          // ==================================================

          const Eigen::Matrix<
              double,
              6,
              1>
              cartesian_velocity =

                  jacobian *
                  dq;


          // ==================================================
          // SLAVE -> MASTER FEEDBACK
          // ==================================================

          feedback_packet[0].store(

              static_cast<double>(
                  feedback_id++),

              std::memory_order_relaxed);


          const double local_time_s =

              std::chrono::duration<double>(

                  std::chrono::steady_clock::now() -
                  control_start)

                  .count();


          feedback_packet[1].store(

              local_time_s *
                  1e6,

              std::memory_order_relaxed);


          // --------------------------------------------------
          // External wrench
          // --------------------------------------------------
          // Packet entries [2..7] are populated later in this callback,
          // after the paper-guided estimator has processed tau_ext_hat_filtered.
          // Keep them at their previous atomic values until then.


          // --------------------------------------------------
          // Position
          // --------------------------------------------------

          feedback_packet[8].store(
              position.x(),
              std::memory_order_relaxed);

          feedback_packet[9].store(
              position.y(),
              std::memory_order_relaxed);

          feedback_packet[10].store(
              position.z(),
              std::memory_order_relaxed);


          // --------------------------------------------------
          // Quaternion
          // --------------------------------------------------

          feedback_packet[11].store(
              orientation.w(),
              std::memory_order_relaxed);

          feedback_packet[12].store(
              orientation.x(),
              std::memory_order_relaxed);

          feedback_packet[13].store(
              orientation.y(),
              std::memory_order_relaxed);

          feedback_packet[14].store(
              orientation.z(),
              std::memory_order_relaxed);


          // --------------------------------------------------
          // Cartesian linear velocity
          // --------------------------------------------------

          feedback_packet[15].store(
              cartesian_velocity[0],
              std::memory_order_relaxed);

          feedback_packet[16].store(
              cartesian_velocity[1],
              std::memory_order_relaxed);

          feedback_packet[17].store(
              cartesian_velocity[2],
              std::memory_order_relaxed);


          // --------------------------------------------------
          // Cartesian angular velocity
          // --------------------------------------------------

          feedback_packet[18].store(
              cartesian_velocity[3],
              std::memory_order_relaxed);

          feedback_packet[19].store(
              cartesian_velocity[4],
              std::memory_order_relaxed);

          feedback_packet[20].store(
              cartesian_velocity[5],
              std::memory_order_relaxed);


          // ==================================================
          // READ MASTER COMMAND
          // ==================================================

          double command[
              kCommandPacketSize]{};


          for (std::size_t i = 0;
               i < kCommandPacketSize;
               ++i) {

            command[i] =

                master_packet[i].load(
                    std::memory_order_relaxed);
          }


          // ==================================================
          // MASTER PACKET AGE
          //
          // Load receive timestamp first, then current time.
          // ==================================================

          const uint64_t last_receive =

              last_master_receive_ns.load(
                  std::memory_order_relaxed);


          const uint64_t current_ns =
              now_ns();


          const bool packet_live =

              last_receive != 0 &&

              current_ns >=
                  last_receive &&

              current_ns -
                      last_receive <
                  kMasterStaleNs;


          const double packet_age_ms =

              last_receive != 0 &&
                      current_ns >=
                          last_receive

                  ? static_cast<double>(
                        current_ns -
                        last_receive) /
                        1e6

                  : 1e9;


          const uint64_t master_packet_id =

              static_cast<uint64_t>(

                  std::max(
                      0.0,
                      command[0]));


          const double master_time_s =

              command[1] *
              1e-6;


          const double force_gain =
              command[9];


          const double torque_gain =
              command[10];


          const int deadman =

              static_cast<int>(
                  std::round(
                      command[12]));


          const int emergency_stop =

              static_cast<int>(
                  std::round(
                      command[13]));


          // ==================================================
          // ALPHA
          // ==================================================

          static double alpha_received =
              0.50;


          if (packet_live &&
              std::isfinite(
                  command[11])) {

            alpha_received =

                clamp_scalar(
                    command[11],
                    0.0,
                    1.0);
          }


          // ==================================================
          // VARIABLE TRANSLATIONAL IMPEDANCE
          // ==================================================

          const double k_requested =

              kMinimumTranslationalStiffness +

              alpha_received *

                  (
                      kMaximumTranslationalStiffness -
                      kMinimumTranslationalStiffness);


          static double k_applied =
              kBaselineTranslationalStiffness;


          const double max_k_step =

              kMaximumStiffnessRate *
              dt;


          k_applied +=

              clamp_scalar(

                  k_requested -
                      k_applied,

                  -max_k_step,

                  max_k_step);


          const double d_applied =

              kBaselineTranslationalDamping *

              std::sqrt(

                  k_applied /
                  kBaselineTranslationalStiffness);


          stiffness.topLeftCorner<
              3,
              3>() =

              k_applied *

              Eigen::Matrix3d::
                  Identity();


          damping.topLeftCorner<
              3,
              3>() =

              d_applied *

              Eigen::Matrix3d::
                  Identity();


          stiffness.bottomRightCorner<
              3,
              3>() =

              (Eigen::Vector3d(
                   kRotationalStiffnessX,
                   kRotationalStiffnessY,
                   kRotationalStiffnessZ))
                  .asDiagonal();


          damping.bottomRightCorner<
              3,
              3>() =

              (Eigen::Vector3d(
                   kRotationalDampingX,
                   kRotationalDampingY,
                   kRotationalDampingZ))
                  .asDiagonal();


          // ==================================================
          // MASTER POSE
          // ==================================================

          const Eigen::Vector3d
              master_position(

                  command[6],
                  command[7],
                  command[8]);


          Eigen::Quaterniond
              master_orientation(

                  command[2],
                  command[3],
                  command[4],
                  command[5]);


          const bool valid_master_pose =

              finite_vector3(
                  master_position) &&

              finite_quaternion(
                  master_orientation);


          if (valid_master_pose) {

            master_orientation.normalize();
          }


          // ==================================================
          // RELATIVE REFERENCES
          // ==================================================

          static bool initialized =
              false;


          static bool reference_ready =
              false;


          static bool previous_active =
              false;


          static bool deadman_release_seen =
              false;


          static Eigen::Vector3d
              desired_position;


          static Eigen::Vector3d
              franka_position_reference;


          static Eigen::Vector3d
              master_position_reference;


          static Eigen::Quaterniond
              desired_orientation;


          // Desired angular velocity of the *accepted* orientation
          // reference, expressed in the Franka base/world frame.
          // This is computed from the exact rate-limited quaternion
          // step below, not by differentiating network pose samples.
          static Eigen::Vector3d
              desired_angular_velocity =
                  Eigen::Vector3d::Zero();


          static Eigen::Quaterniond
              franka_orientation_reference;


          static Eigen::Quaterniond
              master_orientation_reference;


          if (!initialized) {

            desired_position =
                position;


            franka_position_reference =
                position;


            master_position_reference =
                master_position;


            desired_orientation =
                orientation;


            desired_angular_velocity.setZero();


            franka_orientation_reference =
                orientation;


            master_orientation_reference =

                valid_master_pose

                    ? master_orientation

                    : Eigen::Quaterniond::
                          Identity();


            initialized =
                true;
          }


          if (packet_live &&
              deadman == 0) {

            deadman_release_seen =
                true;
          }


          const bool active =

              packet_live &&

              deadman_release_seen &&

              valid_master_pose &&

              deadman ==
                  1 &&

              emergency_stop ==
                  1;


          // ==================================================
          // PAPER-GUIDED COMPENSATED WRENCH ESTIMATOR
          // ==================================================
          //
          // tau_ext_hat_filtered is a joint-space external-torque residual.
          // During the explicit initial free-space calibration interval, an
          // RLS model learns the repeatable configuration/velocity-dependent
          // residual (gravity/model/friction mismatch).  After calibration,
          // that model is frozen, subtracted, and the remaining joint residual
          // is mapped to a wrench at stiffness frame K using J_K.
          // ==================================================

          const EstimatorFeatureVector estimator_phi =
              make_estimator_features(q, dq);

          Eigen::Matrix<double, 7, 1> tau_ext_measured;

          for (int i = 0; i < 7; ++i) {
            tau_ext_measured[i] =
                state.tau_ext_hat_filtered[static_cast<std::size_t>(i)];
          }

          const Eigen::Vector3d raw_estimator_force(
              state.O_F_ext_hat_K[0],
              state.O_F_ext_hat_K[1],
              state.O_F_ext_hat_K[2]);

          const Eigen::Vector3d raw_estimator_torque(
              state.O_F_ext_hat_K[3],
              state.O_F_ext_hat_K[4],
              state.O_F_ext_hat_K[5]);

          const bool calibration_sample_accepted =
              raw_estimator_force.allFinite() &&
              raw_estimator_torque.allFinite() &&
              raw_estimator_force.norm() <
                  kEstimatorCalibrationMaxRawForceNormN &&
              raw_estimator_torque.norm() <
                  kEstimatorCalibrationMaxRawTorqueNormNm;

          if (active &&
              !estimator_ready &&
              calibration_sample_accepted) {

            for (int i = 0; i < 7; ++i) {
              calibration_tau_min[i] =
                  std::min(calibration_tau_min[i], tau_ext_measured[i]);
              calibration_tau_max[i] =
                  std::max(calibration_tau_max[i], tau_ext_measured[i]);
            }

            const Eigen::Matrix<double, 7, 1> predicted_before_update =
                estimator_weights.transpose() * estimator_phi;

            const Eigen::Matrix<double, 7, 1> prediction_error =
                tau_ext_measured - predicted_before_update;

            const EstimatorFeatureVector covariance_phi =
                estimator_covariance * estimator_phi;

            const double denominator =
                kEstimatorRlsForgettingFactor +
                estimator_phi.dot(covariance_phi);

            if (std::isfinite(denominator) &&
                denominator > 1e-12) {

              const EstimatorFeatureVector rls_gain =
                  covariance_phi / denominator;

              estimator_weights.noalias() +=
                  rls_gain * prediction_error.transpose();

              estimator_covariance =
                  (estimator_covariance -
                   rls_gain *
                       (estimator_phi.transpose() * estimator_covariance)) /
                  kEstimatorRlsForgettingFactor;

              // Keep numerical round-off from slowly destroying symmetry.
              estimator_covariance =
                  (0.5 *
                   (estimator_covariance + estimator_covariance.transpose())).eval();
            }

            estimator_calibration_active_s += dt;
          }

          // IMPORTANT: NEVER arm bilateral feedback while the deadman is held.
          // The operator deliberately ends calibration by releasing the deadman.
          // Because previous_active is still true in this cycle, this falling edge
          // is unambiguous.  If the minimum calibration time has not yet been
          // accumulated, the estimator simply remains in calibration mode and the
          // next press continues collecting free-space data.
          if (!active &&
              previous_active &&
              !estimator_ready &&
              estimator_calibration_active_s >=
                  kEstimatorMinimumCalibrationActiveSeconds) {
            estimator_ready = true;

            std::cerr
                << "\n[WRENCH ESTIMATOR CALIBRATION FROZEN]\n"
                << "Release detected after "
                << estimator_calibration_active_s
                << " active calibration seconds.\n"
                << "Next deadman press will start bilateral operation "
                << "with fresh references.\n";
          }

          Eigen::Matrix<double, 7, 1> estimated_joint_bias =
              estimator_weights.transpose() * estimator_phi;

          // Fail-safe against RLS extrapolation.  The predicted no-contact
          // residual for each joint is allowed only slightly outside the range
          // that was actually observed during calibration.
          if (estimator_ready) {
            for (int i = 0; i < 7; ++i) {
              if (std::isfinite(calibration_tau_min[i]) &&
                  std::isfinite(calibration_tau_max[i]) &&
                  calibration_tau_max[i] >= calibration_tau_min[i]) {
                const double observed_range =
                    calibration_tau_max[i] - calibration_tau_min[i];
                const double margin =
                    kJointBiasClampMarginNm +
                    kJointBiasClampRangeFraction * observed_range;
                estimated_joint_bias[i] =
                    clamp_scalar(estimated_joint_bias[i],
                                 calibration_tau_min[i] - margin,
                                 calibration_tau_max[i] + margin);
              }
            }
          }

          const Eigen::Matrix<double, 7, 1> corrected_joint_external_torque =
              tau_ext_measured - estimated_joint_bias;

          Eigen::Matrix<double, 6, 1> reconstructed_wrench =
              Eigen::Matrix<double, 6, 1>::Zero();

          if (estimator_ready) {

            Eigen::Matrix<double, 6, 6> dls_matrix =
                stiffness_jacobian * stiffness_jacobian.transpose();

            dls_matrix.diagonal().array() +=
                kEstimatorDlsLambda * kEstimatorDlsLambda;

            const Eigen::Matrix<double, 6, 1> rhs =
                stiffness_jacobian * corrected_joint_external_torque;

            reconstructed_wrench =
                dls_matrix.ldlt().solve(rhs);

            if (!reconstructed_wrench.allFinite()) {
              reconstructed_wrench.setZero();
            }
          }

          const Eigen::Vector3d reconstructed_force =
              reconstructed_wrench.head<3>();

          const Eigen::Vector3d reconstructed_torque =
              reconstructed_wrench.tail<3>();

          const double raw_force_norm =
              raw_estimator_force.norm();

          if (std::isfinite(raw_force_norm)) {
            if (!filtered_raw_force_norm_initialized) {
              filtered_raw_force_norm = raw_force_norm;
              filtered_raw_force_norm_initialized = true;
            } else {
              const double force_filter_alpha =
                  dt / (kTorqueContactGateForceFilterTauS + dt);
              filtered_raw_force_norm +=
                  force_filter_alpha *
                  (raw_force_norm - filtered_raw_force_norm);
            }
          }

          // Fast adaptive re-tare of the remaining CARTESIAN rotational offset.
          // It updates only in confidently free space and freezes immediately
          // as force rises.  This directly replaces the stale master startup
          // average while preserving genuine contact torque.
          adaptive_retare_updating =
              estimator_ready &&
              active &&
              reconstructed_torque.allFinite() &&
              filtered_raw_force_norm_initialized &&
              filtered_raw_force_norm < kAdaptiveRetareFreeForceNormN &&
              !torque_contact_gate_active;

          if (adaptive_retare_updating) {
            if (!adaptive_retare_initialized) {
              adaptive_retare_bias = reconstructed_torque;
              adaptive_retare_initialized = true;
            } else {
              const double retare_alpha =
                  dt / (kAdaptiveRetareTauS + dt);
              adaptive_retare_bias +=
                  retare_alpha *
                  (reconstructed_torque - adaptive_retare_bias);
            }
          }

          const Eigen::Vector3d retared_torque =
              adaptive_retare_initialized
                  ? reconstructed_torque - adaptive_retare_bias
                  : reconstructed_torque;

          Eigen::Vector3d estimator_force =
              soft_vector_deadband(
                  reconstructed_force,
                  kEstimatorForceDeadbandN);

          Eigen::Vector3d estimator_torque =
              soft_vector_deadband(
                  retared_torque,
                  kEstimatorTorqueDeadbandNm);

          // Bound the torque BEFORE the gate ramp.  Otherwise a very large
          // estimator excursion multiplied by a small ramp scale could still
          // hit the final limit immediately and defeat the purpose of the ramp.
          estimator_torque =
              clamp_vector_norm(
                  estimator_torque,
                  kEstimatorMaxTorqueNormNm);

          // ==================================================
          // ROTATIONAL CONTACT GATE
          // ==================================================
          //
          // The raw force norm is used only to decide whether rotational
          // reflection may pass.  It does not alter the slave controller and
          // it does not gate translational force feedback.
          if (!estimator_ready ||
              !active ||
              !filtered_raw_force_norm_initialized ||
              !std::isfinite(filtered_raw_force_norm)) {
            torque_contact_gate_active = false;
            torque_contact_gate_scale = 0.0;
            torque_contact_gate_on_dwell_s = 0.0;
            torque_contact_gate_off_dwell_s = 0.0;
          } else if (!torque_contact_gate_active) {
            torque_contact_gate_off_dwell_s = 0.0;

            if (filtered_raw_force_norm > kTorqueContactGateOnForceN) {
              torque_contact_gate_on_dwell_s += dt;

              if (torque_contact_gate_on_dwell_s >=
                  kTorqueContactGateOnDwellS) {
                torque_contact_gate_active = true;
                torque_contact_gate_on_dwell_s = 0.0;
              }
            } else {
              torque_contact_gate_on_dwell_s = 0.0;
            }
          } else {
            torque_contact_gate_on_dwell_s = 0.0;

            if (filtered_raw_force_norm < kTorqueContactGateOffForceN) {
              torque_contact_gate_off_dwell_s += dt;

              if (torque_contact_gate_off_dwell_s >=
                  kTorqueContactGateOffDwellS) {
                torque_contact_gate_active = false;
                torque_contact_gate_off_dwell_s = 0.0;
              }
            } else {
              torque_contact_gate_off_dwell_s = 0.0;
            }
          }

          // Smoothly ramp torque reflection rather than stepping it at the
          // contact threshold.  At 1 kHz and 80 ms this changes by ~0.0125/cycle.
          const double gate_target =
              torque_contact_gate_active ? 1.0 : 0.0;

          const double gate_step =
              (kTorqueContactGateRampS > 0.0)
                  ? dt / kTorqueContactGateRampS
                  : 1.0;

          if (torque_contact_gate_scale < gate_target) {
            torque_contact_gate_scale =
                std::min(gate_target,
                         torque_contact_gate_scale + gate_step);
          } else if (torque_contact_gate_scale > gate_target) {
            torque_contact_gate_scale =
                std::max(gate_target,
                         torque_contact_gate_scale - gate_step);
          }

          estimator_torque *=
              torque_contact_gate_scale;

          estimator_force =
              clamp_vector_norm(
                  estimator_force,
                  kEstimatorMaxForceNormN);

          Eigen::Matrix<double, 6, 1> feedback_wrench =
              Eigen::Matrix<double, 6, 1>::Zero();

          if (estimator_ready) {
            feedback_wrench.head<3>() = estimator_force;
            feedback_wrench.tail<3>() = estimator_torque;
          }

          for (int i = 0; i < 6; ++i) {
            feedback_packet[2 + static_cast<std::size_t>(i)].store(
                feedback_wrench[i],
                std::memory_order_relaxed);
          }


          // Tell the matching master whether it may enable ANY backward
          // haptic coupling. estimator_ready can become 1 ONLY on a deadman
          // release, never while the operator is actively moving.
          feedback_packet[21].store(
              estimator_ready ? 1.0 : 0.0,
              std::memory_order_relaxed);


          // ==================================================
          // FORWARD CLUTCH / REFERENCE CAPTURE
          // ==================================================

          if (active &&
              !previous_active) {

            desired_position =
                position;


            franka_position_reference =
                position;


            master_position_reference =
                master_position;


            desired_orientation =
                orientation;


            desired_angular_velocity.setZero();


            franka_orientation_reference =
                orientation;


            master_orientation_reference =
                master_orientation;


            reference_ready =
                true;
          }


          // ==================================================
          // LIMIT DIAGNOSTICS
          // ==================================================

          bool translation_workspace_limited =
              false;


          bool rotation_workspace_limited =
              false;


          bool linear_speed_limited =
              false;


          bool angular_speed_limited =
              false;


          // ==================================================
          // FORWARD TELEOPERATION
          // ==================================================

          // Translation damping is intentionally left exactly as in
          // the validated baseline. Only the rotational damping gets
          // a moving-reference angular velocity, computed from the
          // orientation step actually accepted this cycle.
          desired_angular_velocity.setZero();


          if (active &&
              reference_ready) {


            // =================================================
            // TRANSLATION
            // =================================================

            if (kEnableTranslationFollowing) {

              const Eigen::Vector3d
                  master_delta =

                      master_position -
                      master_position_reference;


              const Eigen::Vector3d
                  mapped_delta =

                      v_to_f *
                      master_delta;


              Eigen::Vector3d
                  raw_target_delta(

                      kPositionScaleX *
                          mapped_delta.x(),

                      kPositionScaleY *
                          mapped_delta.y(),

                      kPositionScaleZ *
                          mapped_delta.z());


              if (

                  std::abs(
                      raw_target_delta.x()) >
                      kMaxTranslationPerAxisM ||

                  std::abs(
                      raw_target_delta.y()) >
                      kMaxTranslationPerAxisM ||

                  std::abs(
                      raw_target_delta.z()) >
                      kMaxTranslationPerAxisM) {

                translation_workspace_limited =
                    true;
              }


              const Eigen::Vector3d
                  target_delta =

                      clamp_components(

                          raw_target_delta,

                          kMaxTranslationPerAxisM);


              const Eigen::Vector3d
                  target_position =

                      franka_position_reference +
                      target_delta;


              Eigen::Vector3d
                  position_step =

                      target_position -
                      desired_position;


              const double maximum_step =

                  kMaxDesiredLinearSpeedMps *
                  dt;


              const double step_norm =

                  position_step.norm();


              if (step_norm >
                      maximum_step &&
                  step_norm >
                      1e-12) {

                linear_speed_limited =
                    true;


                position_step *=

                    maximum_step /
                    step_norm;
              }


              desired_position +=
                  position_step;

            } else {

              desired_position =
                  franka_position_reference;
            }


            // =================================================
            // ROTATION
            // =================================================

            if (kEnableRotationFollowing) {

              const Eigen::Matrix3d
                  master_delta_rotation =

                      master_orientation
                          .toRotationMatrix() *

                      master_orientation_reference
                          .toRotationMatrix()
                          .transpose();


              Eigen::Matrix3d
                  mapped_delta_rotation =

                      v_to_f *

                      master_delta_rotation *

                      v_to_f.transpose();


              // -------------------------------------------------
              // ROTATION-REFERENCE AXIS PROJECTION
              // -------------------------------------------------
              // For the Y-only diagnostic we project the mapped
              // master relative rotation vector onto base-frame Y.
              // This keeps desired X/Z rotation exactly at the
              // captured Franka reference while preserving full
              // X/Z corrective torque in the impedance controller.
              //
              // The normal Full3D path remains available through
              // kRotationReferenceMode for direct A/B comparison.
              // -------------------------------------------------

              if (kRotationReferenceMode ==
                  RotationReferenceMode::Full3D) {

                mapped_delta_rotation =

                    limit_relative_rotation(

                        mapped_delta_rotation,

                        kMaxRelativeRotationRad,

                        &rotation_workspace_limited);

              } else {

                const Eigen::AngleAxisd mapped_delta_aa(
                    mapped_delta_rotation);

                Eigen::Vector3d mapped_rotation_vector =
                    Eigen::Vector3d::Zero();

                if (std::isfinite(mapped_delta_aa.angle()) &&
                    mapped_delta_aa.angle() > 1e-12) {

                  mapped_rotation_vector =
                      mapped_delta_aa.axis() *
                      mapped_delta_aa.angle();
                }

                int selected_axis = 1;  // YOnly default

                if (kRotationReferenceMode ==
                    RotationReferenceMode::XOnly) {
                  selected_axis = 0;
                } else if (kRotationReferenceMode ==
                           RotationReferenceMode::ZOnly) {
                  selected_axis = 2;
                }

                const double raw_selected_angle =
                    mapped_rotation_vector[selected_axis];

                const double selected_angle =
                    clamp_scalar(
                        raw_selected_angle,
                        -kMaxRelativeRotationRad,
                        kMaxRelativeRotationRad);

                rotation_workspace_limited =
                    std::abs(raw_selected_angle) >
                    kMaxRelativeRotationRad;

                Eigen::Vector3d selected_axis_vector =
                    Eigen::Vector3d::Zero();

                selected_axis_vector[selected_axis] =
                    1.0;

                mapped_delta_rotation =
                    Eigen::AngleAxisd(
                        selected_angle,
                        selected_axis_vector)
                        .toRotationMatrix();
              }


              const Eigen::Matrix3d
                  target_rotation =

                      mapped_delta_rotation *

                      franka_orientation_reference
                          .toRotationMatrix();


              Eigen::Quaterniond
                  target_orientation(

                      target_rotation);


              target_orientation.normalize();


              const Eigen::Quaterniond
                  previous_desired_orientation =
                      desired_orientation;


              desired_orientation =

                  step_quaternion_toward(

                      desired_orientation,

                      target_orientation,

                      kMaxDesiredAngularSpeedRadps *
                          dt,

                      &angular_speed_limited);


              // Exact angular velocity of this cycle's accepted
              // desired-orientation step, expressed in base/world
              // coordinates. R_new * R_old^T is a world-frame
              // incremental rotation, so its AngleAxis axis is in
              // the same frame as jacobian*dq angular velocity.
              const Eigen::Matrix3d
                  desired_step_rotation =

                      desired_orientation
                          .toRotationMatrix() *

                      previous_desired_orientation
                          .toRotationMatrix()
                          .transpose();


              const Eigen::AngleAxisd
                  desired_step_aa(
                      desired_step_rotation);


              if (std::isfinite(
                      desired_step_aa.angle()) &&

                  desired_step_aa.angle() >
                      1e-12) {

                desired_angular_velocity =

                    desired_step_aa.axis() *

                    (desired_step_aa.angle() /
                     dt);
              }

            } else {

              desired_orientation =
                  franka_orientation_reference;


              desired_angular_velocity.setZero();
            }

          } else {

            desired_position =
                position;


            desired_orientation =
                orientation;


            desired_angular_velocity.setZero();
          }


          previous_active =
              active;


          // ==================================================
          // CARTESIAN ERROR
          // ==================================================

          Eigen::Matrix<
              double,
              6,
              1>
              error;


          error.head<3>() =

              position -
              desired_position;


          Eigen::Quaterniond
              orientation_for_error =

                  orientation;


          if (

              desired_orientation
                  .coeffs()
                  .dot(
                      orientation_for_error
                          .coeffs()) <
              0.0) {

            orientation_for_error
                .coeffs() *=
                -1.0;
          }


          Eigen::Quaterniond
              error_quaternion =

                  orientation_for_error
                      .inverse() *

                  desired_orientation;


          error_quaternion.normalize();


          const Eigen::Vector3d
              orientation_error =

                  -transform.rotation() *

                  Eigen::Vector3d(

                      error_quaternion.x(),

                      error_quaternion.y(),

                      error_quaternion.z());


          error.tail<3>() =
              orientation_error;


          // ==================================================
          // CARTESIAN IMPEDANCE
          // ==================================================

          // Preserve the validated translational damping exactly:
          //     -D_trans * v
          //
          // For rotation only, damp tracking velocity error:
          //     -D_rot * (w - w_des)
          //
          // w_des comes from the exact accepted quaternion step above,
          // so it respects the existing 60 deg workspace limit and
          // 120 deg/s target-rate limit. No network differentiation,
          // no change to Krot/Drot, and no translational feedforward.
          Eigen::Matrix<
              double,
              6,
              1>
              damping_velocity =
                  cartesian_velocity;


          damping_velocity.tail<3>() -=
              desired_angular_velocity;


          const Eigen::Matrix<
              double,
              6,
              1>
              task_wrench =

                  -stiffness *
                      error

                  -

                  damping *
                      damping_velocity;


          const Eigen::Matrix<
              double,
              7,
              1>
              task_torque =

                  jacobian.transpose() *
                  task_wrench;


          // ==================================================
          // NULLSPACE CONTROL
          // ==================================================

          Eigen::Matrix<
              double,
              6,
              6>
              jj_transpose =

                  jacobian *
                  jacobian.transpose();


          jj_transpose
              .diagonal()
              .array() +=

              kDampedInverseLambda *
              kDampedInverseLambda;


          const Eigen::Matrix<
              double,
              6,
              6>
              jj_inverse =

                  jj_transpose
                      .ldlt()
                      .solve(

                          Eigen::Matrix<
                              double,
                              6,
                              6>::
                              Identity());


          const Eigen::Matrix<
              double,
              7,
              7>
              nullspace_projector =

                  Eigen::Matrix<
                      double,
                      7,
                      7>::
                      Identity()

                  -

                  jacobian.transpose() *
                      jj_inverse *
                      jacobian;


          const Eigen::Matrix<
              double,
              7,
              1>
              nullspace_torque =

                  nullspace_projector *

                  (
                      kNullspaceStiffness *

                          (
                              q_goal_eigen -
                              q)

                      -

                      kNullspaceDamping *
                          dq);


          // ==================================================
          // TOTAL JOINT TORQUE
          // ==================================================

          const Eigen::Matrix<
              double,
              7,
              1>
              desired_joint_torque =

                  task_torque +
                  nullspace_torque +
                  coriolis;


          const std::array<
              double,
              7>
              saturated_torque =

                  saturate_torque_rate(

                      desired_joint_torque,

                      state.tau_J_d);


          // ==================================================
          // LOG
          // ==================================================

          SlaveLogSample s{};


          s.time_s =
              local_time_s;


          s.callback_dt_ms =

              period.toSec() *
              1000.0;


          s.master_time_s =
              master_time_s;


          s.master_packet_id =
              master_packet_id;


          s.packet_age_ms =
              packet_age_ms;


          s.active =
              active
                  ? 1
                  : 0;


          s.alpha_received =
              alpha_received;


          s.k_requested =
              k_requested;


          s.k_applied =
              k_applied;


          s.d_applied =
              d_applied;


          s.k_rot_x =
              kRotationalStiffnessX;

          s.k_rot_y =
              kRotationalStiffnessY;

          s.k_rot_z =
              kRotationalStiffnessZ;

          s.d_rot_x =
              kRotationalDampingX;

          s.d_rot_y =
              kRotationalDampingY;

          s.d_rot_z =
              kRotationalDampingZ;


          s.actual_x =
              position.x();

          s.actual_y =
              position.y();

          s.actual_z =
              position.z();


          s.desired_x =
              desired_position.x();

          s.desired_y =
              desired_position.y();

          s.desired_z =
              desired_position.z();


          s.error_x =
              error[0];

          s.error_y =
              error[1];

          s.error_z =
              error[2];


          s.actual_qw =
              orientation.w();

          s.actual_qx =
              orientation.x();

          s.actual_qy =
              orientation.y();

          s.actual_qz =
              orientation.z();


          s.desired_qw =
              desired_orientation.w();

          s.desired_qx =
              desired_orientation.x();

          s.desired_qy =
              desired_orientation.y();

          s.desired_qz =
              desired_orientation.z();


          s.orientation_error_x =
              orientation_error.x();

          s.orientation_error_y =
              orientation_error.y();

          s.orientation_error_z =
              orientation_error.z();


          s.vx =
              cartesian_velocity[0];

          s.vy =
              cartesian_velocity[1];

          s.vz =
              cartesian_velocity[2];


          s.wx =
              cartesian_velocity[3];

          s.wy =
              cartesian_velocity[4];

          s.wz =
              cartesian_velocity[5];


          s.desired_wx =
              desired_angular_velocity.x();

          s.desired_wy =
              desired_angular_velocity.y();

          s.desired_wz =
              desired_angular_velocity.z();


          // fx..mz are now exactly the compensated wrench transmitted
          // to the master. Raw Franka O_F_ext_hat_K is logged separately.
          s.fx = feedback_wrench[0];
          s.fy = feedback_wrench[1];
          s.fz = feedback_wrench[2];

          s.mx = feedback_wrench[3];
          s.my = feedback_wrench[4];
          s.mz = feedback_wrench[5];


          s.g_force =
              force_gain;


          s.g_torque =
              torque_gain;


          s.deadman =
              deadman;


          s.estop =
              emergency_stop;


          s.translation_workspace_limited =

              translation_workspace_limited
                  ? 1
                  : 0;


          s.rotation_workspace_limited =

              rotation_workspace_limited
                  ? 1
                  : 0;


          s.linear_speed_limited =

              linear_speed_limited
                  ? 1
                  : 0;


          s.angular_speed_limited =

              angular_speed_limited
                  ? 1
                  : 0;


          // ==================================================
          // DIAGNOSTIC-ONLY DATA
          // ==================================================

          for (int i = 0; i < 7; ++i) {
            s.q[i] = q[i];
            s.dq[i] = dq[i];

            s.task_torque[i] = task_torque[i];
            s.nullspace_torque[i] = nullspace_torque[i];
            s.coriolis[i] = coriolis[i];

            s.tau_J[i] = state.tau_J[static_cast<std::size_t>(i)];
            s.tau_J_d[i] = state.tau_J_d[static_cast<std::size_t>(i)];
            s.tau_ext_hat_filtered[i] =
                state.tau_ext_hat_filtered[static_cast<std::size_t>(i)];
            s.commanded_torque[i] = saturated_torque[static_cast<std::size_t>(i)];
            s.estimated_joint_bias[i] = estimated_joint_bias[i];
            s.corrected_joint_external_torque[i] = corrected_joint_external_torque[i];
          }

          s.estimator_ready = estimator_ready ? 1 : 0;
          s.estimator_calibration_active_s = estimator_calibration_active_s;

          s.torque_contact_gate_active =
              torque_contact_gate_active ? 1 : 0;
          s.torque_contact_gate_scale =
              torque_contact_gate_scale;
          s.torque_contact_gate_on_dwell_s =
              torque_contact_gate_on_dwell_s;
          s.torque_contact_gate_off_dwell_s =
              torque_contact_gate_off_dwell_s;
          s.raw_force_norm =
              raw_force_norm;
          s.filtered_raw_force_norm =
              filtered_raw_force_norm;
          s.adaptive_retare_bias[0] = adaptive_retare_bias.x();
          s.adaptive_retare_bias[1] = adaptive_retare_bias.y();
          s.adaptive_retare_bias[2] = adaptive_retare_bias.z();
          s.adaptive_retare_updating = adaptive_retare_updating ? 1 : 0;

          for (int i = 0; i < 6; ++i) {
            s.raw_franka_wrench[i] = state.O_F_ext_hat_K[static_cast<std::size_t>(i)];
            s.reconstructed_wrench[i] = reconstructed_wrench[i];
            s.feedback_wrench[i] = feedback_wrench[i];
          }

          for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 7; ++col) {
              s.jacobian[7 * row + col] = jacobian(row, col);
            }
            s.task_wrench[row] = task_wrench[row];
          }

          for (int row = 0; row < 7; ++row) {
            for (int col = 0; col < 7; ++col) {
              s.mass_matrix[7 * row + col] = mass_matrix(row, col);
            }
          }


          if (!log_queue.push(
                  s)) {

            dropped_samples.fetch_add(
                1,
                std::memory_order_relaxed);
          }


          // ==================================================
          // STOP
          // ==================================================

          if (g_stop_requested) {

            return franka::MotionFinished(

                franka::Torques(
                    saturated_torque));
          }


          return franka::Torques(
              saturated_torque);
        });


  } catch (
      const franka::Exception&
          exception) {

    std::cerr
        << "\nFranka exception: "
        << exception.what()
        << '\n';


    g_stop_requested =
        1;


  } catch (
      const std::exception&
          exception) {

    std::cerr
        << "\nException: "
        << exception.what()
        << '\n';


    g_stop_requested =
        1;
  }


  // ==========================================================
  // SHUTDOWN
  // ==========================================================

  g_stop_requested =
      1;


  if (receive_thread.joinable()) {

    receive_thread.join();
  }


  if (feedback_thread.joinable()) {

    feedback_thread.join();
  }


  close(
      receive_socket);


  logger_stop.store(
      true,
      std::memory_order_relaxed);


  logger_thread.join();


  csv.close();


  std::cout
      << "\nSlave log: "
      << log_name
      << "\nDropped samples: "
      << dropped_samples.load()
      << '\n';


  return 0;
}


