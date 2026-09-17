// ============================================================
// master_compensated_adaptive_retare_yonly.cpp
//
// VIRTUOSE MASTER
// 1 kHz BILATERAL VARIABLE-IMPEDANCE TELEOPERATION
//
// CURRENT STAGE:
//   FULL 6-DOF BACKWARD POSITION/ORIENTATION COUPLING
//   + DIMENSIONALLY-CORRECT DIRECTIONAL DAMPING
//   + PAPER-GUIDED COMPENSATED FRANKA WRENCH INPUT
//   + SLAVE-SIDE ROTATIONAL CONTACT GATING (6 N ON / 3 N OFF)
//
// The slave now transmits a disturbance-compensated wrench reconstructed
// from tau_ext_hat_filtered.  This master deliberately keeps the validated
// haptic architecture and the physical reflection sign unchanged.
//
// Directional damping is independent of wrench reflection.
// Translation uses the external FORCE direction and Virtuose
// linear velocity. Rotation uses the external TORQUE direction
// and Virtuose angular velocity. Force and torque are never
// normalized together as one 6-D vector.
//
// Forward path:
//   Virtuose pose -> Franka desired pose
//
// Backward path:
//   Franka relative pose -> virtual spring-damper -> Virtuose
//
// Directional damping:
//   Translational and rotational terms are treated separately.
//   These are additional dissipative haptic terms; they do not
//   scale, replace, or decompose the reflected wrench itself.
//
// IMPORTANT SAFETY/ARCHITECTURE RULES:
//
//   1) The validated bilateral control law/gains are preserved.
//   2) Startup requires one observed deadman RELEASE before
//      either forward or backward teleoperation can arm.
//   3) Finite Franka wrench feedback never drops out merely
//      because its magnitude is large. The final Virtuose
//      wrench is norm-saturated and remains at the ceiling.
//   4) Physical Virtuose handle pose/speed are sampled only
//      for diagnostics. They do NOT enter the control law,
//      safety gate, references, or feedback computation.
//
// MASTER -> SLAVE packet: 14 doubles
//
// [0]      packet id
// [1]      timestamp us
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
// [1]      timestamp us
// [2..7]   Fx Fy Fz Mx My Mz
// [8..10]  Franka x y z
// [11..14] Franka qw qx qy qz
// [15..17] Franka vx vy vz
// [18..20] Franka wx wy wz
// [21]     estimator_ready (0 during free-space calibration, 1 afterwards)
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
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "virtuoseAPI.h"


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

constexpr const char* kSlaveIp =
    "127.0.0.1";

constexpr int kCommandSendPort =
    11056;

constexpr int kFeedbackReceivePort =
    11055;

constexpr std::size_t kCommandPacketSize =
    14;

constexpr std::size_t kFeedbackPacketSize =
    22;


// ------------------------------------------------------------
// CONTROL RATE / TIMING
// ------------------------------------------------------------

constexpr double kLoopHz =
    1000.0;

constexpr double kFeedbackStaleMs =
    50.0;

constexpr float kVirtuoseApiTimeoutS =
    0.05f;


// ============================================================
// BACKWARD CONTROL SWITCHES
// ============================================================

// Translation remains full 3-D. Rotational haptics are Y-only in this diagnostic
// so the backward rotational channel matches the slave Y-only forward reference.

constexpr bool kEnableBackwardPositionCoupling =
    true;

constexpr bool kEnableBackwardRotationCoupling =
    true;

enum class BackwardRotationMode {
  Full3D,
  YOnly
};

constexpr BackwardRotationMode kBackwardRotationMode =
    BackwardRotationMode::Full3D;


// ------------------------------------------------------------
// DIRECTIONAL DAMPING
//
// Independent from force/torque reflection.
//
// Translation:
//   project Virtuose linear velocity onto Franka external-force
//   direction, then oppose only that projected motion.
//
// Rotation:
//   project Virtuose angular velocity onto Franka external-torque
//   direction, then oppose only that projected motion.
//
// IMPORTANT:
//   Force [N] and torque [Nm] are NOT combined into one normalized
//   6-D wrench, unlike the reference implementation.
// ------------------------------------------------------------

constexpr bool kEnableDirectionalForceDamping =
    true;

constexpr bool kEnableDirectionalTorqueDamping =
    true;


// ------------------------------------------------------------
// WRENCH REFLECTION
//
// Reflection is a separate additive haptic term. These switches
// do not enable/disable directional damping.
// ------------------------------------------------------------

constexpr bool kEnableForceReflection =
    true;

constexpr bool kEnableTorqueReflection =
    true;


// ============================================================
// BACKWARD POSITION COUPLING
//
// Fsync = Kb * (x_target - x_V)
//       + Db * (v_target - v_V)
//
// ============================================================

constexpr double kBackwardPositionStiffness =
    100.0;                       // N/m

constexpr double kBackwardPositionDamping =
    20.0;                         // Ns/m


// ============================================================
// BACKWARD ROTATION COUPLING
//
// Msync = Kr * rotation_error
//       + Dr * (w_target - w_V)
//
// ============================================================

constexpr double kBackwardRotationStiffness =
    1.0;                         // Nm/rad

constexpr double kBackwardRotationDamping =
    0.12;                        // Nms/rad


// ------------------------------------------------------------
// BACKWARD SYNCHRONIZATION ERROR LIMITS
// ------------------------------------------------------------

constexpr double kMaxBackwardPositionErrorM =
    0.10;                        // m

constexpr double kMaxBackwardRotationErrorDeg =
    30.0;                        // deg

constexpr double kMaxBackwardRotationErrorRad =

    kMaxBackwardRotationErrorDeg *
    kPi /
    180.0;


// ============================================================
// SELECTIVE DIRECTIONAL DAMPING
//
// Translation:
//
//   nF        = F / ||F||
//   v_parallel = (v_V . nF) nF
//   Fd        = -bF v_parallel
//
//   Pd_lin = Fd . v_V
//          = -bF ||v_parallel||^2
//          <= 0
//
// Rotation:
//
//   nM         = M / ||M||
//   w_parallel = (w_V . nM) nM
//   Md         = -bM w_parallel
//
//   Pd_rot = Md . w_V
//          = -bM ||w_parallel||^2
//          <= 0
//
// Translation and rotation are kept dimensionally separate.
// The measured wrench is used only to define the damping axis;
// wrench reflection remains an independent additive term.
// ============================================================

constexpr double kDirectionalLinearDamping =
    5.0;                         // Ns/m

constexpr double kDirectionalAngularDamping =
    0.10;                        // Nms/rad

constexpr double kDirectionalForceThresholdN =
    1.0;                         // N

constexpr double kDirectionalTorqueThresholdNm =
    1.0;                         // Nm


// ============================================================
// WRENCH REFLECTION
//
// Slave feedback is already compensated/deadbanded.
// Keep the physical action-reaction reflection sign:
//
// F_V = -gF * F_F
// M_V = -gM * M_F
// ============================================================

constexpr double kForceReflectionGain =
    1;

constexpr double kTorqueReflectionGain =
    0.75;


// ------------------------------------------------------------
// VIRTUOSE SOFTWARE/API SATURATION
// ------------------------------------------------------------

// Haption VIRTUOSE 6D TAO HF published ratings.
// Peak ratings are documented here but are NOT held continuously.
constexpr double kDevicePeakForceN =
    70.0;

constexpr double kDevicePeakTorqueNm =
    5.0;

// Persistent software/API ceilings use the published continuous
// ratings, because the command may remain saturated indefinitely.
constexpr double kDeviceForceCeilingN =
    30.0;

constexpr double kDeviceTorqueCeilingNm =
    1.5;

constexpr double kNearCeilingFraction =
    0.90;


// ------------------------------------------------------------
// COMMAND SLEW LIMITS
// ------------------------------------------------------------

constexpr double kMaxForceStepN =
    0.075;                       // N/cycle @ 1 kHz

constexpr double kMaxTorqueStepNm =
    0.010;                       // Nm/cycle @ 1 kHz

// After calibration finishes, backward haptics are enabled gradually.
// This is separate from the existing per-cycle slew limits and prevents
// a sudden six-axis step when estimator_ready changes from 0 to 1.
constexpr double kHapticEnableRampSeconds =
    0.40;


// ------------------------------------------------------------
// COMMAND LOW-PASS FILTERS
// ------------------------------------------------------------

constexpr double kForceFilterAlpha =
    0.133974596216;

constexpr double kTorqueFilterAlpha =
    0.1339746;


// ------------------------------------------------------------
// FRANKA WRENCH VALIDITY
//
// There is deliberately NO magnitude-triggered dropout here.
// A finite debiased wrench remains usable; the final command sent
// to the Virtuose is saturated to the continuous device ceiling.
// ------------------------------------------------------------


// ------------------------------------------------------------
// MASTER-SIDE WRENCH BIAS
// ------------------------------------------------------------
//
// Disabled deliberately.  The slave already performs joint-space
// disturbance compensation plus a force-gated adaptive re-tare.
// A one-shot 500-sample Cartesian bias was shown to become stale and can
// make the residual worse later in the run.  The master therefore uses the
// slave feedback exactly as received.


// ------------------------------------------------------------
// VIRTUOSE API SCALING
// ------------------------------------------------------------

constexpr float kVirtuoseForceFactor =
    1.0f;

constexpr float kVirtuoseSpeedFactor =
    1.0f;


// ------------------------------------------------------------
// VIRTUOSE -> FRANKA FRAME MAP
//
// Experimentally verified identity mapping.
// ------------------------------------------------------------

constexpr std::array<double, 9>
    kVirtuoseToFranka = {

        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};


// ------------------------------------------------------------
// MANUAL TELEIMPEDANCE INPUT
// ------------------------------------------------------------

constexpr double kInitialAlpha =
    0.50;


// ------------------------------------------------------------
// LOGGING
// ------------------------------------------------------------

constexpr std::size_t kLogQueueCapacity =
    65536;

constexpr uint64_t kLogFlushRows =
    1000;

constexpr int kConsolePrintEveryCycles =
    200;


// ============================================================
// END CONFIGURATION
// ============================================================


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

double clamp_scalar(
    double value,
    double lower,
    double upper) {

  return std::max(
      lower,
      std::min(
          value,
          upper));
}


double rate_limit(
    double target,
    double previous,
    double maximum_step) {

  return previous +

         clamp_scalar(
             target - previous,
             -maximum_step,
             maximum_step);
}


double norm3(
    const double value[3]) {

  return std::sqrt(

      value[0] * value[0] +
      value[1] * value[1] +
      value[2] * value[2]);
}


bool finite3(
    const Eigen::Vector3d& value) {

  return

      std::isfinite(value.x()) &&
      std::isfinite(value.y()) &&
      std::isfinite(value.z());
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


double elapsed_seconds(
    const std::chrono::steady_clock::time_point& start) {

  return std::chrono::duration<double>(

             std::chrono::steady_clock::now() -
             start)

      .count();
}


double elapsed_us(
    const std::chrono::steady_clock::time_point& start) {

  return std::chrono::duration<
             double,
             std::micro>(

             std::chrono::steady_clock::now() -
             start)

      .count();
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


Eigen::Vector3d clamp_vector_norm(
    const Eigen::Vector3d& value,
    double maximum_norm) {

  const double magnitude =
      value.norm();


  if (magnitude <=
          maximum_norm ||

      magnitude <
          1e-12) {

    return value;
  }


  return

      value *

      (
          maximum_norm /
          magnitude);
}


// ------------------------------------------------------------
// World-frame rotation vector taking current -> target
// ------------------------------------------------------------

Eigen::Vector3d orientation_error_world(
    const Eigen::Quaterniond& current_input,
    const Eigen::Quaterniond& target_input) {

  const Eigen::Quaterniond current =
      current_input.normalized();


  const Eigen::Quaterniond target =
      target_input.normalized();


  const Eigen::Matrix3d relative_rotation =

      target.toRotationMatrix() *

      current.toRotationMatrix().transpose();


  Eigen::AngleAxisd aa(
      relative_rotation);


  if (!std::isfinite(
          aa.angle()) ||

      aa.angle() <
          1e-12) {

    return Eigen::Vector3d::
        Zero();
  }


  return

      aa.axis() *
      aa.angle();
}


void zero_wrench(
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


void shutdown_virtuose(
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


const char* power_label(
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


// ============================================================
// LOCK-FREE LOG QUEUE
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
// MASTER LOG
// ============================================================

struct MasterLogSample {

  double time_s;
  double loop_dt_ms;

  uint64_t packet_id;

  int backward_active;
  int backward_reference_ready;

  int force_safe;
  int torque_safe;

  double alpha_command;

  double g_force;
  double g_torque;

  double virtuose_x;
  double virtuose_y;
  double virtuose_z;

  double virtuose_qw;
  double virtuose_qx;
  double virtuose_qy;
  double virtuose_qz;

  double virtuose_vx;
  double virtuose_vy;
  double virtuose_vz;

  double virtuose_wx;
  double virtuose_wy;
  double virtuose_wz;

  // Physical handle state from virtGetPhysicalPosition/Speed.
  // Diagnostic only; never used by the controller.
  double physical_x;
  double physical_y;
  double physical_z;

  double physical_qw;
  double physical_qx;
  double physical_qy;
  double physical_qz;

  double physical_vx;
  double physical_vy;
  double physical_vz;

  double physical_wx;
  double physical_wy;
  double physical_wz;

  int physical_pose_rc;
  int physical_speed_rc;
  int physical_state_valid;

  double franka_x;
  double franka_y;
  double franka_z;

  double franka_qw;
  double franka_qx;
  double franka_qy;
  double franka_qz;

  double franka_vx;
  double franka_vy;
  double franka_vz;

  double franka_wx;
  double franka_wy;
  double franka_wz;

  double target_x;
  double target_y;
  double target_z;

  double target_qw;
  double target_qx;
  double target_qy;
  double target_qz;

  double sync_error_x;
  double sync_error_y;
  double sync_error_z;

  double sync_rot_error_x;
  double sync_rot_error_y;
  double sync_rot_error_z;

  double franka_fx;
  double franka_fy;
  double franka_fz;

  double franka_mx;
  double franka_my;
  double franka_mz;

  double sync_fx;
  double sync_fy;
  double sync_fz;

  double sync_mx;
  double sync_my;
  double sync_mz;

  double reflect_fx;
  double reflect_fy;
  double reflect_fz;

  double reflect_mx;
  double reflect_my;
  double reflect_mz;

  double directional_fx;
  double directional_fy;
  double directional_fz;

  double directional_mx;
  double directional_my;
  double directional_mz;

  double force_direction_x;
  double force_direction_y;
  double force_direction_z;

  double parallel_vx;
  double parallel_vy;
  double parallel_vz;

  double torque_direction_x;
  double torque_direction_y;
  double torque_direction_z;

  double parallel_wx;
  double parallel_wy;
  double parallel_wz;

  double directional_linear_power;
  double directional_rotational_power;

  int directional_force_active;
  int directional_torque_active;

  double desired_fx;
  double desired_fy;
  double desired_fz;

  double desired_mx;
  double desired_my;
  double desired_mz;

  double rendered_fx;
  double rendered_fy;
  double rendered_fz;

  double rendered_mx;
  double rendered_my;
  double rendered_mz;

  double linear_power;
  double rotational_power;
  double total_power;

  int force_rate_limited;
  int torque_rate_limited;

  int force_near_ceiling;
  int torque_near_ceiling;

  int deadman;
  int estop;
  int power_on;
  int indexing;
  int render_on;

  double feedback_age_ms;

  uint32_t alarm;

  int set_force_rc;
};


}  // namespace


// ============================================================
// MAIN
// ============================================================

int main() {

  std::signal(
      SIGINT,
      sigint_handler);


  const Eigen::Matrix3d v_to_f =
      create_frame_map();


  const auto loop_period =

      std::chrono::microseconds(

          static_cast<int>(
              1000000.0 /
              kLoopHz));


  // ==========================================================
  // MANUAL ALPHA
  // ==========================================================

  std::atomic<double>
      alpha_command{
          kInitialAlpha};


  // ==========================================================
  // CSV
  // ==========================================================

  const std::string log_name =

      "master_paper_compensated_wrench_" +
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
      << "loop_dt_ms,"
      << "packet_id,"
      << "backward_active,"
      << "backward_reference_ready,"
      << "force_safe,"
      << "torque_safe,"
      << "alpha_command,"
      << "g_force,"
      << "g_torque,"

      << "virtuose_x,"
      << "virtuose_y,"
      << "virtuose_z,"
      << "virtuose_qw,"
      << "virtuose_qx,"
      << "virtuose_qy,"
      << "virtuose_qz,"

      << "virtuose_vx,"
      << "virtuose_vy,"
      << "virtuose_vz,"
      << "virtuose_wx,"
      << "virtuose_wy,"
      << "virtuose_wz,"

      << "physical_x,"
      << "physical_y,"
      << "physical_z,"
      << "physical_qw,"
      << "physical_qx,"
      << "physical_qy,"
      << "physical_qz,"
      << "physical_vx,"
      << "physical_vy,"
      << "physical_vz,"
      << "physical_wx,"
      << "physical_wy,"
      << "physical_wz,"
      << "physical_pose_rc,"
      << "physical_speed_rc,"
      << "physical_state_valid,"

      << "franka_x,"
      << "franka_y,"
      << "franka_z,"
      << "franka_qw,"
      << "franka_qx,"
      << "franka_qy,"
      << "franka_qz,"

      << "franka_vx,"
      << "franka_vy,"
      << "franka_vz,"
      << "franka_wx,"
      << "franka_wy,"
      << "franka_wz,"

      << "target_x,"
      << "target_y,"
      << "target_z,"
      << "target_qw,"
      << "target_qx,"
      << "target_qy,"
      << "target_qz,"

      << "sync_error_x,"
      << "sync_error_y,"
      << "sync_error_z,"

      << "sync_rot_error_x,"
      << "sync_rot_error_y,"
      << "sync_rot_error_z,"

      << "franka_fx,"
      << "franka_fy,"
      << "franka_fz,"
      << "franka_mx,"
      << "franka_my,"
      << "franka_mz,"

      << "sync_fx,"
      << "sync_fy,"
      << "sync_fz,"
      << "sync_mx,"
      << "sync_my,"
      << "sync_mz,"

      << "reflect_fx,"
      << "reflect_fy,"
      << "reflect_fz,"
      << "reflect_mx,"
      << "reflect_my,"
      << "reflect_mz,"

      << "directional_fx,"
      << "directional_fy,"
      << "directional_fz,"
      << "directional_mx,"
      << "directional_my,"
      << "directional_mz,"

      << "force_direction_x,"
      << "force_direction_y,"
      << "force_direction_z,"
      << "parallel_vx,"
      << "parallel_vy,"
      << "parallel_vz,"
      << "torque_direction_x,"
      << "torque_direction_y,"
      << "torque_direction_z,"
      << "parallel_wx,"
      << "parallel_wy,"
      << "parallel_wz,"
      << "directional_linear_power,"
      << "directional_rotational_power,"
      << "directional_force_active,"
      << "directional_torque_active,"

      << "desired_fx,"
      << "desired_fy,"
      << "desired_fz,"
      << "desired_mx,"
      << "desired_my,"
      << "desired_mz,"

      << "rendered_fx,"
      << "rendered_fy,"
      << "rendered_fz,"
      << "rendered_mx,"
      << "rendered_my,"
      << "rendered_mz,"

      << "linear_power,"
      << "rotational_power,"
      << "total_power,"

      << "force_rate_limited,"
      << "torque_rate_limited,"
      << "force_near_ceiling,"
      << "torque_near_ceiling,"

      << "deadman,"
      << "estop,"
      << "power_on,"
      << "indexing,"
      << "render_on,"

      << "feedback_age_ms,"
      << "alarm,"
      << "set_force_rc\n";


  csv
      << std::fixed
      << std::setprecision(9);


  static SpscRing<
      MasterLogSample,
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

        MasterLogSample s{};

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
                << s.loop_dt_ms << ','
                << s.packet_id << ','
                << s.backward_active << ','
                << s.backward_reference_ready << ','
                << s.force_safe << ','
                << s.torque_safe << ','
                << s.alpha_command << ','
                << s.g_force << ','
                << s.g_torque << ','

                << s.virtuose_x << ','
                << s.virtuose_y << ','
                << s.virtuose_z << ','
                << s.virtuose_qw << ','
                << s.virtuose_qx << ','
                << s.virtuose_qy << ','
                << s.virtuose_qz << ','

                << s.virtuose_vx << ','
                << s.virtuose_vy << ','
                << s.virtuose_vz << ','
                << s.virtuose_wx << ','
                << s.virtuose_wy << ','
                << s.virtuose_wz << ','

                << s.physical_x << ','
                << s.physical_y << ','
                << s.physical_z << ','
                << s.physical_qw << ','
                << s.physical_qx << ','
                << s.physical_qy << ','
                << s.physical_qz << ','
                << s.physical_vx << ','
                << s.physical_vy << ','
                << s.physical_vz << ','
                << s.physical_wx << ','
                << s.physical_wy << ','
                << s.physical_wz << ','
                << s.physical_pose_rc << ','
                << s.physical_speed_rc << ','
                << s.physical_state_valid << ','

                << s.franka_x << ','
                << s.franka_y << ','
                << s.franka_z << ','
                << s.franka_qw << ','
                << s.franka_qx << ','
                << s.franka_qy << ','
                << s.franka_qz << ','

                << s.franka_vx << ','
                << s.franka_vy << ','
                << s.franka_vz << ','
                << s.franka_wx << ','
                << s.franka_wy << ','
                << s.franka_wz << ','

                << s.target_x << ','
                << s.target_y << ','
                << s.target_z << ','
                << s.target_qw << ','
                << s.target_qx << ','
                << s.target_qy << ','
                << s.target_qz << ','

                << s.sync_error_x << ','
                << s.sync_error_y << ','
                << s.sync_error_z << ','

                << s.sync_rot_error_x << ','
                << s.sync_rot_error_y << ','
                << s.sync_rot_error_z << ','

                << s.franka_fx << ','
                << s.franka_fy << ','
                << s.franka_fz << ','
                << s.franka_mx << ','
                << s.franka_my << ','
                << s.franka_mz << ','

                << s.sync_fx << ','
                << s.sync_fy << ','
                << s.sync_fz << ','
                << s.sync_mx << ','
                << s.sync_my << ','
                << s.sync_mz << ','

                << s.reflect_fx << ','
                << s.reflect_fy << ','
                << s.reflect_fz << ','
                << s.reflect_mx << ','
                << s.reflect_my << ','
                << s.reflect_mz << ','

                << s.directional_fx << ','
                << s.directional_fy << ','
                << s.directional_fz << ','
                << s.directional_mx << ','
                << s.directional_my << ','
                << s.directional_mz << ','

                << s.force_direction_x << ','
                << s.force_direction_y << ','
                << s.force_direction_z << ','
                << s.parallel_vx << ','
                << s.parallel_vy << ','
                << s.parallel_vz << ','
                << s.torque_direction_x << ','
                << s.torque_direction_y << ','
                << s.torque_direction_z << ','
                << s.parallel_wx << ','
                << s.parallel_wy << ','
                << s.parallel_wz << ','
                << s.directional_linear_power << ','
                << s.directional_rotational_power << ','
                << s.directional_force_active << ','
                << s.directional_torque_active << ','

                << s.desired_fx << ','
                << s.desired_fy << ','
                << s.desired_fz << ','
                << s.desired_mx << ','
                << s.desired_my << ','
                << s.desired_mz << ','

                << s.rendered_fx << ','
                << s.rendered_fy << ','
                << s.rendered_fz << ','
                << s.rendered_mx << ','
                << s.rendered_my << ','
                << s.rendered_mz << ','

                << s.linear_power << ','
                << s.rotational_power << ','
                << s.total_power << ','

                << s.force_rate_limited << ','
                << s.torque_rate_limited << ','
                << s.force_near_ceiling << ','
                << s.torque_near_ceiling << ','

                << s.deadman << ','
                << s.estop << ','
                << s.power_on << ','
                << s.indexing << ','
                << s.render_on << ','

                << s.feedback_age_ms << ','
                << s.alarm << ','
                << s.set_force_rc
                << '\n';


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
  // VIRTUOSE INITIALIZATION
  // ==========================================================

  VirtContext context =

      virtOpen(
          "127.0.0.1#53210");


  if (!context) {

    std::cerr
        << "ERROR: virtOpen failed.\n";


    logger_stop.store(
        true);


    logger_thread.join();


    return 1;
  }


  float identity_frame[7] = {

      0.0f,
      0.0f,
      0.0f,

      0.0f,
      0.0f,
      0.0f,
      1.0f};


  auto setup_failed =

      [&](const char* operation) {

        std::cerr
            << "ERROR: Virtuose setup failed at "
            << operation
            << '\n';


        shutdown_virtuose(
            context);


        logger_stop.store(
            true);


        logger_thread.join();


        return 1;
      };


  if (virtSetTimeoutValue(
          context,
          kVirtuoseApiTimeoutS) !=
      0) {

    return setup_failed(
        "virtSetTimeoutValue");
  }


  if (virtSetIndexingMode(
          context,
          INDEXING_ALL) !=
      0) {

    return setup_failed(
        "virtSetIndexingMode");
  }


  if (virtSetForceFactor(
          context,
          kVirtuoseForceFactor) !=
      0) {

    return setup_failed(
        "virtSetForceFactor");
  }


  if (virtSetSpeedFactor(
          context,
          kVirtuoseSpeedFactor) !=
      0) {

    return setup_failed(
        "virtSetSpeedFactor");
  }


  if (virtSetTimeStep(
          context,
          static_cast<float>(
              1.0 /
              kLoopHz)) !=
      0) {

    return setup_failed(
        "virtSetTimeStep");
  }


  if (virtSetBaseFrame(
          context,
          identity_frame) !=
      0) {

    return setup_failed(
        "virtSetBaseFrame");
  }


  if (virtSetObservationFrame(
          context,
          identity_frame) !=
      0) {

    return setup_failed(
        "virtSetObservationFrame");
  }


  if (virtSetCommandType(
          context,
          COMMAND_TYPE_IMPEDANCE) !=
      0) {

    return setup_failed(
        "virtSetCommandType");
  }


  if (virtSaturateTorque(
          context,
          static_cast<float>(
              kDeviceForceCeilingN),
          static_cast<float>(
              kDeviceTorqueCeilingNm)) !=
      0) {

    return setup_failed(
        "virtSaturateTorque");
  }


  if (virtEnableForceFeedback(
          context,
          1) !=
      0) {

    return setup_failed(
        "virtEnableForceFeedback");
  }


  if (virtSetPowerOn(
          context,
          1) !=
      0) {

    return setup_failed(
        "virtSetPowerOn");
  }


  // ==========================================================
  // COMMAND SOCKET
  // ==========================================================

  int command_socket =

      socket(
          AF_INET,
          SOCK_DGRAM,
          0);


  if (command_socket < 0) {

    std::perror(
        "command socket");


    shutdown_virtuose(
        context);


    logger_stop.store(
        true);


    logger_thread.join();


    return 1;
  }


  sockaddr_in slave_address{};

  slave_address.sin_family =
      AF_INET;

  slave_address.sin_port =

      htons(
          kCommandSendPort);


  if (inet_pton(
          AF_INET,
          kSlaveIp,
          &slave_address.sin_addr) !=
      1) {

    std::cerr
        << "ERROR: invalid slave IP.\n";


    close(
        command_socket);


    shutdown_virtuose(
        context);


    logger_stop.store(
        true);


    logger_thread.join();


    return 1;
  }


  // ==========================================================
  // FEEDBACK SOCKET
  // ==========================================================

  int feedback_socket =

      socket(
          AF_INET,
          SOCK_DGRAM,
          0);


  if (feedback_socket < 0) {

    std::perror(
        "feedback socket");


    close(
        command_socket);


    shutdown_virtuose(
        context);


    logger_stop.store(
        true);


    logger_thread.join();


    return 1;
  }


  int reuse =
      1;


  setsockopt(
      feedback_socket,
      SOL_SOCKET,
      SO_REUSEADDR,
      &reuse,
      sizeof(reuse));


  sockaddr_in feedback_address{};

  feedback_address.sin_family =
      AF_INET;

  feedback_address.sin_addr.s_addr =

      htonl(
          INADDR_ANY);

  feedback_address.sin_port =

      htons(
          kFeedbackReceivePort);


  if (bind(
          feedback_socket,
          reinterpret_cast<sockaddr*>(
              &feedback_address),
          sizeof(feedback_address)) <
      0) {

    std::perror(
        "feedback bind");


    close(
        feedback_socket);


    close(
        command_socket);


    shutdown_virtuose(
        context);


    logger_stop.store(
        true);


    logger_thread.join();


    return 1;
  }


  const int feedback_flags =

      fcntl(
          feedback_socket,
          F_GETFL,
          0);


  if (feedback_flags < 0 ||

      fcntl(
          feedback_socket,
          F_SETFL,
          feedback_flags |
              O_NONBLOCK) <
          0) {

    std::perror(
        "feedback fcntl");


    close(
        feedback_socket);


    close(
        command_socket);


    shutdown_virtuose(
        context);


    logger_stop.store(
        true);


    logger_thread.join();


    return 1;
  }


  // ==========================================================
  // ALPHA INPUT THREAD
  // ==========================================================

  std::thread alpha_thread(

      [&]() {

        std::cout

            << "\n============================================\n"
            << "CONTROLLER CONFIGURATION: MASTER\n"
            << "============================================\n"

            << "Loop rate                     : "
            << kLoopHz
            << " Hz\n"

            << "Backward translation coupling : "
            << (
                   kEnableBackwardPositionCoupling
                       ? "ON"
                       : "OFF")
            << "\n"

            << "Backward rotation coupling    : "
            << (
                   kEnableBackwardRotationCoupling
                       ? "ON"
                       : "OFF")
            << "\n"
            << "Backward rotation mode        : "
            << (kBackwardRotationMode == BackwardRotationMode::YOnly
                    ? "Y ONLY"
                    : "FULL 3D")
            << "\n"

            << "Kb translation                : "
            << kBackwardPositionStiffness
            << " N/m\n"

            << "Db translation                : "
            << kBackwardPositionDamping
            << " Ns/m\n"

            << "Kb rotation                   : "
            << kBackwardRotationStiffness
            << " Nm/rad\n"

            << "Db rotation                   : "
            << kBackwardRotationDamping
            << " Nms/rad\n"

            << "Directional force damping     : "
            << (
                   kEnableDirectionalForceDamping
                       ? "ON"
                       : "OFF")
            << "\n"

            << "Directional linear damping    : "
            << kDirectionalLinearDamping
            << " Ns/m\n"

            << "Directional force threshold   : "
            << kDirectionalForceThresholdN
            << " N\n"

            << "Directional torque damping    : "
            << (
                   kEnableDirectionalTorqueDamping
                       ? "ON"
                       : "OFF")
            << "\n"

            << "Directional angular damping   : "
            << kDirectionalAngularDamping
            << " Nms/rad\n"

            << "Directional torque threshold  : "
            << kDirectionalTorqueThresholdNm
            << " Nm\n"

            << "Force reflection              : "
            << (
                   kEnableForceReflection
                       ? "ON"
                       : "OFF")
            << "\n"

            << "Torque reflection             : "
            << (
                   kEnableTorqueReflection
                       ? "ON"
                       : "OFF")
            << "\n"

            << "Wrench magnitude dropout      : OFF\n"

            << "Force continuous ceiling      : "
            << kDeviceForceCeilingN
            << " N\n"

            << "Torque continuous ceiling     : "
            << kDeviceTorqueCeilingNm
            << " Nm\n"

            << "Documented peak force/torque  : "
            << kDevicePeakForceN
            << " N / "
            << kDevicePeakTorqueNm
            << " Nm (not held continuously)\n"

            << "Initial alpha                 : "
            << kInitialAlpha
            << "\n"

            << "============================================\n\n"

            << "IMPORTANT:\n"
            << "Release the deadman once after startup to arm teleoperation.\n"
            << "Large finite wrench feedback saturates; it does not drop out.\n\n"

            << "Enter alpha in [0,1].\n"
            << "Keep alpha = 0.50 for this test.\n"
            << "Type q to quit.\n\n";


        pollfd descriptor{};

        descriptor.fd =
            STDIN_FILENO;

        descriptor.events =
            POLLIN;


        while (!g_stop_requested) {

          const int result =

              poll(
                  &descriptor,
                  1,
                  200);


          if (result <= 0) {

            continue;
          }


          if (!(descriptor.revents &
                POLLIN)) {

            continue;
          }


          std::string line;


          if (!std::getline(
                  std::cin,
                  line)) {

            continue;
          }


          if (line == "q" ||
              line == "Q") {

            g_stop_requested =
                1;

            break;
          }


          try {

            std::size_t parsed =
                0;


            const double alpha =

                std::stod(
                    line,
                    &parsed);


            if (parsed !=
                    line.size() ||

                !std::isfinite(
                    alpha) ||

                alpha < 0.0 ||
                alpha > 1.0) {

              std::cerr
                  << "Alpha must be in [0,1].\n";

              continue;
            }


            alpha_command.store(
                alpha,
                std::memory_order_relaxed);


            const double K =

                300.0 +
                400.0 *
                    alpha;


            const double D =

                45.0 *

                std::sqrt(
                    K /
                    500.0);


            std::cout
                << "\nalpha = "
                << alpha

                << "\nFranka K target = "
                << K
                << " N/m"

                << "\nFranka D target = "
                << D
                << " Ns/m\n\n";


          } catch (...) {

            std::cerr
                << "Invalid alpha.\n";
          }
        }
      });


  // ==========================================================
  // RUNTIME STATE
  // ==========================================================

  const auto start_time =

      std::chrono::steady_clock::now();


  auto next_tick =

      std::chrono::steady_clock::now();


  auto previous_loop_time =
      next_tick;


  auto last_feedback_time =
      next_tick;


  uint64_t packet_id =
      0;


  double feedback[
      kFeedbackPacketSize]{};


  bool feedback_received =
      false;


  // ----------------------------------------------------------
  // STARTUP ARM
  //
  // Teleoperation cannot become active until the deadman has
  // been observed RELEASED at least once after startup.
  // ----------------------------------------------------------

  bool deadman_release_seen =
      false;


  // ----------------------------------------------------------
  // WRENCH BIAS
  // ----------------------------------------------------------
  // No master-side static bias.  Compensation lives on the slave.


  // ----------------------------------------------------------
  // COMMAND FILTER / SLEW STATE
  // ----------------------------------------------------------

  double filtered_force[3]{};
  double rendered_force[3]{};

  double filtered_torque[3]{};
  double rendered_torque[3]{};


  // ----------------------------------------------------------
  // BACKWARD RELATIVE REFERENCES
  // ----------------------------------------------------------

  bool backward_reference_ready =
      false;

  bool previous_backward_active =
      false;


  auto backward_activation_time =
      next_tick;


  Eigen::Vector3d
      virtuose_position_reference =
          Eigen::Vector3d::Zero();


  Eigen::Vector3d
      franka_position_reference =
          Eigen::Vector3d::Zero();


  Eigen::Quaterniond
      virtuose_orientation_reference =
          Eigen::Quaterniond::Identity();


  Eigen::Quaterniond
      franka_orientation_reference =
          Eigen::Quaterniond::Identity();


  int terminal_counter =
      0;


  // ==========================================================
  // 1 kHz MASTER LOOP
  // ==========================================================

  while (!g_stop_requested) {

    next_tick +=
        loop_period;


    const auto now =

        std::chrono::steady_clock::now();


    const double loop_dt_ms =

        std::chrono::duration<
            double,
            std::milli>(

            now -
            previous_loop_time)

            .count();


    previous_loop_time =
        now;


    // ========================================================
    // VIRTUOSE STATE
    // ========================================================

    float pose[7]{};

    float speed[6]{};

    // Physical-handle diagnostics. These values are not used
    // anywhere in the controller itself.
    float physical_pose[7]{};

    float physical_speed[6]{};

    float analogue[7]{};


    int deadman =
        0;

    int emergency_stop =
        0;

    int power =
        0;

    int indexing =
        0;


    unsigned int alarm =
        0;


    const int rc_pose =

        virtGetPosition(
            context,
            pose);


    const int rc_speed =

        virtGetSpeed(
            context,
            speed);


    // Diagnostic reads only. Their return codes deliberately
    // do not participate in api_ok/backward_active.
    const int rc_physical_pose =

        virtGetPhysicalPosition(
            context,
            physical_pose);


    const int rc_physical_speed =

        virtGetPhysicalSpeed(
            context,
            physical_speed);


    const int rc_analogue =

        virtGetAnalogicInputs(
            context,
            analogue);


    const int rc_deadman =

        virtGetDeadMan(
            context,
            &deadman);


    const int rc_estop =

        virtGetEmergencyStop(
            context,
            &emergency_stop);


    const int rc_power =

        virtGetPowerOn(
            context,
            &power);


    const int rc_index =

        virtIsInShiftPosition(
            context,
            &indexing);


    const int rc_alarm =

        virtGetAlarm(
            context,
            &alarm);


    Eigen::Vector3d virtuose_position(

        static_cast<double>(
            pose[0]),

        static_cast<double>(
            pose[1]),

        static_cast<double>(
            pose[2]));


    Eigen::Quaterniond virtuose_orientation(

        static_cast<double>(
            pose[6]),

        static_cast<double>(
            pose[3]),

        static_cast<double>(
            pose[4]),

        static_cast<double>(
            pose[5]));


    if (finite_quaternion(
            virtuose_orientation)) {

      virtuose_orientation.normalize();
    }


    Eigen::Vector3d virtuose_linear_velocity(

        static_cast<double>(
            speed[0]),

        static_cast<double>(
            speed[1]),

        static_cast<double>(
            speed[2]));


    Eigen::Vector3d virtuose_angular_velocity(

        static_cast<double>(
            speed[3]),

        static_cast<double>(
            speed[4]),

        static_cast<double>(
            speed[5]));


    Eigen::Vector3d physical_position(

        static_cast<double>(
            physical_pose[0]),

        static_cast<double>(
            physical_pose[1]),

        static_cast<double>(
            physical_pose[2]));


    Eigen::Quaterniond physical_orientation(

        static_cast<double>(
            physical_pose[6]),

        static_cast<double>(
            physical_pose[3]),

        static_cast<double>(
            physical_pose[4]),

        static_cast<double>(
            physical_pose[5]));


    if (finite_quaternion(
            physical_orientation)) {

      physical_orientation.normalize();
    }


    Eigen::Vector3d physical_linear_velocity(

        static_cast<double>(
            physical_speed[0]),

        static_cast<double>(
            physical_speed[1]),

        static_cast<double>(
            physical_speed[2]));


    Eigen::Vector3d physical_angular_velocity(

        static_cast<double>(
            physical_speed[3]),

        static_cast<double>(
            physical_speed[4]),

        static_cast<double>(
            physical_speed[5]));


    const bool physical_state_valid =

        rc_physical_pose == 0 &&
        rc_physical_speed == 0 &&
        finite3(physical_position) &&
        finite_quaternion(physical_orientation) &&
        finite3(physical_linear_velocity) &&
        finite3(physical_angular_velocity);


    // ========================================================
    // RECEIVE NEWEST SLAVE FEEDBACK
    // ========================================================

    while (true) {

      double candidate[
          kFeedbackPacketSize]{};


      const ssize_t received =

          recvfrom(
              feedback_socket,
              reinterpret_cast<char*>(
                  candidate),
              sizeof(candidate),
              0,
              nullptr,
              nullptr);


      if (received ==
          static_cast<ssize_t>(
              sizeof(candidate))) {

        std::memcpy(
            feedback,
            candidate,
            sizeof(feedback));


        feedback_received =
            true;


        last_feedback_time =
            now;


      } else {

        break;
      }
    }


    const double feedback_age_ms =

        std::chrono::duration<
            double,
            std::milli>(

            now -
            last_feedback_time)

            .count();


    const bool feedback_live =

        feedback_received &&

        feedback_age_ms <
            kFeedbackStaleMs;


    // ========================================================
    // FRANKA FEEDBACK STATE
    // ========================================================

    Eigen::Vector3d franka_position =
        Eigen::Vector3d::Zero();


    Eigen::Quaterniond franka_orientation =
        Eigen::Quaterniond::Identity();


    Eigen::Vector3d franka_linear_velocity =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d franka_angular_velocity =
        Eigen::Vector3d::Zero();


    if (feedback_received) {

      franka_position =

          Eigen::Vector3d(

              feedback[8],
              feedback[9],
              feedback[10]);


      franka_orientation =

          Eigen::Quaterniond(

              feedback[11],
              feedback[12],
              feedback[13],
              feedback[14]);


      if (finite_quaternion(
              franka_orientation)) {

        franka_orientation.normalize();
      }


      franka_linear_velocity =

          Eigen::Vector3d(

              feedback[15],
              feedback[16],
              feedback[17]);


      franka_angular_velocity =

          Eigen::Vector3d(

              feedback[18],
              feedback[19],
              feedback[20]);
    }


    // Slave reports estimator_ready only AFTER the operator releases the
    // deadman following sufficient free-space calibration.  Therefore there is
    // no timed transition while the operator is moving.  Until the NEXT press
    // the master remains haptically passive.  That next press captures fresh
    // references and starts the 0.4 s haptic ramp.
    const bool estimator_ready_from_slave =

        feedback_live &&
        std::isfinite(feedback[21]) &&
        feedback[21] > 0.5;


    const bool franka_pose_valid =

        feedback_live &&

        finite3(
            franka_position) &&

        finite_quaternion(
            franka_orientation) &&

        finite3(
            franka_linear_velocity) &&

        finite3(
            franka_angular_velocity);


    // ========================================================
    // FRANKA WRENCH
    // ========================================================

    double raw_wrench[6]{};


    if (feedback_live) {

      for (int i = 0;
           i < 6;
           ++i) {

        raw_wrench[i] =
            feedback[
                2 + i];
      }
    }


    // ========================================================
    // SLAVE-COMPENSATED WRENCH
    // ========================================================
    //
    // Do not subtract a fixed startup average here.  The slave performs
    // joint-space model compensation and a continuously updated free-space
    // rotational re-tare.  Re-biasing the signal again on the master would
    // reintroduce the stale-offset problem.

    const Eigen::Vector3d
        force_franka(

            raw_wrench[0],
            raw_wrench[1],
            raw_wrench[2]);


    const Eigen::Vector3d
        torque_franka(

            raw_wrench[3],
            raw_wrench[4],
            raw_wrench[5]);


    // Franka -> Virtuose frame map.

    const Eigen::Vector3d force_mapped =

        v_to_f.transpose() *
        force_franka;


    const Eigen::Vector3d torque_mapped =

        v_to_f.transpose() *
        torque_franka;


    // The slave is intentionally Y-only for this diagnostic.  Keep the
    // rotational haptic channel reciprocal: X/Z torque estimates are logged
    // but are not rendered to the operator in this build.
    Eigen::Vector3d torque_for_haptics =
        torque_mapped;

    if (kBackwardRotationMode ==
        BackwardRotationMode::YOnly) {
      torque_for_haptics.x() = 0.0;
      torque_for_haptics.z() = 0.0;
    }


    const Eigen::Vector3d
        franka_linear_velocity_mapped =

            v_to_f.transpose() *
            franka_linear_velocity;


    const Eigen::Vector3d
        franka_angular_velocity_mapped =

            v_to_f.transpose() *
            franka_angular_velocity;


    // ========================================================
    // API / DEVICE STATUS
    // ========================================================

    const bool api_ok =

        rc_pose == 0 &&
        rc_speed == 0 &&
        rc_analogue == 0 &&
        rc_deadman == 0 &&
        rc_estop == 0 &&
        rc_power == 0 &&
        rc_index == 0 &&
        rc_alarm == 0;


    if (api_ok &&
        deadman == 0) {

      deadman_release_seen =
          true;
    }


    // ========================================================
    // WRENCH VALIDITY
    //
    // IMPORTANT FIX:
    //
    // These flags control ONLY wrench-dependent terms.
    //
    // They are deliberately NOT part of backward_active.
    // ========================================================

    const bool force_safe =

        estimator_ready_from_slave &&

        finite3(
            force_mapped);


    const bool torque_safe =

        estimator_ready_from_slave &&

        finite3(
            torque_mapped);


    // ========================================================
    // CORE BACKWARD COUPLING GATE
    //
    // High external force/torque does NOT disengage this.
    // ========================================================

    const bool backward_active =

        api_ok &&

        feedback_live &&

        estimator_ready_from_slave &&

        franka_pose_valid &&

        finite3(
            virtuose_position) &&

        finite_quaternion(
            virtuose_orientation) &&

        finite3(
            virtuose_linear_velocity) &&

        finite3(
            virtuose_angular_velocity) &&

        deadman_release_seen &&

        deadman == 1 &&

        emergency_stop == 1 &&

        power == 1 &&

        indexing == 0;


    // ========================================================
    // BACKWARD CLUTCH / REFERENCE CAPTURE
    //
    // Reference is captured only when the real core coupling
    // transitions from inactive to active.
    //
    // Crossing a force/torque trust threshold no longer causes
    // a recapture.
    // ========================================================

    if (backward_active &&
        !previous_backward_active) {

      backward_activation_time =
          now;


      virtuose_position_reference =
          virtuose_position;


      franka_position_reference =
          franka_position;


      virtuose_orientation_reference =
          virtuose_orientation;


      franka_orientation_reference =
          franka_orientation;


      backward_reference_ready =
          true;


      for (int i = 0;
           i < 3;
           ++i) {

        filtered_force[i] =
            0.0;

        rendered_force[i] =
            0.0;

        filtered_torque[i] =
            0.0;

        rendered_torque[i] =
            0.0;
      }


      std::cerr
          << "\n[BACKWARD REFERENCE CAPTURED]\n";
    }


    if (!backward_active) {

      backward_reference_ready =
          false;
    }


    previous_backward_active =
        backward_active;


    double haptic_enable_scale =
        0.0;


    if (backward_active &&
        backward_reference_ready) {

      const double since_activation_s =

          std::chrono::duration<double>(
              now - backward_activation_time)
              .count();


      haptic_enable_scale =

          clamp_scalar(

              since_activation_s /
                  kHapticEnableRampSeconds,

              0.0,
              1.0);
    }


    // ========================================================
    // TARGET VIRTUOSE POSE FROM FRANKA RELATIVE MOTION
    // ========================================================

    Eigen::Vector3d target_virtuose_position =
        virtuose_position;


    Eigen::Quaterniond target_virtuose_orientation =
        virtuose_orientation;


    Eigen::Vector3d sync_position_error =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d sync_rotation_error =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d sync_force =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d sync_torque =
        Eigen::Vector3d::Zero();


    if (backward_active &&
        backward_reference_ready) {


      // ======================================================
      // BACKWARD TRANSLATION
      // ======================================================

      if (kEnableBackwardPositionCoupling) {

        const Eigen::Vector3d
            franka_delta =

                franka_position -
                franka_position_reference;


        const Eigen::Vector3d
            virtuose_delta_target =

                v_to_f.transpose() *
                franka_delta;


        target_virtuose_position =

            virtuose_position_reference +
            virtuose_delta_target;


        sync_position_error =

            target_virtuose_position -
            virtuose_position;


        sync_position_error =

            clamp_vector_norm(

                sync_position_error,

                kMaxBackwardPositionErrorM);


        sync_force =

            kBackwardPositionStiffness *
                sync_position_error +

            kBackwardPositionDamping *

                (
                    franka_linear_velocity_mapped -
                    virtuose_linear_velocity);
      }


      // ======================================================
      // BACKWARD ROTATION
      // ======================================================

      if (kEnableBackwardRotationCoupling) {

        const Eigen::Matrix3d
            franka_delta_rotation =

                franka_orientation
                    .toRotationMatrix() *

                franka_orientation_reference
                    .toRotationMatrix()
                    .transpose();


        Eigen::Matrix3d
            virtuose_delta_target =

                v_to_f.transpose() *

                franka_delta_rotation *

                v_to_f;


        if (kBackwardRotationMode ==
            BackwardRotationMode::YOnly) {

          const Eigen::AngleAxisd delta_aa(
              virtuose_delta_target);

          Eigen::Vector3d delta_rotvec =
              Eigen::Vector3d::Zero();

          if (std::isfinite(delta_aa.angle()) &&
              delta_aa.axis().allFinite()) {
            delta_rotvec =
                delta_aa.axis() * delta_aa.angle();
          }

          virtuose_delta_target =
              Eigen::AngleAxisd(
                  delta_rotvec.y(),
                  Eigen::Vector3d::UnitY())
                  .toRotationMatrix();
        }


        const Eigen::Matrix3d
            target_rotation =

                virtuose_delta_target *

                virtuose_orientation_reference
                    .toRotationMatrix();


        target_virtuose_orientation =

            Eigen::Quaterniond(
                target_rotation);


        target_virtuose_orientation.normalize();


        sync_rotation_error =

            orientation_error_world(

                virtuose_orientation,

                target_virtuose_orientation);


        Eigen::Vector3d sync_angular_velocity_error =

            franka_angular_velocity_mapped -
            virtuose_angular_velocity;


        if (kBackwardRotationMode ==
            BackwardRotationMode::YOnly) {

          sync_rotation_error.x() = 0.0;
          sync_rotation_error.z() = 0.0;

          sync_angular_velocity_error.x() = 0.0;
          sync_angular_velocity_error.z() = 0.0;
        }


        sync_rotation_error =

            clamp_vector_norm(

                sync_rotation_error,

                kMaxBackwardRotationErrorRad);


        sync_torque =

            kBackwardRotationStiffness *
                sync_rotation_error +

            kBackwardRotationDamping *
                sync_angular_velocity_error;
      }
    }


    // ========================================================
    // WRENCH REFLECTION
    //
    // Finite debiased wrench remains active regardless of magnitude.
    // Final output saturation is applied after filtering/slew limiting.
    // ========================================================

    Eigen::Vector3d reflection_force =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d reflection_torque =
        Eigen::Vector3d::Zero();


    if (kEnableForceReflection &&
        force_safe) {

      reflection_force =

          -kForceReflectionGain *
          force_mapped;
    }


    if (kEnableTorqueReflection &&
        torque_safe) {

      reflection_torque =

          -kTorqueReflectionGain *
          torque_for_haptics;
    }


    // ========================================================
    // DIRECTIONAL DAMPING
    //
    // This follows the projection idea from the reference code,
    // but adapts it correctly to this Virtuose-Franka system.
    //
    // The reference normalized a full 6-D wrench and projected a
    // full 6-D twist onto it. That mixes N with Nm and m/s with
    // rad/s. Here translation and rotation are separate:
    //
    //   nF = F / ||F||
    //   v_parallel = (vV . nF) nF
    //   Fd = -bF v_parallel
    //
    //   nM = M / ||M||
    //   w_parallel = (wV . nM) nM
    //   Md = -bM w_parallel
    //
    // Each term is dissipative with respect to Virtuose motion:
    //
    //   Fd . vV = -bF ||v_parallel||^2 <= 0
    //   Md . wV = -bM ||w_parallel||^2 <= 0
    //
    // IMPORTANT:
    //   - The measured Franka wrench defines only the damping axis.
    //   - Force/torque reflection remains a completely separate term.
    //   - No directional damping is generated below its own wrench
    //     threshold, which avoids normalizing noise near zero.
    // ========================================================

    Eigen::Vector3d directional_force =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d directional_torque =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d force_direction =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d torque_direction =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d parallel_linear_velocity =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d parallel_angular_velocity =
        Eigen::Vector3d::Zero();


    bool directional_force_active =
        false;


    bool directional_torque_active =
        false;


    if (kEnableDirectionalForceDamping &&
        force_safe) {

      const double force_norm =
          force_mapped.norm();


      if (force_norm >
          kDirectionalForceThresholdN) {

        force_direction =

            force_mapped /
            force_norm;


        parallel_linear_velocity =

            force_direction *

            virtuose_linear_velocity.dot(
                force_direction);


        directional_force =

            -kDirectionalLinearDamping *

            parallel_linear_velocity;


        directional_force_active =
            true;
      }
    }


    if (kEnableDirectionalTorqueDamping &&
        torque_safe) {

      const double torque_norm =
          torque_for_haptics.norm();


      if (torque_norm >
          kDirectionalTorqueThresholdNm) {

        torque_direction =

            torque_for_haptics /
            torque_norm;


        parallel_angular_velocity =

            torque_direction *

            virtuose_angular_velocity.dot(
                torque_direction);


        directional_torque =

            -kDirectionalAngularDamping *

            parallel_angular_velocity;


        directional_torque_active =
            true;
      }
    }


    const double directional_linear_power =

        directional_force.dot(
            virtuose_linear_velocity);


    const double directional_rotational_power =

        directional_torque.dot(
            virtuose_angular_velocity);


    // ========================================================
    // TOTAL BACKWARD COMMAND
    // ========================================================

    Eigen::Vector3d desired_force =
        Eigen::Vector3d::Zero();


    Eigen::Vector3d desired_torque =
        Eigen::Vector3d::Zero();


    if (backward_active &&
        backward_reference_ready) {

      desired_force =

          haptic_enable_scale *

          (sync_force +
           reflection_force +
           directional_force);


      desired_torque =

          haptic_enable_scale *

          (sync_torque +
           reflection_torque +
           directional_torque);
    }


    // ========================================================
    // LOW-PASS + SLEW LIMIT
    // ========================================================

    bool force_rate_limited =
        false;


    bool torque_rate_limited =
        false;


    for (int i = 0;
         i < 3;
         ++i) {

      filtered_force[i] =

          kForceFilterAlpha *
              desired_force[i] +

          (
              1.0 -
              kForceFilterAlpha) *

              filtered_force[i];


      const double next_force =

          rate_limit(

              filtered_force[i],

              rendered_force[i],

              kMaxForceStepN);


      if (std::abs(
              next_force -
              filtered_force[i]) >
          1e-12) {

        force_rate_limited =
            true;
      }


      rendered_force[i] =
          next_force;


      filtered_torque[i] =

          kTorqueFilterAlpha *
              desired_torque[i] +

          (
              1.0 -
              kTorqueFilterAlpha) *

              filtered_torque[i];


      const double next_torque =

          rate_limit(

              filtered_torque[i],

              rendered_torque[i],

              kMaxTorqueStepNm);


      if (std::abs(
              next_torque -
              filtered_torque[i]) >
          1e-12) {

        torque_rate_limited =
            true;
      }


      rendered_torque[i] =
          next_torque;
    }


    // ========================================================
    // FINAL VIRTUOSE WRENCH SATURATION
    //
    // Saturate by vector norm. Demand beyond the ceiling remains
    // pinned at the ceiling instead of being zeroed or disabled.
    // ========================================================

    const Eigen::Vector3d rendered_force_vector =

        clamp_vector_norm(

            Eigen::Vector3d(
                rendered_force[0],
                rendered_force[1],
                rendered_force[2]),

            kDeviceForceCeilingN);


    const Eigen::Vector3d rendered_torque_vector =

        clamp_vector_norm(

            Eigen::Vector3d(
                rendered_torque[0],
                rendered_torque[1],
                rendered_torque[2]),

            kDeviceTorqueCeilingNm);


    for (int i = 0;
         i < 3;
         ++i) {

      rendered_force[i] =
          rendered_force_vector[i];

      rendered_torque[i] =
          rendered_torque_vector[i];
    }


    const bool force_near_ceiling =

        norm3(
            rendered_force) >=

        kNearCeilingFraction *
            kDeviceForceCeilingN;


    const bool torque_near_ceiling =

        norm3(
            rendered_torque) >=

        kNearCeilingFraction *
            kDeviceTorqueCeilingNm;


    // ========================================================
    // APPLY WRENCH TO VIRTUOSE
    // ========================================================

    int set_force_rc =
        0;


    bool wrench_applied =
        false;


    if (backward_active &&
        backward_reference_ready) {

      float wrench[6] = {

          static_cast<float>(
              rendered_force[0]),

          static_cast<float>(
              rendered_force[1]),

          static_cast<float>(
              rendered_force[2]),

          static_cast<float>(
              rendered_torque[0]),

          static_cast<float>(
              rendered_torque[1]),

          static_cast<float>(
              rendered_torque[2])};


      set_force_rc =

          virtSetForce(
              context,
              wrench);


      wrench_applied =

          set_force_rc ==
          0;


      if (!wrench_applied) {

        zero_wrench(
            context);
      }


    } else {

      zero_wrench(
          context);


      for (int i = 0;
           i < 3;
           ++i) {

        filtered_force[i] =
            0.0;

        rendered_force[i] =
            0.0;

        filtered_torque[i] =
            0.0;

        rendered_torque[i] =
            0.0;
      }
    }


    // ========================================================
    // HAPTIC POWER
    // ========================================================

    double linear_power =
        0.0;


    double rotational_power =
        0.0;


    if (wrench_applied) {

      linear_power =

          rendered_force[0] *
              speed[0] +

          rendered_force[1] *
              speed[1] +

          rendered_force[2] *
              speed[2];


      rotational_power =

          rendered_torque[0] *
              speed[3] +

          rendered_torque[1] *
              speed[4] +

          rendered_torque[2] *
              speed[5];
    }


    const double total_power =

        linear_power +
        rotational_power;


    // ========================================================
    // MASTER -> SLAVE
    // ========================================================

    const double current_alpha =

        alpha_command.load(
            std::memory_order_relaxed);


    const double effective_force_gain =

        kEnableForceReflection

            ? kForceReflectionGain

            : 0.0;


    const double effective_torque_gain =

        kEnableTorqueReflection

            ? kTorqueReflectionGain

            : 0.0;


    const double command_packet[
        kCommandPacketSize] = {

        static_cast<double>(
            packet_id),

        elapsed_us(
            start_time),

        static_cast<double>(
            pose[6]),

        static_cast<double>(
            pose[3]),

        static_cast<double>(
            pose[4]),

        static_cast<double>(
            pose[5]),

        static_cast<double>(
            pose[0]),

        static_cast<double>(
            pose[1]),

        static_cast<double>(
            pose[2]),

        effective_force_gain,

        effective_torque_gain,

        current_alpha,

        static_cast<double>(
            deadman),

        static_cast<double>(
            emergency_stop)};


    sendto(
        command_socket,
        command_packet,
        sizeof(command_packet),
        0,
        reinterpret_cast<sockaddr*>(
            &slave_address),
        sizeof(slave_address));


    // ========================================================
    // LOG SAMPLE
    // ========================================================

    MasterLogSample s{};


    s.time_s =
        elapsed_seconds(
            start_time);


    s.loop_dt_ms =
        loop_dt_ms;


    s.packet_id =
        packet_id;


    s.backward_active =

        backward_active
            ? 1
            : 0;


    s.backward_reference_ready =

        backward_reference_ready
            ? 1
            : 0;


    s.force_safe =

        force_safe
            ? 1
            : 0;


    s.torque_safe =

        torque_safe
            ? 1
            : 0;


    s.alpha_command =
        current_alpha;


    s.g_force =
        effective_force_gain;


    s.g_torque =
        effective_torque_gain;


    s.virtuose_x =
        virtuose_position.x();

    s.virtuose_y =
        virtuose_position.y();

    s.virtuose_z =
        virtuose_position.z();


    s.virtuose_qw =
        virtuose_orientation.w();

    s.virtuose_qx =
        virtuose_orientation.x();

    s.virtuose_qy =
        virtuose_orientation.y();

    s.virtuose_qz =
        virtuose_orientation.z();


    s.virtuose_vx =
        virtuose_linear_velocity.x();

    s.virtuose_vy =
        virtuose_linear_velocity.y();

    s.virtuose_vz =
        virtuose_linear_velocity.z();


    s.virtuose_wx =
        virtuose_angular_velocity.x();

    s.virtuose_wy =
        virtuose_angular_velocity.y();

    s.virtuose_wz =
        virtuose_angular_velocity.z();


    s.physical_x =
        physical_position.x();

    s.physical_y =
        physical_position.y();

    s.physical_z =
        physical_position.z();


    s.physical_qw =
        physical_orientation.w();

    s.physical_qx =
        physical_orientation.x();

    s.physical_qy =
        physical_orientation.y();

    s.physical_qz =
        physical_orientation.z();


    s.physical_vx =
        physical_linear_velocity.x();

    s.physical_vy =
        physical_linear_velocity.y();

    s.physical_vz =
        physical_linear_velocity.z();


    s.physical_wx =
        physical_angular_velocity.x();

    s.physical_wy =
        physical_angular_velocity.y();

    s.physical_wz =
        physical_angular_velocity.z();


    s.physical_pose_rc =
        rc_physical_pose;

    s.physical_speed_rc =
        rc_physical_speed;

    s.physical_state_valid =
        physical_state_valid
            ? 1
            : 0;


    s.franka_x =
        franka_position.x();

    s.franka_y =
        franka_position.y();

    s.franka_z =
        franka_position.z();


    s.franka_qw =
        franka_orientation.w();

    s.franka_qx =
        franka_orientation.x();

    s.franka_qy =
        franka_orientation.y();

    s.franka_qz =
        franka_orientation.z();


    s.franka_vx =
        franka_linear_velocity.x();

    s.franka_vy =
        franka_linear_velocity.y();

    s.franka_vz =
        franka_linear_velocity.z();


    s.franka_wx =
        franka_angular_velocity.x();

    s.franka_wy =
        franka_angular_velocity.y();

    s.franka_wz =
        franka_angular_velocity.z();


    s.target_x =
        target_virtuose_position.x();

    s.target_y =
        target_virtuose_position.y();

    s.target_z =
        target_virtuose_position.z();


    s.target_qw =
        target_virtuose_orientation.w();

    s.target_qx =
        target_virtuose_orientation.x();

    s.target_qy =
        target_virtuose_orientation.y();

    s.target_qz =
        target_virtuose_orientation.z();


    s.sync_error_x =
        sync_position_error.x();

    s.sync_error_y =
        sync_position_error.y();

    s.sync_error_z =
        sync_position_error.z();


    s.sync_rot_error_x =
        sync_rotation_error.x();

    s.sync_rot_error_y =
        sync_rotation_error.y();

    s.sync_rot_error_z =
        sync_rotation_error.z();


    s.franka_fx =
        force_franka.x();

    s.franka_fy =
        force_franka.y();

    s.franka_fz =
        force_franka.z();


    s.franka_mx =
        torque_franka.x();

    s.franka_my =
        torque_franka.y();

    s.franka_mz =
        torque_franka.z();


    s.sync_fx =
        sync_force.x();

    s.sync_fy =
        sync_force.y();

    s.sync_fz =
        sync_force.z();


    s.sync_mx =
        sync_torque.x();

    s.sync_my =
        sync_torque.y();

    s.sync_mz =
        sync_torque.z();


    s.reflect_fx =
        reflection_force.x();

    s.reflect_fy =
        reflection_force.y();

    s.reflect_fz =
        reflection_force.z();


    s.reflect_mx =
        reflection_torque.x();

    s.reflect_my =
        reflection_torque.y();

    s.reflect_mz =
        reflection_torque.z();


    s.directional_fx =
        directional_force.x();

    s.directional_fy =
        directional_force.y();

    s.directional_fz =
        directional_force.z();


    s.directional_mx =
        directional_torque.x();

    s.directional_my =
        directional_torque.y();

    s.directional_mz =
        directional_torque.z();


    s.force_direction_x =
        force_direction.x();

    s.force_direction_y =
        force_direction.y();

    s.force_direction_z =
        force_direction.z();


    s.parallel_vx =
        parallel_linear_velocity.x();

    s.parallel_vy =
        parallel_linear_velocity.y();

    s.parallel_vz =
        parallel_linear_velocity.z();


    s.torque_direction_x =
        torque_direction.x();

    s.torque_direction_y =
        torque_direction.y();

    s.torque_direction_z =
        torque_direction.z();


    s.parallel_wx =
        parallel_angular_velocity.x();

    s.parallel_wy =
        parallel_angular_velocity.y();

    s.parallel_wz =
        parallel_angular_velocity.z();


    s.directional_linear_power =
        directional_linear_power;


    s.directional_rotational_power =
        directional_rotational_power;


    s.directional_force_active =
        directional_force_active
            ? 1
            : 0;


    s.directional_torque_active =
        directional_torque_active
            ? 1
            : 0;


    s.desired_fx =
        desired_force.x();

    s.desired_fy =
        desired_force.y();

    s.desired_fz =
        desired_force.z();


    s.desired_mx =
        desired_torque.x();

    s.desired_my =
        desired_torque.y();

    s.desired_mz =
        desired_torque.z();


    s.rendered_fx =
        rendered_force[0];

    s.rendered_fy =
        rendered_force[1];

    s.rendered_fz =
        rendered_force[2];


    s.rendered_mx =
        rendered_torque[0];

    s.rendered_my =
        rendered_torque[1];

    s.rendered_mz =
        rendered_torque[2];


    s.linear_power =
        linear_power;


    s.rotational_power =
        rotational_power;


    s.total_power =
        total_power;


    s.force_rate_limited =

        force_rate_limited
            ? 1
            : 0;


    s.torque_rate_limited =

        torque_rate_limited
            ? 1
            : 0;


    s.force_near_ceiling =

        force_near_ceiling
            ? 1
            : 0;


    s.torque_near_ceiling =

        torque_near_ceiling
            ? 1
            : 0;


    s.deadman =
        deadman;


    s.estop =
        emergency_stop;


    s.power_on =
        power;


    s.indexing =
        indexing;


    s.render_on =

        wrench_applied
            ? 1
            : 0;


    s.feedback_age_ms =
        feedback_age_ms;


    s.alarm =

        static_cast<uint32_t>(
            alarm);


    s.set_force_rc =
        set_force_rc;


    if (!log_queue.push(
            s)) {

      dropped_samples.fetch_add(
          1,
          std::memory_order_relaxed);
    }


    ++packet_id;


    // ========================================================
    // TERMINAL OUTPUT
    // ========================================================

    ++terminal_counter;


    if (terminal_counter >=
        kConsolePrintEveryCycles) {

      terminal_counter =
          0;


      std::fprintf(

          stderr,

          "\n[MASTER BILATERAL] "
          "dt=%.3f ms active=%d ref=%d armed=%d "
          "forceValid=%d torqueValid=%d alpha=%.2f\n"

          "ePos=[%+.4f %+.4f %+.4f] m "
          "Fsync=[%+.2f %+.2f %+.2f] N\n"

          "eRot=[%+.3f %+.3f %+.3f] rad "
          "Msync=[%+.3f %+.3f %+.3f] Nm\n"

          "Fext=[%+.2f %+.2f %+.2f] "
          "Fdamp=[%+.2f %+.2f %+.2f] "
          "Fcmd=[%+.2f %+.2f %+.2f]\n"

          "Mext=[%+.3f %+.3f %+.3f] "
          "Mcmd=[%+.3f %+.3f %+.3f]\n"

          "PdirLin=%+.4f W PdirRot=%+.4f W "
          "dirF=%d dirM=%d\n"

          "P=%+.4f W [%s] age=%.2f ms "
          "rateF=%d rateM=%d render=%d\n",

          loop_dt_ms,

          backward_active
              ? 1
              : 0,

          backward_reference_ready
              ? 1
              : 0,

          deadman_release_seen
              ? 1
              : 0,

          force_safe
              ? 1
              : 0,

          torque_safe
              ? 1
              : 0,

          current_alpha,

          sync_position_error.x(),
          sync_position_error.y(),
          sync_position_error.z(),

          sync_force.x(),
          sync_force.y(),
          sync_force.z(),

          sync_rotation_error.x(),
          sync_rotation_error.y(),
          sync_rotation_error.z(),

          sync_torque.x(),
          sync_torque.y(),
          sync_torque.z(),

          force_franka.x(),
          force_franka.y(),
          force_franka.z(),

          directional_force.x(),
          directional_force.y(),
          directional_force.z(),

          rendered_force[0],
          rendered_force[1],
          rendered_force[2],

          torque_franka.x(),
          torque_franka.y(),
          torque_franka.z(),

          rendered_torque[0],
          rendered_torque[1],
          rendered_torque[2],

          directional_linear_power,
          directional_rotational_power,

          directional_force_active
              ? 1
              : 0,

          directional_torque_active
              ? 1
              : 0,

          total_power,

          power_label(
              total_power),

          feedback_age_ms,

          force_rate_limited
              ? 1
              : 0,

          torque_rate_limited
              ? 1
              : 0,

          wrench_applied
              ? 1
              : 0);
    }


    // ========================================================
    // 1 kHz TIMING
    // ========================================================

    const auto after_work =

        std::chrono::steady_clock::now();


    if (after_work >

        next_tick +
            std::chrono::milliseconds(
                5)) {

      next_tick =
          after_work;
    }


    std::this_thread::sleep_until(
        next_tick);
  }


  // ==========================================================
  // SHUTDOWN
  // ==========================================================

  g_stop_requested =
      1;


  if (alpha_thread.joinable()) {

    alpha_thread.join();
  }


  zero_wrench(
      context);


  virtEnableForceFeedback(
      context,
      0);


  virtSetPowerOn(
      context,
      0);


  close(
      feedback_socket);


  close(
      command_socket);


  virtClose(
      context);


  logger_stop.store(
      true,
      std::memory_order_relaxed);


  logger_thread.join();


  csv.close();


  std::cout
      << "\nMaster log: "
      << log_name
      << "\nDropped samples: "
      << dropped_samples.load()
      << '\n';


  return 0;
}


