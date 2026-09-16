// ============================================================
// slaveo.cpp
//
// FRANKA SLAVE
// 1 kHz FULL 6-DOF VARIABLE-IMPEDANCE TELEOPERATION
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
// SLAVE -> MASTER feedback: 21 doubles
//
// [0]      packet id
// [1]      timestamp [us]
// [2..7]   Fx Fy Fz Mx My Mz
// [8..10]  Franka x y z
// [11..14] Franka qw qx qy qz
// [15..17] Franka vx vy vz
// [18..20] Franka wx wy wz
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
// Fixed rotation:
//
// Krot = 8 Nm/rad
// Drot = 2 Nms/rad
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
    21;


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
// FIXED ROTATIONAL IMPEDANCE
// ------------------------------------------------------------

constexpr double kRotationalStiffness =
    12.0;

constexpr double kRotationalDamping =
    4.0;


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
    8.0 ; //was 8

constexpr double kNullspaceDamping =
    2.5; //was 2.5


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

  double k_rot;
  double d_rot;

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

      "slave_backward_position_" +
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
      << "k_rot,"
      << "d_rot,"

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
      << "angular_speed_limited\n";


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
                << s.k_rot << ','
                << s.d_rot << ','

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
                << s.angular_speed_limited
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

        << "Krot                      : "
        << kRotationalStiffness
        << " Nm/rad\n"

        << "Drot                      : "
        << kRotationalDamping
        << " Nms/rad\n"

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

        kRotationalStiffness *
        Eigen::Matrix3d::
            Identity();


    damping.bottomRightCorner<
        3,
        3>() =

        kRotationalDamping *
        Eigen::Matrix3d::
            Identity();


    const auto control_start =
        std::chrono::steady_clock::now();


    uint64_t feedback_id =
        0;


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


          const std::array<double, 7>
              coriolis_array =

                  model.coriolis(
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
                  7,
                  1>>
              coriolis(
                  coriolis_array.data());


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

          for (std::size_t i = 0;
               i < 6;
               ++i) {

            feedback_packet[
                2 + i]
                .store(

                    state.O_F_ext_hat_K[i],

                    std::memory_order_relaxed);
          }


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

              kRotationalStiffness *

              Eigen::Matrix3d::
                  Identity();


          damping.bottomRightCorner<
              3,
              3>() =

              kRotationalDamping *

              Eigen::Matrix3d::
                  Identity();


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


              mapped_delta_rotation =

                  limit_relative_rotation(

                      mapped_delta_rotation,

                      kMaxRelativeRotationRad,

                      &rotation_workspace_limited);


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
              7,
              1>
              task_torque =

                  jacobian.transpose() *

                  (
                      -stiffness *
                          error

                      -

                      damping *
                          damping_velocity);


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


          s.k_rot =
              kRotationalStiffness;


          s.d_rot =
              kRotationalDamping;


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


          s.fx =
              state.O_F_ext_hat_K[0];

          s.fy =
              state.O_F_ext_hat_K[1];

          s.fz =
              state.O_F_ext_hat_K[2];


          s.mx =
              state.O_F_ext_hat_K[3];

          s.my =
              state.O_F_ext_hat_K[4];

          s.mz =
              state.O_F_ext_hat_K[5];


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


