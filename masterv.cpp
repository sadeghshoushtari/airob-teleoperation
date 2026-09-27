// ============================================================
// master_bilateral_variable_impedance.cpp
//
// VIRTUOSE MASTER
// 1 kHz BILATERAL VARIABLE-IMPEDANCE TELEOPERATION
//
// The Virtuose analogue finger trigger commands variable impedance
// on BOTH sides of the bilateral system:
//   - Franka slave Cartesian translation + rotation
//   - Virtuose-side virtual translational + rotational coupling
// The same normalized trigger command drives separate Franka and master-side
// impedance mappings. Force/torque reflection gains remain independent.
//
// CURRENT STAGE:
//   FULL 6-DOF BACKWARD POSITION/ORIENTATION COUPLING
//   + DIMENSIONALLY-CORRECT DIRECTIONAL DAMPING
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
// [11]     translational stiffness command Kt [N/m]
//          (slave derives the matching rotational K from the same normalized command)
//          generated from Virtuose analogue finger trigger (300..700 N/m)
// [12]     deadman
// [13]     emergency stop
//
// SLAVE -> MASTER feedback: 21 doubles
//
// [0]      packet id
// [1]      timestamp us
// [2..7]   Fx Fy Fz Mx My Mz
// [8..10]  Franka x y z
// [11..14] Franka qw qx qy qz
// [15..17] Franka vx vy vz
// [18..20] Franka wx wy wz
// [21]     Franka applied translational stiffness Kt [N/m]
// [22]     Franka applied rotational stiffness Kr [Nm/rad]
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
#include "teleop_common.hpp"
#include "master_config.hpp"


namespace {




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


double normalize_analogue_trigger(
    double raw_value) {

  const double denominator =
      kAnalogFullyPressedValue -
      kAnalogReleasedValue;

  if (!std::isfinite(raw_value) ||
      std::abs(denominator) < 1e-12) {
    return 0.0;
  }

  double normalized =
      (raw_value - kAnalogReleasedValue) /
      denominator;

  normalized =
      clamp_scalar(normalized, 0.0, 1.0);

  // Keep the first small part of lever travel at minimum stiffness,
  // then remap the remaining travel back to the full 0..1 range.
  if (normalized <= kAnalogLowDeadzone) {
    return 0.0;
  }

  normalized =
      (normalized - kAnalogLowDeadzone) /
      (1.0 - kAnalogLowDeadzone);

  return clamp_scalar(normalized, 0.0, 1.0);
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
  // ANALOGUE-GRIPPER STIFFNESS COMMAND
  // ==========================================================
  // The actual command is updated in the 1 kHz loop from the
  // Virtuose finger trigger.  Start safely at minimum stiffness.

  double analog_trigger_filtered =
      0.0;

  bool analog_trigger_initialized =
      false;

  double current_stiffness =
      kMinimumTranslationalStiffness;

  double current_rotational_stiffness =
      kMinimumRotationalStiffness;

  // Requested and applied Virtuose-side virtual impedance.
  double current_master_translational_stiffness_command =
      kMinimumBackwardPositionStiffness;

  double current_master_rotational_stiffness_command =
      kMinimumBackwardRotationStiffness;

  double current_master_translational_stiffness_applied =
      kMinimumBackwardPositionStiffness;

  double current_master_rotational_stiffness_applied =
      kMinimumBackwardRotationStiffness;

  double current_master_translational_damping_applied =
      kBaselineBackwardPositionDamping *
      std::sqrt(
          kMinimumBackwardPositionStiffness /
          kBaselineBackwardPositionStiffness);

  double current_master_rotational_damping_applied =
      kBaselineBackwardRotationDamping *
      std::sqrt(
          kMinimumBackwardRotationStiffness /
          kBaselineBackwardRotationStiffness);

  double current_analog_raw =
      kAnalogReleasedValue;

  double current_analog_normalized =
      0.0;


  // ==========================================================
  // VIRTUOSE INITIALIZATION
  // ==========================================================

  VirtContext context =

      virtOpen(
          "127.0.0.1#53210");


  if (!context) {

    std::cerr
        << "ERROR: virtOpen failed.\n";


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


    return 1;
  }


  // ==========================================================
  // CONSOLE THREAD (QUIT ONLY)
  // ==========================================================

  std::thread console_thread(

      [&]() {

        std::cout

            << "\n============================================\n"
            << "CONTROLLER CONFIGURATION: MASTER\n"
            << "============================================\n"

            << "Loop rate                     : "
            << kLoopHz
            << " Hz\n"

            << "Backward translation coupling : "
            << (kEnableBackwardPositionCoupling ? "ON" : "OFF")
            << "\n"

            << "Backward rotation coupling    : "
            << (kEnableBackwardRotationCoupling ? "ON" : "OFF")
            << "\n"

            << "Directional force damping     : "
            << (kEnableDirectionalForceDamping ? "ON" : "OFF")
            << "\n"

            << "Directional torque damping    : "
            << (kEnableDirectionalTorqueDamping ? "ON" : "OFF")
            << "\n"

            << "Force reflection              : "
            << (kEnableForceReflection ? "ON" : "OFF")
            << "\n"

            << "Torque reflection             : "
            << (kEnableTorqueReflection ? "ON" : "OFF")
            << "\n"

            << "Analogue stiffness channel    : "
            << kStiffnessAnalogChannel
            << "\n"

            << "Analogue released/full scale  : "
            << kAnalogReleasedValue
            << " / "
            << kAnalogFullyPressedValue
            << "\n"

            << "Analogue low deadzone         : "
            << 100.0 * kAnalogLowDeadzone
            << " %\n"

            << "Translational K range         : "
            << kMinimumTranslationalStiffness
            << " ... "
            << kMaximumTranslationalStiffness
            << " N/m\n"

            << "Franka rotational K range     : "
            << kMinimumRotationalStiffness
            << " ... "
            << kMaximumRotationalStiffness
            << " Nm/rad\n"

            << "Master virtual Kt range       : "
            << kMinimumBackwardPositionStiffness
            << " ... "
            << kMaximumBackwardPositionStiffness
            << " N/m\n"

            << "Master virtual Kr range       : "
            << kMinimumBackwardRotationStiffness
            << " ... "
            << kMaximumBackwardRotationStiffness
            << " Nm/rad\n"

            << "============================================\n\n"

            << "The Virtuose finger trigger commands variable impedance on BOTH sides.\n"
            << "Franka: variable translational + rotational K/D.\n"
            << "Virtuose virtual coupling: variable translational + rotational K/D.\n"
            << "Released/light press = minimum impedance; full press = maximum impedance.\n"
            << "Type q to quit.\n\n";

        pollfd descriptor{};
        descriptor.fd = STDIN_FILENO;
        descriptor.events = POLLIN;

        while (!g_stop_requested) {

          const int result =
              poll(&descriptor, 1, 200);

          if (result <= 0 ||
              !(descriptor.revents & POLLIN)) {
            continue;
          }

          std::string line;

          if (!std::getline(std::cin, line)) {
            continue;
          }

          if (line == "q" || line == "Q") {
            g_stop_requested = 1;
            break;
          }

          std::cerr
              << "Stiffness is controlled by the analogue trigger; type q to quit.\n";
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

  double bias_sum[6]{};

  double wrench_bias[6]{};

  int bias_count =
      0;

  bool bias_ready =
      false;


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


    // ========================================================
    // ANALOGUE TRIGGER -> TRANSLATIONAL + ROTATIONAL STIFFNESS
    // ========================================================
    // Haption's finger trigger is treated as a normalized 0..1 input.
    // A small low-end deadzone keeps a light touch at minimum stiffness.
    // Input is smoothed here. The slave applies independent translational
    // and rotational stiffness slew limits before using the commands.

    if (rc_analogue == 0 &&
        kStiffnessAnalogChannel < 7 &&
        std::isfinite(
            static_cast<double>(
                analogue[kStiffnessAnalogChannel]))) {

      current_analog_raw =
          static_cast<double>(
              analogue[kStiffnessAnalogChannel]);

      const double normalized_raw =
          normalize_analogue_trigger(
              current_analog_raw);

      if (!analog_trigger_initialized) {

        analog_trigger_filtered =
            normalized_raw;

        analog_trigger_initialized =
            true;

      } else {

        const double loop_dt_s =
            std::max(
                loop_dt_ms * 1e-3,
                1.0 / kLoopHz);

        const double analog_alpha =
            loop_dt_s /
            (kAnalogFilterTimeConstantS +
             loop_dt_s);

        analog_trigger_filtered +=
            analog_alpha *
            (normalized_raw -
             analog_trigger_filtered);
      }

      current_analog_normalized =
          clamp_scalar(
              analog_trigger_filtered,
              0.0,
              1.0);

      current_stiffness =
          kMinimumTranslationalStiffness +
          current_analog_normalized *
              (kMaximumTranslationalStiffness -
               kMinimumTranslationalStiffness);

      current_rotational_stiffness =
          kMinimumRotationalStiffness +
          current_analog_normalized *
              (kMaximumRotationalStiffness -
               kMinimumRotationalStiffness);

      current_master_translational_stiffness_command =
          kMinimumBackwardPositionStiffness +
          current_analog_normalized *
              (kMaximumBackwardPositionStiffness -
               kMinimumBackwardPositionStiffness);

      current_master_rotational_stiffness_command =
          kMinimumBackwardRotationStiffness +
          current_analog_normalized *
              (kMaximumBackwardRotationStiffness -
               kMinimumBackwardRotationStiffness);
    }

    // Apply independent master-side slew limits. Full trigger travel
    // changes the master and slave stiffnesses over approximately 2 s.
    const double master_impedance_dt_s =
        std::max(
            loop_dt_ms * 1e-3,
            1.0 / kLoopHz);

    current_master_translational_stiffness_applied =
        rate_limit(
            current_master_translational_stiffness_command,
            current_master_translational_stiffness_applied,
            kMaximumBackwardPositionStiffnessRate *
                master_impedance_dt_s);

    current_master_rotational_stiffness_applied =
        rate_limit(
            current_master_rotational_stiffness_command,
            current_master_rotational_stiffness_applied,
            kMaximumBackwardRotationStiffnessRate *
                master_impedance_dt_s);

    current_master_translational_damping_applied =
        kBaselineBackwardPositionDamping *
        std::sqrt(
            current_master_translational_stiffness_applied /
            kBaselineBackwardPositionStiffness);

    current_master_rotational_damping_applied =
        kBaselineBackwardRotationDamping *
        std::sqrt(
            current_master_rotational_stiffness_applied /
            kBaselineBackwardRotationStiffness);


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
    // STARTUP WRENCH BIAS
    // ========================================================

    if (feedback_live &&
        !bias_ready) {

      for (int i = 0;
           i < 6;
           ++i) {

        bias_sum[i] +=
            raw_wrench[i];
      }


      ++bias_count;


      if (bias_count >=
          kBiasSamplesRequired) {

        for (int i = 0;
             i < 6;
             ++i) {

          wrench_bias[i] =

              bias_sum[i] /

              static_cast<double>(
                  bias_count);
        }


        bias_ready =
            true;


        std::cerr
            << "\n[WRENCH BIAS READY]\n";
      }
    }


    double debiased[6]{};


    if (feedback_live &&
        bias_ready) {

      for (int i = 0;
           i < 6;
           ++i) {

        debiased[i] =

            raw_wrench[i] -
            wrench_bias[i];
      }
    }


    const Eigen::Vector3d
        force_franka(

            debiased[0],
            debiased[1],
            debiased[2]);


    const Eigen::Vector3d
        torque_franka(

            debiased[3],
            debiased[4],
            debiased[5]);


    // Franka -> Virtuose frame map.

    const Eigen::Vector3d force_mapped =

        v_to_f.transpose() *
        force_franka;


    const Eigen::Vector3d torque_mapped =

        v_to_f.transpose() *
        torque_franka;


    const Eigen::Vector3d
        franka_linear_velocity_mapped =

            v_to_f.transpose() *
            franka_linear_velocity;


    const Eigen::Vector3d
        franka_angular_velocity_mapped =

            v_to_f.transpose() *
            franka_angular_velocity;


    double franka_translational_stiffness_applied =
        feedback_received ? feedback[21] : 0.0;

    double franka_rotational_stiffness_applied =
        feedback_received ? feedback[22] : 0.0;


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

        bias_ready &&

        finite3(
            force_mapped);


    const bool torque_safe =

        bias_ready &&

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

            current_master_translational_stiffness_applied *
                sync_position_error +

            current_master_translational_damping_applied *

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


        const Eigen::Matrix3d
            virtuose_delta_target =

                v_to_f.transpose() *

                franka_delta_rotation *

                v_to_f;


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
	

        sync_rotation_error =

            clamp_vector_norm(

                sync_rotation_error,

                kMaxBackwardRotationErrorRad);

	if(sync_rotation_error.norm() > kMinBackwardRotationErrorRad){
		sync_torque =

		    current_master_rotational_stiffness_applied *
		        sync_rotation_error +

		    current_master_rotational_damping_applied *

		        (
		            franka_angular_velocity_mapped -
		            virtuose_angular_velocity);
        }
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
          torque_mapped;
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
          torque_mapped.norm();


      if (torque_norm >
          kDirectionalTorqueThresholdNm) {

        torque_direction =

            torque_mapped /
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

          sync_force +
          reflection_force +
          directional_force;


      desired_torque =

          sync_torque +
          reflection_torque +
          directional_torque;
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

    // current_stiffness is updated continuously from the analogue
    // finger trigger above and transmitted directly to the slave.

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

        current_stiffness,

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
          "forceValid=%d torqueValid=%d "
          "Franka Kt_cmd=%.1f Kt_appl=%.1f N/m "
          "Kr_cmd=%.2f Kr_appl=%.2f Nm/rad\n"
          "MasterVirt Kt_cmd=%.2f Kt_appl=%.2f Dt=%.3f "
          "Kr_cmd=%.3f Kr_appl=%.3f Dr=%.4f "
          "Araw=%.4f Agrip=%.3f\n"

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

          current_stiffness,

          franka_translational_stiffness_applied,

          current_rotational_stiffness,

          franka_rotational_stiffness_applied,

          current_master_translational_stiffness_command,

          current_master_translational_stiffness_applied,

          current_master_translational_damping_applied,

          current_master_rotational_stiffness_command,

          current_master_rotational_stiffness_applied,

          current_master_rotational_damping_applied,

          current_analog_raw,

          current_analog_normalized,

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


  if (console_thread.joinable()) {

    console_thread.join();
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


  return 0;
}
