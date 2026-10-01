// master_bilateral_variable_impedance

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "virtuoseAPI.h"
#include "teleop_common.hpp"
#include "teleop_math.hpp"
#include "master_config.hpp"
#include "teleop_runtime.hpp"
#include "master_runtime.hpp"

namespace {
using Vector6d = Eigen::Matrix<double, 6, 1>;
}

int main() {
  std::signal(SIGINT, sigint_handler);

  const Eigen::Matrix3d v_to_f = create_frame_map();
  const auto loop_period =
      std::chrono::microseconds(static_cast<int>(1000000.0 / kLoopHz));

  const auto interpolate = [](double minimum, double maximum, double alpha) {
    return minimum + alpha * (maximum - minimum);
  };

  double analog_trigger_filtered = 0.0;
  bool analog_trigger_initialized = false;
  double current_analog_normalized = 0.0;
  double current_stiffness = kMinimumTranslationalStiffness;

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
      std::sqrt(kMinimumBackwardPositionStiffness /
                kBaselineBackwardPositionStiffness);
  double current_master_rotational_damping_applied =
      kBaselineBackwardRotationDamping *
      std::sqrt(kMinimumBackwardRotationStiffness /
                kBaselineBackwardRotationStiffness);

  VirtContext context = virtOpen("127.0.0.1#53210");
  if (!context) {
    std::cerr << "ERROR: virtOpen failed.\n";
    return 1;
  }

  float identity_frame[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const auto setup_ok = [&](int result, const char* operation) {
    if (result == 0) {
      return true;
    }
    std::cerr << "ERROR: Virtuose setup failed at " << operation << '\n';
    shutdown_virtuose(context);
    return false;
  };

  if (!setup_ok(virtSetTimeoutValue(context, kVirtuoseApiTimeoutS),
                "virtSetTimeoutValue") ||
      !setup_ok(virtSetIndexingMode(context, INDEXING_ALL),
                "virtSetIndexingMode") ||
      !setup_ok(virtSetForceFactor(context, kVirtuoseForceFactor),
                "virtSetForceFactor") ||
      !setup_ok(virtSetSpeedFactor(context, kVirtuoseSpeedFactor),
                "virtSetSpeedFactor") ||
      !setup_ok(virtSetTimeStep(context, static_cast<float>(1.0 / kLoopHz)),
                "virtSetTimeStep") ||
      !setup_ok(virtSetBaseFrame(context, identity_frame), "virtSetBaseFrame") ||
      !setup_ok(virtSetObservationFrame(context, identity_frame),
                "virtSetObservationFrame") ||
      !setup_ok(virtSetCommandType(context, COMMAND_TYPE_IMPEDANCE),
                "virtSetCommandType") ||
      !setup_ok(virtSaturateTorque(
                    context,
                    static_cast<float>(kDeviceForceCeilingN),
                    static_cast<float>(kDeviceTorqueCeilingNm)),
                "virtSaturateTorque") ||
      !setup_ok(virtEnableForceFeedback(context, 1),
                "virtEnableForceFeedback") ||
      !setup_ok(virtSetPowerOn(context, 1), "virtSetPowerOn")) {
    return 1;
  }

  const int command_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (command_socket < 0) {
    std::perror("command socket");
    shutdown_virtuose(context);
    return 1;
  }

  sockaddr_in slave_address{};
  slave_address.sin_family = AF_INET;
  slave_address.sin_port = htons(kCommandSendPort);
  if (inet_pton(AF_INET, kSlaveIp, &slave_address.sin_addr) != 1) {
    std::cerr << "ERROR: invalid slave IP.\n";
    close(command_socket);
    shutdown_virtuose(context);
    return 1;
  }

  const int feedback_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (feedback_socket < 0) {
    std::perror("feedback socket");
    close(command_socket);
    shutdown_virtuose(context);
    return 1;
  }

  int reuse = 1;
  setsockopt(feedback_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in feedback_address{};
  feedback_address.sin_family = AF_INET;
  feedback_address.sin_addr.s_addr = htonl(INADDR_ANY);
  feedback_address.sin_port = htons(kFeedbackReceivePort);
  if (bind(feedback_socket,
           reinterpret_cast<sockaddr*>(&feedback_address),
           sizeof(feedback_address)) < 0) {
    std::perror("feedback bind");
    close(feedback_socket);
    close(command_socket);
    shutdown_virtuose(context);
    return 1;
  }

  const int feedback_flags = fcntl(feedback_socket, F_GETFL, 0);
  if (feedback_flags < 0 ||
      fcntl(feedback_socket, F_SETFL, feedback_flags | O_NONBLOCK) < 0) {
    std::perror("feedback fcntl");
    close(feedback_socket);
    close(command_socket);
    shutdown_virtuose(context);
    return 1;
  }

  const auto start_time = std::chrono::steady_clock::now();
  auto next_tick = std::chrono::steady_clock::now();
  auto previous_loop_time = next_tick;
  auto last_feedback_time = next_tick;

  uint64_t packet_id = 0;
  double feedback[kFeedbackPacketSize]{};
  bool feedback_received = false;
  bool deadman_release_seen = false;

  Vector6d bias_sum = Vector6d::Zero();
  Vector6d wrench_bias = Vector6d::Zero();
  int bias_count = 0;
  bool bias_ready = false;

  Vector6d filtered_wrench = Vector6d::Zero();
  Vector6d rendered_wrench = Vector6d::Zero();

  bool backward_reference_ready = false;
  bool previous_backward_active = false;
  Eigen::Vector3d virtuose_position_reference = Eigen::Vector3d::Zero();
  Eigen::Vector3d franka_position_reference = Eigen::Vector3d::Zero();
  Eigen::Quaterniond virtuose_orientation_reference =
      Eigen::Quaterniond::Identity();
  Eigen::Quaterniond franka_orientation_reference =
      Eigen::Quaterniond::Identity();

  int terminal_counter = 0;

  while (!g_stop_requested) {
    next_tick += loop_period;

    const auto now = std::chrono::steady_clock::now();
    const double loop_dt_ms =
        std::chrono::duration<double, std::milli>(now - previous_loop_time)
            .count();
    previous_loop_time = now;

    float pose[7]{};
    float speed[6]{};
    float analogue[7]{};
    int deadman = 0;
    int emergency_stop = 0;
    int power = 0;
    int indexing = 0;
    unsigned int alarm = 0;

    const int rc_pose = virtGetPosition(context, pose);
    const int rc_speed = virtGetSpeed(context, speed);
    const int rc_analogue = virtGetAnalogicInputs(context, analogue);

    if (rc_analogue == 0 &&
        kStiffnessAnalogChannel < 7 &&
        std::isfinite(static_cast<double>(analogue[kStiffnessAnalogChannel]))) {
      const double normalized_raw = normalize_analogue_trigger(
          static_cast<double>(analogue[kStiffnessAnalogChannel]));

      if (!analog_trigger_initialized) {
        analog_trigger_filtered = normalized_raw;
        analog_trigger_initialized = true;
      } else {
        const double loop_dt_s = std::max(loop_dt_ms * 1e-3, 1.0 / kLoopHz);
        const double analog_alpha =
            loop_dt_s / (kAnalogFilterTimeConstantS + loop_dt_s);
        analog_trigger_filtered +=
            analog_alpha * (normalized_raw - analog_trigger_filtered);
      }

      current_analog_normalized =
          clamp_scalar(analog_trigger_filtered, 0.0, 1.0);
      current_stiffness = interpolate(
          kMinimumTranslationalStiffness,
          kMaximumTranslationalStiffness,
          current_analog_normalized);
      current_master_translational_stiffness_command = interpolate(
          kMinimumBackwardPositionStiffness,
          kMaximumBackwardPositionStiffness,
          current_analog_normalized);
      current_master_rotational_stiffness_command = interpolate(
          kMinimumBackwardRotationStiffness,
          kMaximumBackwardRotationStiffness,
          current_analog_normalized);
    }

    const double master_impedance_dt_s =
        std::max(loop_dt_ms * 1e-3, 1.0 / kLoopHz);
    current_master_translational_stiffness_applied = rate_limit(
        current_master_translational_stiffness_command,
        current_master_translational_stiffness_applied,
        kMaximumBackwardPositionStiffnessRate * master_impedance_dt_s);
    current_master_rotational_stiffness_applied = rate_limit(
        current_master_rotational_stiffness_command,
        current_master_rotational_stiffness_applied,
        kMaximumBackwardRotationStiffnessRate * master_impedance_dt_s);
    current_master_translational_damping_applied =
        kBaselineBackwardPositionDamping *
        std::sqrt(current_master_translational_stiffness_applied /
                  kBaselineBackwardPositionStiffness);
    current_master_rotational_damping_applied =
        kBaselineBackwardRotationDamping *
        std::sqrt(current_master_rotational_stiffness_applied /
                  kBaselineBackwardRotationStiffness);

    const int rc_deadman = virtGetDeadMan(context, &deadman);
    const int rc_estop = virtGetEmergencyStop(context, &emergency_stop);
    const int rc_power = virtGetPowerOn(context, &power);
    const int rc_index = virtIsInShiftPosition(context, &indexing);
    const int rc_alarm = virtGetAlarm(context, &alarm);

    Eigen::Vector3d virtuose_position(
        static_cast<double>(pose[0]),
        static_cast<double>(pose[1]),
        static_cast<double>(pose[2]));
    Eigen::Quaterniond virtuose_orientation(
        static_cast<double>(pose[6]),
        static_cast<double>(pose[3]),
        static_cast<double>(pose[4]),
        static_cast<double>(pose[5]));
    if (finite_quaternion(virtuose_orientation)) {
      virtuose_orientation.normalize();
    }

    const Eigen::Vector3d virtuose_linear_velocity(
        static_cast<double>(speed[0]),
        static_cast<double>(speed[1]),
        static_cast<double>(speed[2]));
    const Eigen::Vector3d virtuose_angular_velocity(
        static_cast<double>(speed[3]),
        static_cast<double>(speed[4]),
        static_cast<double>(speed[5]));

    while (true) {
      double candidate[kFeedbackPacketSize]{};
      const ssize_t received = recvfrom(
          feedback_socket,
          reinterpret_cast<char*>(candidate),
          sizeof(candidate),
          0,
          nullptr,
          nullptr);

      if (received != static_cast<ssize_t>(sizeof(candidate))) {
        break;
      }

      std::memcpy(feedback, candidate, sizeof(feedback));
      feedback_received = true;
      last_feedback_time = now;
    }

    const double feedback_age_ms =
        std::chrono::duration<double, std::milli>(now - last_feedback_time)
            .count();
    const bool feedback_live =
        feedback_received && feedback_age_ms < kFeedbackStaleMs;

    Eigen::Vector3d franka_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond franka_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d franka_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d franka_angular_velocity = Eigen::Vector3d::Zero();

    if (feedback_received) {
      franka_position =
          Eigen::Vector3d(feedback[8], feedback[9], feedback[10]);
      franka_orientation = Eigen::Quaterniond(
          feedback[11], feedback[12], feedback[13], feedback[14]);

      if (finite_quaternion(franka_orientation)) {
        franka_orientation.normalize();
      }

      franka_linear_velocity =
          Eigen::Vector3d(feedback[15], feedback[16], feedback[17]);
      franka_angular_velocity =
          Eigen::Vector3d(feedback[18], feedback[19], feedback[20]);
    }

    const bool franka_pose_valid =
        feedback_live &&
        finite_vector3(franka_position) &&
        finite_quaternion(franka_orientation) &&
        finite_vector3(franka_linear_velocity) &&
        finite_vector3(franka_angular_velocity);

    Vector6d raw_wrench = Vector6d::Zero();
    if (feedback_live) {
      for (int i = 0; i < 6; ++i) {
        raw_wrench[i] = feedback[2 + i];
      }
    }

    if (feedback_live && !bias_ready) {
      bias_sum += raw_wrench;
      ++bias_count;

      if (bias_count >= kBiasSamplesRequired) {
        wrench_bias = bias_sum / static_cast<double>(bias_count);
        bias_ready = true;
        std::cerr << "\n[WRENCH BIAS READY]\n";
      }
    }

    Vector6d debiased_wrench = Vector6d::Zero();
    if (feedback_live && bias_ready) {
      debiased_wrench = raw_wrench - wrench_bias;
    }

    const Eigen::Vector3d force_mapped =
        v_to_f.transpose() * debiased_wrench.head<3>();
    const Eigen::Vector3d torque_mapped =
        v_to_f.transpose() * debiased_wrench.tail<3>();
    const Eigen::Vector3d franka_linear_velocity_mapped =
        v_to_f.transpose() * franka_linear_velocity;
    const Eigen::Vector3d franka_angular_velocity_mapped =
        v_to_f.transpose() * franka_angular_velocity;

    const double franka_translational_stiffness_applied =
        feedback_received ? feedback[21] : 0.0;
    const double franka_rotational_stiffness_applied =
        feedback_received ? feedback[22] : 0.0;

    const bool api_ok =
        rc_pose == 0 &&
        rc_speed == 0 &&
        rc_analogue == 0 &&
        rc_deadman == 0 &&
        rc_estop == 0 &&
        rc_power == 0 &&
        rc_index == 0 &&
        rc_alarm == 0;

    if (api_ok && deadman == 0) {
      deadman_release_seen = true;
    }

    const bool force_safe = bias_ready && finite_vector3(force_mapped);
    const bool torque_safe = bias_ready && finite_vector3(torque_mapped);
    const bool backward_active =
        api_ok &&
        feedback_live &&
        franka_pose_valid &&
        finite_vector3(virtuose_position) &&
        finite_quaternion(virtuose_orientation) &&
        finite_vector3(virtuose_linear_velocity) &&
        finite_vector3(virtuose_angular_velocity) &&
        deadman_release_seen &&
        deadman == 1 &&
        emergency_stop == 1 &&
        power == 1 &&
        indexing == 0;

    if (backward_active && !previous_backward_active) {
      virtuose_position_reference = virtuose_position;
      franka_position_reference = franka_position;
      virtuose_orientation_reference = virtuose_orientation;
      franka_orientation_reference = franka_orientation;
      backward_reference_ready = true;
      filtered_wrench.setZero();
      rendered_wrench.setZero();
      std::cerr << "\n[BACKWARD REFERENCE CAPTURED]\n";
    }

    if (!backward_active) {
      backward_reference_ready = false;
    }
    previous_backward_active = backward_active;

    Vector6d sync_wrench = Vector6d::Zero();
    if (backward_active && backward_reference_ready) {
      if (kEnableBackwardPositionCoupling) {
        const Eigen::Vector3d franka_delta =
            franka_position - franka_position_reference;
        const Eigen::Vector3d target_position =
            virtuose_position_reference + v_to_f.transpose() * franka_delta;
        const Eigen::Vector3d position_error = clamp_vector_norm(
            target_position - virtuose_position,
            kMaxBackwardPositionErrorM);

        sync_wrench.head<3>() =
            current_master_translational_stiffness_applied * position_error +
            current_master_translational_damping_applied *
                (franka_linear_velocity_mapped - virtuose_linear_velocity);
      }

      if (kEnableBackwardRotationCoupling) {
        const Eigen::Matrix3d franka_delta_rotation =
            franka_orientation.toRotationMatrix() *
            franka_orientation_reference.toRotationMatrix().transpose();
        const Eigen::Matrix3d target_rotation =
            (v_to_f.transpose() * franka_delta_rotation * v_to_f) *
            virtuose_orientation_reference.toRotationMatrix();

        Eigen::Quaterniond target_orientation(target_rotation);
        target_orientation.normalize();

        const Eigen::Vector3d rotation_error = clamp_vector_norm(
            orientation_error_world(virtuose_orientation, target_orientation),
            kMaxBackwardRotationErrorRad);

        if (rotation_error.norm() > kMinBackwardRotationErrorRad) {
          sync_wrench.tail<3>() =
              current_master_rotational_stiffness_applied * rotation_error +
              current_master_rotational_damping_applied *
                  (franka_angular_velocity_mapped -
                   virtuose_angular_velocity);
        }
      }
    }

    Vector6d reflection_wrench = Vector6d::Zero();
    if (kEnableForceReflection && force_safe) {
      reflection_wrench.head<3>() =
          -kForceReflectionGain * force_mapped;
    }
    if (kEnableTorqueReflection && torque_safe) {
      reflection_wrench.tail<3>() =
          -kTorqueReflectionGain * torque_mapped;
    }

    Vector6d directional_wrench = Vector6d::Zero();
    if (kEnableDirectionalForceDamping && force_safe) {
      const double force_norm = force_mapped.norm();
      if (force_norm > kDirectionalForceThresholdN) {
        const Eigen::Vector3d direction = force_mapped / force_norm;
        directional_wrench.head<3>() =
            -kDirectionalLinearDamping *
            direction * virtuose_linear_velocity.dot(direction);
      }
    }

    if (kEnableDirectionalTorqueDamping && torque_safe) {
      const double torque_norm = torque_mapped.norm();
      if (torque_norm > kDirectionalTorqueThresholdNm) {
        const Eigen::Vector3d direction = torque_mapped / torque_norm;
        directional_wrench.tail<3>() =
            -kDirectionalAngularDamping *
            direction * virtuose_angular_velocity.dot(direction);
      }
    }

    Vector6d desired_wrench = Vector6d::Zero();
    if (backward_active && backward_reference_ready) {
      desired_wrench = sync_wrench + reflection_wrench + directional_wrench;
    }

    filtered_wrench.head<3>() =
        kForceFilterAlpha * desired_wrench.head<3>() +
        (1.0 - kForceFilterAlpha) * filtered_wrench.head<3>();
    filtered_wrench.tail<3>() =
        kTorqueFilterAlpha * desired_wrench.tail<3>() +
        (1.0 - kTorqueFilterAlpha) * filtered_wrench.tail<3>();

    for (int i = 0; i < 3; ++i) {
      rendered_wrench[i] =
          rate_limit(filtered_wrench[i], rendered_wrench[i], kMaxForceStepN);
      rendered_wrench[3 + i] = rate_limit(
          filtered_wrench[3 + i],
          rendered_wrench[3 + i],
          kMaxTorqueStepNm);
    }

    rendered_wrench.head<3>() = clamp_vector_norm(
        Eigen::Vector3d(rendered_wrench.head<3>()),
        kDeviceForceCeilingN);
    rendered_wrench.tail<3>() = clamp_vector_norm(
        Eigen::Vector3d(rendered_wrench.tail<3>()),
        kDeviceTorqueCeilingNm);

    if (backward_active && backward_reference_ready) {
      float wrench[6];
      for (int i = 0; i < 6; ++i) {
        wrench[i] = static_cast<float>(rendered_wrench[i]);
      }

      if (virtSetForce(context, wrench) != 0) {
        zero_wrench(context);
      }
    } else {
      zero_wrench(context);
      filtered_wrench.setZero();
      rendered_wrench.setZero();
    }

    const double command_packet[kCommandPacketSize] = {
        static_cast<double>(packet_id),
        elapsed_us(start_time),
        static_cast<double>(pose[6]),
        static_cast<double>(pose[3]),
        static_cast<double>(pose[4]),
        static_cast<double>(pose[5]),
        static_cast<double>(pose[0]),
        static_cast<double>(pose[1]),
        static_cast<double>(pose[2]),
        kEnableForceReflection ? kForceReflectionGain : 0.0,
        kEnableTorqueReflection ? kTorqueReflectionGain : 0.0,
        current_stiffness,
        static_cast<double>(deadman),
        static_cast<double>(emergency_stop)};

    sendto(
        command_socket,
        command_packet,
        sizeof(command_packet),
        0,
        reinterpret_cast<sockaddr*>(&slave_address),
        sizeof(slave_address));
    ++packet_id;

    if (++terminal_counter >= kConsolePrintEveryCycles) {
      terminal_counter = 0;
      std::fprintf(
          stderr,
          "[MASTER] active=%d trigger=%.2f "
          "Slave Kt=%.1f Kr=%.1f | Master Kt=%.2f Kr=%.2f | age=%.2f ms\n",
          backward_active ? 1 : 0,
          current_analog_normalized,
          franka_translational_stiffness_applied,
          franka_rotational_stiffness_applied,
          current_master_translational_stiffness_applied,
          current_master_rotational_stiffness_applied,
          feedback_age_ms);
    }

    const auto after_work = std::chrono::steady_clock::now();
    if (after_work > next_tick + std::chrono::milliseconds(5)) {
      next_tick = after_work;
    }
    std::this_thread::sleep_until(next_tick);
  }

  g_stop_requested = 1;
  zero_wrench(context);
  virtEnableForceFeedback(context, 0);
  virtSetPowerOn(context, 0);
  close(feedback_socket);
  close(command_socket);
  virtClose(context);
  return 0;
}