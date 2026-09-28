// master_bilateral_variable_impedance.cpp

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <thread>
#include "virtuoseAPI.h"
#include "teleop_common.hpp"
#include "teleop_math.hpp"
#include "master_config.hpp"
#include "teleop_runtime.hpp"
#include "master_runtime.hpp"

// MAIN

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


  // ANALOGUE-GRIPPER STIFFNESS COMMAND
  
  double analog_trigger_filtered =
      0.0;
  bool analog_trigger_initialized =
      false;
  double current_stiffness =
      kMinimumTranslationalStiffness;
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
  double current_analog_normalized =
      0.0;

  
  // VIRTUOSE INITIALIZATION

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

 
  // COMMAND SOCKET

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

 
  // FEEDBACK SOCKET
 
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


  // RUNTIME STATE
 
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

  // STARTUP ARM

  bool deadman_release_seen =
      false;
 
  // WRENCH BIAS
 
  Eigen::Matrix<double, 6, 1> bias_sum =
      Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> wrench_bias =
      Eigen::Matrix<double, 6, 1>::Zero();
  int bias_count =
      0;
  bool bias_ready =
      false;

  // COMMAND FILTER / SLEW STATE
 
  Eigen::Vector3d filtered_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d rendered_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d filtered_torque = Eigen::Vector3d::Zero();
  Eigen::Vector3d rendered_torque = Eigen::Vector3d::Zero();
 
  // BACKWARD RELATIVE REFERENCES

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

  // 1 kHz MASTER LOOP
 
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

    // VIRTUOSE STATE

    float pose[7]{};
    float speed[6]{};
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
    const int rc_analogue =
        virtGetAnalogicInputs(
            context,
            analogue);

    // ANALOGUE TRIGGER -> TRANSLATIONAL + ROTATIONAL STIFFNESS

    if (rc_analogue == 0 &&
        kStiffnessAnalogChannel < 7 &&
        std::isfinite(
            static_cast<double>(
                analogue[kStiffnessAnalogChannel]))) {
      const double analog_raw =
          static_cast<double>(analogue[kStiffnessAnalogChannel]);
      const double normalized_raw =
          normalize_analogue_trigger(analog_raw);
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

    // RECEIVE NEWEST SLAVE FEEDBACK

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

    // FRANKA FEEDBACK STATE

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
        finite_vector3(
            franka_position) &&
        finite_quaternion(
            franka_orientation) &&
        finite_vector3(
            franka_linear_velocity) &&
        finite_vector3(
            franka_angular_velocity);

    // FRANKA WRENCH
    
    Eigen::Matrix<double, 6, 1> raw_wrench =
        Eigen::Matrix<double, 6, 1>::Zero();
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
    Eigen::Matrix<double, 6, 1> debiased =
        Eigen::Matrix<double, 6, 1>::Zero();
    if (feedback_live && bias_ready) {
      debiased = raw_wrench - wrench_bias;
    }
    const Eigen::Vector3d force_franka = debiased.head<3>();
    const Eigen::Vector3d torque_franka = debiased.tail<3>();
    const Eigen::Vector3d force_mapped =
        v_to_f.transpose() * force_franka;
    const Eigen::Vector3d torque_mapped =
        v_to_f.transpose() * torque_franka;
    const Eigen::Vector3d franka_linear_velocity_mapped =
        v_to_f.transpose() * franka_linear_velocity;
    const Eigen::Vector3d franka_angular_velocity_mapped =
        v_to_f.transpose() * franka_angular_velocity;
    const double franka_translational_stiffness_applied =
        feedback_received ? feedback[21] : 0.0;
    const double franka_rotational_stiffness_applied =
        feedback_received ? feedback[22] : 0.0;
  
    // API / DEVICE STATUS

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

    // WRENCH VALIDITY
    
    const bool force_safe =
        bias_ready &&
        finite_vector3(
            force_mapped);
    const bool torque_safe =
        bias_ready &&
        finite_vector3(
            torque_mapped);

    // CORE BACKWARD COUPLING GATE

    const bool backward_active =
        api_ok &&
        feedback_live &&
        franka_pose_valid &&
        finite_vector3(
            virtuose_position) &&
        finite_quaternion(
            virtuose_orientation) &&
        finite_vector3(
            virtuose_linear_velocity) &&
        finite_vector3(
            virtuose_angular_velocity) &&
        deadman_release_seen &&
        deadman == 1 &&
        emergency_stop == 1 &&
        power == 1 &&
        indexing == 0;

    // BACKWARD CLUTCH / REFERENCE CAPTURE

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
      filtered_force.setZero();
      rendered_force.setZero();
      filtered_torque.setZero();
      rendered_torque.setZero();
      std::cerr
          << "\n[BACKWARD REFERENCE CAPTURED]\n";
    }
    if (!backward_active) {
      backward_reference_ready =
          false;
    }
    previous_backward_active =
        backward_active;

    // TARGET VIRTUOSE POSE FROM FRANKA RELATIVE MOTION
    
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

      // BACKWARD TRANSLATION

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

      // BACKWARD ROTATION
      
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

    // WRENCH REFLECTION
    
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

    // DIRECTIONAL DAMPING

    Eigen::Vector3d directional_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d directional_torque = Eigen::Vector3d::Zero();
    if (kEnableDirectionalForceDamping && force_safe) {
      const double force_norm = force_mapped.norm();
      if (force_norm > kDirectionalForceThresholdN) {
        const Eigen::Vector3d direction = force_mapped / force_norm;
        const Eigen::Vector3d parallel_velocity =
            direction * virtuose_linear_velocity.dot(direction);
        directional_force = -kDirectionalLinearDamping * parallel_velocity;
      }
    }
    if (kEnableDirectionalTorqueDamping && torque_safe) {
      const double torque_norm = torque_mapped.norm();
      if (torque_norm > kDirectionalTorqueThresholdNm) {
        const Eigen::Vector3d direction = torque_mapped / torque_norm;
        const Eigen::Vector3d parallel_velocity =
            direction * virtuose_angular_velocity.dot(direction);
        directional_torque = -kDirectionalAngularDamping * parallel_velocity;
      }
    }
    
    // TOTAL BACKWARD COMMAND
    
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

    // LOW-PASS + SLEW LIMIT
    
    filtered_force =
        kForceFilterAlpha * desired_force +
        (1.0 - kForceFilterAlpha) * filtered_force;
    filtered_torque =
        kTorqueFilterAlpha * desired_torque +
        (1.0 - kTorqueFilterAlpha) * filtered_torque;
    for (int i = 0; i < 3; ++i) {
      rendered_force[i] =
          rate_limit(filtered_force[i], rendered_force[i], kMaxForceStepN);
      rendered_torque[i] =
          rate_limit(filtered_torque[i], rendered_torque[i], kMaxTorqueStepNm);
    }
    rendered_force = clamp_vector_norm(rendered_force, kDeviceForceCeilingN);
    rendered_torque = clamp_vector_norm(rendered_torque, kDeviceTorqueCeilingNm);

    // APPLY WRENCH TO VIRTUOSE
    
    if (backward_active && backward_reference_ready) {
      float wrench[6] = {
          static_cast<float>(rendered_force[0]),
          static_cast<float>(rendered_force[1]),
          static_cast<float>(rendered_force[2]),
          static_cast<float>(rendered_torque[0]),
          static_cast<float>(rendered_torque[1]),
          static_cast<float>(rendered_torque[2])};
      if (virtSetForce(context, wrench) != 0) {
        zero_wrench(context);
      }
    } else {
      zero_wrench(context);
      filtered_force.setZero();
      rendered_force.setZero();
      filtered_torque.setZero();
      rendered_torque.setZero();
    }
    
    // MASTER -> SLAVE

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

    // TERMINAL OUTPUT
    
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
    
    // 1 kHz TIMING

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

  // SHUTDOWN
  
  g_stop_requested =
      1;
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
