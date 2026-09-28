// slave_bilateral_variable_impedance.cpp
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
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
#include "teleop_common.hpp"
#include "teleop_math.hpp"
#include "slave_config.hpp"
#include "teleop_runtime.hpp"
#include "slave_control_utils.hpp"

// MAIN
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

  // SHARED UDP STATE
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

  // MASTER COMMAND SOCKET
  int receive_socket =
      socket(
          AF_INET,
          SOCK_DGRAM,
          0);
  if (receive_socket < 0) {
    std::perror(
        "command socket");
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
    return 1;
  }

  // RECEIVE THREAD
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

  // FEEDBACK THREAD
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

  // FRANKA
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
    std::cout
        << "[SLAVE] " << kControlRateHz << " Hz | "
        << "Kt=" << kMinimumTranslationalStiffness << ".."
        << kMaximumTranslationalStiffness << " N/m | "
        << "Kr=" << kMinimumRotationalStiffness << ".."
        << kMaximumRotationalStiffness << " Nm/rad\n"
        << "WARNING: Franka will move to the initial pose. "
        << "Keep emergency stop accessible. Press Enter to continue..."
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

    // COLLISION BEHAVIOR
    std::array<double, 7> joint_thresholds{};
    joint_thresholds.fill(kCollisionThreshold);
    std::array<double, 6> cartesian_thresholds{};
    cartesian_thresholds.fill(kCollisionThreshold);
    robot.setCollisionBehavior(
        joint_thresholds,
        joint_thresholds,
        cartesian_thresholds,
        cartesian_thresholds);
        
    // IMPEDANCE MATRICES
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
        kBaselineRotationalStiffness *
        Eigen::Matrix3d::
            Identity();
    damping.bottomRightCorner<
        3,
        3>() =
        kBaselineRotationalDamping *
        Eigen::Matrix3d::
            Identity();
    const auto control_start =
        std::chrono::steady_clock::now();
    uint64_t feedback_id =
        0;
    Eigen::Vector3d prev_master_position(0, 0, 0);
    double prev_command[7]{};

    // FRANKA 1 kHz CALLBACK
    robot.control(
        [&](const franka::RobotState& state,
            franka::Duration period)
            -> franka::Torques {
          const double dt =
              std::max(
                  period.toSec(),
                  kMinimumCallbackDtS);

          // MODEL
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

          // CURRENT FRANKA POSE
          Eigen::Affine3d transform(
              Eigen::Matrix4d::Map(
                  state.O_T_EE.data()));
          const Eigen::Vector3d position =
              transform.translation();
          Eigen::Quaterniond orientation(
              transform.rotation());
          orientation.normalize();

          // CARTESIAN VELOCITY
          const Eigen::Matrix<
              double,
              6,
              1>
              cartesian_velocity =
                  jacobian *
                  dq;

          // SLAVE -> MASTER FEEDBACK
          feedback_packet[0].store(
              static_cast<double>(feedback_id++),
              std::memory_order_relaxed);
          const double local_time_s =
              std::chrono::duration<double>(
                  std::chrono::steady_clock::now() - control_start)
                  .count();
          feedback_packet[1].store(
              local_time_s * 1e6,
              std::memory_order_relaxed);
          for (std::size_t i = 0; i < 3; ++i) {
            feedback_packet[2 + i].store(
                state.O_F_ext_hat_K[i], std::memory_order_relaxed);
            feedback_packet[5 + i].store(
                state.O_F_ext_hat_K[3 + i], std::memory_order_relaxed);
            feedback_packet[8 + i].store(
                position[static_cast<int>(i)], std::memory_order_relaxed);
            feedback_packet[15 + i].store(
                cartesian_velocity[static_cast<int>(i)],
                std::memory_order_relaxed);
            feedback_packet[18 + i].store(
                cartesian_velocity[3 + static_cast<int>(i)],
                std::memory_order_relaxed);
          }
          feedback_packet[11].store(
              orientation.w(), std::memory_order_relaxed);
          feedback_packet[12].store(
              orientation.x(), std::memory_order_relaxed);
          feedback_packet[13].store(
              orientation.y(), std::memory_order_relaxed);
          feedback_packet[14].store(
              orientation.z(), std::memory_order_relaxed);
              
          // READ MASTER COMMAND
          double command[
              kCommandPacketSize]{};
          for (std::size_t i = 0;
               i < kCommandPacketSize;
               ++i) {
            command[i] =
                master_packet[i].load(
                    std::memory_order_relaxed);
          }

          // MASTER PACKET AGE
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
          const int deadman =
              static_cast<int>(
                  std::round(
                      command[12]));
          const int emergency_stop =
              static_cast<int>(
                  std::round(
                      command[13]));

          // DIRECT REAL-TIME TRANSLATIONAL STIFFNESS COMMAND
          static double stiffness_received =
              kBaselineTranslationalStiffness;
          if (packet_live &&
              std::isfinite(
                  command[11]) &&
              command[11] >= 0.0) {
            stiffness_received =
                command[11];
          }

          // VARIABLE CARTESIAN IMPEDANCE
          const double k_requested =
              stiffness_received;
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

          // VARIABLE ROTATIONAL IMPEDANCE
          const double stiffness_span =
              kMaximumTranslationalStiffness -
              kMinimumTranslationalStiffness;
          const double analogue_command =
              stiffness_span > 1e-12
                  ? clamp_scalar(
                        (k_requested -
                         kMinimumTranslationalStiffness) /
                            stiffness_span,
                        0.0,
                        1.0)
                  : 0.0;
          const double k_rot_requested =
              kMinimumRotationalStiffness +
              analogue_command *
                  (kMaximumRotationalStiffness -
                   kMinimumRotationalStiffness);
          static double k_rot_applied =
              kBaselineRotationalStiffness;
          const double max_k_rot_step =
              kMaximumRotationalStiffnessRate *
              dt;
          k_rot_applied +=
              clamp_scalar(
                  k_rot_requested -
                      k_rot_applied,
                  -max_k_rot_step,
                  max_k_rot_step);
          const double d_rot_applied =
              kBaselineRotationalDamping *
              std::sqrt(
                  k_rot_applied /
                  kBaselineRotationalStiffness);
          stiffness.bottomRightCorner<
              3,
              3>() =
              k_rot_applied *
              Eigen::Matrix3d::
                  Identity();
          damping.bottomRightCorner<
              3,
              3>() =
              d_rot_applied *
              Eigen::Matrix3d::
                  Identity();
          feedback_packet[21].store(
              k_applied,
              std::memory_order_relaxed);
          feedback_packet[22].store(
              k_rot_applied,
              std::memory_order_relaxed);

          // MASTER POSE
          Eigen::Vector3d
              master_position(
                  command[6],
                  command[7],
                  command[8]);
          Eigen::Quaterniond
              master_orientation(
                  0.95 * command[2] + 0.05 * prev_command[3],
                  0.95 * command[3] + 0.05 * prev_command[4],
                  0.95 * command[4] + 0.05 * prev_command[5],
                  0.95 * command[5] + 0.05 * prev_command[6]);
          prev_command[3] = master_orientation.w();
          prev_command[4] = master_orientation.x();
          prev_command[5] = master_orientation.y();
          prev_command[6] = master_orientation.z();
          const bool valid_master_pose =
              finite_vector3(
                  master_position) &&
              finite_quaternion(
                  master_orientation);
          if (valid_master_pose) {
      master_position = 0.99 * master_position + 0.01 * prev_master_position;
      prev_master_position = master_position;
            master_orientation.normalize();
          }

          // RELATIVE REFERENCES
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

          // FORWARD CLUTCH / REFERENCE CAPTURE
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

          // FORWARD TELEOPERATION
          desired_angular_velocity.setZero();
          if (active &&
              reference_ready) {

            // TRANSLATION
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

            // ROTATION
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
                      nullptr);
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
                      nullptr);
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

          // CARTESIAN ERROR
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

          // CARTESIAN IMPEDANCE
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

          // NULLSPACE CONTROL
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

          // TOTAL JOINT TORQUE
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

          // STOP
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

  // SHUTDOWN
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
  return 0;
}
