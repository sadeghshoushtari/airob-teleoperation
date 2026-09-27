#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Dense>
#include <Eigen/Geometry>

inline constexpr double kPi =
    3.14159265358979323846;

inline constexpr const char* kLoopbackIp =
    "127.0.0.1";

inline constexpr const char* kSlaveIp =
    kLoopbackIp;

inline constexpr const char* kFeedbackDestinationIp =
    kLoopbackIp;

inline constexpr int kCommandPort =
    11056;

inline constexpr int kFeedbackPort =
    11055;

inline constexpr int kCommandSendPort =
    kCommandPort;

inline constexpr int kCommandReceivePort =
    kCommandPort;

inline constexpr int kFeedbackSendPort =
    kFeedbackPort;

inline constexpr int kFeedbackReceivePort =
    kFeedbackPort;

inline constexpr std::size_t kCommandPacketSize =
    14;

inline constexpr std::size_t kFeedbackPacketSize =
    23;

inline constexpr std::array<double, 9> kVirtuoseToFranka = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};

inline double clamp_scalar(
    double value,
    double lower,
    double upper) {

  return std::max(
      lower,
      std::min(
          value,
          upper));
}

inline bool finite_quaternion(
    const Eigen::Quaterniond& q) {

  return
      std::isfinite(q.w()) &&
      std::isfinite(q.x()) &&
      std::isfinite(q.y()) &&
      std::isfinite(q.z()) &&
      q.squaredNorm() >
          1e-10;
}

inline Eigen::Matrix3d create_frame_map() {

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
