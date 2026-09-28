#pragma once

#include <array>
#include <cstddef>

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
