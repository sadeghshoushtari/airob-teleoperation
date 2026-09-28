#pragma once

#include <chrono>
#include <csignal>
#include <cstdint>

inline volatile std::sig_atomic_t g_stop_requested = 0;

inline void sigint_handler(int) {
  g_stop_requested = 1;
}

inline uint64_t now_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

inline double elapsed_seconds(
    const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now() - start)
      .count();
}

inline double elapsed_us(
    const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double, std::micro>(
             std::chrono::steady_clock::now() - start)
      .count();
}
