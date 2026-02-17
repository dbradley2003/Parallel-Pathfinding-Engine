#pragma once

#include <atomic>
#include <chrono>
#include <iostream>
#include <string>

#if defined(__SANITIZE_THREAD__)
#define NO_TSAN_FENCE 1
#endif

class Timer {
private:
  // Use high_resolution_clock for maximum precision
  std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_endTime;
  bool m_isRunning = false;

public:
  void start() {
#ifndef NO_TSAN_FENCE
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
    m_startTime = std::chrono::high_resolution_clock::now();
#ifndef NO_TSAN_FENCE
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  void stop() {
#ifndef NO_TSAN_FENCE
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
    m_endTime = std::chrono::high_resolution_clock::now();
#ifndef NO_TSAN_FENCE
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  double elapsedMilliseconds() const {
    auto endTime =
        m_isRunning ? std::chrono::high_resolution_clock::now() : m_endTime;
    return std::chrono::duration<double, std::milli>(endTime - m_startTime)
        .count();
  }

  double elapsedSeconds() const { return elapsedMilliseconds() / 1000.0; }

  // Helper to print standard benchmark format
  void printResult(const std::string &label) {
    std::cout << "[" << label << "] Time: " << elapsedMilliseconds() << " ms"
              << std::endl;

    std::cout << "[" << label << "] Time: " << elapsedSeconds() << " s"
              << std::endl;
  }
};
