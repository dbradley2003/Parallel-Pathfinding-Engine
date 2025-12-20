#pragma once

#include <chrono>
#include <atomic>
#include <iostream>
#include <string>

class Timer {
private:
    // Use high_resolution_clock for maximum precision
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_endTime;
    bool m_isRunning = false;

public:
    void start() {
        std::atomic_thread_fence(std::memory_order_seq_cst);
        m_startTime = std::chrono::high_resolution_clock::now();
        std::atomic_thread_fence(std::memory_order_seq_cst);
    }

    void stop() {
        std::atomic_thread_fence(std::memory_order_seq_cst);
        m_endTime = std::chrono::high_resolution_clock::now();
        std::atomic_thread_fence(std::memory_order_seq_cst);
    }

    double elapsedMilliseconds() const {
        auto endTime = m_isRunning ? std::chrono::high_resolution_clock::now() : m_endTime;
        return std::chrono::duration<double, std::milli>(endTime - m_startTime).count();
    }

    double elapsedSeconds() const {
        return elapsedMilliseconds() / 1000.0;
    }

    // Helper to print standard benchmark format
    void printResult(const std::string& label) {
        std::cout << "[" << label << "] Time: "
            << elapsedMilliseconds() << " ms" << std::endl;

        std::cout << "[" << label << "] Time: "
            << elapsedSeconds() << " s" << std::endl;
    }
};
