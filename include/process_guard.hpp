/**
 * @file process_guard.hpp
 * @brief Process resource monitoring and protection
 * @author Dashboard Team
 */

#pragma once

#include <cstdint>
#include <functional>
#include <chrono>
#include <atomic>
#include <thread>
#include <memory>
#include <string>

namespace ros2_dashboard {

/**
 * @class ProcessGuard
 * @brief Monitor process resources (CPU, memory, handles)
 * 
 * Periodically samples process resource usage and triggers callbacks
 * when thresholds are exceeded.
 */
class ProcessGuard {
public:
    struct ResourceLimits {
        double max_cpu_percent = 95.0;          ///< CPU threshold
        uint64_t max_memory_mb = 2048;           ///< Memory threshold
        uint64_t max_open_files = 4096;          ///< File descriptor threshold
        std::chrono::seconds check_interval{1};  ///< How often to check
    };
    
    using alert_callback = std::function<void(const std::string& alert)>;
    
    /**
     * @brief Constructor with limits
     */
    ProcessGuard() : limits_(), running_(false), callback_(nullptr) {}
    
    explicit ProcessGuard(const ResourceLimits& limits)
        : limits_(limits), running_(false), callback_(nullptr) {
        // Pre-allocate pool
    }
    
    ~ProcessGuard();
    
    /**
     * @brief Start monitoring
     */
    void start(alert_callback callback);
    
    /**
     * @brief Stop monitoring
     */
    void stop();
    
    /**
     * @brief Check if currently monitoring
     */
    bool is_running() const { return running_.load(); }
    
    /**
     * @brief Update resource limits
     */
    void set_limits(const ResourceLimits& limits) {
        limits_ = limits;
    }

private:
    ResourceLimits limits_;
    std::atomic<bool> running_;
    alert_callback callback_;
    std::unique_ptr<std::thread> monitor_thread_;
    
    void monitor_loop_();
};

/**
 * @class ProcessHealthMonitor
 * @brief Detect and report process anomalies (crashes, hangs, memory leaks)
 */
class ProcessHealthMonitor {
public:
    enum class HealthStatus {
        HEALTHY = 0,
        WARNING = 1,
        CRITICAL = 2
    };
    
    struct HealthReport {
        HealthStatus status;
        std::string message;
        uint64_t timestamp_ms;
    };
    
    /**
     * @brief Get current health status
     */
    HealthStatus get_status() const { return status_; }
    
    /**
     * @brief Get last health report
     */
    HealthReport get_report() const { return last_report_; }
    
    /**
     * @brief Mark a successful operation (resets error counter)
     */
    void mark_healthy() { error_count_ = 0; }
    
    /**
     * @brief Record an error/anomaly
     */
    void record_error(const std::string& error);
    
    /**
     * @brief Get consecutive error count
     */
    size_t get_error_count() const { return error_count_; }

private:
    HealthStatus status_ = HealthStatus::HEALTHY;
    HealthReport last_report_;
    size_t error_count_ = 0;
    static constexpr size_t ERROR_THRESHOLD = 10;
};

/**
 * @class CPUPriorityManager
 * @brief Manage process priority and CPU affinity
 */
class CPUPriorityManager {
public:
    enum class Priority {
        LOW = -20,
        NORMAL = 0,
        HIGH = 20
    };
    
    /**
     * @brief Set process priority
     */
    static bool set_priority(Priority priority);
    
    /**
     * @brief Get current priority
     */
    static Priority get_priority();
    
    /**
     * @brief Pin process to specific CPU cores
     */
    static bool set_cpu_affinity(int core_id);
    
    /**
     * @brief Get number of available CPU cores
     */
    static int get_cpu_count();
};

/**
 * @class FailsafeController
 * @brief Auto-recovery and graceful shutdown on failures
 */
class FailsafeController {
public:
    enum class RecoveryStrategy {
        IMMEDIATE_RESTART,  ///< Restart immediately
        EXPONENTIAL_BACKOFF, ///< Restart with exponential backoff
        GRACEFUL_SHUTDOWN,   ///< Shutdown gracefully
        ALERT_ONLY          ///< Just notify (no action)
    };
    
    /**
     * @brief Trigger failsafe action
     */
    void trigger_failsafe(RecoveryStrategy strategy);
    
    /**
     * @brief Get recovery attempts count
     */
    size_t get_recovery_attempts() const { return recovery_attempts_; }
    
    /**
     * @brief Reset recovery counter
     */
    void reset_recovery_attempts() { recovery_attempts_ = 0; }

private:
    size_t recovery_attempts_ = 0;
    static constexpr size_t MAX_RECOVERY_ATTEMPTS = 5;
};

}  // namespace ros2_dashboard
