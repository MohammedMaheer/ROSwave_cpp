/**
 * @file process_guard.cpp
 * @brief Implementation of process resource monitoring
 */

#include "process_guard.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/sysinfo.h>

namespace ros2_dashboard {

// ===== ProcessGuard Implementation =====

ProcessGuard::~ProcessGuard() {
    stop();
}

void ProcessGuard::start(alert_callback callback) {
    if (running_.load()) {
        return;  // Already running
    }
    
    running_ = true;
    callback_ = callback;
    
    monitor_thread_ = std::make_unique<std::thread>([this]() {
        monitor_loop_();
    });
}

void ProcessGuard::stop() {
    running_ = false;
    if (monitor_thread_ && monitor_thread_->joinable()) {
        monitor_thread_->join();
    }
}

void ProcessGuard::monitor_loop_() {
    while (running_.load()) {
        // Read /proc/self/stat for CPU usage
        std::ifstream stat_file("/proc/self/stat");
        if (stat_file) {
            std::string line;
            std::getline(stat_file, line);
            
            // Parse CPU ticks (fields 13-14 = utime + stime)
            // Note: This is simplified; real implementation would calculate delta
            std::istringstream iss(line);
            for (int i = 0; i < 13; ++i) {
                std::string dummy;
                iss >> dummy;
            }
            
            uint64_t utime, stime;
            iss >> utime >> stime;
            
            double uptime = sysconf(_SC_CLK_TCK);
            double cpu_time = (utime + stime) / uptime;
            double cpu_percent = (cpu_time / limits_.check_interval.count()) * 100.0;
            
            if (callback_ && cpu_percent > limits_.max_cpu_percent) {
                callback_("⚠️ HIGH CPU: " + std::to_string(cpu_percent) + "%");
            }
        }
        
        // Read /proc/self/status for memory
        std::ifstream status_file("/proc/self/status");
        if (status_file) {
            std::string line;
            while (std::getline(status_file, line)) {
                if (line.find("VmRSS:") == 0) {
                    std::istringstream iss(line);
                    std::string label;
                    uint64_t rss_kb;
                    iss >> label >> rss_kb;
                    
                    uint64_t rss_mb = rss_kb / 1024;
                    if (callback_ && rss_mb > limits_.max_memory_mb) {
                        callback_("⚠️ HIGH MEMORY: " + std::to_string(rss_mb) + "MB");
                    }
                    break;
                }
            }
        }
        
        std::this_thread::sleep_for(limits_.check_interval);
    }
}

// ===== ProcessHealthMonitor Implementation =====

void ProcessHealthMonitor::record_error(const std::string& error) {
    error_count_++;
    
    if (error_count_ >= ERROR_THRESHOLD) {
        status_ = HealthStatus::CRITICAL;
        last_report_.status = HealthStatus::CRITICAL;
        last_report_.message = "🔴 CRITICAL: Multiple consecutive errors - " + error;
    } else if (error_count_ >= ERROR_THRESHOLD / 2) {
        status_ = HealthStatus::WARNING;
        last_report_.status = HealthStatus::WARNING;
        last_report_.message = "🟡 WARNING: Error detected - " + error;
    }
    
    last_report_.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// ===== CPUPriorityManager Implementation =====

bool CPUPriorityManager::set_priority(Priority priority) {
    if (setpriority(PRIO_PROCESS, 0, static_cast<int>(priority)) == 0) {
        return true;
    }
    return false;
}

CPUPriorityManager::Priority CPUPriorityManager::get_priority() {
    errno = 0;
    int prio = getpriority(PRIO_PROCESS, 0);
    if (errno != 0) {
        return Priority::NORMAL;
    }
    return static_cast<Priority>(prio);
}

bool CPUPriorityManager::set_cpu_affinity(int core_id) {
#ifdef __linux__
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(core_id, &cpu_set);
    return sched_setaffinity(0, sizeof(cpu_set_t), &cpu_set) == 0;
#endif
    return false;
}

int CPUPriorityManager::get_cpu_count() {
    return sysconf(_SC_NPROCESSORS_ONLN);
}

// ===== FailsafeController Implementation =====

void FailsafeController::trigger_failsafe(RecoveryStrategy strategy) {
    recovery_attempts_++;
    
    if (recovery_attempts_ > MAX_RECOVERY_ATTEMPTS) {
        strategy = RecoveryStrategy::GRACEFUL_SHUTDOWN;
    }
    
    switch (strategy) {
        case RecoveryStrategy::IMMEDIATE_RESTART:
            std::cerr << "[FailsafeController] 🔄 Immediate restart triggered\n";
            // In real implementation: exec() to restart process
            break;
            
        case RecoveryStrategy::EXPONENTIAL_BACKOFF: {
            unsigned int backoff_ms = 100 * (1 << std::min(recovery_attempts_, size_t(5)));
            std::cerr << "[FailsafeController] ⏳ Exponential backoff: " 
                      << backoff_ms << "ms\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
            break;
        }
            
        case RecoveryStrategy::GRACEFUL_SHUTDOWN:
            std::cerr << "[FailsafeController] 🛑 Graceful shutdown triggered\n";
            std::exit(1);
            break;
            
        case RecoveryStrategy::ALERT_ONLY:
            std::cerr << "[FailsafeController] ⚠️ Alert only - no recovery action\n";
            break;
    }
}

}  // namespace ros2_dashboard
