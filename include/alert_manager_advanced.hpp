/**
 * @file alert_manager_advanced.hpp
 * @brief Advanced alerting system with configurable thresholds and notifications
 * @author Dashboard Team
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <cstdint>
#include <mutex>

namespace ros2_dashboard {

/**
 * @enum AlertSeverity
 * @brief Alert severity levels
 */
enum class AlertSeverity {
    INFO = 0,
    WARNING = 1,
    CRITICAL = 2
};

/**
 * @enum AlertType
 * @brief Types of alerts that can be generated
 */
enum class AlertType {
    TOPIC_NO_MESSAGES,      // Topic stopped publishing
    TOPIC_HIGH_LATENCY,     // Message latency exceeds threshold
    TOPIC_HIGH_RATE,        // Publish rate exceeds threshold
    TOPIC_DROPPED_MESSAGES, // Message drops detected
    NODE_UNRESPONSIVE,      // Node not responding
    DISK_SPACE_LOW,         // Disk space below threshold
    MEMORY_USAGE_HIGH,      // Memory usage exceeds threshold
    RECORDING_FAILED,       // Recording encountered error
    SERVICE_TIMEOUT,        // Service call timeout
    CUSTOM_THRESHOLD        // Custom user-defined threshold
};

/**
 * @struct Alert
 * @brief Individual alert record
 */
struct Alert {
    AlertType type;
    AlertSeverity severity;
    std::string topic_name;      // If topic-related
    std::string message;
    uint64_t timestamp_ms;
    std::string component;       // Which component raised alert
    bool auto_resolved = false;
    uint64_t resolution_time_ms = 0;
    int occurrence_count = 1;    // How many times has this alert occurred?
};

/**
 * @struct AlertThreshold
 * @brief Configurable threshold for topic alerts
 */
struct AlertThreshold {
    std::string topic_name;
    AlertType alert_type;
    double threshold_value = 0.0;
    AlertSeverity severity = AlertSeverity::WARNING;
    uint32_t duration_seconds = 5;  // How long condition must persist
    bool enabled = true;
};

/**
 * @class AlertManager
 * @brief Manages system alerts with configurable thresholds
 * 
 * Features:
 * - Configurable per-topic thresholds
 * - Alert history with timestamps
 * - Severity levels (INFO, WARNING, CRITICAL)
 * - Auto-resolution when condition clears
 * - Alert aggregation (prevent duplicate spam)
 * - Notification callbacks
 * - Alert correlation
 * - Statistics and analytics
 */
class AlertManager {
public:
    AlertManager();
    ~AlertManager() = default;
    
    // Prevent copying
    AlertManager(const AlertManager&) = delete;
    AlertManager& operator=(const AlertManager&) = delete;

    // Alert generation
    void raise_alert(const Alert& alert);
    void raise_topic_alert(AlertType type, const std::string& topic_name,
                          AlertSeverity severity, const std::string& message);
    void raise_system_alert(AlertType type, AlertSeverity severity, 
                           const std::string& message);

    // Threshold management
    void set_threshold(const AlertThreshold& threshold);
    void remove_threshold(const std::string& topic_name, AlertType type);
    std::vector<AlertThreshold> get_thresholds() const;
    AlertThreshold* get_threshold(const std::string& topic_name, AlertType type);

    // Alert queries
    std::vector<Alert> get_active_alerts() const;
    std::vector<Alert> get_alert_history(uint32_t limit = 100) const;
    std::vector<Alert> get_alerts_for_topic(const std::string& topic_name) const;
    std::vector<Alert> get_alerts_by_severity(AlertSeverity severity) const;

    // Alert acknowledgment
    void acknowledge_alert(size_t alert_id);
    void clear_resolved_alerts();
    void clear_all_alerts();

    // Notification callbacks
    using AlertCallback = std::function<void(const Alert&)>;
    void on_alert(AlertCallback callback);
    void on_severity_change(AlertCallback callback);

    // Statistics
    struct AlertStats {
        uint32_t total_alerts_raised = 0;
        uint32_t active_alerts_count = 0;
        uint32_t critical_count = 0;
        uint32_t warning_count = 0;
        uint32_t info_count = 0;
        double avg_resolution_time_ms = 0.0;
    };
    AlertStats get_statistics() const;

    // Check thresholds and auto-generate alerts
    void check_topic_metrics(const std::string& topic_name, 
                            double message_rate, 
                            double latency_ms,
                            uint32_t dropped_messages);

    // Reset
    void reset();

private:
    mutable std::mutex mtx_;
    
    std::vector<Alert> active_alerts_;
    std::vector<Alert> alert_history_;
    std::vector<AlertThreshold> thresholds_;
    
    std::vector<AlertCallback> alert_callbacks_;
    std::vector<AlertCallback> severity_change_callbacks_;
    
    // Alert deduplication
    std::map<std::string, uint64_t> last_alert_time_;  // topic+type -> timestamp
    static constexpr uint64_t ALERT_DEDUPE_WINDOW_MS = 5000;  // 5s
    
    static constexpr size_t MAX_HISTORY = 1000;
    
    std::string alert_key_(const Alert& alert) const;
    bool should_dedupe_(const Alert& alert);
    void trigger_callbacks_(const Alert& alert);
};

}  // namespace ros2_dashboard
