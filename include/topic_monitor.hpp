/**
 * @file topic_monitor.hpp
 * @brief Topic health monitoring and rate tracking
 * @author Dashboard Team
 */

#pragma once

#include <string>
#include <chrono>
#include <cstdint>
#include <optional>
#include <map>
#include <vector>
#include <mutex>

namespace ros2_dashboard {

/**
 * @enum TopicHealthStatus
 * @brief Health status of a monitored topic
 */
enum class TopicHealthStatus {
    UNKNOWN,      ///< No data available yet
    HEALTHY,      ///< Topic is publishing normally (green)
    DEGRADED,     ///< Message rate below threshold (yellow)
    FAILED,       ///< No messages received or rate critical (red)
};

/**
 * @struct TopicMonitor
 * @brief Extended topic information with health monitoring
 */
struct TopicMonitor {
    // Basic topic information
    std::string name;
    std::string msg_type;
    std::string data_type;                    ///< Detailed message data type (e.g., std_msgs/msg/Float64)
    
    // Publisher/Subscriber info
    int publisher_count = 0;
    int subscription_count = 0;
    
    // Message rate tracking
    double current_rate_hz = 0.0;            ///< Current measured message rate
    double average_rate_hz = 0.0;            ///< Average rate over monitoring period
    double expected_rate_hz = 0.0;           ///< Expected/configured rate threshold
    int64_t message_count = 0;               ///< Total messages received
    
    // Health monitoring
    TopicHealthStatus health_status = TopicHealthStatus::UNKNOWN;
    std::string last_message_time;           ///< ISO 8601 timestamp
    int64_t time_since_last_message_ms = -1; ///< Milliseconds since last message
    
    // Rate degradation tracking
    double rate_threshold_percent = 80.0;    ///< Alert threshold (e.g., 80% of expected)
    bool is_rate_critical = false;           ///< True if rate < threshold
    int consecutive_slow_readings = 0;       ///< Consecutive slow readings
    int slow_reading_threshold = 3;          ///< Threshold for marking degraded
    
    // Transfer health
    bool is_transferring = true;             ///< Currently receiving messages
    int64_t last_check_timestamp_ms = 0;     ///< Last health check timestamp
    
    // Additional metadata
    std::string description;                  ///< User-added description/notes
    bool is_monitored = false;               ///< Whether actively monitoring this topic
    int64_t monitoring_start_time_ms = 0;    ///< When monitoring started
};

/**
 * @class TopicHealthMonitor
 * @brief Monitors topic health and message rates
 * 
 * Tracks message delivery rates, detects slow or failing topics,
 * and provides health status color coding (red/green).
 */
class TopicHealthMonitor {
public:
    TopicHealthMonitor();
    ~TopicHealthMonitor();

    /**
     * @brief Record a message received on a topic
     * @param topic_name Name of the topic
     * @param expected_rate_hz Expected message rate in Hz (optional)
     */
    void record_message(const std::string& topic_name, double expected_rate_hz = 0.0);

    /**
     * @brief Update topic monitoring
     * @param topic Name of the topic to monitor
     * @param expected_rate_hz Expected message rate threshold
     */
    void start_monitoring(const std::string& topic, double expected_rate_hz);

    /**
     * @brief Stop monitoring a topic
     * @param topic Name of the topic
     */
    void stop_monitoring(const std::string& topic);

    /**
     * @brief Get current health status for a topic
     * @param topic_name Name of the topic
     * @return TopicHealthStatus enum
     */
    TopicHealthStatus get_health_status(const std::string& topic_name) const;

    /**
     * @brief Get health percentage (0-100)
     * @param topic_name Name of the topic
     * @return Health score percentage, 0 if not monitored
     */
    double get_health_percentage(const std::string& topic_name) const;

    /**
     * @brief Get current message rate for a topic
     * @param topic_name Name of the topic
     * @return Current rate in Hz, -1 if not monitored
     */
    double get_current_rate_hz(const std::string& topic_name) const;

    /**
     * @brief Check if topic is transferring (actively receiving messages)
     * @param topic_name Name of the topic
     * @param timeout_ms Consider not transferring if no messages in this time
     * @return true if actively receiving messages
     */
    bool is_topic_transferring(const std::string& topic_name, int64_t timeout_ms = 1000) const;

    /**
     * @brief Get time since last message on a topic
     * @param topic_name Name of the topic
     * @return Milliseconds since last message, -1 if no messages received
     */
    int64_t get_time_since_last_message_ms(const std::string& topic_name) const;

    /**
     * @brief Calculate current rate based on message history
     * @details Analyzes message timestamps to determine current rate
     * @return Rate in Hz, 0 if insufficient data
     */
    void update_all_rates();

    /**
     * @brief Get color for health status (for UI rendering)
     * @param status The health status
     * @return Color hex code: "#00AA00" (green), "#FFAA00" (yellow), "#FF0000" (red)
     */
    static std::string get_status_color(TopicHealthStatus status);

    /**
     * @brief Get status string for display
     * @param status The health status
     * @return Human-readable status string
     */
    static std::string get_status_string(TopicHealthStatus status);

    /**
     * @brief Clear all monitoring data
     */
    void clear_all();

    /**
     * @brief Get number of monitored topics
     */
    size_t get_monitored_topics_count() const;

private:
    /**
     * @struct RateTracker
     * @brief Internal structure for tracking message rates
     */
    struct RateTracker {
        std::chrono::steady_clock::time_point last_message_time;
        std::chrono::steady_clock::time_point rate_window_start;
        int64_t messages_in_window = 0;
        std::vector<std::chrono::steady_clock::time_point> recent_messages;  // Last 100 messages
        double last_calculated_rate = 0.0;
        double expected_rate = 0.0;
    };

    mutable std::mutex monitor_mutex_;
    std::map<std::string, RateTracker> rate_trackers_;
    std::map<std::string, TopicMonitor> monitored_topics_;
    
    static constexpr int64_t RATE_WINDOW_MS = 1000;  // 1 second window
    static constexpr size_t MAX_MESSAGE_HISTORY = 100;
    static constexpr int SLOW_READING_THRESHOLD = 3;
    static constexpr double CRITICAL_RATE_THRESHOLD = 50.0;  // 50% of expected
};

}  // namespace ros2_dashboard

