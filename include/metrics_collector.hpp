/**
 * @file metrics_collector.hpp
 * @brief Real-time system, recording, and network metrics collection
 * @author Dashboard Team
 */

#pragma once

#include <string>
#include <map>
#include <vector>
#include <cstdint>
#include <mutex>
#include <memory>

namespace ros2_dashboard {

/**
 * @struct SystemMetrics
 * @brief System-level performance metrics
 */
struct SystemMetrics {
    double cpu_usage_percent = 0.0;      ///< CPU usage 0-100%
    int64_t memory_usage_mb = 0;         ///< Memory usage in MB
    int64_t disk_free_mb = 0;            ///< Free disk space in MB
    int64_t disk_total_mb = 0;           ///< Total disk space in MB
    int32_t temperature_celsius = -1;    ///< CPU temperature (-1 if unavailable)
    int32_t thermal_throttle_count = 0;  ///< Thermal throttling events
};

/**
 * @struct RecordingMetrics
 * @brief Metrics specific to active recording
 */
struct RecordingMetrics {
    double write_speed_mbps = 0.0;       ///< Current write speed MB/s
    int64_t messages_per_second = 0;     ///< Message throughput
    double compression_ratio = 1.0;      ///< Current compression ratio
    int64_t bytes_written = 0;           ///< Total bytes written
    std::string elapsed_time;            ///< HH:MM:SS format
};

/**
 * @struct NetworkMetrics
 * @brief Network performance metrics
 */
struct NetworkMetrics {
    double bandwidth_mbps = 0.0;         ///< Network bandwidth usage
    bool connected = false;              ///< Network connectivity status
    int32_t pending_uploads = 0;         ///< Queued uploads count
    int32_t active_uploads = 0;          ///< Currently uploading count
};

/**
 * @struct ApplicationMetrics
 * @brief Application-level performance metrics
 */
struct ApplicationMetrics {
    double ui_frame_rate_fps = 0.0;      ///< UI frame rate
    double cache_hit_ratio = 0.0;        ///< Cache effectiveness 0-1.0
    int32_t active_thread_count = 0;     ///< Active worker threads
    int32_t task_queue_size = 0;         ///< Pending tasks in queue
};

/**
 * @struct MetricsSnapshot
 * @brief Complete metrics snapshot at a point in time
 */
struct MetricsSnapshot {
    uint64_t timestamp_ms = 0;           ///< UNIX timestamp in ms
    SystemMetrics system;
    RecordingMetrics recording;
    NetworkMetrics network;
    ApplicationMetrics application;
};

/**
 * @class MetricsCollector
 * @brief Collects and maintains real-time metrics
 * 
 * Thread-safe collector for system, recording, and network metrics.
 * Supports historical metric storage and export.
 */
class MetricsCollector {
public:
    MetricsCollector();
    ~MetricsCollector();

    /**
     * @brief Get current system metrics
     * @return SystemMetrics struct
     */
    SystemMetrics get_system_metrics();

    /**
     * @brief Get current recording metrics
     * @return RecordingMetrics struct
     */
    RecordingMetrics get_recording_metrics();

    /**
     * @brief Get current network metrics
     * @return NetworkMetrics struct
     */
    NetworkMetrics get_network_metrics();

    /**
     * @brief Get current application metrics
     * @return ApplicationMetrics struct
     */
    ApplicationMetrics get_application_metrics();

    /**
     * @brief Get complete metrics snapshot
     * @return MetricsSnapshot with all metrics
     */
    MetricsSnapshot get_snapshot();

    /**
     * @brief Get historical metrics
     * @param period_seconds History period in seconds (3600, 86400, 604800)
     * @return Vector of MetricsSnapshot samples
     */
    std::vector<MetricsSnapshot> get_history(int period_seconds);

    /**
     * @brief Clear historical metrics
     */
    void clear_history();

    /**
     * @brief Export metrics to CSV format
     * @param filepath Output file path
     * @param period_seconds History period to export
     * @return true if export succeeded
     */
    bool export_csv(const std::string& filepath, int period_seconds);

    /**
     * @brief Export metrics to JSON format
     * @param filepath Output file path
     * @param period_seconds History period to export
     * @return true if export succeeded
     */
    bool export_json(const std::string& filepath, int period_seconds);

    /**
     * @brief Set metric collection interval
     * @param interval_ms Interval in milliseconds (500-10000ms)
     */
    void set_collection_interval(int interval_ms);

    /**
     * @brief Update recording metrics
     * @param write_speed_mbps Write speed in MB/s
     * @param messages_per_sec Message rate
     * @param compression_ratio Compression ratio
     * @param bytes_written Total bytes written
     */
    void update_recording_metrics(double write_speed_mbps,
                                  int64_t messages_per_sec,
                                  double compression_ratio,
                                  int64_t bytes_written);

    /**
     * @brief Update network metrics
     * @param bandwidth_mbps Bandwidth usage
     * @param connected Network connectivity
     * @param pending_uploads Queued uploads
     * @param active_uploads Currently uploading
     */
    void update_network_metrics(double bandwidth_mbps,
                                bool connected,
                                int32_t pending_uploads,
                                int32_t active_uploads);

    /**
     * @brief Update application metrics
     * @param frame_rate UI frame rate
     * @param cache_hit_ratio Cache effectiveness
     * @param thread_count Active threads
     * @param queue_size Task queue size
     */
    void update_application_metrics(double frame_rate,
                                    double cache_hit_ratio,
                                    int32_t thread_count,
                                    int32_t queue_size);

private:
    mutable std::mutex metrics_mutex_;
    
    SystemMetrics current_system_;
    RecordingMetrics current_recording_;
    NetworkMetrics current_network_;
    ApplicationMetrics current_application_;
    
    std::vector<MetricsSnapshot> history_;
    int max_history_samples_ = 3600;  // ~1 hour at 1s intervals
    int collection_interval_ms_ = 1000;

    // Internal methods
    SystemMetrics collect_system_metrics_();
    void trim_history_();
    double parse_cpu_usage_();
    int64_t parse_memory_usage_();
};

}  // namespace ros2_dashboard
