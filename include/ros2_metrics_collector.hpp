/**
 * @file ros2_metrics_collector.hpp
 * @brief ROS 2 specific metrics collection (topics, nodes, services, subscriptions)
 * @author Dashboard Team
 */

#pragma once

#include <string>
#include <map>
#include <vector>
#include <cstdint>
#include <mutex>
#include <chrono>

namespace ros2_dashboard {

/**
 * @struct ROS2Metrics
 * @brief ROS 2 ecosystem metrics
 */
struct ROS2Metrics {
    // Topic metrics
    uint32_t total_topics = 0;
    uint32_t new_topics_per_min = 0;
    double avg_topic_discovery_time_ms = 0.0;
    
    // Node metrics
    uint32_t total_nodes = 0;
    uint32_t active_publishers = 0;
    uint32_t active_subscribers = 0;
    
    // Message throughput
    uint64_t total_messages_per_sec = 0;
    uint64_t peak_messages_per_sec = 0;
    double avg_message_size_bytes = 0.0;
    
    // Service metrics
    uint32_t total_services = 0;
    uint32_t service_calls_per_min = 0;
    double avg_service_latency_ms = 0.0;
    
    // Recording metrics (ROS 2 specific)
    uint32_t topics_being_recorded = 0;
    double recording_throughput_mbps = 0.0;
    uint64_t total_messages_recorded = 0;
    
    // Network metrics
    double estimated_network_load_percent = 0.0;
    uint32_t message_drops = 0;
    
    // Performance indicators
    double system_latency_ms = 0.0;
    uint32_t rmw_memory_usage_mb = 0;
};

/**
 * @class ROS2MetricsCollector
 * @brief Collects ROS 2 specific metrics
 * 
 * Tracks:
 * - Topic discovery rate and history
 * - Node activity (publishers, subscribers)
 * - Message throughput across system
 * - Service call frequency and latency
 * - Recording session statistics
 * - Overall ROS 2 system health
 */
class ROS2MetricsCollector {
public:
    ROS2MetricsCollector();
    ~ROS2MetricsCollector() = default;
    
    // Prevent copying
    ROS2MetricsCollector(const ROS2MetricsCollector&) = delete;
    ROS2MetricsCollector& operator=(const ROS2MetricsCollector&) = delete;
    
    // Record topic discovery event
    void record_topic_discovery(uint32_t total_topics, double discovery_time_ms);
    
    // Record node activity
    void record_node_activity(uint32_t total_nodes, uint32_t publishers, uint32_t subscribers);
    
    // Record message throughput
    void record_message_throughput(uint64_t messages_per_sec, double avg_size_bytes);
    
    // Record service activity
    void record_service_activity(uint32_t total_services, uint32_t calls_per_min, double latency_ms);
    
    // Record recording session statistics
    void record_session_stats(uint32_t topics_recorded, double throughput_mbps, uint64_t total_messages);
    
    // Record network metrics
    void record_network_metrics(double load_percent, uint32_t message_drops);
    
    // Record system latency
    void record_system_latency(double latency_ms, uint32_t rmw_memory_mb);
    
    // Get current metrics snapshot
    ROS2Metrics get_metrics() const;
    
    // Get historical data for charting
    std::vector<double> get_topic_discovery_history() const;
    std::vector<double> get_message_throughput_history() const;
    std::vector<double> get_service_latency_history() const;
    std::vector<double> get_recording_throughput_history() const;
    
    // Reset all metrics
    void reset();

private:
    mutable std::mutex mtx_;
    
    ROS2Metrics current_metrics_;
    
    // History buffers (last 60 data points for ~1 minute history)
    static constexpr size_t HISTORY_SIZE = 60;
    std::vector<double> topic_discovery_history_;
    std::vector<double> message_throughput_history_;
    std::vector<double> service_latency_history_;
    std::vector<double> recording_throughput_history_;
    
    void add_to_history_(std::vector<double>& history, double value);
};

}  // namespace ros2_dashboard
