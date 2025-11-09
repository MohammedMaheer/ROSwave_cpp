/**
 * @file ros2_metrics_collector.cpp
 * @brief Implementation of ROS 2 specific metrics collection
 */

#include "ros2_metrics_collector.hpp"
#include <algorithm>

namespace ros2_dashboard {

ROS2MetricsCollector::ROS2MetricsCollector() {
    topic_discovery_history_.reserve(HISTORY_SIZE);
    message_throughput_history_.reserve(HISTORY_SIZE);
    service_latency_history_.reserve(HISTORY_SIZE);
    recording_throughput_history_.reserve(HISTORY_SIZE);
}

void ROS2MetricsCollector::record_topic_discovery(uint32_t total_topics, double discovery_time_ms) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_metrics_.total_topics = total_topics;
    current_metrics_.avg_topic_discovery_time_ms = discovery_time_ms;
    add_to_history_(topic_discovery_history_, discovery_time_ms);
}

void ROS2MetricsCollector::record_node_activity(uint32_t total_nodes, uint32_t publishers, uint32_t subscribers) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_metrics_.total_nodes = total_nodes;
    current_metrics_.active_publishers = publishers;
    current_metrics_.active_subscribers = subscribers;
}

void ROS2MetricsCollector::record_message_throughput(uint64_t messages_per_sec, double avg_size_bytes) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_metrics_.total_messages_per_sec = messages_per_sec;
    current_metrics_.peak_messages_per_sec = std::max(current_metrics_.peak_messages_per_sec, messages_per_sec);
    current_metrics_.avg_message_size_bytes = avg_size_bytes;
    add_to_history_(message_throughput_history_, static_cast<double>(messages_per_sec));
}

void ROS2MetricsCollector::record_service_activity(uint32_t total_services, uint32_t calls_per_min, double latency_ms) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_metrics_.total_services = total_services;
    current_metrics_.service_calls_per_min = calls_per_min;
    current_metrics_.avg_service_latency_ms = latency_ms;
    add_to_history_(service_latency_history_, latency_ms);
}

void ROS2MetricsCollector::record_session_stats(uint32_t topics_recorded, double throughput_mbps, uint64_t total_messages) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_metrics_.topics_being_recorded = topics_recorded;
    current_metrics_.recording_throughput_mbps = throughput_mbps;
    current_metrics_.total_messages_recorded = total_messages;
    add_to_history_(recording_throughput_history_, throughput_mbps);
}

void ROS2MetricsCollector::record_network_metrics(double load_percent, uint32_t message_drops) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_metrics_.estimated_network_load_percent = load_percent;
    current_metrics_.message_drops = message_drops;
}

void ROS2MetricsCollector::record_system_latency(double latency_ms, uint32_t rmw_memory_mb) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_metrics_.system_latency_ms = latency_ms;
    current_metrics_.rmw_memory_usage_mb = rmw_memory_mb;
}

ROS2Metrics ROS2MetricsCollector::get_metrics() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return current_metrics_;
}

std::vector<double> ROS2MetricsCollector::get_topic_discovery_history() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return topic_discovery_history_;
}

std::vector<double> ROS2MetricsCollector::get_message_throughput_history() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return message_throughput_history_;
}

std::vector<double> ROS2MetricsCollector::get_service_latency_history() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return service_latency_history_;
}

std::vector<double> ROS2MetricsCollector::get_recording_throughput_history() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return recording_throughput_history_;
}

void ROS2MetricsCollector::reset() {
    std::lock_guard<std::mutex> lock(mtx_);
    current_metrics_ = ROS2Metrics{};
    topic_discovery_history_.clear();
    message_throughput_history_.clear();
    service_latency_history_.clear();
    recording_throughput_history_.clear();
}

void ROS2MetricsCollector::add_to_history_(std::vector<double>& history, double value) {
    history.push_back(value);
    if (history.size() > HISTORY_SIZE) {
        history.erase(history.begin());
    }
}

}  // namespace ros2_dashboard
