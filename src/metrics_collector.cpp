/**
 * @file metrics_collector.cpp
 * @brief Real-time metrics collection implementation
 */

#include "metrics_collector.hpp"
#include <chrono>
#include <fstream>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <sys/statvfs.h>

namespace ros2_dashboard {

MetricsCollector::MetricsCollector()
    : collection_interval_ms_(1000) {
}

MetricsCollector::~MetricsCollector() = default;

SystemMetrics MetricsCollector::get_system_metrics() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return collect_system_metrics_();
}

RecordingMetrics MetricsCollector::get_recording_metrics() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return current_recording_;
}

NetworkMetrics MetricsCollector::get_network_metrics() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return current_network_;
}

ApplicationMetrics MetricsCollector::get_application_metrics() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return current_application_;
}

MetricsSnapshot MetricsCollector::get_snapshot() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    MetricsSnapshot snapshot;
    snapshot.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    snapshot.system = collect_system_metrics_();
    snapshot.recording = current_recording_;
    snapshot.network = current_network_;
    snapshot.application = current_application_;
    
    history_.push_back(snapshot);
    trim_history_();
    
    return snapshot;
}

std::vector<MetricsSnapshot> MetricsCollector::get_history(
    int period_seconds) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    std::vector<MetricsSnapshot> result;
    auto cutoff_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() - 
        (period_seconds * 1000);
    
    for (const auto& snapshot : history_) {
        if (static_cast<int64_t>(snapshot.timestamp_ms) >= cutoff_time) {
            result.push_back(snapshot);
        }
    }
    
    return result;
}

void MetricsCollector::clear_history() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    history_.clear();
}

bool MetricsCollector::export_csv(const std::string& filepath,
                                 int period_seconds) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    try {
        std::ofstream file(filepath);
        if (!file.is_open()) return false;

        // Write CSV header
        file << "timestamp_ms,cpu_percent,memory_mb,disk_free_mb,"
             << "write_speed_mbps,messages_per_sec,frame_rate_fps,"
             << "cache_hit_ratio\n";

        // Write data rows
        for (const auto& snapshot : get_history(period_seconds)) {
            file << snapshot.timestamp_ms << ","
                 << snapshot.system.cpu_usage_percent << ","
                 << snapshot.system.memory_usage_mb << ","
                 << snapshot.system.disk_free_mb << ","
                 << snapshot.recording.write_speed_mbps << ","
                 << snapshot.recording.messages_per_second << ","
                 << snapshot.application.ui_frame_rate_fps << ","
                 << snapshot.application.cache_hit_ratio << "\n";
        }

        file.close();
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool MetricsCollector::export_json(const std::string& filepath,
                                  int period_seconds) {
    // TODO: Implement JSON export using nlohmann/json
    return false;
}

void MetricsCollector::set_collection_interval(int interval_ms) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    collection_interval_ms_ = std::max(500, std::min(10000, interval_ms));
}

void MetricsCollector::update_recording_metrics(double write_speed_mbps,
                                               int64_t messages_per_sec,
                                               double compression_ratio,
                                               int64_t bytes_written) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    current_recording_.write_speed_mbps = write_speed_mbps;
    current_recording_.messages_per_second = messages_per_sec;
    current_recording_.compression_ratio = compression_ratio;
    current_recording_.bytes_written = bytes_written;
}

void MetricsCollector::update_network_metrics(double bandwidth_mbps,
                                             bool connected,
                                             int32_t pending_uploads,
                                             int32_t active_uploads) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    current_network_.bandwidth_mbps = bandwidth_mbps;
    current_network_.connected = connected;
    current_network_.pending_uploads = pending_uploads;
    current_network_.active_uploads = active_uploads;
}

void MetricsCollector::update_application_metrics(double frame_rate,
                                                 double cache_hit_ratio,
                                                 int32_t thread_count,
                                                 int32_t queue_size) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    current_application_.ui_frame_rate_fps = frame_rate;
    current_application_.cache_hit_ratio = cache_hit_ratio;
    current_application_.active_thread_count = thread_count;
    current_application_.task_queue_size = queue_size;
}

SystemMetrics MetricsCollector::collect_system_metrics_() {
    SystemMetrics metrics;
    metrics.cpu_usage_percent = parse_cpu_usage_();
    metrics.memory_usage_mb = parse_memory_usage_();
    
    // Get disk space
    struct statvfs statfs_buf;
    if (statvfs("/", &statfs_buf) == 0) {
        metrics.disk_free_mb = (statfs_buf.f_bavail * statfs_buf.f_frsize) / (1024 * 1024);
        metrics.disk_total_mb = (statfs_buf.f_blocks * statfs_buf.f_frsize) / (1024 * 1024);
    }
    
    return metrics;
}

void MetricsCollector::trim_history_() {
    if (history_.size() > static_cast<size_t>(max_history_samples_)) {
        history_.erase(history_.begin());
    }
}

double MetricsCollector::parse_cpu_usage_() {
    try {
        std::ifstream file("/proc/stat");
        if (!file.is_open()) return 0.0;

        std::string line;
        std::getline(file, line);
        file.close();

        // Simple CPU usage estimation (can be enhanced)
        return 25.0;  // Placeholder
    } catch (const std::exception&) {
        return 0.0;
    }
}

int64_t MetricsCollector::parse_memory_usage_() {
    try {
        std::ifstream file("/proc/self/status");
        if (!file.is_open()) return 0;

        std::string line;
        while (std::getline(file, line)) {
            if (line.find("VmRSS:") == 0) {
                // Extract memory in KB and convert to MB
                std::istringstream iss(line);
                std::string label;
                int64_t memory_kb;
                iss >> label >> memory_kb;
                file.close();
                return memory_kb / 1024;
            }
        }
        file.close();
    } catch (const std::exception&) {
    }
    return 0;
}

}  // namespace ros2_dashboard
