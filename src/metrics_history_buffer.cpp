/**
 * @file metrics_history_buffer.cpp
 * @brief Implementation of circular metrics buffer
 */

#include "metrics_history_buffer.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace ros2_dashboard {

MetricsHistoryBuffer::MetricsHistoryBuffer(size_t capacity)
    : buffer_() {
    // Note: buffer_ is always fixed to DEFAULT_CAPACITY (600)
    // capacity parameter is kept for API compatibility
    // To change capacity, modify the template parameter in header
}

void MetricsHistoryBuffer::push_point(const MetricsPoint& point) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_.push_back(point);
}

void MetricsHistoryBuffer::push_point(MetricsPoint&& point) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_.push_back(std::move(point));
}

std::vector<MetricsPoint> MetricsHistoryBuffer::get_all_points() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return buffer_.to_vector();
}

std::vector<MetricsPoint> MetricsHistoryBuffer::get_last_n_points(size_t n) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    size_t actual_n = std::min(n, buffer_.size());
    
    std::vector<MetricsPoint> result;
    result.reserve(actual_n);
    
    // Get last n points (iterate backwards from end)
    for (size_t i = buffer_.size() - actual_n; i < buffer_.size(); ++i) {
        result.push_back(buffer_[i]);
    }
    return result;
}

std::vector<MetricsPoint> MetricsHistoryBuffer::get_points_in_range(
    uint64_t from_ms, uint64_t to_ms) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    std::vector<MetricsPoint> result;
    for (size_t i = 0; i < buffer_.size(); ++i) {
        const auto& pt = buffer_[i];
        if (pt.timestamp_ms >= from_ms && pt.timestamp_ms <= to_ms) {
            result.push_back(pt);
        }
    }
    return result;
}

bool MetricsHistoryBuffer::get_first_point(MetricsPoint& out_point) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (buffer_.empty()) return false;
    out_point = buffer_.front();
    return true;
}

bool MetricsHistoryBuffer::get_last_point(MetricsPoint& out_point) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (buffer_.empty()) return false;
    out_point = buffer_.back();
    return true;
}

bool MetricsHistoryBuffer::get_point_at(size_t index, MetricsPoint& out_point) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (index >= buffer_.size()) return false;
    out_point = buffer_[index];
    return true;
}

MetricsHistoryBuffer::MetricStats MetricsHistoryBuffer::get_cpu_stats() const {
    return calculate_stats_([](const MetricsPoint& pt) { return pt.cpu_percent; });
}

MetricsHistoryBuffer::MetricStats MetricsHistoryBuffer::get_memory_stats() const {
    return calculate_stats_([](const MetricsPoint& pt) { 
        return static_cast<double>(pt.memory_mb); 
    });
}

MetricsHistoryBuffer::MetricStats MetricsHistoryBuffer::get_disk_io_stats() const {
    return calculate_stats_([](const MetricsPoint& pt) { return pt.disk_io_mbps; });
}

MetricsHistoryBuffer::MetricStats MetricsHistoryBuffer::get_network_stats() const {
    return calculate_stats_([](const MetricsPoint& pt) { return pt.network_mbps; });
}

MetricsHistoryBuffer::MetricStats MetricsHistoryBuffer::get_frame_rate_stats() const {
    return calculate_stats_([](const MetricsPoint& pt) { return pt.frame_rate_fps; });
}

void MetricsHistoryBuffer::clear() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_.clear();
}

size_t MetricsHistoryBuffer::size() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return buffer_.size();
}

bool MetricsHistoryBuffer::empty() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return buffer_.empty();
}

size_t MetricsHistoryBuffer::capacity() const {
    return DEFAULT_CAPACITY;
}

bool MetricsHistoryBuffer::full() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return buffer_.full();
}

MetricsHistoryBuffer::MetricStats MetricsHistoryBuffer::calculate_stats_(
    std::function<double(const MetricsPoint&)> accessor) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    MetricStats stats;
    if (buffer_.empty()) {
        return stats;
    }

    std::vector<double> values;
    values.reserve(buffer_.size());
    
    for (size_t i = 0; i < buffer_.size(); ++i) {
        values.push_back(accessor(buffer_[i]));
    }

    // Calculate min/max
    auto min_max = std::minmax_element(values.begin(), values.end());
    stats.min = *min_max.first;
    stats.max = *min_max.second;

    // Calculate average
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    stats.avg = sum / values.size();

    // Latest is last value
    stats.latest = accessor(buffer_[buffer_.size() - 1]);

    return stats;
}

}  // namespace ros2_dashboard
