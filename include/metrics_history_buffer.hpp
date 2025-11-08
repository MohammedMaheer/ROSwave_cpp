/**
 * @file metrics_history_buffer.hpp
 * @brief Circular buffer storage for historical metrics data
 * @author Dashboard Team
 * 
 * Maintains bounded circular buffers for all metrics (CPU, memory, disk, network, etc).
 * Perfect for live charting - no unbounded growth, automatic eviction of old data.
 * 
 * Capacity: 600 samples (at 1Hz = 10 minutes of history)
 */

#pragma once

#include <cstdint>
#include <mutex>
#include <memory>
#include "containers/bounded_deque.hpp"

namespace ros2_dashboard {

/**
 * @struct MetricsPoint
 * @brief Single point in time for all metrics
 */
struct MetricsPoint {
    uint64_t timestamp_ms = 0;       ///< Unix time in milliseconds
    double cpu_percent = 0.0;        ///< CPU usage 0-100%
    int64_t memory_mb = 0;           ///< Memory in MB
    double disk_io_mbps = 0.0;       ///< Disk I/O throughput
    double network_mbps = 0.0;       ///< Network bandwidth
    int32_t temperature_c = -1;      ///< CPU temperature (-1 if N/A)
    double frame_rate_fps = 0.0;     ///< UI frame rate
    double cache_ratio = 0.0;        ///< Cache hit ratio 0-1.0
};

/**
 * @class MetricsHistoryBuffer
 * @brief Bounded circular buffers for metrics time-series data
 * 
 * Thread-safe read operations, single-threaded write pattern.
 * Designed for high-frequency updates without allocation overhead.
 * 
 * Capacity: 600 points per series (tunable)
 * Memory footprint: ~62KB for all metrics at 600pt capacity
 * 
 * Example:
 * @code
 *   MetricsHistoryBuffer history;
 *   
 *   // Add metrics periodically
 *   MetricsPoint pt;
 *   pt.timestamp_ms = now_ms();
 *   pt.cpu_percent = 42.5;
 *   pt.memory_mb = 256;
 *   history.push_point(std::move(pt));
 *   
 *   // Read all history for charting
 *   auto points = history.get_all_points();
 *   for (const auto& p : points) {
 *       chart_update(p.timestamp_ms, p.cpu_percent);
 *   }
 * @endcode
 */
class MetricsHistoryBuffer {
public:
    static constexpr size_t DEFAULT_CAPACITY = 600;  ///< 10 min at 1Hz

    /**
     * @brief Constructor with configurable capacity
     * @param capacity Maximum number of points to store
     */
    explicit MetricsHistoryBuffer(size_t capacity = DEFAULT_CAPACITY);

    ~MetricsHistoryBuffer() = default;

    /**
     * @brief Add a metrics snapshot to history
     * 
     * When buffer is full, automatically evicts oldest point.
     * Thread-safe for reading while writing (atomic push).
     * 
     * @param point The metrics snapshot to add
     */
    void push_point(const MetricsPoint& point);

    /**
     * @brief Add metrics using move semantics
     */
    void push_point(MetricsPoint&& point);

    /**
     * @brief Get all historical points as a vector copy
     * 
     * Linearizes the circular buffer for easy iteration.
     * Thread-safe snapshot.
     * 
     * @return Vector of all MetricsPoints in chronological order
     */
    std::vector<MetricsPoint> get_all_points() const;

    /**
     * @brief Get last N points
     * 
     * Useful for partial updates or windowing charts.
     * 
     * @param n Number of recent points (clamped to size)
     * @return Vector of last N points in chronological order
     */
    std::vector<MetricsPoint> get_last_n_points(size_t n) const;

    /**
     * @brief Get points within time window
     * 
     * @param from_ms Start time (inclusive)
     * @param to_ms End time (inclusive)
     * @return Vector of points in [from_ms, to_ms]
     */
    std::vector<MetricsPoint> get_points_in_range(uint64_t from_ms, uint64_t to_ms) const;

    /**
     * @brief Get oldest point
     * 
     * @param out_point Output parameter
     * @return true if buffer has at least one point
     */
    bool get_first_point(MetricsPoint& out_point) const;

    /**
     * @brief Get newest point
     * 
     * @param out_point Output parameter
     * @return true if buffer has at least one point
     */
    bool get_last_point(MetricsPoint& out_point) const;

    /**
     * @brief Get point at index
     * 
     * Index 0 = oldest, size-1 = newest
     * 
     * @param index Position
     * @param out_point Output parameter
     * @return true if index is valid
     */
    bool get_point_at(size_t index, MetricsPoint& out_point) const;

    /**
     * @brief Get statistics for a metric over all history
     */
    struct MetricStats {
        double min = 0.0;
        double max = 0.0;
        double avg = 0.0;
        double latest = 0.0;
    };

    /**
     * @brief Calculate CPU statistics
     */
    MetricStats get_cpu_stats() const;

    /**
     * @brief Calculate memory statistics
     */
    MetricStats get_memory_stats() const;

    /**
     * @brief Calculate disk I/O statistics
     */
    MetricStats get_disk_io_stats() const;

    /**
     * @brief Calculate network statistics
     */
    MetricStats get_network_stats() const;

    /**
     * @brief Calculate frame rate statistics
     */
    MetricStats get_frame_rate_stats() const;

    /**
     * @brief Clear all historical data
     */
    void clear();

    /**
     * @brief Get current number of points stored
     */
    size_t size() const;

    /**
     * @brief Check if buffer is empty
     */
    bool empty() const;

    /**
     * @brief Get maximum capacity
     */
    size_t capacity() const;

    /**
     * @brief Check if buffer is full
     */
    bool full() const;

private:
    using PointBuffer = containers::BoundedDeque<MetricsPoint, DEFAULT_CAPACITY>;
    
    mutable std::mutex buffer_mutex_;
    PointBuffer buffer_;

    /**
     * @brief Helper to calculate min/max/avg for a metric accessor
     */
    MetricStats calculate_stats_(
        std::function<double(const MetricsPoint&)> accessor) const;
};

}  // namespace ros2_dashboard
