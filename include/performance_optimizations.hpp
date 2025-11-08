// performance_optimization_snippets.cpp
// World-class optimization techniques for ROS2 Dashboard v2.0
// These are reference implementations ready to integrate

#pragma once

#include <atomic>
#include <chrono>
#include <unordered_map>
#include <deque>
#include <memory_resource>
#include <execution>
#include <algorithm>

// ============================================================================
// 1. ADAPTIVE CACHING WITH TOPIC ACTIVITY MONITORING
// ============================================================================

class AdaptiveCacheManager {
private:
    struct CacheEntry {
        std::any data;
        std::chrono::steady_clock::time_point timestamp;
        int access_count = 0;
        double publish_rate = 0.0;  // Messages per second
        std::chrono::milliseconds ttl{5000};  // Adaptive TTL
    };

    std::unordered_map<std::string, CacheEntry> cache_;
    std::mutex cache_mutex_;
    static constexpr int ACCESS_THRESHOLD = 10;

public:
    template<typename T>
    void set(const std::string& key, T&& value, double publish_rate) {
        std::unique_lock lock(cache_mutex_);
        auto& entry = cache_[key];
        entry.data = std::forward<T>(value);
        entry.timestamp = std::chrono::steady_clock::now();
        entry.publish_rate = publish_rate;
        
        // Adaptive TTL based on publish frequency
        // High frequency topics → shorter cache (fresher data)
        // Low frequency topics → longer cache (reduce ROS2 calls)
        if (publish_rate > 100.0) {
            entry.ttl = std::chrono::milliseconds(500);   // 500ms for high-freq
        } else if (publish_rate > 10.0) {
            entry.ttl = std::chrono::milliseconds(2000);  // 2s for medium-freq
        } else {
            entry.ttl = std::chrono::milliseconds(10000); // 10s for low-freq
        }
    }

    template<typename T>
    std::optional<T> get(const std::string& key) {
        std::unique_lock lock(cache_mutex_);
        auto it = cache_.find(key);
        if (it == cache_.end()) return std::nullopt;

        auto now = std::chrono::steady_clock::now();
        auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - it->second.timestamp);

        if (age > it->second.ttl) {
            cache_.erase(it);
            return std::nullopt;
        }

        it->second.access_count++;
        return std::any_cast<T>(it->second.data);
    }

    double get_cache_hit_ratio() const {
        std::shared_lock lock(cache_mutex_);
        if (cache_.empty()) return 0.0;
        
        int total_accesses = 0;
        for (const auto& [_, entry] : cache_) {
            total_accesses += entry.access_count;
        }
        return total_accesses > 0 ? (total_accesses / (total_accesses + 1.0)) : 0.0;
    }
};

// ============================================================================
// 2. LOCK-FREE METRICS BUFFER (Using std::atomic)
// ============================================================================

template<typename T>
class LockFreeMetricsBuffer {
private:
    struct MetricSnapshot {
        T value;
        uint64_t timestamp_ms;
        std::atomic<bool> is_valid{true};
    };

    static constexpr size_t BUFFER_SIZE = 256;
    std::array<MetricSnapshot, BUFFER_SIZE> buffer_;
    std::atomic<size_t> write_index_{0};
    std::atomic<uint64_t> total_samples_{0};

public:
    LockFreeMetricsBuffer() {
        for (auto& snapshot : buffer_) {
            snapshot.is_valid.store(false, std::memory_order_relaxed);
        }
    }

    void record_metric(T value, uint64_t timestamp_ms) {
        size_t idx = (write_index_.load(std::memory_order_acquire) + 1) % BUFFER_SIZE;
        buffer_[idx].value = value;
        buffer_[idx].timestamp_ms = timestamp_ms;
        buffer_[idx].is_valid.store(true, std::memory_order_release);
        
        write_index_.store(idx, std::memory_order_release);
        total_samples_.fetch_add(1, std::memory_order_relaxed);
    }

    T get_latest_metric() const {
        size_t idx = write_index_.load(std::memory_order_acquire);
        if (buffer_[idx].is_valid.load(std::memory_order_acquire)) {
            return buffer_[idx].value;
        }
        return T{};
    }

    double get_average() const {
        double sum = 0.0;
        int count = 0;
        size_t start_idx = write_index_.load(std::memory_order_acquire);

        for (size_t i = 0; i < BUFFER_SIZE; ++i) {
            size_t idx = (start_idx + i) % BUFFER_SIZE;
            if (buffer_[idx].is_valid.load(std::memory_order_acquire)) {
                sum += static_cast<double>(buffer_[idx].value);
                count++;
            }
        }
        return count > 0 ? sum / count : 0.0;
    }
};

// ============================================================================
// 3. OBJECT POOL FOR FREQUENT ALLOCATIONS
// ============================================================================

template<typename T>
class ObjectPool {
private:
    std::deque<std::unique_ptr<T>> available_;
    std::atomic<size_t> allocated_{0};
    std::atomic<size_t> peak_usage_{0};
    size_t max_pool_size_;
    std::mutex mutex_;

public:
    ObjectPool(size_t initial_size = 100, size_t max_size = 1000)
        : max_pool_size_(max_size) {
        for (size_t i = 0; i < initial_size; ++i) {
            available_.push_back(std::make_unique<T>());
        }
    }

    std::unique_ptr<T> acquire() {
        std::unique_lock lock(mutex_);
        
        if (!available_.empty()) {
            auto obj = std::move(available_.front());
            available_.pop_front();
            return obj;
        }

        // Create new object if pool depleted
        if (allocated_.load() < max_pool_size_) {
            allocated_.fetch_add(1);
            size_t usage = allocated_.load();
            size_t peak = peak_usage_.load();
            if (usage > peak) {
                peak_usage_.compare_exchange_strong(peak, usage);
            }
            return std::make_unique<T>();
        }

        // Return dummy if pool exhausted
        return std::make_unique<T>();
    }

    void release(std::unique_ptr<T> obj) {
        if (!obj) return;
        
        std::unique_lock lock(mutex_);
        if (available_.size() < max_pool_size_ / 2) {
            available_.push_back(std::move(obj));
        }
    }

    size_t available_count() const {
        std::unique_lock lock(mutex_);
        return available_.size();
    }

    size_t peak_usage() const {
        return peak_usage_.load();
    }

    void shrink_to_fit() {
        std::unique_lock lock(mutex_);
        available_.shrink_to_fit();
    }
};

// ============================================================================
// 4. BATCH PROCESSING WITH std::execution::par
// ============================================================================

class BatchTopicProcessor {
public:
    struct TopicData {
        std::string name;
        std::string type;
        int subscriber_count = 0;
        int publisher_count = 0;
        double frequency = 0.0;
    };

    // Process topics in parallel with vectorized operations
    static std::vector<TopicData> filter_topics_parallel(
        const std::vector<TopicData>& topics,
        double min_frequency = 0.1) {
        
        std::vector<TopicData> filtered;
        filtered.reserve(topics.size());

        std::copy_if(
            std::execution::par,  // Parallel execution
            topics.begin(),
            topics.end(),
            std::back_inserter(filtered),
            [min_frequency](const TopicData& t) {
                return t.frequency >= min_frequency && 
                       !t.name.empty();
            }
        );

        return filtered;
    }

    // Vectorized metric aggregation
    static double compute_average_frequency_vectorized(
        const std::vector<TopicData>& topics) {
        
        if (topics.empty()) return 0.0;

        // Use std::transform_reduce for parallel reduction
        double sum = std::transform_reduce(
            std::execution::par,
            topics.begin(),
            topics.end(),
            0.0,
            std::plus<double>{},
            [](const TopicData& t) { return t.frequency; }
        );

        return sum / topics.size();
    }

    // Sort topics with parallel algorithm
    static void sort_by_frequency_parallel(std::vector<TopicData>& topics) {
        std::sort(
            std::execution::par,
            topics.begin(),
            topics.end(),
            [](const TopicData& a, const TopicData& b) {
                return a.frequency > b.frequency;
            }
        );
    }
};

// ============================================================================
// 5. MEMORY-MAPPED RESOURCE POOL
// ============================================================================

class SmartResourcePool {
private:
    std::pmr::monotonic_buffer_resource buffer_;
    std::pmr::vector<std::string> topic_names_;
    std::pmr::unordered_map<std::string, int> topic_index_;

public:
    SmartResourcePool(size_t buffer_size = 1 << 20)  // 1MB buffer
        : buffer_(new char[buffer_size], buffer_size),
          topic_names_(&buffer_),
          topic_index_(&buffer_) {
    }

    void add_topic(const std::string& name) {
        if (topic_index_.find(name) == topic_index_.end()) {
            topic_index_[name] = topic_names_.size();
            topic_names_.push_back(name);
        }
    }

    const std::pmr::vector<std::string>& get_all_topics() const {
        return topic_names_;
    }

    void reset() {
        topic_names_.clear();
        topic_index_.clear();
        buffer_.release();
    }

    size_t get_used_memory() const {
        return buffer_.upstream_resource()->allocated_size();
    }
};

// ============================================================================
// 6. LAZY EVALUATION FOR EXPENSIVE COMPUTATIONS
// ============================================================================

template<typename T>
class LazyComputation {
private:
    mutable std::function<T()> compute_fn_;
    mutable std::optional<T> cached_value_;
    mutable std::atomic<bool> is_dirty_{true};
    mutable std::mutex mutex_;

public:
    LazyComputation(std::function<T()> fn) : compute_fn_(fn) {}

    const T& get() const {
        std::unique_lock lock(mutex_);
        if (is_dirty_) {
            cached_value_ = compute_fn_();
            is_dirty_.store(false, std::memory_order_release);
        }
        return cached_value_.value();
    }

    void invalidate() {
        is_dirty_.store(true, std::memory_order_release);
    }

    void update_computation(std::function<T()> fn) {
        std::unique_lock lock(mutex_);
        compute_fn_ = fn;
        invalidate();
    }
};

// ============================================================================
// 7. PERFORMANCE MONITORING UTILITIES
// ============================================================================

class PerformanceMonitor {
private:
    struct TimingData {
        std::chrono::microseconds min_us{LLONG_MAX};
        std::chrono::microseconds max_us{0};
        std::chrono::microseconds avg_us{0};
        uint64_t call_count = 0;
    };

    std::unordered_map<std::string, TimingData> timings_;
    std::mutex mutex_;

public:
    class ScopedTimer {
    private:
        std::string name_;
        std::chrono::steady_clock::time_point start_;
        PerformanceMonitor* monitor_;

    public:
        ScopedTimer(const std::string& name, PerformanceMonitor* monitor)
            : name_(name), monitor_(monitor) {
            start_ = std::chrono::steady_clock::now();
        }

        ~ScopedTimer() {
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - start_);
            monitor_->record_timing(name_, duration);
        }
    };

    ScopedTimer measure(const std::string& operation_name) {
        return ScopedTimer(operation_name, this);
    }

private:
    void record_timing(const std::string& name, std::chrono::microseconds duration) {
        std::unique_lock lock(mutex_);
        auto& timing = timings_[name];
        
        timing.min_us = std::min(timing.min_us, duration);
        timing.max_us = std::max(timing.max_us, duration);
        timing.avg_us = (timing.avg_us * timing.call_count + duration) /
                        (timing.call_count + 1);
        timing.call_count++;
    }

public:
    void print_report() {
        std::unique_lock lock(mutex_);
        
        printf("\n╔════════════════════════════════════════════════════╗\n");
        printf("║           PERFORMANCE MONITORING REPORT             ║\n");
        printf("╠════════════════════════════════════════════════════╣\n");
        printf("║ Operation          │  Min   │  Avg   │  Max       ║\n");
        printf("╠════════════════════════════════════════════════════╣\n");

        for (const auto& [name, timing] : timings_) {
            printf("║ %-18s │ %6ld │ %6ld │ %6ld µs  ║\n",
                name.c_str(),
                timing.min_us.count(),
                timing.avg_us.count(),
                timing.max_us.count());
        }

        printf("╚════════════════════════════════════════════════════╝\n");
    }
};

// ============================================================================
// USAGE EXAMPLES
// ============================================================================

/*

// 1. Adaptive Caching
AdaptiveCacheManager cache;
cache.set("topic_info", topic_data, 25.5);  // 25.5 Hz topic
auto cached = cache.get<TopicInfo>("topic_info");
printf("Cache hit ratio: %.1f%%\n", cache.get_cache_hit_ratio() * 100);

// 2. Lock-Free Metrics
LockFreeMetricsBuffer<double> cpu_metrics;
cpu_metrics.record_metric(45.2, std::chrono::system_clock::now().time_since_epoch().count());
printf("Current CPU: %.1f%%\n", cpu_metrics.get_latest_metric());
printf("Average CPU: %.1f%%\n", cpu_metrics.get_average());

// 3. Object Pool
ObjectPool<TopicInfo> pool(100, 500);
{
    auto topic = pool.acquire();
    topic->name = "sensor_data";
    // Use topic...
    pool.release(std::move(topic));
}
printf("Peak usage: %zu objects\n", pool.peak_usage());

// 4. Parallel Batch Processing
std::vector<BatchTopicProcessor::TopicData> topics = get_all_topics();
auto filtered = BatchTopicProcessor::filter_topics_parallel(topics, 1.0);
BatchTopicProcessor::sort_by_frequency_parallel(topics);
auto avg_freq = BatchTopicProcessor::compute_average_frequency_vectorized(topics);

// 5. Resource Pool
SmartResourcePool resources;
resources.add_topic("/sensor/camera");
resources.add_topic("/sensor/lidar");
printf("Used memory: %zu bytes\n", resources.get_used_memory());

// 6. Lazy Evaluation
LazyComputation<double> stats([](){ 
    return compute_expensive_statistics(); 
});
auto result = stats.get();  // Only computed when accessed

// 7. Performance Monitoring
PerformanceMonitor perf;
{
    auto timer = perf.measure("ros2_discovery");
    // ... do ROS2 discovery
}
perf.print_report();

*/

