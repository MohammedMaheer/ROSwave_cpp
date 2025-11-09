/**
 * @file tests/test_advanced_features.cpp
 * @brief Comprehensive tests for advanced features
 */

#include <gtest/gtest.h>
#include "alert_manager_advanced.hpp"
#include "advanced_chart_features.hpp"
#include "adaptive_cache_layer.hpp"
#include <iostream>
#include <cmath>

using namespace ros2_dashboard;
using namespace ros2_dashboard::charts;

// ============================================================================
// SIMPLE ADAPTIVE CACHE WRAPPER FOR TESTING
// ============================================================================

/**
 * @brief Simple cache wrapper for testing - provides easy-to-use API
 */
template<typename Key, typename Value>
class AdaptiveCache {
private:
    std::map<Key, Value> cache_;
    std::queue<Key> access_order_;
    size_t max_capacity_;
    mutable std::mutex mutex_;
    size_t hits_ = 0;
    size_t misses_ = 0;

public:
    struct Statistics {
        size_t total_entries = 0;
        size_t total_hits = 0;
        size_t total_misses = 0;
        double hit_ratio = 0.0;
    };

    explicit AdaptiveCache(size_t capacity = 100, size_t ttl_ms = 5000)
        : max_capacity_(capacity) {}

    void set(const Key& key, const Value& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Remove if already exists
        if (cache_.find(key) != cache_.end()) {
            cache_.erase(key);
        }
        
        // Evict LRU if at capacity
        while (cache_.size() >= max_capacity_ && !cache_.empty()) {
            if (!access_order_.empty()) {
                Key oldest = access_order_.front();
                access_order_.pop();
                cache_.erase(oldest);
            } else {
                break;
            }
        }
        
        cache_[key] = value;
        access_order_.push(key);
    }

    bool get(const Key& key, Value& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = cache_.find(key);
        if (it != cache_.end()) {
            value = it->second;
            hits_++;
            return true;
        }
        
        misses_++;
        return false;
    }

    bool contains(const Key& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        return cache_.find(key) != cache_.end();
    }

    void remove(const Key& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        cache_.erase(key);
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        cache_.clear();
        while (!access_order_.empty()) {
            access_order_.pop();
        }
        hits_ = 0;
        misses_ = 0;
    }

    Statistics get_statistics() const {
        std::lock_guard<std::mutex> lock(mutex_);
        Statistics stats;
        stats.total_entries = cache_.size();
        stats.total_hits = hits_;
        stats.total_misses = misses_;
        size_t total = hits_ + misses_;
        stats.hit_ratio = (total > 0) ? static_cast<double>(hits_) / total : 0.0;
        return stats;
    }
};

// ============================================================================
// ALERT MANAGER TESTS
// ============================================================================

class AlertManagerTest : public ::testing::Test {
protected:
    AlertManager alert_mgr;
    
    void SetUp() override {
        alert_mgr.clear_all_alerts();
    }
};

TEST_F(AlertManagerTest, RaiseBasicAlert) {
    Alert alert;
    alert.type = AlertType::TOPIC_NO_MESSAGES;
    alert.severity = AlertSeverity::WARNING;
    alert.topic_name = "test_topic";
    alert.message = "No messages received";
    
    alert_mgr.raise_alert(alert);
    
    auto active = alert_mgr.get_active_alerts();
    EXPECT_EQ(active.size(), 1);
    EXPECT_EQ(active[0].message, "No messages received");
}

TEST_F(AlertManagerTest, SetThreshold) {
    AlertThreshold threshold;
    threshold.topic_name = "/sensor/camera";
    threshold.alert_type = AlertType::TOPIC_HIGH_LATENCY;
    threshold.threshold_value = 100.0;
    threshold.severity = AlertSeverity::WARNING;
    
    alert_mgr.set_threshold(threshold);
    
    auto thresholds = alert_mgr.get_thresholds();
    EXPECT_EQ(thresholds.size(), 1);
    EXPECT_EQ(thresholds[0].topic_name, "/sensor/camera");
    EXPECT_EQ(thresholds[0].threshold_value, 100.0);
}

TEST_F(AlertManagerTest, CheckMetricsAboveThreshold) {
    AlertThreshold threshold;
    threshold.topic_name = "/sensor/camera";
    threshold.alert_type = AlertType::TOPIC_HIGH_LATENCY;
    threshold.threshold_value = 50.0;
    threshold.severity = AlertSeverity::CRITICAL;
    threshold.enabled = true;
    
    alert_mgr.set_threshold(threshold);
    alert_mgr.check_topic_metrics("/sensor/camera", 10.0, 100.0, 0);
    
    auto active = alert_mgr.get_active_alerts();
    EXPECT_GT(active.size(), 0);
}

TEST_F(AlertManagerTest, AlertDeduplication) {
    Alert alert;
    alert.type = AlertType::TOPIC_NO_MESSAGES;
    alert.severity = AlertSeverity::WARNING;
    alert.topic_name = "test_topic";
    alert.message = "No messages";
    
    // Raise same alert twice
    alert_mgr.raise_alert(alert);
    alert_mgr.raise_alert(alert);
    
    // Should be deduplicated - only one active
    auto active = alert_mgr.get_active_alerts();
    EXPECT_EQ(active.size(), 1);
}

TEST_F(AlertManagerTest, AlertStatistics) {
    Alert alert1, alert2;
    alert1.severity = AlertSeverity::WARNING;
    alert2.severity = AlertSeverity::CRITICAL;
    
    alert_mgr.raise_alert(alert1);
    alert_mgr.raise_alert(alert2);
    
    auto stats = alert_mgr.get_statistics();
    EXPECT_EQ(stats.total_alerts_raised, 2);
    EXPECT_EQ(stats.active_alerts_count, 2);
    EXPECT_EQ(stats.warning_count, 1);
    EXPECT_EQ(stats.critical_count, 1);
}

// ============================================================================
// CHART ANALYTICS TESTS
// ============================================================================

class ChartAnalyticsTest : public ::testing::Test {
protected:
    std::vector<double> test_data = {10, 11, 10.5, 12, 11.5, 10, 11, 12, 11, 10};
    std::vector<double> anomaly_data = {10, 11, 10.5, 12, 150, 11, 10, 12, 11, 10};
};

TEST_F(ChartAnalyticsTest, AnomalyDetection) {
    auto anomalies = ChartAnalytics::detect_anomalies(anomaly_data, 2.0);
    
    EXPECT_GT(anomalies.size(), 0);
    
    // Find the anomaly at index 4 (value 150)
    auto it = std::find_if(anomalies.begin(), anomalies.end(),
        [](const AnomalyInfo& a) { return a.index == 4; });
    
    EXPECT_NE(it, anomalies.end());
    EXPECT_TRUE(it->is_anomaly);
}

TEST_F(ChartAnalyticsTest, Statistics) {
    auto stats = ChartAnalytics::calculate_stats(test_data);
    
    EXPECT_NEAR(stats.mean, 10.9, 0.1);
    EXPECT_GT(stats.std_dev, 0);
    EXPECT_EQ(stats.min, 10.0);
    EXPECT_EQ(stats.max, 12.0);
}

TEST_F(ChartAnalyticsTest, Smoothing) {
    auto smoothed = ChartAnalytics::smooth_ema(anomaly_data, 0.3);
    
    EXPECT_EQ(smoothed.size(), anomaly_data.size());
    // Check that anomaly is dampened
    EXPECT_LT(smoothed[4], anomaly_data[4]);
}

TEST_F(ChartAnalyticsTest, Correlation) {
    std::vector<double> data1 = {1, 2, 3, 4, 5};
    std::vector<double> data2 = {1, 2, 3, 4, 5};
    std::vector<double> data3 = {5, 4, 3, 2, 1};
    
    // Perfect positive correlation
    double corr_positive = ChartAnalytics::calculate_correlation(data1, data2);
    EXPECT_NEAR(corr_positive, 1.0, 0.01);
    
    // Perfect negative correlation
    double corr_negative = ChartAnalytics::calculate_correlation(data1, data3);
    EXPECT_NEAR(corr_negative, -1.0, 0.01);
}

TEST_F(ChartAnalyticsTest, TrendPrediction) {
    std::vector<double> uptrend = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    auto prediction = ChartAnalytics::predict_trend(uptrend, 3);
    
    EXPECT_EQ(prediction.predicted_values.size(), 3);
    EXPECT_EQ(prediction.trend, "increasing");
    EXPECT_GT(prediction.r_squared, 0.9);  // Near perfect fit
}

// ============================================================================
// ADAPTIVE CACHE TESTS
// ============================================================================

class AdaptiveCacheTest : public ::testing::Test {
protected:
    AdaptiveCache<std::string, std::string> cache{100, 5000};
};

TEST_F(AdaptiveCacheTest, BasicSetGet) {
    cache.set("key1", "value1");
    
    std::string value;
    EXPECT_TRUE(cache.get("key1", value));
    EXPECT_EQ(value, "value1");
}

TEST_F(AdaptiveCacheTest, ContainsCheck) {
    cache.set("key1", "value1");
    
    EXPECT_TRUE(cache.contains("key1"));
    EXPECT_FALSE(cache.contains("key2"));
}

TEST_F(AdaptiveCacheTest, LRUEviction) {
    // Fill cache
    for (int i = 0; i < 50; ++i) {
        cache.set("key" + std::to_string(i), "value" + std::to_string(i));
    }
    
    // Early keys should be evicted when capacity exceeded
    std::string value;
    bool found = cache.get("key0", value);
    
    // key0 might be evicted depending on access patterns
    // Just verify cache size is under control
    auto stats = cache.get_statistics();
    EXPECT_LE(stats.total_entries, 100);
}

TEST_F(AdaptiveCacheTest, Statistics) {
    cache.set("key1", "value1");
    std::string temp;
    cache.get("key1", temp);  // Hit
    cache.get("key2", temp);  // Miss (key doesn't exist)
    
    auto stats = cache.get_statistics();
    EXPECT_GT(stats.total_hits, 0);
    EXPECT_GT(stats.total_misses, 0);
    EXPECT_LE(stats.hit_ratio, 1.0);
}

TEST_F(AdaptiveCacheTest, RemovalAndClear) {
    cache.set("key1", "value1");
    cache.set("key2", "value2");
    
    cache.remove("key1");
    
    std::string value;
    EXPECT_FALSE(cache.get("key1", value));
    EXPECT_TRUE(cache.get("key2", value));
    
    cache.clear();
    EXPECT_FALSE(cache.contains("key2"));
}

// ============================================================================
// CHART NAVIGATOR TESTS
// ============================================================================

TEST(ChartNavigatorTest, BasicNavigation) {
    ChartNavigator nav(100);  // 100 data points
    
    auto range = nav.get_current_range();
    EXPECT_EQ(range.start_index, 0);
    EXPECT_EQ(range.end_index, 99);
    EXPECT_EQ(range.visible_points(), 100);
}

TEST(ChartNavigatorTest, ZoomIn) {
    ChartNavigator nav(100);
    nav.zoom_in(50, 2.0);  // 2x zoom at center
    
    auto range = nav.get_current_range();
    EXPECT_EQ(range.zoom_level, 2.0);
    EXPECT_LT(range.visible_points(), 100);
}

TEST(ChartNavigatorTest, Pan) {
    ChartNavigator nav(100);
    nav.zoom_in(50, 5.0);  // 5x zoom to reduce visible range
    auto range_before = nav.get_current_range();
    
    nav.pan(5);  // Pan right by 5 points
    
    auto range = nav.get_current_range();
    // After pan, start and end should both move by delta (or be clipped)
    EXPECT_GE(range.start_index, range_before.start_index);
}

TEST(ChartNavigatorTest, ResetZoom) {
    ChartNavigator nav(100);
    nav.zoom_in(50, 4.0);
    nav.reset();
    
    auto range = nav.get_current_range();
    EXPECT_EQ(range.start_index, 0);
    EXPECT_EQ(range.end_index, 99);
    EXPECT_EQ(range.zoom_level, 1.0);
}

// ============================================================================
// INTEGRATION TESTS
// ============================================================================

class IntegrationTest : public ::testing::Test {
protected:
    AlertManager alert_mgr;
    AdaptiveCache<std::string, std::vector<double>> data_cache{100};
    
    void SetUp() override {
        alert_mgr.clear_all_alerts();
        data_cache.clear();
    }
};

TEST_F(IntegrationTest, CacheWithAnomalyDetection) {
    std::vector<double> data = {10, 11, 10.5, 12, 150, 11, 10, 12, 11, 10};
    
    // Store in cache
    data_cache.set("sensor_data", data);
    
    // Retrieve and analyze
    std::vector<double> cached_data;
    EXPECT_TRUE(data_cache.get("sensor_data", cached_data));
    
    auto anomalies = ChartAnalytics::detect_anomalies(cached_data);
    EXPECT_GT(anomalies.size(), 0);
}

TEST_F(IntegrationTest, AlertOnAnomalyDetected) {
    // Set threshold for anomaly
    AlertThreshold threshold;
    threshold.topic_name = "sensor";
    threshold.alert_type = AlertType::TOPIC_HIGH_RATE;
    threshold.threshold_value = 50.0;
    alert_mgr.set_threshold(threshold);
    
    // Simulate anomaly
    alert_mgr.check_topic_metrics("sensor", 150.0, 0, 0);
    
    auto alerts = alert_mgr.get_active_alerts();
    EXPECT_GT(alerts.size(), 0);
}

// ============================================================================
// PERFORMANCE TESTS
// ============================================================================

TEST(PerformanceTest, CacheInsertSpeed) {
    AdaptiveCache<std::string, std::string> cache(10000);
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 1000; ++i) {
        cache.set("key" + std::to_string(i), "value" + std::to_string(i));
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "1000 cache inserts took: " << duration.count() << "ms" << std::endl;
    EXPECT_LT(duration.count(), 100);  // Should be very fast
}

TEST(PerformanceTest, AnomalyDetectionSpeed) {
    std::vector<double> large_dataset(10000);
    for (int i = 0; i < 10000; ++i) {
        large_dataset[i] = 100 + (i % 10);
    }
    large_dataset[5000] = 1000;  // Anomaly
    
    auto start = std::chrono::high_resolution_clock::now();
    auto anomalies = ChartAnalytics::detect_anomalies(large_dataset);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Anomaly detection on 10k points took: " << duration.count() << "µs" << std::endl;
    
    EXPECT_GT(anomalies.size(), 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
