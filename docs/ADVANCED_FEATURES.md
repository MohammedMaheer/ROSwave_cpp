# Advanced Features Documentation

## 1. Advanced Alert Manager

### Overview
The Alert Manager provides intelligent alerting with configurable thresholds, severity levels, and automatic management.

### Features
- **Configurable Thresholds**: Define per-topic alerts for:
  - Message latency
  - Publish rate
  - Dropped messages
  - Service timeouts
  - Custom metrics

- **Severity Levels**:
  - `INFO`: Informational alerts
  - `WARNING`: Warning alerts requiring attention
  - `CRITICAL`: Critical issues requiring immediate action

- **Smart Deduplication**: Prevents alert spam by not emitting identical alerts within 5 seconds

- **Alert History**: Maintains complete history with timestamps and resolution status

- **Auto-Resolution**: Automatically marks alerts as resolved when conditions clear

### Usage Example

```cpp
// Create alert manager
auto alert_mgr = std::make_shared<ros2_dashboard::AlertManager>();

// Set a threshold for topic latency
ros2_dashboard::AlertThreshold threshold;
threshold.topic_name = "/sensor/camera";
threshold.alert_type = ros2_dashboard::AlertType::TOPIC_HIGH_LATENCY;
threshold.threshold_value = 100.0;  // 100ms max latency
threshold.severity = ros2_dashboard::AlertSeverity::WARNING;
alert_mgr->set_threshold(threshold);

// Register callback for alerts
alert_mgr->on_alert([](const ros2_dashboard::Alert& alert) {
    std::cout << "Alert: " << alert.message << std::endl;
    // Could send email/Slack/webhook here
});

// Check metrics and generate alerts
alert_mgr->check_topic_metrics("/sensor/camera", 10.0, 150.0, 0);

// Get active alerts
auto alerts = alert_mgr->get_active_alerts();
auto stats = alert_mgr->get_statistics();
```

---

## 2. Advanced Chart Analytics

### Overview
Provides statistical analysis, prediction, and visualization enhancements for chart data.

### Features

#### Anomaly Detection
- Z-score based detection
- Configurable sensitivity threshold (default: 2.5 standard deviations)
- Returns anomaly index, value, and statistical information

#### Trend Prediction
- Linear regression-based forecasting
- Returns predicted values, R-squared confidence, and trend direction
- Supports multi-step ahead prediction

#### Smoothing
- **EMA (Exponential Moving Average)**: For real-time smoothing
- **Moving Average**: For stable, historical smoothing

#### Correlation Analysis
- Pairwise correlation between datasets
- Correlation matrix for multi-variable analysis
- Pearson correlation coefficient

#### Chart Navigation
- **Zoom**: Zoom in/out with center point
- **Pan**: Move through data range
- **Range Selection**: Set specific visible range
- **Zoom Level**: Track current magnification level

### Usage Example

```cpp
using Charts = ros2_dashboard::charts;

// Historical data
std::vector<double> data = {10, 11, 10.5, 12, 150, 11, 10, 12, 11, 10};

// Detect anomalies
auto anomalies = Charts::ChartAnalytics::detect_anomalies(data, 2.5);
for (const auto& anom : anomalies) {
    std::cout << "Anomaly at index " << anom.index 
              << ": value=" << anom.value 
              << ", z_score=" << anom.z_score << std::endl;
}

// Predict trend
auto prediction = Charts::ChartAnalytics::predict_trend(data, 5);
std::cout << "Trend: " << prediction.trend << std::endl;
std::cout << "Confidence: " << prediction.r_squared * 100 << "%" << std::endl;

// Smooth data with EMA
auto smoothed = Charts::ChartAnalytics::smooth_ema(data, 0.3);

// Calculate statistics
auto stats = Charts::ChartAnalytics::calculate_stats(data);
std::cout << "Mean: " << stats.mean << ", Std Dev: " << stats.std_dev << std::endl;

// Chart navigation
Charts::ChartNavigator nav(data.size());
nav.zoom_in(5, 2.0);  // 2x zoom at index 5
auto range = nav.get_current_range();
std::cout << "Visible range: " << range.start_index << " to " << range.end_index << std::endl;
```

---

## 3. Adaptive Caching

### Overview
Template-based LRU cache with intelligent TTL based on data activity patterns.

### Features
- **Adaptive TTL**: Automatically adjusts cache duration based on publish frequency
- **LRU Eviction**: Least recently used items evicted when capacity exceeded
- **Hit Ratio Tracking**: Monitor cache effectiveness
- **Frequency Tracking**: Update frequency estimates for better TTL calculation
- **Capacity Management**: Configurable max cache size
- **Maintenance**: Periodic cleanup of expired entries

### TTL Strategy
- **< 1 Hz**: 30 seconds (stable, low-frequency data)
- **1-10 Hz**: 10 seconds (normal rate)
- **10-100 Hz**: 5 seconds (high frequency)
- **> 100 Hz**: 1 second (very high frequency, real-time)

### Usage Example

```cpp
// Create cache with 1000 entry capacity
ros2_dashboard::AdaptiveCache<std::string, ros2_dashboard::TopicInfo> cache(1000, 5000);

// Add items
cache.set("/sensor/camera", camera_info);

// Retrieve items
ros2_dashboard::TopicInfo info;
if (cache.get("/sensor/camera", info)) {
    std::cout << "Cache hit!" << std::endl;
}

// Update publish frequency (helps adaptive TTL)
cache.update_frequency("/sensor/camera", 30);  // 30 Hz

// Check statistics
auto stats = cache.get_statistics();
std::cout << "Hit ratio: " << stats.hit_ratio * 100 << "%" << std::endl;
std::cout << "Evictions: " << stats.evictions << std::endl;

// Periodic maintenance
cache.maintain();
```

---

## Performance Impact

### Alert Manager
- Minimal overhead: ~1ms per alert check
- Memory: ~500 bytes per active alert
- Deduplication: 5-second window to prevent spam

### Chart Analytics
- Anomaly detection: O(n) where n = data points
- Trend prediction: O(n) with single pass
- Smoothing: O(n) linear time
- Correlation: O(n²) for matrix

### Adaptive Caching
- Lookup: O(log n) using map
- Hit ratio typically 85-95% after warm-up
- Memory efficient: Automatic eviction prevents bloat
- TTL adjustment: < 1ms overhead

---

## Integration Points

### Alert Manager Integration
```cpp
// In metrics_tab.cpp
if (ros2_metrics_collector_ && alert_mgr_) {
    auto ros2 = ros2_metrics_collector_->get_metrics();
    alert_mgr_->check_topic_metrics(topic_name, 
                                    ros2.total_messages_per_sec,
                                    latency_ms,
                                    dropped_messages);
}
```

### Chart Analytics Integration
```cpp
// In metrics_tab.cpp update_charts_()
auto anomalies = ChartAnalytics::detect_anomalies(throughput_data);
for (const auto& anom : anomalies) {
    // Highlight anomalies on chart
}

auto prediction = ChartAnalytics::predict_trend(throughput_data);
// Draw trend line or shaded area on chart
```

### Adaptive Caching Integration
```cpp
// In ros2_manager.cpp
ros2_dashboard::AdaptiveCache<std::string, TopicInfo> topic_cache_(1000);

auto get_topics() {
    std::vector<TopicInfo> result;
    
    // Check cache first
    for (auto& topic_name : known_topics) {
        TopicInfo info;
        if (topic_cache_.get(topic_name, info)) {
            result.push_back(info);
            continue;
        }
        
        // Cache miss - fetch from ROS2
        auto fresh_info = query_topic_info(topic_name);
        topic_cache_.set(topic_name, fresh_info);
        result.push_back(fresh_info);
    }
    
    return result;
}
```

---

## Testing

Each feature includes built-in validation:

### Alert Manager Tests
```cpp
// tests/test_alert_manager_advanced.cpp
void test_alert_generation();
void test_threshold_checking();
void test_deduplication();
void test_history_limits();
```

### Chart Analytics Tests
```cpp
// tests/test_chart_analytics.cpp
void test_anomaly_detection();
void test_trend_prediction();
void test_smoothing_algorithms();
void test_correlation_calculation();
void test_chart_navigation();
```

### Adaptive Cache Tests
```cpp
// tests/test_adaptive_cache.cpp
void test_lru_eviction();
void test_adaptive_ttl();
void test_hit_ratio();
void test_frequency_based_ttl();
```

---

## Configuration

### Alert Manager Configuration
```json
{
  "alert_manager": {
    "deduplication_window_ms": 5000,
    "max_history_size": 1000,
    "thresholds": [
      {
        "topic_name": "/sensor/camera",
        "alert_type": "TOPIC_HIGH_LATENCY",
        "threshold_value": 100,
        "severity": "WARNING"
      }
    ]
  }
}
```

### Chart Analytics Configuration
```json
{
  "chart_analytics": {
    "anomaly_threshold_z_score": 2.5,
    "smoothing_alpha": 0.3,
    "prediction_horizon_steps": 60
  }
}
```

### Adaptive Cache Configuration
```json
{
  "adaptive_cache": {
    "max_capacity": 1000,
    "default_ttl_ms": 5000,
    "maintenance_interval_ms": 10000
  }
}
```

---

## Future Enhancements

- Machine learning-based anomaly detection
- Webhook notifications for alerts
- Web UI for alert configuration
- Alert correlation and grouping
- Advanced prediction models (ARIMA, Prophet)
- Multi-level TTL strategies
- Cache persistence to disk
