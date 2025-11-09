# Advanced Analytics Dashboard - Developer Guide

## Overview

The Advanced Analytics Dashboard provides comprehensive time-series analysis, predictive capabilities, and statistical reporting for ROS2 system metrics. It enables:

- **Trend Analysis**: Identify upward/downward trends with slope calculation
- **Anomaly Detection**: Z-score based detection of unusual metric values
- **Correlation Analysis**: Discover relationships between different metrics
- **Predictive Forecasting**: Simple linear regression for trend prediction
- **Statistical Reports**: Comprehensive summaries in JSON, CSV, and Markdown formats
- **Performance Analysis**: Identify bottlenecks and optimization opportunities
- **Risk Assessment**: Predictive risk scoring based on current and projected metrics

## Architecture

```
AnalyticsDashboard
├── Trend Analysis (Linear Regression)
│   ├── CPU percent trends
│   ├── Memory usage trends
│   ├── Disk I/O trends
│   └── Network bandwidth trends
│
├── Anomaly Detection (Z-Score)
│   ├── Per-metric anomaly detection
│   ├── Severity assessment
│   └── Historical anomaly reporting
│
├── Correlation Analysis
│   ├── Pairwise metric correlations
│   ├── Correlation matrix generation
│   └── Relationship classification
│
├── Predictive Analytics
│   ├── Trend prediction (linear)
│   ├── Prediction accuracy scoring
│   └── Actionable recommendations
│
├── Report Generation
│   ├── JSON export (machine-readable)
│   ├── CSV export (spreadsheet-friendly)
│   ├── Markdown export (human-readable)
│   └── Executive summaries
│
└── Risk Assessment
    ├── Risk scoring (0-100)
    ├── Risk factors identification
    ├── Recommendations
    └── Baseline comparisons
```

## Core Components

### 1. TrendAnalysis

Analyzes time-series data to identify trends and patterns.

```cpp
struct TrendAnalysis {
    std::string metric_name;
    std::vector<uint64_t> timestamps;
    std::vector<double> values;
    
    double slope;              // Rate of change
    double trend_strength;     // 0-1 (0=weak, 1=strong)
    std::string direction;     // "up", "down", "stable"
    
    bool has_periodicity;      // Cyclic pattern detected?
    int period_minutes;        // Cycle length if periodic
    
    double change_last_hour;   // Absolute change
    double percent_change_last_hour;  // Relative change %
};
```

**Calculation Method**: Linear regression (least squares)

```
slope = (n * Σ(x*y) - Σx * Σy) / (n * Σ(x²) - (Σx)²)
direction: slope > threshold → "up", slope < -threshold → "down", else → "stable"
```

### 2. AnomalyReport

Identifies unusual values in metrics using statistical methods.

```cpp
struct AnomalyReport {
    std::string metric_name;
    std::vector<charts::AnomalyInfo> anomalies;
    size_t total_points;
    double anomaly_rate;  // % of points marked as anomalies
    std::string severity; // "info", "warning", "critical"
};
```

**Detection Method**: Z-score with threshold 2.5

```
z_score = (value - mean) / std_dev
is_anomaly = |z_score| > 2.5
severity: anomaly_rate > 10% → "critical", > 5% → "warning", else → "info"
```

### 3. CorrelationPair

Reveals relationships between different metrics.

```cpp
struct CorrelationPair {
    std::string metric1;
    std::string metric2;
    double correlation;  // -1.0 to 1.0
    std::string relationship;  // e.g., "strong_positive"
};
```

**Correlation Classification**:
- `|r| > 0.7`: Strong correlation
- `|r| > 0.3`: Weak correlation
- `|r| ≤ 0.3`: No significant correlation

### 4. PredictionReport

Forecasts future metric behavior.

```cpp
struct PredictionReport {
    std::string metric_name;
    charts::PredictionInfo prediction;
    std::vector<double> actual_values;
    std::vector<double> predicted_values;
    double accuracy_percent;    // 75-100%
    std::string recommendation; // Action guidance
};
```

**Prediction Method**: Linear regression extrapolation

```
predicted_value[i] = slope * i + intercept
accuracy = 75% + (r_squared * 25%)  // Scale to 75-100%
```

### 5. StatisticalReport

Comprehensive metrics summary.

```cpp
struct StatisticalReport {
    uint64_t report_time_ms;
    std::string time_window;  // "last_5min", "last_hour", "last_day"
    
    std::map<std::string, Stats> metric_stats;
    std::vector<CorrelationPair> correlations;
    std::vector<AnomalyReport> anomalies;
    std::vector<PredictionReport> predictions;
    
    struct SummaryMetrics {
        double overall_health_percent;  // 0-100
        std::string status;
        int alert_count;
        int anomaly_count;
        std::string primary_concern;
    };
    SummaryMetrics summary;
};
```

### 6. RiskAssessment

Predictive risk evaluation.

```cpp
struct RiskAssessment {
    double risk_score;  // 0-100
    std::string level;  // "low", "medium", "high", "critical"
    std::vector<std::string> risk_factors;
    std::vector<std::string> recommendations;
};
```

**Risk Scoring**:
- Base: 30 points
- Upward trends (strength > 0.7): +20 points
- Warning anomalies: +15 points
- Critical anomalies: +30 points
- Maximum clamped at 100

## API Reference

### Initialization

```cpp
#include "analytics_dashboard.hpp"
#include "metrics_history_buffer.hpp"

// Get reference to existing history buffer
MetricsHistoryBuffer& history = main_window->metrics_tab_->metrics_history_;

// Create analytics dashboard
AnalyticsDashboard analytics(history);
```

### Trend Analysis

```cpp
// Analyze all metrics for trends
std::vector<TrendAnalysis> trends = analytics.analyze_trends();

for (const auto& trend : trends) {
    std::cout << trend.metric_name << ": " 
              << trend.direction << " (" 
              << trend.slope << " units/sample)" << std::endl;
    
    if (trend.has_periodicity) {
        std::cout << "  Periodic pattern: " << trend.period_minutes 
                  << " minute cycle" << std::endl;
    }
}
```

### Anomaly Detection

```cpp
// Detect anomalies in all metrics
auto anomalies = analytics.detect_anomalies();

for (const auto& report : anomalies) {
    std::cout << "Metric: " << report.metric_name << std::endl;
    std::cout << "  Anomalies: " << report.anomalies.size() 
              << " / " << report.total_points 
              << " (" << report.anomaly_rate << "%)" << std::endl;
    std::cout << "  Severity: " << report.severity << std::endl;
}
```

### Correlation Analysis

```cpp
// Analyze correlations (threshold = 0.5 minimum)
auto correlations = analytics.analyze_correlations(0.5);

for (const auto& pair : correlations) {
    std::cout << pair.metric1 << " ↔ " << pair.metric2 
              << ": " << pair.correlation 
              << " (" << pair.relationship << ")" << std::endl;
}

// Get correlation matrix for heatmap
auto matrix = analytics.get_correlation_matrix();
// Display as heatmap in UI
```

### Predictive Analytics

```cpp
// Predict next 5 minutes
auto predictions = analytics.predict_trends(5);

for (const auto& pred : predictions) {
    std::cout << "Metric: " << pred.metric_name << std::endl;
    std::cout << "  Trend: " << pred.prediction.trend << std::endl;
    std::cout << "  Slope: " << pred.prediction.slope << std::endl;
    std::cout << "  Accuracy: " << pred.accuracy_percent << "%" << std::endl;
    std::cout << "  Recommendation: " << pred.recommendation << std::endl;
}
```

### Report Generation

```cpp
// Generate comprehensive report
auto report = analytics.generate_statistical_report("last_hour");

// Export to JSON
nlohmann::json json_report = analytics.export_report_json(report);
std::cout << json_report.dump(2) << std::endl;

// Export to CSV
std::string csv_report = analytics.export_report_csv(report);
// Save to file or send to client

// Export to Markdown
std::string md_report = analytics.export_report_markdown(report);
// Display in UI or send as email
```

### Performance Analysis

```cpp
// Analyze performance characteristics
auto perf = analytics.analyze_performance();

std::cout << "CPU Response Times:" << std::endl;
std::cout << "  Min: " << perf.cpu_response.min_ms << "ms" << std::endl;
std::cout << "  Max: " << perf.cpu_response.max_ms << "ms" << std::endl;
std::cout << "  Avg: " << perf.cpu_response.avg_ms << "ms" << std::endl;
std::cout << "  P95: " << perf.cpu_response.p95_ms << "ms" << std::endl;
std::cout << "  P99: " << perf.cpu_response.p99_ms << "ms" << std::endl;

// Check identified bottlenecks
for (const auto& bottleneck : perf.identified_bottlenecks) {
    std::cout << "⚠️ Bottleneck: " << bottleneck << std::endl;
}

// Get optimization suggestions
for (const auto& suggestion : perf.optimization_suggestions) {
    std::cout << "💡 Suggestion: " << suggestion << std::endl;
}
```

### Risk Assessment

```cpp
// Assess risk for next 5 minutes
auto risk = analytics.assess_risk(5);

std::cout << "Risk Score: " << risk.risk_score << " / 100" << std::endl;
std::cout << "Risk Level: " << risk.level << std::endl;

for (const auto& factor : risk.risk_factors) {
    std::cout << "  Risk Factor: " << factor << std::endl;
}

for (const auto& rec : risk.recommendations) {
    std::cout << "  Recommendation: " << rec << std::endl;
}
```

### Baseline Comparison

```cpp
// Compare current metrics to historical baseline
auto comparisons = analytics.compare_to_baseline();

for (const auto& comp : comparisons) {
    double deviation = comp.deviation_percent;
    std::cout << comp.metric_name << ": "
              << "Current=" << comp.current_value
              << " Baseline=" << comp.baseline_avg
              << " Deviation=" << deviation << "%" << std::endl;
    
    if (comp.status != "normal") {
        std::cout << "  ⚠️ Status: " << comp.status << std::endl;
    }
}
```

### Executive Summary

```cpp
// Generate high-level summary
auto summary = analytics.generate_executive_summary();

std::cout << "Status: " << summary.overall_status << std::endl;
std::cout << "Uptime: " << summary.uptime_percent << "%" << std::endl;
std::cout << "Critical Events: " << summary.critical_events << std::endl;
std::cout << "Warning Events: " << summary.warning_events << std::endl;
std::cout << "\nKey Findings:" << std::endl;
std::cout << summary.key_findings << std::endl;

std::cout << "\nTop Issues:" << std::endl;
for (const auto& issue : summary.top_issues) {
    std::cout << "  - " << issue << std::endl;
}

std::cout << "\nRecommendations:" << std::endl;
for (const auto& rec : summary.recommendations) {
    std::cout << "  - " << rec << std::endl;
}
```

## Integration with REST API

The analytics dashboard can be exposed via REST API endpoints:

```cpp
// In src/server/api_routes.cpp

server.get("/api/analytics/trends", [&analytics](const HttpRequest& req) {
    auto trends = analytics.analyze_trends();
    
    nlohmann::json response = nlohmann::json::array();
    for (const auto& trend : trends) {
        response.push_back({
            {"metric", trend.metric_name},
            {"direction", trend.direction},
            {"slope", trend.slope},
            {"trend_strength", trend.trend_strength},
            {"change_last_hour", trend.change_last_hour},
            {"percent_change", trend.percent_change_last_hour}
        });
    }
    
    return HttpResponse::ok(response);
});

server.get("/api/analytics/anomalies", [&analytics](const HttpRequest& req) {
    auto anomalies = analytics.detect_anomalies();
    
    nlohmann::json response;
    for (const auto& report : anomalies) {
        response[report.metric_name] = {
            {"severity", report.severity},
            {"count", report.anomalies.size()},
            {"total_points", report.total_points},
            {"rate", report.anomaly_rate}
        };
    }
    
    return HttpResponse::ok(response);
});

server.get("/api/analytics/report", [&analytics](const HttpRequest& req) {
    std::string window = "last_hour";
    if (req.query_params.count("window")) {
        window = req.query_params.at("window");
    }
    
    auto report = analytics.generate_statistical_report(window);
    return HttpResponse::ok(analytics.export_report_json(report));
});

server.get("/api/analytics/risk", [&analytics](const HttpRequest& req) {
    int minutes = 5;
    if (req.query_params.count("minutes")) {
        minutes = std::stoi(req.query_params.at("minutes"));
    }
    
    auto risk = analytics.assess_risk(minutes);
    
    return HttpResponse::ok({
        {"score", risk.risk_score},
        {"level", risk.level},
        {"factors", nlohmann::json(risk.risk_factors)},
        {"recommendations", nlohmann::json(risk.recommendations)}
    });
});
```

## Performance Considerations

### Memory Usage

- **Base**: ~1-2 KB per metric
- **Per Data Point**: ~40 bytes per metric per timestamp
- **Example**: 600 points × 4 metrics × 40 bytes = ~96 KB

### Computation Time

- **Trend Analysis**: O(n) where n = data points
- **Anomaly Detection**: O(n) + O(1) for stats
- **Correlation**: O(n) per pair
- **Full Report Generation**: ~5-10ms for 600 points

### Optimization Tips

1. **Cache Results**: Store analysis results and invalidate on new data
2. **Downsampling**: For long history, downsample before analysis
3. **Parallel Processing**: Use std::execution for multiple metric analysis
4. **Incremental Updates**: Update statistics incrementally as new points arrive

Example caching:

```cpp
class CachedAnalytics : public AnalyticsDashboard {
private:
    std::map<std::string, std::pair<uint64_t, std::vector<TrendAnalysis>>> trend_cache_;
    std::chrono::milliseconds cache_ttl_{5000};  // 5 second TTL
    
public:
    std::vector<TrendAnalysis> analyze_trends_cached() {
        auto now = std::chrono::steady_clock::now();
        
        if (trend_cache_.count("trends")) {
            auto& [timestamp, cached] = trend_cache_["trends"];
            auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - 
                std::chrono::steady_clock::time_point(std::chrono::milliseconds(timestamp)));
            
            if (age < cache_ttl_) {
                return cached;
            }
        }
        
        auto results = analyze_trends();
        trend_cache_["trends"] = {now.time_since_epoch().count(), results};
        return results;
    }
};
```

## Testing

### Unit Tests

```cpp
#include <gtest/gtest.h>
#include "analytics_dashboard.hpp"

TEST(AnalyticsDashboard, TrendAnalysis) {
    MetricsHistoryBuffer history;
    
    // Populate with synthetic data
    for (int i = 0; i < 100; ++i) {
        MetricsPoint point;
        point.timestamp_ms = i * 1000;
        point.cpu_percent = 20.0 + i * 0.5;  // Upward trend
        history.push_point(point);
    }
    
    AnalyticsDashboard analytics(history);
    auto trends = analytics.analyze_trends();
    
    ASSERT_GT(trends.size(), 0);
    EXPECT_EQ(trends[0].direction, "up");
    EXPECT_GT(trends[0].slope, 0);
}

TEST(AnalyticsDashboard, AnomalyDetection) {
    MetricsHistoryBuffer history;
    
    // Normal data with one spike
    for (int i = 0; i < 100; ++i) {
        MetricsPoint point;
        point.cpu_percent = i < 50 || i >= 55 ? 30.0 : 100.0;  // Spike at 50-55
        history.push_point(point);
    }
    
    AnalyticsDashboard analytics(history);
    auto anomalies = analytics.detect_anomalies();
    
    EXPECT_GT(anomalies[0].anomalies.size(), 0);
}

TEST(AnalyticsDashboard, CorrelationAnalysis) {
    MetricsHistoryBuffer history;
    
    // Correlated metrics
    for (int i = 0; i < 100; ++i) {
        MetricsPoint point;
        point.cpu_percent = 20.0 + i * 0.5;
        point.memory_mb = 256 + i;  // Correlated
        history.push_point(point);
    }
    
    AnalyticsDashboard analytics(history);
    auto correlations = analytics.analyze_correlations(0.5);
    
    EXPECT_GT(correlations.size(), 0);
    EXPECT_GT(correlations[0].correlation, 0.7);
}
```

## Future Enhancements

1. **Advanced Prediction**: ARIMA, exponential smoothing
2. **Machine Learning**: Anomaly detection with neural networks
3. **Pattern Recognition**: Identify repeating patterns
4. **Forecasting Confidence Intervals**: Predict ranges, not just points
5. **Causal Analysis**: Identify which metrics cause which
6. **Streaming Algorithms**: Handle unbounded data streams efficiently
7. **Distributed Analysis**: Analyze across multiple nodes
8. **Interactive Dashboards**: Real-time analytics visualization

## See Also

- [Advanced Chart Features](ADVANCED_FEATURES.md)
- [Metrics Collection](../include/metrics_collector.hpp)
- [REST API Documentation](REST_WEBSOCKET_API.md)
- [Developer Guide](DEVELOPER.md)
